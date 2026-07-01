# Thunderbolt 3/4/5 on FreeBSD/arm64 + SMB-over-Thunderbolt for Mac clients — Design & Development Document

**Status:** design draft · **Author:** networkextension · **Target:** FreeBSD 15.x / arm64
**Scope:** a native FreeBSD Thunderbolt/USB4 stack (host side) and, on top of it, a
host-to-host **Thunderbolt networking** interface used to serve **SMB** to macOS clients
at near-cable speed.

> Convention of this document: **the document is the deliverable; code is written only to
> verify the document's claims.** Every section that asserts a hardware/protocol fact ends
> with a **`VERIFY:`** note describing the minimal probe that confirms it. If a probe
> contradicts the text, the text is wrong and must be corrected first.

---

## 0. Goals and non-goals

### 0.1 Goals
1. **G1 — FreeBSD/arm64 Thunderbolt host stack.** Bring a Thunderbolt 3 host controller up
   on FreeBSD/arm64: enumerate it on PCIe, drive its Native Host Interface (NHI), talk to
   its connection manager, and establish a host-to-host (**XDomain**) link.
2. **G2 — Thunderbolt networking (`if_tbt`).** Implement the Apple *ThunderboltIP* /
   Linux *thunderbolt-net* protocol so that two TB-connected hosts each expose a standard
   Ethernet `ifnet`. This is the "雷电网桥" (Thunderbolt Bridge).
3. **G3 — SMB storage for Mac.** Run Samba over `if_tbt` so a Mac connects a TB cable to the
   FreeBSD/arm64 box and gets a fast SMB share (target: saturate the link, ~10–20 Gbit/s
   effective on TB3).
4. **G4 — Forward path to TB4/TB5 (USB4).** Structure the stack around the USB4 *router*
   model so TB4 (Goshen/Maple Ridge) and TB5 (Barlow Ridge) are incremental, not rewrites.

### 0.2 Non-goals (initially)
- Full **software connection manager** with arbitrary PCIe/DisplayPort tunnel setup and
  device authorization policy. We start in **firmware-CM (ICM)** mode and do **only XDomain
  networking** — no PCIe tunneling of downstream devices, no DP.
  - **Concrete consequence — a plugged-in TB device (SSD enclosure, dock, eGPU) will NOT
    appear.** Those need a *PCIe/DP/USB3 tunnel* through the fabric (a different data path
    from the host-interface DMA rings we drive): the flow is `ICM_APPROVE_DEVICE` → firmware
    builds the tunnel → the tunneled endpoint hotplugs onto the host PCIe tree (`pci`/`nvme`
    bind it). The ICM raises `DEVICE_CONNECTED` (code 0x03) for such a device, distinct from
    the `XDOMAIN_CONNECTED` (0x06) we act on; `nhi_icm_handle_event()` **only logs** 0x03 and
    does not approve/tunnel it. ThunderboltIP networking is special precisely because it rides
    the host DMA rings directly (just `APPROVE_XDOMAIN_PATHS`, no router path programming),
    which is why it fits the small ICM v1. Real-device tunneling is **Phase 5 / SW-CM** (§ Phase 5).
- Security hardening (DMA attack surface) beyond putting the controller behind the SMMU
  where the platform allows it. Threat-model work is a later milestone (§9).
- Windows/Mac-style GUI device approval. CLI/sysctl policy only.

### 0.3 Why this is worth doing
A Mac's built-in **Thunderbolt Bridge** gives a zero-config, ~20 Gbit/s point-to-point IP
link with no switch. A FreeBSD/arm64 box that speaks the same protocol becomes a **direct-
attached, very fast SMB target** — effectively external storage at internal-bus speeds,
without 10/25 GbE NICs or a switch. ARM server hosts (Ampere) with many PCIe lanes make
good, cheap, low-power heads for this.

---

## 1. Thunderbolt / USB4 architecture primer

### 1.1 What Thunderbolt actually is
Thunderbolt is a **packet-switched fabric** that *tunnels* other protocols over a duplex
high-speed link:
- **TB3**: up to 40 Gbit/s/port; tunnels **PCIe** (gen3 x4 worth) + **DisplayPort** + (TB3)
  USB; Intel-proprietary at the link layer.
- **TB4**: same 40 Gbit/s, but the link layer is **USB4**; stricter requirements (min PCIe
  tunneling, min 2 4K displays, 32 Gbit/s PCIe), and a standardized **router** model.
- **TB5**: **USB4 v2**, 80 Gbit/s symmetric / 120 Gbit/s asymmetric (Barlow Ridge).
- **USB4** is the umbrella spec; TB4/TB5 are Intel's certified supersets.

The fabric is built from **routers** (TB3 calls them *switches*). The host has a **host
router**; each device/hub is a router; they form a tree. Each router has **adapters** (lane
adapters = ports, plus protocol adapters for PCIe/DP/USB). **Paths** (tunnels) are set up
between protocol adapters across the tree.

**VERIFY:** none yet — definitional. (Confirmed against USB4 spec §1, Intel TB3 docs, and
Linux `drivers/thunderbolt/` which is the canonical open implementation.)

### 1.2 The host controller and the NHI
The host controller (Alpine/Titan Ridge for TB3; Maple Ridge for TB4 AICs) appears on the
host **PCIe** bus as one or more functions. The relevant one is the **NHI — Native Host
Interface**:
- A PCIe function with an Intel vendor ID `0x8086` and a controller-specific device ID (e.g.
  Alpine Ridge 2C `0x1578`, Titan Ridge `0x15e8/0x15eb`, Maple Ridge `0x1136`; **Meteor Lake-P
  integrated `0x7ec2`/`0x7ec3`** — two functions, CONFIRMED on the ThinkPad T14s Gen 5).
  PCI **class depends on generation**: discrete TB3 NHIs show **`0x088000`** (generic system
  peripheral), but the **integrated USB4/TB4 NHI shows `0x0c0340`** (Serial bus / *USB4 Host
  Interface*) — confirmed on the ThinkPad. Match on vendor+device ID, not class.
- BAR0 = memory-mapped NHI registers: **TX/RX descriptor rings**, per-ring producer/consumer
  indices, interrupt status/mask, and a **firmware mailbox** (the path to the on-chip ICM).
- The NHI is the *only* thing the host OS DMA-talks to. Everything else (fabric config,
  device approval) is either (a) sent as **control packets through ring 0**, or (b) issued
  as **mailbox commands** to the firmware CM.

**VERIFY (V1.2):** `pciconf -lv` shows the controller; a tiny `nhi` attach reads BAR0 and
prints the ring count and firmware/mailbox capability register. Cross-check offsets against
Linux `drivers/thunderbolt/nhi_regs.h`.

### 1.3 Connection manager: ICM vs SW-CM
Two ways the fabric is managed:
- **ICM (Internal Connection Manager)** — firmware *inside* the controller manages the
  topology, hotplug, tunneling, and device authorization. The OS sends high-level commands
  (`DRIVER_READY`, `APPROVE_DEVICE`, `APPROVE_XDOMAIN_PATHS`, …) via the **NHI mailbox** and
  receives **notifications** on ring 0. This is what Windows and modern Linux use on
  ICM-capable controllers. **Much smaller OS surface.**
- **SW-CM (software CM)** — the OS owns the fabric: it walks routers by issuing config
  read/write **control packets** on ring 0, allocates paths, programs adapters, handles
  hotplug. This is the "Apple/`tb`" model and the full Linux CM. **Large, complex.**

**Design decision D1:** *FreeBSD starts in ICM mode and implements only what XDomain
networking needs.* Rationale: the GIGABYTE TB3 AIC (Titan Ridge) is ICM-capable; XDomain
networking needs only `DRIVER_READY`, XDomain discovery, and `APPROVE_XDOMAIN_PATHS` — no
PCIe tunnel programming. SW-CM is a later, optional milestone for tunneling real devices.

> **Which hardware gives us ICM (important).** ICM is a property of the controller's
> firmware, not of "Thunderbolt" in general. **Discrete** TB3 controllers (Alpine/Titan
> Ridge), as on the GIGABYTE AIC, run ICM. **Integrated** Intel Thunderbolt (Ice Lake
> onward, including the **Meteor Lake** controller in the first x86 test box, ThinkPad T14s
> Gen 5 — see `TESTPLAN-thinkpad-t14s-gen5.md`) is expected to run **no ICM**: Linux drives
> those controllers with its *software* CM (`tb.c`), not the ICM client (`icm.c`). So D1's
> ICM path is tied to the **discrete Titan Ridge card**; an integrated controller likely
> validates only Phase 0/1 + DMA until SW-CM (Phase 5) exists. Confirmed/refuted by the gate
> probe TP-1b (reads FW_STS / OUTMAIL opmode).
>
> **Observed (TP-1, swift@192.168.11.83, ThinkPad T14s Gen 5, BOTH NHIs):** the prediction is
> only *half* right. `fw_sts=0x80000121` has **bit0 `ICM_EN` set** — Linux's
> `icm_firmware_running()` test — so ICM firmware **is present** on this integrated MTL
> controller, contradicting "no ICM". **But** OUTMAIL `opmode=0x0 (SAFE)`, not `CM (0x3)`: the
> ICM is **present but dormant / in safe mode**, not actively managing the fabric. Static read
> is therefore **inconclusive** — settle with the `DRIVER_READY` round-trip (does FW go to CM
> mode and ACK, or error/stay safe?). Until then treat ICM here as *unconfirmed, not absent*.
> (Also: `hop_count=12`, `caps_ver=0x0`, MSI-X alloc failed — see TESTPLAN.)

**VERIFY (V1.3) — RESOLVED ✅ (ICM works):** `DRIVER_READY` succeeds on the Meteor Lake box and
the ICM returns a valid ready-response: `slevel=0 proto=3 nvm=0x50205 devid=0x7ec2` (the
`devid` matching the controller proves it's the real firmware answering). **The integrated
Meteor Lake ICM is present and host-responsive — D1's ICM path is alive on this ThinkPad.**

> **⚠️ The two journey blocks below are SUPERSEDED.** They concluded "ICM is a dead end / native
> USB4" — that was a **driver bug, not the hardware**: our ring-0 transport was not byteswapping
> the control-frame payload to big-endian (Linux `ctl.c cpu_to_be32_array`), so every
> `DRIVER_READY` was malformed with a CRC over the wrong bytes and the firmware silently dropped
> it. After fixing `nhi_ctl_tx`/`nhi_ctl_rx` to byteswap per dword, DRIVER_READY succeeds (above)
> with **no change to the controller**. `opmode` still reads `SAFE`, confirming opmode/OUTMAIL is
> meaningless on integrated parts (Linux has no `get_mode` hook for them) — the DRIVER_READY
> response is the real signal. **Net: the SW-CM pivot is withdrawn; the ICM/XDomain path (D1)
> stands.** Force-power (kept) and the ring-0 engine were both validated along the way.

> **Round-trip RESULT (ring-0 DRIVER_READY, both NHIs, swift@192.168.11.83):** ring 0 was
> brought up and a `DRIVER_READY` ICM frame (PDF=11) transmitted. A register read-back proves
> the **transport works**: the TX index's consumer half advanced to the producer and the
> descriptor came back with `COMPLETED` set (`desc.attrs` flags `0xE`), rings `ENABLE|RAW`
> (`opt=0xc0000000`), `notify` bit0 latched. **But the ICM never replied** — the RX producer
> index stayed 0, `nhi_ctl_rx` timed out, `intr_count=0`. So the integrated-MTL ICM is
> **present but dormant in SAFE mode and does not answer DRIVER_READY as-is.** The earlier
> "integrated controllers use ICM and it'll just work" (from the Linux read of
> `icm_icl_driver_ready`) is **not borne out on silicon** without first waking the CM. Next:
> investigate the ICL/TGL **force-power / FW-ready VSEC** wake sequence (PCI config `0xc8`
> bit31 *is* set, suggesting FW-ready) vs. driving this controller with the **native USB4 /
> SW-CM** path. The ring-0 transport itself is done and reusable for either. (`ring0_rx`/V1.4
> is half-proven: TX path verified end-to-end; RX path exercised but no peer/CM frame yet.)
>
> **Force-power RESULT (VSEC `icl_nhi_force_power`, both NHIs):** asserting `FORCE_POWER` in
> `VS_CAP_22` (with the documented 0x22 DMA delay) succeeds and `VS_CAP_9` reports FW-ready
> immediately — **but `opmode` stays `SAFE` and DRIVER_READY still gets no ICM response.** Our
> frame matches Linux's request byte-for-byte and the firmware is powered+ready, so the
> conclusion is that **this controller does not run a host-responsive ICM — it is in native
> USB4 mode.** On integrated parts Linux reads no opmode (no `get_mode` hook), so our `SAFE`
> reading is a non-signal; the real signal is the silent ICM. **D1's ICM path is a dead end on
> this ThinkPad; the way forward is the software CM (raw ring-0 config read/write over the USB4
> router model, Linux `tb.c`)** — what DESIGN.md filed as the Phase-5 SW-CM stretch. Force-power
> stays (the controller must be powered to drive it at all, SW-CM included). The discrete Titan
> Ridge card remains the ICM track (topology T1).

### 1.4 The control path (ring 0)
Ring 0 carries **Thunderbolt control protocol** frames: config read/write to a router's
config spaces (router / adapter / path / counters), plus **events** (hotplug, XDomain
requests). Each frame carries a **route string** (the path from the host router to the
target router, nibble-per-hop) and a CRC. In ICM mode the OS mostly receives *notifications*
here and sends *XDomain* request/response frames; it does **not** need to drive raw config
read/write for the networking use case.

**VERIFY (V1.4):** arm the RX ring, plug a second host, and observe an **XDomain connection
request** notification arriving on ring 0.

---

## 2. Thunderbolt networking — the "bridge" (the core of this project)

### 2.1 XDomain: host meets host
When the far end of the cable is **another host** (not a device), the two host routers form
two **domains**; each sees the other as an **XDomain**. They run the **XDomain protocol** on
ring 0:
1. **UUID exchange / discovery** — each host has a domain UUID; they exchange it.
2. **Properties request/response** — each host serves a **property directory**: a nested
   key→value tree (`KEY`, type, value) describing the host (vendor/device name) and the
   **services** it offers. Networking is one such service.
3. The **network service** entry advertises: `prtcid` (protocol ID), `prtcvers`,
   `prtcrevs`, `prtcstns` (a settings bitmask), and a service UUID. Apple's ThunderboltIP
   uses a well-known protocol UUID; both ends must match.

**VERIFY (V2.1):** dump the remote property directory after connect; confirm a `network`
service entry with the ThunderboltIP protocol UUID and `prtc*` keys. Compare to Linux
`tb_xdomain` and `drivers/net/thunderbolt/` (the `APPLE_THUNDERBOLT_IP_PROTOCOL_UUID`).

> **Progress (XDOMAIN_CONNECTED captured, MacBook Pro 2018 ⟷ ThinkPad):** with the Mac plugged
> in, `nhi_icm_drain()` reaped a ring-0 ICM event `pdf=10 code=0x06` (XDOMAIN_CONNECTED, 56 B)
> carrying two domain UUIDs — remote (Mac) `21d9e378-566a-ab53-a28f-fb379786cb9d` and local
> (our box) `479f8780-511f-cb84-ffff-ffffffffffff`. The local UUID **matches what macOS shows**
> for our box ("Unknown Cross Domain Device"), confirming the XDomain link from both ends. The
> all-`F` lower half is a firmware-default UUID (the OS never set one — `tb(4)` will). The Mac
> says "Unknown" only because we don't yet **serve a property directory**: that (respond to
> XDomain `PROPERTIES` requests with our host name + the ThunderboltIP network service) is the
> next Phase-2 step toward V2.1 proper.

> **Progress 2 (XDomain discovery responder live).** Added a ring-0 RX **event-loop kthread**
> (`nhi_event_loop`) + an XDomain responder (`nhi_xdomain.c`): builds our property directory
> (host identity), and answers the Mac's `UUID_REQUEST` / `PROPERTIES_REQUEST` /
> `PROPERTIES_CHANGED` over ring 0 (PDF 6/7). Key fix: responses must use the **peer route**
> from `XDOMAIN_CONNECTED` (`0:1`), not the echoed request route — the request carries the
> `0x80000000` reply-flag in `route_hi`, and echoing it bounced our reply back to our own host
> router (observed as a `pdf=7` loopback). With peer-route, the loopback is gone.
> **Two open items:** (a) macOS labels non-Apple peers "Unknown" by device type regardless, so
> the panel name is *not* the success signal — the **network bridge** (link-local + mDNS) is;
> (b) a recurring `pdf=12` (ICM_RESP, 28 B) after our XDomain TX suggests the **ICM may mediate
> XDomain property serving** in ICM mode — to confirm before relying on raw ring-0 serving.
> Next: advertise the `network` service, then Phase 3 (APPROVE_XDOMAIN_PATHS → data rings →
> `tbt0` → the Mac's Thunderbolt Bridge gets a `169.254` IP → mDNS/SMB).

### 2.2 Path setup and the data rings
Once both ends agree on the network service:
- The CM (ICM, via `APPROVE_XDOMAIN_PATHS`) sets up a **bidirectional high-speed path**
  (a tunnel) between the two host routers dedicated to this service, mapped to a pair of
  **NHI rings** (one TX, one RX) distinct from ring 0.
- The driver allocates those rings (descriptors + frame buffers) and a **per-direction E2E
  flow-control** credit scheme (the rings have hop/credit config).

**VERIFY (V2.2):** after approval, confirm the assigned ring numbers and that descriptors
posted to the RX ring get consumed when the peer transmits.

### 2.3 ThunderboltIP framing and the `ifnet`
The networking protocol carries **Ethernet frames fragmented into TB packets**:
- A small **per-fragment header**: `frame_size`, `frame_index`, `frame_id`, `frame_count`
  — used to split an Ethernet frame across TB ring descriptors and reassemble on RX. There
  is a CRC/checksum field over the frame.
- Before data flows, a **login** handshake: a `ThunderboltIP login` request/response
  (sequence number, the peer's properties) establishes the per-direction settings (e.g.
  whether checksums/segmentation are offloaded).
- The driver presents a **standard Ethernet `ifnet`** (`tbt0`): a synthetic MAC (derived
  from the route/UUID), MTU (Apple uses ~64 KB jumbo internally; expose a sane MTU), and the
  usual `if_transmit`/input path. To the IP stack it's just a NIC. To Samba it's just an
  interface.

**Design decision D2:** model `if_tbt` exactly like a normal `ifnet` driver
(`ether_ifattach`, `iflib`-style or classic). Multichannel SMB, jumbo frames, and RSS are
out of scope for v1; get a single fast queue working first.

**VERIFY (V2.3):** with `if_tbt` up on both ends and static IPs assigned, `ping` then
`iperf3` across the cable; confirm Ethernet frames round-trip and measure throughput.

### 2.4 macOS interop
macOS exposes the same thing as **Thunderbolt Bridge** (a built-in `bridgeXXX`/`enX`
interface over TB). Apple's protocol is the reference; matching the **protocol UUID,
property directory shape, and login handshake** is what makes a Mac talk to us with **zero
configuration on the Mac side** (the user just assigns/uses the Thunderbolt Bridge IP).

**VERIFY (V2.4):** plug a Mac into the FreeBSD host; the Mac's *Thunderbolt Bridge*
interface comes up and `ping`s the FreeBSD `tbt0` IP without any Mac-side driver.

---

## 3. FreeBSD gap analysis

| Capability | FreeBSD today | Needed |
|---|---|---|
| PCIe enumeration of the NHI | yes (`pci`/ACPI) | confirm the controller appears; force-power (ARM) |
| MMIO / busdma rings | yes (`bus_dma`) | TB-specific ring/descriptor management |
| Interrupts (MSI/MSI-X) | yes | NHI ring + mailbox interrupts |
| **TB core / router model** | **none** | minimal ICM client (mailbox, notifications) |
| **XDomain protocol** | **none** | discovery, properties, login |
| **ThunderboltIP net driver** | **none** | `if_tbt` (`ifnet`) |
| SMMU/IOMMU | arm64 SMMU exists | optional: place NHI DMA behind SMMU |
| ACPI TB objects / `_OSC` | n/a on these ARM platforms | firmware exposes the device; force-power handled out-of-band |
| `ifnet`, `ether_*`, bpf | yes | reuse as-is |
| Samba | in ports | reuse as-is |

**Net:** FreeBSD has every *generic* building block (PCIe, busdma, MSI, `ifnet`). The
missing pieces are entirely **Thunderbolt-specific**: an NHI driver, an ICM client, the
XDomain protocol, and `if_tbt`. There is **no existing FreeBSD Thunderbolt code** to start
from; Linux `drivers/thunderbolt/` + `drivers/net/thunderbolt/` and the USB4 spec are the
references. (NetBSD/OpenBSD have none either.)

---

## 4. Proposed FreeBSD architecture

### 4.1 Module / newbus layout
```
pci (NHI function)
└── nhi(4)            BAR0 regs, ring engine, MSI/MSI-X, mailbox to ICM
    └── tb(4)         "thunderbolt domain" bus: ICM client, control-ring dispatch,
        │             XDomain discovery + property service, event handling
        ├── tbxd      an XDomain peer (a discovered remote host)
        │   └── if_tbt(4)   ThunderboltIP service → ifnet "tbt0"
        └── (future) tunnel manager for PCIe/DP (SW-CM)
```
- **`nhi(4)`** — pure transport: map BAR0, set up TX/RX **ring engine** (descriptor rings in
  coherent DMA, producer/consumer doorbells), MSI-X vectors, and the **mailbox** read/write
  primitives. Exposes a kobj/`device_if`-style API upward: "send control frame", "register
  RX handler for ring N", "issue mailbox command", "alloc data ring pair".
- **`tb(4)`** — the **domain**: issues `DRIVER_READY`, processes ICM notifications
  (`DEVICE_CONNECTED`, `XDOMAIN_CONNECTED`, hotplug), runs the XDomain state machine, builds
  and serves our **local property directory**, and creates child devices per discovered
  service.
- **`if_tbt(4)`** — the **net driver**: owns a data-ring pair (from `nhi`), does the
  ThunderboltIP login handshake, fragments/reassembles Ethernet frames, and presents an
  `ifnet`.

This mirrors Linux's split (`nhi` ↔ `tb`/`xdomain` ↔ `thunderbolt-net`) but in newbus terms,
which keeps each layer independently testable.

### 4.2 Why this layering
- The **transport vs. protocol** split means the hard, hardware-specific DMA/ring work lives
  only in `nhi(4)` and is verified once. The protocol layers are pure software state
  machines over a clean "send/recv control frame + alloc data ring" API — easy to unit-probe.
- It leaves a clean seam to later drop in a **SW-CM** beside the ICM client without touching
  `nhi(4)` or `if_tbt(4)`.

### 4.3 DMA, rings, and coherency
- Descriptor rings and frame buffers via `bus_dma(9)`. Descriptors must be **DMA-coherent**;
  frame buffers can be streaming with explicit `bus_dmamap_sync`.
- **arm64 coherency matters:** on **Ampere (N1/Altra)** the PCIe path is cache-coherent
  (CCA), so coherent allocations behave like x86. On **LX2160A** parts of DPAA2 are
  non-coherent and PCIe coherency depends on the RC config; ring memory must be allocated
  with the correct `BUS_DMA_COHERENT`/attributes and every descriptor/frame touched with
  `bus_dmamap_sync`. Getting coherency wrong = silent corruption that looks like protocol
  bugs. (We already learned the cost of coherency mistakes on this board's DPAA2 path.)

**VERIFY (V4.3):** a loopback/self-test that posts a descriptor and reads back the
producer/consumer movement with `sync`; confirm no stale-data anomalies under load.

### 4.4 IOMMU / SMMU
Thunderbolt DMA is the classic **DMA-attack** surface. arm64 FreeBSD has an SMMU driver.
For functionality-first we may run the NHI with the SMMU in pass-through, but the design
keeps `bus_dma` tags so that enabling SMMU translation later is a config change, not a
rewrite. (macOS/Windows require VT-d/SMMU for TB DMA protection; we treat it as a milestone.)

---

## 5. Phased implementation plan

Each phase is **gated by a verification probe**; code exists to prove the doc.

### Phase 0 — Get the controller on the bus (prerequisite, partly hardware)
- Make the TB host controller **appear in `pciconf`**. On the test ARM hosts this is the
  current blocker for the GIGABYTE TB3 AIC: **force-power**. A TB3 add-in card keeps its
  controller powered off until a force-power signal is asserted (mainboard TBT header / an
  on-card jumper). Without it: no PCIe link, nothing to drive. (See the LX2160A field note:
  the x8 slot enumerates a normal NIC fine, but the TB3 card is invisible — consistent with
  force-power.)
- Decide the host: **Ampere N1/Altra** is the better primary target (many PCIe lanes,
  coherent, SMMU, standard ACPI PCIe). LX2160A is a secondary target (force-power + non-
  coherent + only 2 of 6 PCIe controllers exposed by firmware).
- **VERIFY (V5.0):** `pciconf -lv` lists the Intel NHI function with a known device ID.

### Phase 1 — `nhi(4)`: transport up
- Attach to the NHI PCI function; map BAR0; identify ring count, mailbox.
- Bring up **ring 0** (control) TX/RX with coherent descriptors; wire MSI-X.
- Implement **mailbox** primitive; issue **`DRIVER_READY`**; parse the ready response (ICM
  present? mode? route?).
- **VERIFY (V5.1):** `DRIVER_READY` succeeds and an ICM "driver-ready response" is parsed;
  ring 0 RX delivers at least one ICM notification (e.g. when a peer is connected).

### Phase 2 — `tb(4)`: domain + XDomain
- ICM event loop: handle `XDOMAIN_CONNECTED`/`DISCONNECTED`.  `DEVICE_CONNECTED` (0x03, a
  downstream TB device — SSD enclosure/dock/eGPU) is **logged only, not tunnelled** (§0.2;
  PCIe tunnelling = Phase 5).
- XDomain discovery: exchange UUIDs; **request the remote property directory**; parse it;
  **serve our own** directory advertising the network service (matching Apple's protocol
  UUID + `prtc*`).
- On match, request **`APPROVE_XDOMAIN_PATHS`** and learn the assigned data-ring pair.
- **VERIFY (V5.2):** with two ARM hosts (or ARM↔Mac), dump both property directories and
  confirm the network service is negotiated and paths approved.

### Phase 3 — `if_tbt(4)`: the bridge
- Alloc the data-ring pair; ThunderboltIP **login** handshake.
- Frame fragmentation/reassembly; `ifnet` create (`ether_ifattach`), TX (`if_transmit`),
  RX (`if_input`); interrupt coalescing.
- **VERIFY (V5.3):** `tbt0` up on both ends; `ping`, then `iperf3` saturating the link;
  confirm a **Mac's Thunderbolt Bridge** reaches `tbt0` with zero Mac-side setup (V2.4).

### Phase 4 — SMB for Mac
- Run **Samba** (smbd) bound to the `tbt0` subnet; enable SMB3, large MTU, multichannel
  off→on experimentation; tune socket/`if_tbt` queue sizes.
- Mac: assign the Thunderbolt Bridge IP, `smb://<tbt0-ip>`; measure sequential and random
  throughput vs. a 10 GbE baseline.
- **VERIFY (V5.4):** Mac mounts the share over TB and read/write throughput approaches the
  `iperf3` ceiling from V5.3.
- **Block storage over `tbt0` (added application, not PCIe passthrough).**  Host↔host TB
  carries an *IP network*, not a PCIe tunnel, so a local disk can NOT be PCIe-passed-through to
  the peer as a raw disk — that would need device/target-mode controller hardware we don't have
  (Intel host controllers don't expose it; Apple Target Disk Mode is Apple-specific — §0.2).
  To hand the peer a *disk* instead of a share, export block storage over the tbt0 network:
  **NVMe-over-TCP** (FreeBSD 15 `nvmft` + `ctld`) → peer `nvme connect` sees an NVMe namespace;
  or **iSCSI** (`ctld`/CTL over a zvol/disk) → peer sees a LUN.  It's a network block device
  (not transparent passthrough) but functionally a fast remote disk at ~cable speed, needing
  no tunnelling — pure software on top of `tbt0`.
- **VERIFY (V5.5, block):** peer `nvme connect` (or iSCSI login) over `tbt0` exposes a
  namespace/LUN; `dd`/`fio` throughput approaches the V5.3 `iperf3` ceiling.

### Phase 5 — TB4 / TB5 (USB4) and SW-CM (stretch)
- Generalize `tb(4)` to the **USB4 router** model; support Maple Ridge (TB4 host) and Barlow
  Ridge (TB5). The XDomain/networking layer is largely protocol-stable across generations;
  the deltas are router config spaces, link training, and higher ring throughput.
- Optional **SW-CM**: raw config read/write on ring 0 to set up **PCIe tunnels** so the box
  can host real TB *devices* (docks, NVMe enclosures), not just host-to-host networking.
- **Other tunnel types beyond PCIe (all require SW-CM/firmware tunnel setup; none in v1 —
  recorded so the scope is explicit).** A full Thunderbolt/USB4 stack tunnels several
  protocols; each is a distinct data path over the fabric:
  - **PCIe tunnelling** — downstream devices (NVMe enclosure, eGPU); tunneled endpoint hotplugs
    onto the host PCIe tree → `pci`/`nvme` bind (§0.2, Phase 5 above).
  - **USB3 tunnelling** — a TB dock's USB hub and everything behind it (USB **sound card**, NIC,
    flash drive) rides a USB3 tunnel.  Key contrast with PCIe: once the tunnel is up the USB
    peripherals **auto-enumerate under the normal FreeBSD USB stack** (`xhci`/`usb`) with no
    per-device driver work — a USB DAC just shows up as `uaudio`/`pcm`.  So "plug the dock and
    the sound card appears" is a *USB3-tunnel* capability gated on tunnel setup, not on us
    writing an audio driver.
  - **DisplayPort tunnelling** — drive an external monitor: a DP-in adapter → DP tunnel → the
    display.  Needs DP-adapter path setup (SW-CM) plus a KMS/DRM sink; the largest of the three.
  - **DDC/CI (monitor brightness / input switch)** — runs as DDC/CI over the DP AUX channel
    *inside* the DP tunnel (`ddccontrol`/`i2c`-style), so it depends on DP tunnelling being up
    first; then external-display brightness/inputs become controllable from the host.
- **VERIFY (V5.6, tunnels):** with SW-CM, a TB NVMe enclosure enumerates as `nvmeN`; a TB dock's
  USB audio appears as `uaudio`/`pcm` (auto, no new driver); an external display lights up and
  DDC/CI brightness adjusts.

---

## 6. ARM-specific concerns (consolidated)

1. **Force-power (TB3 AIC).** Mandatory to even see the controller. Options: on-card jumper;
   wire the card's force-power pin to a board GPIO; or a powered TB4 host AIC that doesn't
   gate. **This is the current gating issue on the LX2160A x8 slot.**
2. **PCIe enumeration / firmware.** FreeBSD inherits whatever UEFI trained. On LX2160A the
   firmware exposes only PCIe #1/#2 in ACPI and uses SerDes lanes for 10 GbE — the TB card's
   slot/controller must actually be lane-routed and ACPI-exposed. On Ampere this is standard.
3. **Cache coherency.** §4.3 — correct `bus_dma` attributes + `sync`; non-coherent LX2160A
   is the risky one.
4. **IOMMU/SMMU.** §4.4 — pass-through first, translation as a milestone.
5. **Endianness / 64-bit MMIO.** arm64 is LE like x86 here; NHI register access is
   straightforward `bus_space`. Watch 64-bit register access width.
6. **MSI-X routing through the host bridge** (ITS on arm64) — already handled by FreeBSD's
   GICv3/ITS; confirm vectors actually fire.

---

## 7. Hardware / test matrix

| Host | Role | PCIe | Coherent | Notes |
|---|---|---|---|---|
| **ThinkPad T14s Gen 5 (Intel)** | **first bring-up host (x86)** | on-die | **yes** | integrated TB4 (Meteor Lake); **no force-power**; coherent DMA baseline; **likely SW-CM, no ICM** (TP-1b). Caps at Phase 0/1 + `dma_selftest`. See `TESTPLAN-thinkpad-t14s-gen5.md` |
| **Ampere N1/Altra** | primary arm64 dev host | many lanes | yes | best NHI host; SMMU; standard ACPI PCIe |
| **SolidRun LX2160A CEX7** | secondary target | 6 ctrls (2 exposed) | partial | force-power + non-coherent; harder |
| **GIGABYTE TB3 AIC** (Titan Ridge) | the TB controller (ICM) | x4 in x8 slot | — | needs **force-power**; **ICM-capable** (carries the ICM path) |
| **Mac (TB3/4)** | client / interop reference | — | — | Thunderbolt Bridge = the protocol reference |

Topologies:
- **T0 (first / x86):** ThinkPad T14s Gen 5 alone — proves Phase 0/1 (`nhi` attach, BAR0,
  caps, MSI-X) and the **DMA ring engine** (`dma_selftest`, V4.3) on coherent, always-on
  hardware, free of force-power/coherency noise. The x86 pass is the baseline the arm64
  coherency work is triaged against. No ICM expected here (TP-1b) → no XDomain on this box.
- **T1 (primary, ICM):** Ampere N1 + GIGABYTE TB3 AIC ⟷ TB cable ⟷ **Mac**. Validates the
  whole G1→G3 path (ICM/XDomain) against Apple's reference.
- **T2:** Ampere N1 ⟷ LX2160A (both with TB AICs) — two FreeBSD/arm64 peers, validates our
  XDomain/networking against *itself*.
- **T3:** LX2160A as host (after force-power solved) — proves the embedded target.

**Available now:** ThinkPad T14s Gen 5 (Intel) in the homelab; 1× GIGABYTE TB3 card.
**Procurement notes:** a second TB AIC (for T2), a TB4 host AIC (Maple Ridge) for Phase 5,
TB/USB4 cables, and ideally a force-power solution (jumper card or TBT-header breakout).

---

## 8. Verification strategy (how "code verifies the doc")

The probes are deliberately tiny and map 1:1 to the `VERIFY:` tags above:

| Probe | Confirms | Form |
|---|---|---|
| `nhi_probe` | V1.2, V5.0 | attach + dump BAR0 ring/mailbox caps |
| `mbox_ready` | V1.3, V5.1 | `DRIVER_READY` round-trip, parse response |
| `ring0_rx` | V1.4, V5.1 | receive one ICM notification |
| `xdomain_dump` | V2.1, V5.2 | print remote property directory + service UUID |
| `paths_approve` | V2.2, V5.2 | `APPROVE_XDOMAIN_PATHS`, learn ring pair |
| `tbt_login` | V2.3, V5.3 | login handshake completes |
| `tbt_ping` | V2.3/2.4, V5.3 | ping/iperf3, Mac interop |
| `dma_selftest` | V4.3 | ring sync correctness under load |
| `smb_bench` | V5.4 | Mac SMB throughput vs iperf3 |

A probe that fails is a **document correction first**, code second. Each probe cites the
Linux source file / spec section it was cross-checked against, so divergences are traceable.

---

## 9. Risks, unknowns, milestones

### 9.1 Top risks
- **R1 — force-power / controller never powers on.** Blocks everything; hardware-level.
  *Mitigation:* jumper/GPIO; or use a host AIC that self-powers.
- **R2 — ICM command/notification ABI is undocumented.** We reverse it from Linux + observed
  traffic; ABI varies by controller generation. *Mitigation:* start with the exact controller
  Linux supports (Titan/Maple Ridge); pin device IDs.
- **R3 — Apple ThunderboltIP details (login/handshake quirks).** Interop is unforgiving.
  *Mitigation:* the Mac is the reference; capture and match byte-for-byte.
- **R4 — arm64 coherency/IOMMU corruption** masquerading as protocol bugs (§4.3). *Mitigation:*
  the `dma_selftest` gate before any protocol work.
- **R5 — throughput far below cable.** XDomain networking shares the link and has framing
  overhead; expect well under the 40 Gbit/s nominal. *Mitigation:* tune ring depth, coalescing,
  jumbo frames, SMB multichannel; set realistic targets (10–20 Gbit/s TB3).
- **R6 — RESOLVED: Meteor Lake ICM works; the earlier "dead end" was a driver byteswap bug.**
  After force-power the `ICM_EN` controller still appeared to ignore `DRIVER_READY` — but the
  real cause was that our ring-0 transport did not byteswap the control payload to big-endian
  (Linux `cpu_to_be32_array`), so the frame + CRC were malformed. With `nhi_ctl_tx`/`nhi_ctl_rx`
  fixed, `DRIVER_READY` succeeds (`slevel=0 proto=3 nvm=0x50205 devid=0x7ec2`). *Net:* the ICM /
  XDomain path (D1) is viable on the integrated ThinkPad after all; the SW-CM pivot is
  withdrawn. *Lesson:* control-frame endianness is load-bearing — verify byte-for-byte against
  Linux `ctl.c` before concluding a controller is unresponsive. `opmode=SAFE` is a non-signal on
  integrated parts (no Linux `get_mode` hook); trust the DRIVER_READY response.

### 9.2 Milestones
- **M1:** controller in `pciconf`, `nhi(4)` attaches, `DRIVER_READY` OK (Phases 0–1).
- **M2:** XDomain negotiated with a Mac; paths approved (Phase 2).
- **M3:** `tbt0` up, `iperf3` across to a Mac (Phase 3).
- **M4:** Mac mounts SMB over TB at target speed (Phase 4) — **the headline result**.
- **M5:** second FreeBSD peer (T2) and/or TB4 (Phase 5).

### 9.3 Open questions to resolve early
1. Exact NHI **device IDs** of the GIGABYTE card (Titan Ridge variant) → pins R2. ThinkPad
   T14s Gen 5 Meteor Lake-P NHI IDs **RESOLVED (TP-0, swift@192.168.11.101): `0x7ec2`+`0x7ec3`,
   two functions, class `0x0c0340`, BAR0 256 KiB** — added to `nhi_idents[]`.
1b. Does the integrated Meteor Lake controller actually present **ICM**, or only SW-CM? → R6.
   **Partial (TP-1):** `ICM_EN` bit IS set (ICM firmware present) but `opmode=SAFE` (dormant),
   so neither "ICM" nor "SW-CM only" is confirmed — the `DRIVER_READY` round-trip is the
   decisive test, still TODO (mbox_ready probe not yet implemented).
2. Does the GIGABYTE card expose a **force-power jumper**, or must we drive a pin? → R1.
3. Ampere host: is the NHI behind an **SMMU stream** we must program even for pass-through?
4. Does macOS require us to advertise any **additional services** in the property directory
   before it brings up Thunderbolt Bridge? (capture real Mac↔Mac traffic if possible).

---

## 10. References (cross-check targets)
- USB4 specification (router/adapter/path model; XDomain).
- Linux `drivers/thunderbolt/` — `nhi.c`, `nhi_regs.h`, `icm.c`, `xdomain.c`, `tb.c`,
  `ctl.c`; and `drivers/net/thunderbolt/` (ThunderboltIP). Canonical open reference for ICM
  ABI, ring layout, property format, and the Apple protocol UUID/handshake.
- Apple "Thunderbolt Bridge" behavior (the interop reference; observe, don't guess).
- Intel Thunderbolt 3 / Titan Ridge / Maple Ridge product briefs for device IDs and BARs.
- FreeBSD `bus_dma(9)`, `pci(4)`, `ifnet(9)`, arm64 SMMU/GIC-ITS docs.

---

### Appendix A — minimal `nhi(4)` attach sketch (illustrative; verifies V1.2/V5.0 only)
> *Illustrative pseudo-driver — the point is to confirm the BAR/register story, not to ship.*
```c
/* match table: Intel TB host NHIs (fill exact IDs from the card, R2) */
static const struct { uint16_t ven, dev; const char *name; } nhi_ids[] = {
  { 0x8086, 0x15eb, "Titan Ridge 4C NHI" },   /* VERIFY against the GIGABYTE card */
  { 0x8086, 0x1137, "Maple Ridge NHI"     },
};
/* attach: pci_alloc BAR0 (mem), read ring-count + mailbox cap, print, alloc MSI-X */
/* then: set up ring0 descriptors (bus_dma, COHERENT), DRIVER_READY mailbox cmd */
```
Real register offsets, mailbox command encodings, and ring descriptor layout are **left to
the verification probes** (V5.1+), each cross-checked against `nhi_regs.h`/`icm.c`. If the
probe disagrees with this document, fix the document.
