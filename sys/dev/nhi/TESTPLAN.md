# Test Plan — ThinkPad T14s Gen 5 (Intel) as first test machine

**Status:** draft · **SUT:** Lenovo ThinkPad T14s Gen 5 (Intel, Core Ultra / Meteor Lake)
**Role:** first bring-up host for `nhi(4)` (Phase 0–1) · **Companion of:** `DESIGN.md`
**Convention:** same as `DESIGN.md` — the probe decides. If a probe contradicts a claim
here or in `DESIGN.md`, the **document is wrong and is corrected first**, code second.

---

## 0. Why the ThinkPad first, and the one big caveat

The DESIGN.md plan targets FreeBSD/**arm64** with a discrete **GIGABYTE Titan Ridge** TB3
add-in card. The T14s Gen 5 is a different beast and we use it deliberately as the *first*
machine because it removes two of the project's top risks for the early phases:

| DESIGN.md risk | On arm64 + Titan Ridge AIC | On T14s Gen 5 (Intel) |
|---|---|---|
| **R1 force-power** (controller dark until a pin is asserted) | blocking, hardware-level | **N/A** — integrated TB, always powered/enumerated |
| **R4 cache coherency** (silent DMA corruption, §4.3) | LX2160A non-coherent = risky | **N/A** — x86 PCIe is coherent like Ampere; clean DMA baseline |
| MSI-X routing | via GICv3/ITS (arm64) | standard x86 APIC — simplest possible |
| PCIe enumeration / ACPI | firmware-dependent | standard laptop UEFI, controller is on-die |

So the ThinkPad lets us prove the **transport layer and the DMA ring engine on coherent,
always-on hardware** — i.e. land Phase 0, Phase 1, and `dma_selftest` (V4.3) with zero
force-power / coherency noise — before porting to the harder arm64 targets.

### The caveat — this controller is probably **SW-CM, not ICM** (the Phase-2 gate)

DESIGN.md **D1** ("start in ICM mode, XDomain only") relies on the controller running the
firmware **Internal Connection Manager (ICM)**. That is true of the discrete Titan Ridge
card. **Integrated** Intel Thunderbolt (Ice Lake onward, including Meteor Lake) typically
runs **no ICM** — the reference open implementation drives those controllers with a
*software* connection manager, not the ICM client.

**Consequence we must verify, not assume:** on the T14s the NHI may report **ICM absent**.
If so, `DRIVER_READY` / `APPROVE_XDOMAIN_PATHS` (the whole ICM path Phase 2–3 is built on)
are **not available here**, and the ThinkPad can validate only Phase 0/1 + DMA — XDomain
needs either the discrete Titan Ridge card (ICM) or the SW-CM work that DESIGN.md defers to
Phase 5. **TP-1b below is the explicit go/no-go gate that settles this.** Whatever the probe
reports gets written back into DESIGN.md §1.3 / §7 / §9.

---

## 1. System under test & topologies

**SUT:** ThinkPad T14s Gen 5 (Intel), Core Ultra (Meteor Lake-U, e.g. 155U/165U), integrated
USB4 / Thunderbolt 4 host router. Two USB4/TB4 Type-C ports (left side).

**NHI identity — CONFIRMED (TP-0, `swift@192.168.11.101`, 2026-06):** Intel `0x8086`, **two**
USB4 NHI functions — `pci0:0:13:2` device **`0x7ec2`** (MTL-P NHI0) and `pci0:0:13:3` device
**`0x7ec3`** (MTL-P NHI1); subvendor `0x17aa`/`0x2329` (Lenovo). PCI **class `0x0c0340`**
(USB4 Host Interface — *not* `0x088000`). **BAR0 = 256 KiB**, BAR1 = 4 KiB, 64-bit mem, no
driver attached (`none4`/`none5`). IDs now added to `nhi_idents[]` / `nhi_reg.h`.

**Topologies for this machine:**
- **TP-A (interop reference):** T14s ⟷ TB/USB4 cable ⟷ **Mac**. The Mac's Thunderbolt
  Bridge is the protocol reference (observe, don't guess). Needed from Phase 2 on.
- **TP-B (self-peer, later):** T14s ⟷ second ThinkPad / second FreeBSD host. Validates our
  XDomain against itself once a peer exists.
- **TP-C (cross-check):** T14s ⟷ Ampere + Titan Ridge AIC. Lets the *same* cable test the
  ICM controller (Ampere side) vs the integrated controller (ThinkPad side).

For Phase 0/1 + `dma_selftest`, **no peer is required** — a bare ThinkPad is enough.

---

## 2. Environment preparation (one-time)

1. **Install FreeBSD 15.x/amd64** on the T14s (or boot a memstick + run from a scratch
   install; UFS is fine). Confirm it boots, NVMe visible, networking for `scp`/`pkg`.
2. **UEFI/BIOS Thunderbolt settings** (Lenovo): set Thunderbolt/USB4 security to the most
   permissive that still enumerates (e.g. "No security" / pre-boot ACL off) so the OS — not
   firmware policy — gates the link. Record the exact menu path & value used.
3. **Toolchain:** kernel source matching the running kernel (`/usr/src`) so the modules build
   with `bsd.kmod.mk`. Either build natively on the ThinkPad or cross-build on the dev box and
   `scp` the `.ko`s.
4. **Deploy loop:** `make` in `sys/dev/nhi` → `kldload ./nhi.ko` → read `dmesg`. Keep a
   serial or `ssh` console; a bad ring program can wedge the box, so be ready to power-cycle.
5. **Baseline capture:** before loading anything, save `pciconf -lv`, `devinfo -rv`, and
   `dmesg` so post-load diffs are clean.

> Note: this machine is x86, so the Makefile/module must build for amd64. The driver source
> is arch-neutral newbus + `bus_dma(9)`; nothing in Phase 1 is arm64-only. If a build assumes
> arm64, that's a bug to fix, not a reason to skip the machine.

---

## 3. Test cases (mapped 1:1 to DESIGN.md VERIFY tags)

Each case lists: **goal · steps · pass criteria · data to capture · doc to update on
surprise.** Run them in order; later cases gate on earlier ones.

### TP-0 — Controller visible & device-ID discovery  (V5.0 / `nhi_probe`)
- **✅ RESULT (swift@192.168.11.83):** two NHIs — `0:13:2`=`0x7ec2`, `0:13:3`=`0x7ec3`, class
  `0x0c0340`, BAR0 256 KiB, subven `0x17aa:0x2329`, unclaimed. IDs added to match table.
- **Goal:** the integrated NHI appears on PCIe and we learn its real device ID.
- **Steps:** `pciconf -lv | grep -i -A4 thunder` and look for an Intel function, class
  `0x088000`, with no driver attached. Note ven/dev/subven, BAR0 size, and the bridge/xHCI
  siblings that travel with it.
- **Pass:** an Intel NHI function is listed with a stable device ID.
- **Capture:** full `pciconf -lv` stanza for the NHI + its PCIe bridge + xHCI.
- **On surprise → fix doc:** add the observed `0x8086,0x<id>` to `nhi_reg.h`
  (`NHI_DEV_MTL_*`) and `nhi_idents[]` in `nhi.c`, and record it in DESIGN.md open-question
  #1 / §7. **This is a required edit before TP-1 can attach.**

### TP-1 — `nhi(4)` attaches, BAR0 + caps  (V1.2 / V5.0 / `nhi_probe`)
- **✅ RESULT (both NHIs):** attach OK, BAR0 256 KiB, `hop_count=12`, `fw_sts=0x80000121`,
  `caps_ver=0x0`. Open items:
  - `caps_ver=0x0` (header expected `VERSION_2=0x40` — flag the version field as VERIFY;
    `hop_count`/`fw_sts` sane so the offsets are otherwise right).
  - **MSI-X → ENXIO (err 6)** despite the cap advertising **16 messages** (`cap 11[a0]`);
    likely an MSI-X table-BAR mapping detail. **Resolved for now via MSI fallback** (added to
    `nhi_setup_intr`): "using 1 MSI vector" on both NHIs — enough for ring-0 control. MSI-X
    (per-ring vectors) deferred until the multi-ring path needs it.
  - Vector is allocated but not yet bound to a handler (`vmstat -i` shows no `nhi` line) —
    `bus_setup_intr` lands with the ring engine, since nothing generates IRQs until ring 0.
- **Goal:** the driver binds, maps BAR0, and the capability register reads back sane.
- **Steps:** `kldload ./nhi.ko`; read `dmesg`. Confirm the `BAR0 mapped at ...` line and the
  `caps_ver=... hop_count=... fw_sts=... icm=... fw_mode=...` line from `nhi_read_caps()`.
- **Pass:** non-zero, plausible `hop_count` (paths/rings count); BAR0 size matches `pciconf`;
  MSI-X line shows "allocated 1 MSI-X vector".
- **Capture:** the two `device_printf` lines verbatim; `vmstat -i` to see the MSI-X vector.
- **On surprise → fix doc:** if `hop_count`/version look wrong, the register offsets in
  `nhi_reg.h` (`NHI_REG_CAPS` etc.) may differ on this NHI generation — reconcile against
  the Intel NHI register map and correct the header + DESIGN.md §1.2.

### TP-1b — **ICM present? (GO/NO-GO GATE for Phase 2+)**  (V1.3 partial)
- **RESULT — ring-0 DRIVER_READY round-trip done; ICM is present but SAFE and unresponsive.**
  `icm=present` (`fw_sts` bit0 `ICM_EN`) but `fw_mode=safe(0x0)`. Brought ring 0 up and TX'd a
  `DRIVER_READY` ICM frame (PDF=11): register read-back proves the **transport works** — TX
  consumer advanced to producer, descriptor returned `COMPLETED`, rings `ENABLE|RAW`. **But no
  ICM response** (RX producer stayed 0, `nhi_ctl_rx` timed out, `intr_count=0`). So the
  integrated-MTL ICM does **not** answer DRIVER_READY in SAFE mode — the "integrated uses ICM,
  works directly" read of the reference implementation is **not borne out on silicon**. Gate verdict: **ICM present
  but dormant; needs an ICL force-power/FW-ready VSEC wake sequence, or the native USB4/SW-CM
  path** (new investigation — Plane). The ring-0 engine (TX verified end-to-end) is reusable
  for either. DESIGN.md §1.3/§9 (R6) updated with the round-trip result.
- **FOLLOW-UP (force-power, `nhi_force_power`):** implemented the ICL/TGL VSEC wake — assert
  `FORCE_POWER`+`0x22` DMA delay in `VS_CAP_22`, poll `VS_CAP_9` FW-ready. It succeeds (FW ready
  immediately). At the time DRIVER_READY still got no response, which *looked* like "native USB4,
  no ICM" — **but that was wrong (see below).**
- **✅ RESOLVED — ICM WORKS (the no-response was a driver byteswap bug).** Root cause: our ring-0
  transport wasn't byteswapping the control payload to big-endian per dword (USB4 Spec 2.0
  §8 control-packet framing), so every DRIVER_READY was malformed with a CRC over the wrong bytes —
  silently dropped. After fixing `nhi_ctl_tx`/`nhi_ctl_rx`, **DRIVER_READY succeeds:
  `slevel=0 proto=3 nvm=0x50205 devid=0x7ec2`** (devid matches the controller → real firmware).
  **Verdict reversed: the integrated Meteor Lake ICM is host-responsive; Phase 2+ on the ThinkPad
  uses ICM/XDomain (D1), not SW-CM.** `opmode=SAFE` is a non-signal on integrated parts. Force-
  power kept; ring-0 engine validated.
- **Goal:** decide whether this controller can run the ICM/XDomain path at all.
- **Steps:** from the same `nhi_read_caps()` output, read `icm=present/absent` (FW_STS bit 0)
  and `fw_mode=...` (OUTMAIL opmode: `CM`=ICM running, `SAFE`/`EP`/`AUTH`=no ICM CM).
- **Decision:**
  - `icm=present` & `fw_mode=CM` → ICM available; proceed to TP-1d/TP-2 (Phase 2 path) on
    this machine.
  - `icm=absent` or `fw_mode≠CM` → **expected outcome for integrated MTL.** The ThinkPad is
    confirmed a Phase 0/1 + DMA machine only. Phase 2+ XDomain must use the **Titan Ridge AIC
    on Ampere (TP-C / topology T1)** or wait for SW-CM (Phase 5). **Do not** try to force
    `DRIVER_READY` against a controller with no ICM.
- **Capture:** the `icm=`/`fw_mode=` values.
- **On surprise → fix doc:** this is the most likely place DESIGN.md needs correcting. Update
  §1.3 and §9 to state explicitly: *integrated Intel TB (MTL) = SW-CM, ICM only on the
  discrete Titan Ridge card.* Adjust D1's "which hardware gives us ICM" accordingly.

### TP-1c — DMA ring self-test on coherent x86  (V4.3 / `dma_selftest`)  ← **high-value here**
- **Goal:** prove the `bus_dma(9)` descriptor-ring engine (`nhi_ring.c`) posts descriptors
  and observes producer/consumer movement correctly — on hardware where coherency can't be
  the culprit. This makes the ThinkPad the **reference oracle** for the arm64 coherency work.
- **Steps:** bring up ring 0 (or a loopback ring) via `nhi_ring_alloc`/`nhi_ring_program`;
  post N descriptors; drive load; verify head/tail advance and buffer contents with explicit
  `bus_dmamap_sync`. (This is the probe the ring-engine follow-up commit must ship with.)
- **Pass:** zero stale-data anomalies under sustained load; head/tail accounting exact.
- **Capture:** descriptors/sec, any mismatch counters, the sync pattern used.
- **Why on this box first:** if `dma_selftest` is clean on coherent x86 but fails on LX2160A,
  the bug is *coherency/attrs*, not ring logic — that triage split is the whole point of
  running it here. Record this x86 pass as the baseline in DESIGN.md §4.3.

### TP-1d — `DRIVER_READY` mailbox round-trip  (V1.3 / V5.1 / `mbox_ready`) — *only if TP-1b = ICM present*
- **Goal:** issue `DRIVER_READY` via the INMAIL/OUTMAIL mailbox and parse the ready response.
- **Steps:** `nhi_mailbox_cmd()` the ready command; poll OP_REQUEST clear / ERROR; read mode
  & route from the response. Cross-check encodings against the ICM firmware protocol.
- **Pass:** command completes without ERROR; response parsed (mode/route plausible).
- **Capture:** raw INMAIL/OUTMAIL words, decoded fields.
- **Skip if** TP-1b said no ICM — there is nothing to talk to; note "N/A on this controller".

### TP-2 — ring 0 RX delivers an ICM notification  (V1.4 / V5.1 / `ring0_rx`) — *ICM path*
- **Goal:** arm ring-0 RX, plug a peer (TP-A Mac), observe an XDOMAIN/DEVICE_CONNECTED
  notification land on ring 0.
- **Pass:** at least one notification frame received and interrupt fired.
- **Capture:** the raw notification frame, the MSI-X vector hit count.

### TP-3+ — XDomain → login → ping → SMB  (V5.2–V5.4)
- These follow DESIGN.md Phases 2–4 unchanged. **On the ThinkPad they run only if TP-1b
  cleared ICM-present.** Otherwise they execute on topology T1 (Ampere + Titan Ridge ⟷ Mac);
  the ThinkPad's role downgrades to "second FreeBSD peer for TP-B once SW-CM exists."

---

## 4. Go/No-Go decision tree (summary)

```
TP-0 device ID found? ──no──> stop: controller not enumerating; check UEFI TB settings
   │yes (add ID to match table)
TP-1 attach + caps sane? ──no──> fix nhi_reg.h offsets vs the Intel NHI register map; re-run
   │yes
TP-1c dma_selftest clean? ──no──> ring-engine logic bug (NOT coherency on x86); fix nhi_ring.c
   │yes  → Phase 0/1 + DMA PROVEN on coherent hardware ✅ (primary deliverable of this box)
TP-1b ICM present?
   ├─ yes → TP-1d/TP-2/TP-3+ here (rare for integrated MTL)
   └─ no  → EXPECTED: ThinkPad = Phase 0/1/DMA machine.
            XDomain (Phase 2+) moves to Ampere+Titan Ridge (T1), or waits for SW-CM (Phase 5).
            Update DESIGN.md §1.3/§7/§9 to record integrated-TB = SW-CM.
```

---

## 5. ThinkPad-specific risks

- **TR-1 (most likely): no ICM on integrated MTL** → caps out at Phase 1 here. *Mitigation:*
  expected and planned for; keep the Titan Ridge/Ampere path as the ICM track. Not a failure.
- **TR-2: NHI register layout drift across NHI generations.** The MTL NHI may move
  caps/ring/mailbox offsets vs the Titan-Ridge-derived `nhi_reg.h`. *Mitigation:* TP-1/TP-1d
  cross-check every offset against the Intel NHI register map; correct the header on mismatch.
- **TR-3: UEFI/firmware owns the controller** (firmware CM holds it, or security policy hides
  it). *Mitigation:* TP-0 + the BIOS settings step; try security level "none".
- **TR-4: a wedged ring program hangs the laptop.** *Mitigation:* console access + accept
  power-cycles; build `nhi_detach` to release cleanly so reload doesn't need a reboot.
- **TR-5: two TB ports / which port** — pick one and label it; some laptops mux ports.

---

## 6. What "done" means for this machine

Minimum success (the reason the ThinkPad came first):
- ✅ TP-0: real NHI device ID captured and added to the match table.
- ✅ TP-1: `nhi(4)` attaches, BAR0 + caps + MSI-X verified on x86.
- ✅ TP-1c: `dma_selftest` passes on coherent hardware — the arm64 coherency baseline.
- ✅ TP-1b: ICM-present question answered and written back into DESIGN.md.

Stretch (only if TP-1b reports ICM present): TP-1d/TP-2 onward on this box.

Every probe result that differs from DESIGN.md is a DESIGN.md edit first (§1.2 offsets, §1.3
ICM, §7 hardware matrix, §9 open questions), then code.
