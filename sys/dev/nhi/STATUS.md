# 版本报告 — FreeBSD Thunderbolt 栈(2026-07-03,commit 系列至本次)

## 一句话

**M4 headline 达成**:Mac 经 Thunderbolt 线挂载 FreeBSD 的 SMB 共享(SMB 3.1.1,
零 Mac 侧配置);裸隧道 FreeBSD→Mac **4.32 Gbit/s**;#26 中断根因已实证定位
(边沿/电平失配),修复已写入但**未验证**(box 在验证轮失联)。

## 里程碑状态(DESIGN.md 阶段)

| 阶段 | 状态 | 证据 |
|---|---|---|
| Phase 0 力电/PCIe 可见 | ✅ | MTL 集成 NHI 0x7ec2/0x7ec3,force-power VSEC |
| Phase 1 nhi(4) 传输 | ✅ | ring0 双向、DRIVER_READY、mailbox |
| Phase 2 XDomain | ✅ | 发现 + 属性目录 + APPROVE,Mac 识别 "T14 / FreeBSD" |
| Phase 3 if_tbt 数据面 | ✅ | 双向 ping 0% loss,900/900;TBIP 登录互认 ~2s |
| **Phase 4 SMB(M4)** | ✅ **功能达成** | `mount_smbfs //swift@10.10.10.83/tbshare` → SMB 3.1.1,列目录、读文件;Samba 4.23 |
| Phase 5 USB4/SW-CM | ⏸ backlog | 基于 in-tree HCM(sys/dev/thunderbolt)|

## 性能(iperf3/ping 裸隧道 —— 非 SMB)

| 指标 | 数值 | 条件 |
|---|---|---|
| FreeBSD → Mac | **4.32 Gbit/s**,Retr 20 | iperf3 -P1,MTU 1500,TX 流控 |
| Mac → FreeBSD | **283 Mbit/s** | 同上;受 20kHz 忙轮询率封顶 |
| RTT | avg **0.49 ms** | ping,稳态 0% loss |

演进:FreeBSD→Mac 305M→3.01G→4.32G;Mac→FreeBSD 0.5M→32.5M→283M;RTT 47ms→0.49ms。

**SMB 实际吞吐未测得**:持续大文件 dd 会把隧道打崩(弱方向扛不住持续负载)——
"能挂"已达成,"能跑满"卡在 #26 中断修复。

## #26 中断根因(本版本的核心诊断成果)

`dev.nhi.N.intrdump` 探针实测(MTL,单 MSI 回退,MSI-X alloc err 6):

- PCI 层 MSI **已使能**(cap05 enabled,busmaster on,INTx off)—— 不是分配问题
- IVR(0x38c40)全 0 → 所有 ring 路由到向量 0 = 正确 —— 不是路由问题
- THROTTLE=0,无合并 —— 不是节流问题
- **铁证:270 次 CFG 往返只产生 1 次 MSI**;NOTIFY 状态位堆积到 0x01001001
- **根因:NOTIFY 状态位是电平语义,MSI 是边沿**。首帧 0→1 触发一次,之后位
  恒 1、无新上升沿 → 永不再触发。硬件 auto-clear 位(DMA_MISC bit2)在 MTL 上
  **并未真正清位**(Linux 也只在 MSI-X 路径用它;单 MSI/legacy 路径显式写
  INT_CLEAR)。

**修复(已写入,未验证)**:`nhi_ring.c` —— 关硬件 auto-clear
(`DMA_MISC &= ~AUTO_CLEAR; |= DISABLE_AUTO_CLEAR`)+ `nhi_ring0_intr` 读完
NOTIFY 后显式写 `INT_CLEAR(0x37808)+0/+4 = ~0` 重新武装边沿,对齐 Linux
`nhi_clear_interrupt` 非-auto-clear 路径。

**未验证原因**:加载该修复后 box 失联(全网段扫描无踪)。两种可能:又是
if_ure panic(已知会杀整机,见下),或 INT_CLEAR 触发中断风暴。等 console
判读;若是风暴,handler 需加"仅在 ring0 有事时清 + 限流"。

## 已知风险 / 运维事实

1. **if_ure 会杀整机**(#27,HIGH):实拍 panic `ure_miibus_readreg →
   usbd_do_request → page fault`(mii 轮询碰已死 USB 设备)。大流量必掉,掉后
   任何接口状态查询即 panic。缓解:tbt1 隧道当管理通道(已验证可行)。
2. Mac 每次重启会杀掉其 iperf3 server;隧道会话靠 reboot Mac 自举(~90s)。
3. `nhi_icm` 仅手动 kldload,不在 loader.conf —— 重启永远是干净系统。

## 调试工具箱(sysctl dev.nhi.N.*)

`rescan`(re-DRIVER_READY)/ `reset`(力电循环)/ `hopdump`(HOPS 表)/
`portdump`(适配器类型)/ `pathfix`(手动装 TX hop)/ `credits`(RX 路径
credits 实验)/ `intrdump`(MSI/IVR/NOTIFY 快照)—— 全部经事件循环线程执行。

## 关键技术发现存档(对 Linux 逐字核对)

1. **MTL ICM 只编程 lane 侧 RX 路径,NHI 侧 TX hop 空缺** → OS 必须自己
   CFG_WRITE(`nhi_install_tx_hop`)—— 数据面通零的破案点。
2. TBIP 帧 UUID 方向:initiator=发送方(所有帧),macOS 严格校验否则静默丢。
3. RX 环 repost 必须在滚动洞位(head),否则硬件停摆致聋。
4. E2E 两步使能(valid 先,E2E 位后);FRAME 模式 sof/eof mask 在 options+4。
5. TX 描述符所有权检查(POSTED&&!COMPLETED → ENOBUFS)消灭了 1.5 万重传。
6. 轮询率=RX 天花板(吞吐随轮询率线性)—— 中断修复是性能总钥匙。

## 下一步(优先序)

1. console 判读本次失联(ure vs 中断风暴)→ 补验 INT_CLEAR 修复
   (Phase D:vmstat 计数随流量增长、Mac→FreeBSD 上 Gbit、SMB 大文件不崩)
2. RX 转中断驱动,拆忙轮询(省一个核)
3. 补 SMB 真实读写 MB/s(#7 的遗留数字)
4. DESIGN.md 回写全部勘误(#29);cdce 补丁上游(#28)
