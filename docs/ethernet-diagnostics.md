# F767 Ethernet diagnostics candidate

This change configures the Linux build for STM32F767xx, the existing F767
startup file and STM32F767ZITX_FLASH.ld. CubeIDE Debug now uses the project
linker script instead of a Windows absolute path. Clock setup is unchanged.
Both lwipopts.h copies enable software generation/checking of IP, UDP, TCP
and ICMP checksums. The active Core/Src/ethernetif.c disables TX hardware
checksum insertion. tools/sources.mk and CubeIDE compile the Core driver;
LWIP/Target/ethernetif.c is an inactive alternative, not this candidate.

## Read-only debugger telemetry

Inspect `g_eth_diag` using the ELF from this exact candidate. Do not use these
symbol addresses to interpret another firmware. HAL init/start status starts
at UINT32_MAX, then records HAL_OK (0) or the actual HAL error status. PHY init
records the signed LAN8742 result as uint32_t; phy_link_state is signed.

- rx_irq_count / tx_irq_count: HAL completion callbacks.
- rx_frames: frames returned by the driver; rx_input_errors: rejected by lwIP.
- rx_alloc_fail_count: RX pool allocation failures.
- tx_requests: packets entering HAL submission; tx_submit_errors includes BUSY
  retries, not only dropped frames; tx_last_status is the latest HAL result.
- dma_error_count / last_dma_error / last_hal_error: errors captured by the
  callback or TX failure.
- phy_link_state / netif_flags / ipv4_address / dma_status / rx_desc_ready:
  sampled by the existing link thread every approximately 100 ms. IP is in
  network byte order. Descriptor count means non-null DESC2 addresses only;
  it does not prove that RX DMA is progressing.

There is no new task, logging, recovery policy, or register write for sampling.
Counters may wrap, and reads while running are not an atomic snapshot. Compare
successive observations while sending ARP/ping to locate where traffic stops.
PHY up with no RX callbacks suggests checking MAC/DMA/interrupt reception;
RX callbacks without accepted frames suggests the input path; TX failures
require checking the recorded HAL/DMA errors. These are diagnostic leads,
not automatic proof of a root cause.

## Build validation

The checkout lacks Drivers, Middlewares and .tools. Validation was performed
in /tmp/stm32-f767-review-build using copies of the changed project sources,
and read-only dependency paths from /home/khoaiuh/stm32_micro_ros_zackon:
Drivers, Middlewares, micro_ros_stm32cubemx_utils and its GCC toolchain.
No files in zackon were changed. Thus this validates against those installed
dependencies, not a fully self-contained clean checkout.

Command in the staging directory:

    make -f Makefile.linux -j4 TOOL_ROOT=/home/khoaiuh/stm32_micro_ros_zackon/.tools/root/usr

ELF/BIN/HEX/map are copied to build-linux/candidates/f767-ethernet-diagnostics/.
Verified descriptor locations: RX 0x2007C000, TX 0x2007C0A0, in SRAM2.
RX pool is in DMA-capable SRAM1. XML parsing, checksum configuration checks
and git diff --check passed. The initial full build emitted existing warnings
in application code; the changed Ethernet translation unit compiled without
warnings. No firmware was flashed or hardware reset for this change.

Runtime connectivity and telemetry still require a separately authorized
flash and test. Build success does not establish that ARP/ping is fixed.

## Authorized hardware test, 2026-11-11

The user authorized flashing this candidate. A fresh full 2 MB backup was
read to build-linux/backups/stm32-before-f767-test-20261111.bin. OpenOCD
programmed the 262760-byte candidate at 0x08000000 and reported Verified OK,
then reset the target. No mass erase was requested.

After boot, the exact candidate ELF resolved g_eth_diag at 0x200201C8.
HAL init/start and PHY init returned 0; PHY link state was 2. netif flags
were 0x0F, IPv4 bytes C0 A8 04 32 (192.168.4.50), RX descriptors ready = 4.
During the first ping test TX requests/completion callbacks reached 26,
while RX callbacks and rx_frames remained zero; DMA error and RX allocation
failure counts were zero. g_uros_connected was zero. ARP was INCOMPLETE
and ping had no responses. Thus connectivity is not fixed. TX completion
only proves the HAL/DMA completion path, not reception by the PC. Next
investigation should examine the receive path, MAC/DMA configuration,
RMII pins and PHY/MAC reception before assuming recovery is the fix.

## Receive-path investigation

Live GPIO reads confirmed AF11 for PA1/PA2/PA7, PC1/PC4/PC5, PB13 and
PG11/PG13, with alternate-function mode selected. SYSCFG RMII selection
and ETH AHB clocks were enabled. DMASR 0x00660404 indicates RX waiting for
a packet, rather than RX suspended for lack of descriptors. MAC TX-good,
RX CRC/alignment and RX-good-unicast counters read zero; the PC eno1 RX
counter also remained zero while its TX counter increased during ping.

SystemCoreClock was 216000000, but MACMIIAR CR was initially zero (HCLK/42).
The link-thread reinitialization path resets the MAC without calling
HAL_ETH_SetMDIOClockRange again. This is a concrete configuration defect,
though not established as the cause of missing data traffic. For diagnostic
PHY reads, CPU was briefly halted and MDIO commands used CR=0x10 (HCLK/102);
CPU was resumed and confirmed running afterward. No PHY configuration
register was written. This left the volatile MDIO divider corrected.
PHY registers 0/1/2/3/18/27/31 returned 0x3100/0x782D/0x0007/0xC131/
0x60E0/0x000A/0x1058. Basic control has no power-down, isolate or loopback
bit set. Ping remained unsuccessful after this divider correction; RX
callbacks remained zero while TX completions continued increasing.

Recommended next code change: restore the MDIO divider after each successful
HAL_ETH_Init. This change is not yet applied. Further hardware diagnosis
should verify the PHY-to-MCU RMII reference clock and signal continuity;
register snapshots alone cannot prove those signals are physically present.
No additional source changes or flash operations were made in this phase.

## MDIO fix applied and tested

After user approval, eth_init_diagnostic now calls HAL_ETH_SetMDIOClockRange
after every successful HAL_ETH_Init. All active initializations already pass
through this wrapper. No recovery policy or PHY configuration was changed.
Built successfully with no warnings from the changed translation unit.
Candidate: build-linux/candidates/f767-mdio-fix/MotorControlwithMicroROS.bin
(262776 bytes; SHA256 5d04eb9ae8be2007e870b1b03bb113201e003c8ceb43e3472005d0092d2acf0c).
Full pre-flash backup: build-linux/backups/stm32-before-mdio-fix-20261111.bin.
OpenOCD reported Verified OK and reset the MCU.

MACMIIAR read 0x7D0 after boot and after ping: CR=0x10, HCLK/102, without
debugger correction. HAL init/start and PHY init remained successful.
Five ping attempts received no replies; ARP remained incomplete. RX IRQs
and received frames remained 0, while TX requests/completions reached 26.
The PC eno1 also reported zero RX frames. micro-ROS remained disconnected.
The MDIO defect is fixed, but missing Ethernet data traffic is unresolved.
Confirm board identity and external wiring before further experiments.
Physical verification of the 50 MHz RMII reference clock requires suitable
measurement equipment; GPIO register reads do not establish its presence.

## Test with external modules removed

The user removed all external modules and reconnected board power. ST-Link
re-enumerated as USB device 011. eno1 retained only 192.168.3.150 after the
link interruption; 192.168.4.150/24 was reapplied temporarily before testing.
Five ping attempts still received no replies and ARP remained incomplete.
The exact candidate telemetry showed PHY link state 2, netif flags 0x0F,
IP 192.168.4.50, four RX buffers, 36 TX submissions/completions, zero RX
callbacks/frames and zero reported DMA/allocation errors. PC eno1 RX stayed
zero. MDIO divider remained correct (MACMIIAR 0x7D0), CFSR/HFSR were zero,
MCU running, micro-ROS disconnected. Removing modules did not change the
failure. No code changes, reset commands or flash writes were performed.

## Loopback isolation tests

Correction: earlier notes about MAC TX-good counter being zero used an
incorrect offset. MMCTGFCR is at 0x40028168 (not 0x4002816C). The correct
counter reported 56 frames, agreeing with TX completions.

A MAC loopback test immediately after boot increased TX completions from
4 to 16 and RX callbacks/frames from 0 to 12, without allocation/DMA errors.
MACCR loopback was confirmed set (0x0200DE0C), then restored to 0x0200CE0C.
This establishes that internal MAC RX/DMA/lwIP can receive looped frames.
Earlier loopback windows without new TX traffic were inconclusive.

PHY loopback test: BMCR 0x6100 (100Mb/s full duplex, loopback, autoneg disabled)
was read back after writing. Over 12 seconds, TX completions increased from
4 to 16 but RX stayed zero. BMCR was restored and read back as 0x3100, then
MCU reset to normal operation. Final check confirmed MAC loopback off,
MDIO divider correct and CPU running. No flash write occurred. This narrows
investigation toward the MCU/PHY RMII path or PHY behavior, but does not
alone prove a failed component.

ST UM1974 documents JP6 connecting PA7 to PHY RMII_DV and JP7 connecting
PB13 to PHY RMII_TXD1. Inspect both physical jumpers before further firmware
changes: removing external modules does not establish these links are closed.
Reference: https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo-144-boards-mb1137-stmicroelectronics.pdf

## Connection restored after JP6/JP7 installation

The user fitted both JP6 and JP7. After reapplying temporary eno1 IP
192.168.4.150/24, ping returned 5/5 replies with 0% loss and mean 0.323 ms.
ARP resolved 192.168.4.50 to 00:80:E1:00:00:00. Telemetry showed RX and TX
progressing, no DMA/allocation errors, and g_uros_connected=1. Agent logs
confirmed session establishment from 192.168.4.50 and entity creation.
ros2 topic echo /odomfromSTM32 --once returned an Odometry message with
frame_id odom and child_frame_id base_link. No control commands were sent.
This confirms restored Ethernet and micro-ROS data reception with the MDIO
fix candidate. The physical jumper change resolved the observed no-RX
failure; long-duration stability and reconnection stress were not tested.
The host IP remains temporary and may be lost on interface reconnection.

## Wheel-motion observation

A read-only ROS subscriber observed /odomfromSTM32 for 60 seconds and
received zero messages (/tmp/stm32-wheel-odom.jsonl). A Fast DDS SHM warning
was also emitted; independently, two ping attempts failed and MCU
g_uros_connected was zero, confirming the outage is not solely the observer.
Hall signed counts changed: left 6534 to 3524, right 4496 to 7601; Hall
transition error counters remained zero. Internal odom at the later sample
was x=-13.081623 m, y=4.335842 m, theta=-1.215425 rad, vx=wz=0.
This verifies Hall activity, but does not establish physical distance/scale
accuracy or successful ROS delivery during movement.
Ethernet diagnostic error callback count was 2; last DMA status 0x8480
includes RX-buffer-unavailable (not fatal bus error), HAL error 0x8.
RX callbacks increased from 0x666A to 0x66AA between final reads while TX
completions remained 0x6400. No RX pool allocation failures were recorded.
This suggests investigating TX progress/session loss and buffer ownership;
it does not by itself establish motor interference or a single root cause.
No command velocities, resets or flash writes were issued during this test.

## TX heap exhaustion evidence and candidate

Read-only SRAM snapshot: /tmp/stm32-tx-stall-sram.bin. LwIP uses default
MEM_SIZE=1600; all 23 heap blocks were used, many containing 42-byte ARP
pbufs with ref=1. HAL TxDescList had BuffersInUse=0 and all PacketAddress
slots null, with no DMA-owned TX descriptors. The tcpip mailbox was empty;
mem_mutex and core lock had no owner. This supports retained TX pbuf
references rather than an outstanding DMA transfer or held heap mutex.

The candidate moves reclamation out of HAL_ETH_TxCpltCallback (IRQ) into
the submitting task, waits for completion with a bounded timeout, retries
BUSY with a bound and preserves HAL's submitted reference on DMA timeout.
The next submission reaps late completions. No heap-size increase or new
recovery/reset policy was added. Added tx_free_count and tx_timeout_count.
Build succeeded; host harness using the extracted low_level_output passed
immediate/delayed completion, BUSY retry, persistent BUSY timeout, and
DMA timeout followed by eventual reclaim. These mocks do not validate
hardware timing or physical network behavior.

## TX task-reclaim candidate flashed and observed

User confirmed wheels stopped and external run commands disabled, then
authorized flashing. Full backup: build-linux/backups/stm32-before-tx-reclaim-20261111.bin.
Candidate f767-tx-task-reclaim was programmed with Verified OK, then reset.
After restoring temporary eno1 IP, ping worked before and after monitoring.
A 60-second ROS observation received 5853 messages, from t=1.479 to 60.000 s
(approximately 100 Hz once discovered); largest inter-message gap 13 ms.
All pose/twist values stayed zero with stationary wheels. Raw readings are
in the candidate folder as stationary-odom-60s.jsonl.

Heap snapshots at start/end both showed one free block with 1592 payload
bytes free and zero allocated blocks, without increasing MEM_SIZE.
At the later telemetry sample TX completions and tx_free_count both equaled
13858; tx_submit_errors and tx_timeout_count were zero. One RX-buffer-
unavailable DMA callback was recorded, but RX/TX and ping continued to work.
No fatal DMA bus error was observed. This supports successful reclamation
for the observed stationary workload; it does not prove long-duration
stability, reconnection recovery, or odom correctness during wheel motion.
A new wheel-motion trial is still required. Firmware remains on MCU.

## Wheel-motion retest after TX fix

User started another run/stop trial. Over 60 seconds the ROS observer
received 5994 Odometry messages at approximately 100 Hz; maximum gap 12 ms.
Pose and twist changed while wheels moved; the last sample had vx=wz=0.
Both Hall counters changed and both transition-error counters stayed zero.
Ping during motion returned 3/3 replies. g_uros_connected remained 1.
TX completion/free counts matched at 48651; no TX submit errors/timeouts.
DMA error callback count remained 5, unchanged from the start of the trial.
Heap snapshot free/used payload bytes: 1524/60.
Raw messages and summary are in f767-tx-task-reclaim/moving-wheel-odom-60s.jsonl
and moving-wheel-summary.json. This verifies live Hall-based odometry
reaches ROS through run/stop activity. Distance/yaw calibration and wheel
sign correctness still require a known physical motion reference. No motor
commands, resets or firmware modifications were made during this retest.

## ASIX AX88179 permanent host profile and restoration

The user replaced the adapter with USB 0B95:1790, MAC 34:29:8F:60:02:E3,
interface enx34298f6002e3. NetworkManager profile UUID
016f97b4-443e-3120-9264-7e39ebd63dbe was persistently modified to
STM32_AX88179, autoconnect=yes, IPv4 manual 192.168.4.150/24, no gateway
or DNS, never-default=yes, matched by adapter MAC rather than USB port.

After repeated link changes, MCU telemetry showed a fatal DMA bus error
(last_dma_error=0x00602000), allocation failures and zero RX descriptors.
Wheel counts did not change over one second and velocities were zero.
A reset run command restored Ethernet without rewriting firmware.
The adapter retained the saved IP across link recovery. Ping then returned
5/5 replies (mean 1.624 ms), and /odomfromSTM32 returned position data.
Follow-up telemetry showed four RX descriptors, micro-ROS connected,
RX/TX progressing and no errors since reset. This restores the current
connection; firmware hot-unplug/reinitialization recovery remains unresolved
and may require reset after changing adapters or Ethernet link interruptions.
