#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
ROBOT DIFFERENTIAL DRIVE TELEMETRY EXTRACTOR & DIAGNOSTIC TOOL
=============================================================================
Công cụ thu thập dữ liệu và chẩn đoán vi sai 2 bánh xe (BLDC STM32):
- ĐỌC TRỰC TIẾP QUA ST-LINK (SWD) mà không cần thêm dây cắm UART / USB-TTL:
  + Chế độ Live: Đọc biến thời gian thực (20Hz / 50ms) không dừng CPU.
  + Chế độ Dump RAM: Trích xuất toàn bộ Ring Buffer 400 mẫu (~20s lịch sử) trong 1 giây.
- Đọc luồng dữ liệu thời gian thực từ cổng nối tiếp UART3 (115200 baud).
- Đọc và phân tích từ tệp log đã lưu trước đó.
- Hiển thị bảng điều khiển trực quan (Live Dashboard) với độ phân giải 20Hz.
- Tự động tạo tệp CSV chuẩn (Chuẩn định dạng phân tích dữ liệu Robot).
- Tự động chẩn đoán lỗi lệch bánh:
  + Lệch tốc độ giữa 2 bánh khi đi thẳng (|v1 - v2|).
  + Chênh lệch công suất PWM giữa 2 bánh (Power Imbalance).
  + Kiểm tra chiều đếm xung Hall (phát hiện cảm biến Hall đếm ngược dấu).
  + Phân biệt lỗi do Cơ khí (ma sát tải nặng 50kg) hay do Phần mềm (Vòng đồng bộ ghìm).
- Vẽ biểu đồ phân tích 4 khung hình chuyên sâu (Vận tốc, PWM, Xung Hall, Sai số đồng bộ).

Tác giả: Kỹ sư Điều khiển Tự hành (Senior Embedded Robotics Engineer)
=============================================================================
"""

import sys
import os
import glob
import time
import struct
import shutil
import argparse
import subprocess
from datetime import datetime

# =============================================================================
# KIỂM TRA THƯ VIỆN NGOÀI
# =============================================================================
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import pyocd
    from pyocd.core.helpers import ConnectHelper
    HAS_PYOCD = True
except ImportError:
    HAS_PYOCD = False

try:
    from elftools.elf.elffile import ELFFile
    HAS_ELFTOOLS = True
except ImportError:
    HAS_ELFTOOLS = False


CSV_HEADER = [
    "time_ms", "v_cmd", "w_cmd",
    "des_v1", "cur_v1", "pwm1", "enc1",
    "des_v2", "cur_v2", "pwm2", "enc2",
    "e_sync_m", "v_corr", "sync_act"
]

# Định dạng nhị phân của RobotTelemetry_t trên ARM Cortex-M7 (56 bytes, 4-byte aligned)
# struct { uint32_t; float; float; float; float; int16_t; pad2; int32_t; float; float; int16_t; pad2; int32_t; float; float; uint8_t; pad3; }
STRUCT_TELEMETRY_FMT = "<Iffffh2xiffh2xiffB3x"
TELEMETRY_ENTRY_SIZE = struct.calcsize(STRUCT_TELEMETRY_FMT)  # 56 bytes

# Địa chỉ mặc định dự phòng nếu không đọc được từ file ELF
DEFAULT_ADDR_LATEST   = 0x200410A0
DEFAULT_ADDR_BUF      = 0x2003B920
DEFAULT_ADDR_IDX      = 0x200410D8
DEFAULT_ADDR_TOTAL    = 0x200410DC
DEFAULT_LOG_BUF_COUNT = 400


# =============================================================================
# HỖ TRỢ TRÍCH XUẤT ĐỊA CHỈ BIẾN TỪ FILE ELF (.elf)
# =============================================================================
def find_default_elf():
    """Tự động tìm kiếm file ELF của firmware trong thư mục Debug/"""
    candidates = glob.glob("Debug/*.elf") + glob.glob("*.elf")
    if candidates:
        return candidates[0]
    return None


def get_symbols_from_elf(elf_path):
    """
    Trích xuất địa chỉ thực tế của các biến telemetry trong RAM từ file ELF
    """
    syms = {
        "g_telemetry_latest": DEFAULT_ADDR_LATEST,
        "g_telemetry_buf": DEFAULT_ADDR_BUF,
        "g_telemetry_idx": DEFAULT_ADDR_IDX,
        "g_telemetry_total_samples": DEFAULT_ADDR_TOTAL
    }

    if not elf_path or not os.path.exists(elf_path):
        return syms

    if not HAS_ELFTOOLS:
        return syms

    try:
        with open(elf_path, "rb") as f:
            elf = ELFFile(f)
            symtab = elf.get_section_by_name(".symtab")
            if not symtab:
                return syms

            for sym in symtab.iter_symbols():
                if sym.name in syms:
                    syms[sym.name] = sym.entry.st_value
    except Exception as e:
        print(f"[*] Cảnh báo khi đọc ELF: {e}. Sử dụng địa chỉ mặc định.")

    return syms


def find_stm32_programmer_cli():
    """Tự động tìm đường dẫn STM32_Programmer_CLI trên máy tính Windows"""
    p = shutil.which("STM32_Programmer_CLI")
    if p:
        return p

    search_patterns = [
        "E:/ST/**/STM32_Programmer_CLI.exe",
        "C:/ST/**/STM32_Programmer_CLI.exe",
        "C:/Program Files/STMicroelectronics/**/STM32_Programmer_CLI.exe",
        "C:/Program Files (x86)/STMicroelectronics/**/STM32_Programmer_CLI.exe"
    ]
    for pat in search_patterns:
        matches = glob.glob(pat, recursive=True)
        if matches:
            return matches[0]

    return None


def unpack_telemetry_entry(raw_bytes, offset=0):
    """
    Giải mã 56 bytes nhị phân trong RAM thành bản ghi dict Telemetry chuẩn
    """
    if len(raw_bytes) < offset + TELEMETRY_ENTRY_SIZE:
        return None

    try:
        data = struct.unpack_from(STRUCT_TELEMETRY_FMT, raw_bytes, offset)
        entry = {
            "time_ms":   int(data[0]),
            "v_cmd":     round(float(data[1]), 4),
            "w_cmd":     round(float(data[2]), 4),
            "des_v1":    round(float(data[3]), 4),
            "cur_v1":    round(float(data[4]), 4),
            "pwm1":      int(data[5]),
            "enc1":      int(data[6]),
            "des_v2":    round(float(data[7]), 4),
            "cur_v2":    round(float(data[8]), 4),
            "pwm2":      int(data[9]),
            "enc2":      int(data[10]),
            "e_sync_m":  round(float(data[11]), 5),
            "v_corr":    round(float(data[12]), 4),
            "sync_act":  int(data[13])
        }
        return entry
    except Exception:
        return None


# =============================================================================
# GIAO DIỆN KẾT NỐI ST-LINK
# =============================================================================
class STLinkInterface:
    """
    Lớp điều khiển kết nối ST-Link qua STM32_Programmer_CLI hoặc PyOCD.
    Cho phép đọc RAM trực tiếp mà KHÔNG làm dừng hoạt động của STM32/FreeRTOS.
    """
    def __init__(self, elf_path=None, use_pyocd=False):
        self.elf_path = elf_path or find_default_elf()
        self.symbols = get_symbols_from_elf(self.elf_path)
        self.use_pyocd = use_pyocd
        self.pyocd_session = None
        self.cli_path = None
        self.backend = None

    def connect(self):
        # 1. STM32_Programmer_CLI (Mặc định: Chính hãng ST, cực kỳ ổn định, không xung đột driver)
        if not self.use_pyocd:
            self.cli_path = find_stm32_programmer_cli()
            if self.cli_path:
                print(f"[*] Đang kết nối ST-Link qua STM32_Programmer_CLI...")
                cmd = [self.cli_path, "-c", "port=SWD", "mode=HOTPLUG", "shared"]
                try:
                    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
                    if "Error" in res.stdout or "No debug probe" in res.stdout or "No ST-Link" in res.stdout or "DEV_NO_STLINK" in res.stdout or "DEV_CONNECT_ERR" in res.stdout:
                        # Kiểm tra xem có phải do STM32CubeIDE đang chạy debug không
                        is_gdb_running = False
                        try:
                            import psutil
                            for p in psutil.process_iter(['name']):
                                if p.info['name'] and 'ST-LINK_gdbserver' in p.info['name']:
                                    is_gdb_running = True
                                    break
                        except Exception:
                            pass

                        print(f"\n[!] Không thể kết nối tới vi điều khiển qua ST-Link SWD.")
                        if is_gdb_running:
                            print("=" * 80)
                            print(">>> NGUYÊN NHÂN: STM32CubeIDE ĐANG MỞ PHIÊN DEBUG!")
                            print("    Tiến trình 'ST-LINK_gdbserver' đang giữ độc quyền kết nối phần cứng ST-Link.")
                            print(">>> CÁCH KHẮC PHỤC:")
                            print("    1. Trong STM32CubeIDE, hãy bấm nút TERMINATE (hình ô vuông màu ĐỎ / phím tắt Ctrl+F2)")
                            print("       để dừng phiên debug và giải phóng ST-Link.")
                            print("    2. Sau đó chạy lại lệnh: python extract_robot_log.py --stlink --dump-ram --plot")
                            print("=" * 80 + "\n")
                        else:
                            print(f"    Chi tiết: {res.stdout.strip().splitlines()[-1] if res.stdout.strip() else 'No probe'}")
                            print("    Vui lòng kiểm tra lại cáp kết nối ST-Link hoặc nguồn cấp cho board STM32.")
                        return False
                    self.backend = "cli"
                    print("[+] Kết nối ST-Link thành công (mode=HOTPLUG, shared)!")
                    return True
                except subprocess.TimeoutExpired:
                    print("[!] Kết nối ST-Link timeout (5s).")
                    return False
            else:
                print("[!] Không tìm thấy STM32_Programmer_CLI.exe trong các thư mục cài đặt của ST.")

        # 2. PyOCD (Tùy chọn khi truyền cờ --pyocd)
        if HAS_PYOCD:
            try:
                print("[*] Đang tìm kiếm probe ST-Link qua PyOCD...")
                self.pyocd_session = ConnectHelper.session_with_chosen_probe(
                    target_override="cortex_m",
                    options={
                        "connect_mode": "attach",    # Kết nối vào chip ĐANG CHẠY, không reset
                        "resume_on_connect": False,  # Không can thiệp vào trạng thái chạy của FreeRTOS
                        "auto_unlock": False
                    }
                )
                if self.pyocd_session:
                    self.pyocd_session.open()
                    self.backend = "pyocd"
                    print(f"[+] Kết nối ST-Link thành công qua PyOCD!")
                    return True
            except Exception as e:
                print(f"[!] PyOCD không thể mở probe: {e}")
                if self.pyocd_session:
                    try: self.pyocd_session.close()
                    except Exception: pass
                    self.pyocd_session = None

        print("[!] Không tìm thấy ST-Link khả dụng. Hãy cắm cáp ST-Link vào cổng USB của máy tính.")
        return False

    def close(self):
        if self.pyocd_session:
            try:
                self.pyocd_session.close()
            except Exception:
                pass
            self.pyocd_session = None

    def read_memory(self, addr, size):
        """Đọc một vùng nhớ RAM liên tục (size bytes)"""
        if self.backend == "pyocd" and self.pyocd_session:
            try:
                data = self.pyocd_session.target.read_memory_block8(addr, size)
                return bytes(data)
            except Exception as e:
                print(f"[!] Lỗi đọc RAM PyOCD: {e}")
                return None

        elif self.backend == "cli" and self.cli_path:
            temp_bin = os.path.abspath("temp_telemetry_dump.bin")
            cmd = [
                self.cli_path, "-c", "port=SWD", "mode=HOTPLUG", "shared",
                "-u", f"0x{addr:08X}", str(size), temp_bin
            ]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if os.path.exists(temp_bin):
                with open(temp_bin, "rb") as f:
                    data = f.read()
                try: os.remove(temp_bin)
                except Exception: pass
                return data
            else:
                print(f"[!] CLI Upload thất bại:\n{res.stdout}")
                return None

        return None

    def read_latest(self):
        """Đọc biến g_telemetry_latest tức thời (56 bytes)"""
        addr = self.symbols["g_telemetry_latest"]
        raw = self.read_memory(addr, TELEMETRY_ENTRY_SIZE)
        if raw:
            return unpack_telemetry_entry(raw)
        return None

    def read_ring_buffer(self):
        """
        Đọc toàn bộ Ring Buffer 400 mẫu từ RAM và sắp xếp theo trình tự thời gian
        """
        raw_idx = self.read_memory(self.symbols["g_telemetry_idx"], 4)
        raw_total = self.read_memory(self.symbols["g_telemetry_total_samples"], 4)
        if not raw_idx or not raw_total:
            print("[!] Không thể đọc con trỏ chỉ mục Ring Buffer.")
            return []

        cur_idx = struct.unpack("<H", raw_idx[:2])[0]
        total_samples = struct.unpack("<I", raw_total[:4])[0]

        total_bytes = DEFAULT_LOG_BUF_COUNT * TELEMETRY_ENTRY_SIZE
        print(f"[*] Đang tải {total_bytes} bytes Ring Buffer (400 mẫu) từ RAM (0x{self.symbols['g_telemetry_buf']:08X})...")
        raw_buf = self.read_memory(self.symbols["g_telemetry_buf"], total_bytes)
        if not raw_buf or len(raw_buf) < total_bytes:
            print("[!] Đọc Ring Buffer không đủ kích thước.")
            return []

        all_entries = []
        for i in range(DEFAULT_LOG_BUF_COUNT):
            e = unpack_telemetry_entry(raw_buf, i * TELEMETRY_ENTRY_SIZE)
            if e and e["time_ms"] > 0:
                all_entries.append((i, e))

        if not all_entries:
            print("[!] Ring Buffer trong RAM hiện đang trống (chưa có mẫu nào).")
            return []

        if total_samples >= DEFAULT_LOG_BUF_COUNT:
            ordered_entries = []
            for k in range(DEFAULT_LOG_BUF_COUNT):
                slot = (cur_idx + k) % DEFAULT_LOG_BUF_COUNT
                e = unpack_telemetry_entry(raw_buf, slot * TELEMETRY_ENTRY_SIZE)
                if e and e["time_ms"] > 0:
                    ordered_entries.append(e)
            return ordered_entries
        else:
            ordered_entries = []
            for k in range(cur_idx):
                e = unpack_telemetry_entry(raw_buf, k * TELEMETRY_ENTRY_SIZE)
                if e and e["time_ms"] > 0:
                    ordered_entries.append(e)
            return ordered_entries


# =============================================================================
# PARSER LOG CHO DÒNG UART DẠNG TEXT ($LOG,...)
# =============================================================================
def parse_log_line(line):
    line = line.strip()
    if not line.startswith("$LOG,"):
        return None

    parts = line.split(",")
    if len(parts) < 15:
        return None

    try:
        entry = {
            "time_ms":   int(parts[1]),
            "v_cmd":     float(parts[2]),
            "w_cmd":     float(parts[3]),
            "des_v1":    float(parts[4]),
            "cur_v1":    float(parts[5]),
            "pwm1":      int(parts[6]),
            "enc1":      int(parts[7]),
            "des_v2":    float(parts[8]),
            "cur_v2":    float(parts[9]),
            "pwm2":      int(parts[10]),
            "enc2":      int(parts[11]),
            "e_sync_m":  float(parts[12]),
            "v_corr":    float(parts[13]),
            "sync_act":  int(parts[14]),
        }
        return entry
    except (ValueError, IndexError):
        return None


def print_dashboard_header():
    print("=" * 105)
    print(f"{'TIME(s)':<8} | {'V_CMD':<7} {'W_CMD':<7} | {'BÁNH 1 (PHẢI)':<26} | {'BÁNH 2 (TRÁI)':<26} | {'ĐỒNG BỘ 2 BÁNH':<18}")
    print(f"{'':<8} | {'(m/s)':<7} {'(rad/s)':<7} | {'Des':<6} {'Cur':<6} {'PWM':<5} {'Hall':<7} | {'Des':<6} {'Cur':<6} {'PWM':<5} {'Hall':<7} | {'e_sync(mm)':<10} {'v_corr':<7}")
    print("-" * 105)


def print_dashboard_row(entry, t0_ms=0):
    t_sec = (entry["time_ms"] - t0_ms) / 1000.0 if t0_ms else entry["time_ms"] / 1000.0
    sync_str = f"{entry['e_sync_m'] * 1000.0:+.1f}" if entry['sync_act'] else "OFF"
    corr_str = f"{entry['v_corr']:+.3f}" if entry['sync_act'] else "0.000"

    warn_mark = ""
    if abs(entry["des_v1"]) > 0.01 or abs(entry["des_v2"]) > 0.01:
        spd_diff = abs(entry["cur_v1"] - entry["cur_v2"])
        if spd_diff > 0.012:
            warn_mark = " <!>"

    print(f"{t_sec:7.2f}s | {entry['v_cmd']:+6.3f} {entry['w_cmd']:+6.3f} | "
          f"{entry['des_v1']:+6.3f} {entry['cur_v1']:+6.3f} {entry['pwm1']:5d} {entry['enc1']:7d} | "
          f"{entry['des_v2']:+6.3f} {entry['cur_v2']:+6.3f} {entry['pwm2']:5d} {entry['enc2']:7d} | "
          f"{sync_str:>9} mm {corr_str:>7}{warn_mark}")


# =============================================================================
# CHẨN ĐOÁN TỰ ĐỘNG & BÁO CÁO PHÂN TÍCH
# =============================================================================
def analyze_telemetry(records):
    if not records:
        print("\n[!] Không có dữ liệu để phân tích.")
        return

    n = len(records)
    print("\n" + "=" * 80)
    print("   BÁO CÁO PHÂN TÍCH TỰ ĐỘNG HIỆN TƯỢNG LỆCH BÁNH ROBOT (DIAGNOSTIC REPORT)")
    print("=" * 80)

    t_start = records[0]["time_ms"] / 1000.0
    t_end   = records[-1]["time_ms"] / 1000.0
    dur     = max(0.001, t_end - t_start)
    print(f"[1] Tổng số mẫu thu thập : {n} mẫu ({dur:.1f} giây, tần số trung bình: {n/dur:.1f} Hz)")

    active_samples = [r for r in records if abs(r["v_cmd"]) > 0.01]
    straight_samples = [r for r in active_samples if abs(r["w_cmd"]) < 0.05]

    if not active_samples:
        print("[!] Không có khoảng thời gian nào robot nhận lệnh chạy (|v_cmd| > 0.01 m/s).")
        return

    print(f"[2] Thời gian robot nhận lệnh di chuyển: {len(active_samples) * 0.05:.1f}s "
          f"(Trong đó lệnh đi thẳng: {len(straight_samples) * 0.05:.1f}s)")

    avg_des1 = sum(r["des_v1"] for r in active_samples) / len(active_samples)
    avg_cur1 = sum(r["cur_v1"] for r in active_samples) / len(active_samples)
    avg_pwm1 = sum(r["pwm1"] for r in active_samples) / len(active_samples)

    avg_des2 = sum(r["des_v2"] for r in active_samples) / len(active_samples)
    avg_cur2 = sum(r["cur_v2"] for r in active_samples) / len(active_samples)
    avg_pwm2 = sum(r["pwm2"] for r in active_samples) / len(active_samples)

    print("\n--- [3] THỐNG KÊ HOẠT ĐỘNG KHI CÓ LỆNH DI CHUYỂN ---")
    print(f"  * BÁNH SỐ 1 (Bánh Phải):")
    print(f"      + Tốc độ mục tiêu trung bình (Des1): {avg_des1:+.4f} m/s")
    print(f"      + Tốc độ thực tế trung bình   (Cur1): {avg_cur1:+.4f} m/s (Sai số: {avg_des1 - avg_cur1:+.4f} m/s)")
    print(f"      + Công suất PWM trung bình   (PWM1): {avg_pwm1:+.1f} / 1200 ({abs(avg_pwm1)/12.0:.1f}%)")
    print(f"  * BÁNH SỐ 2 (Bánh Trái):")
    print(f"      + Tốc độ mục tiêu trung bình (Des2): {avg_des2:+.4f} m/s")
    print(f"      + Tốc độ thực tế trung bình   (Cur2): {avg_cur2:+.4f} m/s (Sai số: {avg_des2 - avg_cur2:+.4f} m/s)")
    print(f"      + Công suất PWM trung bình   (PWM2): {avg_pwm2:+.1f} / 1200 ({abs(avg_pwm2)/12.0:.1f}%)")

    # Kiểm tra chiều đếm Hall
    print("\n--- [4] KIỂM TRA TÍNH TOÀN VẸN CỦA CẢM BIẾN HALL & ENCODER ---")
    hall1_reversed_count = sum(1 for r in active_samples if (r["des_v1"] > 0.015 and r["cur_v1"] < -0.005) or (r["des_v1"] < -0.015 and r["cur_v1"] > 0.005))
    hall2_reversed_count = sum(1 for r in active_samples if (r["des_v2"] > 0.015 and r["cur_v2"] < -0.005) or (r["des_v2"] < -0.015 and r["cur_v2"] > 0.005))

    if hall1_reversed_count > 5:
        print(f"  [X] CẢNH BÁO NGUY HIỂM: Bánh 1 có {hall1_reversed_count} mẫu tốc độ CurSpeed1 NGƯỢC DẤU với DesiredSpeed1!")
        print("      => Cảm biến Hall Bánh 1 đang bị đảo pha hoặc ngược chiều đếm!")
    else:
        print("  [OK] Chiều đếm Hall Bánh 1 đồng pha với lệnh điều khiển.")

    if hall2_reversed_count > 5:
        print(f"  [X] CẢNH BÁO NGUY HIỂM: Bánh 2 có {hall2_reversed_count} mẫu tốc độ CurSpeed2 NGƯỢC DẤU với DesiredSpeed2!")
        print("      => Cảm biến Hall Bánh 2 đang bị đảo pha hoặc ngược chiều đếm!")
    else:
        print("  [OK] Chiều đếm Hall Bánh 2 đồng pha với lệnh điều khiển.")

    # Phân tích nguyên nhân chênh lệch
    if straight_samples:
        print("\n--- [5] CHẨN ĐOÁN HIỆN TƯỢNG '1 BÁNH QUAY CHẬM, 1 BÁNH QUAY NHANH' ---")
        avg_st_cur1 = sum(r["cur_v1"] for r in straight_samples) / len(straight_samples)
        avg_st_cur2 = sum(r["cur_v2"] for r in straight_samples) / len(straight_samples)
        avg_st_pwm1 = sum(abs(r["pwm1"]) for r in straight_samples) / len(straight_samples)
        avg_st_pwm2 = sum(abs(r["pwm2"]) for r in straight_samples) / len(straight_samples)

        speed_diff = avg_st_cur1 - avg_st_cur2
        print(f"  * Chênh lệch vận tốc thực tế trung bình (Cur1 - Cur2): {speed_diff:+.4f} m/s")
        print(f"  * Chênh lệch công suất PWM trung bình   (|PWM1| - |PWM2|): {avg_st_pwm1 - avg_st_pwm2:+.1f}")

        if abs(speed_diff) > 0.008:
            slower_wheel = "BÁNH 1 (PHẢI)" if speed_diff < 0 else "BÁNH 2 (TRÁI)"
            faster_wheel = "BÁNH 2 (TRÁI)" if speed_diff < 0 else "BÁNH 1 (PHẢI)"
            print(f"\n  [!] KẾT LUẬN: {slower_wheel} đang quay CHẬM HƠN {faster_wheel}!")

            pwm_slower = avg_st_pwm1 if speed_diff < 0 else avg_st_pwm2
            pwm_faster = avg_st_pwm2 if speed_diff < 0 else avg_st_pwm1

            if pwm_slower < pwm_faster * 0.75:
                print("  => NGUYÊN NHÂN PHẦN MỀM / VÒNG THAM CHIẾU ĐỒNG BỘ:")
                print("     Vòng bù tham chiếu đồng bộ đang ghìm PWM của bánh chậm và dồn PWM sang bánh nhanh,")
                print("     hoặc Deadband đồng bộ đang bị vượt ngưỡng (Deadband < 1 tick xung Hall).")
            elif pwm_slower >= pwm_faster:
                print("  => NGUYÊN NHÂN CƠ KHÍ / PHẦN CỨNG (MECHANICAL LOAD / FRICTION):")
                print(f"     {slower_wheel} đã được cấp PWM tương đương hoặc cao hơn ({pwm_slower:.0f} vs {pwm_faster:.0f})")
                print("     nhưng vẫn quay chậm hơn. Nguyên nhân có thể do:")
                print("     1. Bánh xe chịu tải nặng hơn do trọng tâm xe 50kg nghiêng về bên đó.")
                print("     2. Bánh đa hướng bên đó bị kẹt phương xoay (castor resistance).")
                print("     3. Vòng bi hoặc phanh cơ của động cơ BLDC bị bó.")
        else:
            print("  [OK] Hai bánh duy trì vận tốc đồng đều trong dải sai số cho phép (< 0.008 m/s).")

    # Vòng đồng bộ
    sync_active_samples = [r for r in records if r["sync_act"] == 1]
    print("\n--- [6] HIỆU NĂNG VÒNG THAM CHIẾU ĐỒNG BỘ 2 BÁNH (SYNC REFERENCE LOOP) ---")
    print(f"  * Số mẫu vòng đồng bộ kích hoạt: {len(sync_active_samples)} mẫu ({len(sync_active_samples)*0.05:.1f}s)")
    if sync_active_samples:
        max_sync_e = max(abs(r["e_sync_m"]) for r in sync_active_samples)
        avg_sync_corr = sum(abs(r["v_corr"]) for r in sync_active_samples) / len(sync_active_samples)
        max_sync_corr = max(abs(r["v_corr"]) for r in sync_active_samples)
        print(f"  * Sai số quãng đường tích lũy lớn nhất (|e_sync|): {max_sync_e * 1000.0:.2f} mm")
        print(f"  * Lượng bù tốc độ trung bình (|v_corr|): {avg_sync_corr:.4f} m/s (Cực đại: {max_sync_corr:.4f} m/s)")
    print("=" * 80 + "\n")


# =============================================================================
# VẼ ĐỒ THỊ CHẨN ĐOÁN (4 KHUNG HÌNH)
# =============================================================================
def generate_plots(records, out_img_path="telemetry_analysis.png", show_plot=False):
    if not HAS_MATPLOTLIB:
        print("[!] Không tìm thấy thư viện matplotlib để vẽ biểu đồ. Cài đặt bằng: pip install matplotlib")
        return

    if not records:
        print("[!] Không có dữ liệu để vẽ biểu đồ.")
        return

    t0 = records[0]["time_ms"]
    time_s   = [(r["time_ms"] - t0) / 1000.0 for r in records]
    des_v1   = [r["des_v1"] for r in records]
    cur_v1   = [r["cur_v1"] for r in records]
    pwm1     = [r["pwm1"] for r in records]
    enc1     = [r["enc1"] for r in records]

    des_v2   = [r["des_v2"] for r in records]
    cur_v2   = [r["cur_v2"] for r in records]
    pwm2     = [r["pwm2"] for r in records]
    enc2     = [r["enc2"] for r in records]

    e_sync_mm = [r["e_sync_m"] * 1000.0 for r in records]
    v_corr    = [r["v_corr"] for r in records]
    sync_act  = [r["sync_act"] for r in records]

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    fig.suptitle("PHÂN TÍCH TELEMETRY ĐIỀU KHIỂN ĐỘNG CƠ ROBOT VI SAI (ST-LINK LOG)", fontsize=14, fontweight="bold")

    # 1. Vận tốc
    ax1 = axes[0]
    ax1.plot(time_s, des_v1, "r--", label="Target V1 (Phải)", alpha=0.8, linewidth=1.5)
    ax1.plot(time_s, cur_v1, "r-",  label="Actual V1 (Phải)", linewidth=1.5)
    ax1.plot(time_s, des_v2, "b--", label="Target V2 (Trái)", alpha=0.8, linewidth=1.5)
    ax1.plot(time_s, cur_v2, "b-",  label="Actual V2 (Trái)", linewidth=1.5)
    ax1.set_ylabel("Vận tốc (m/s)", fontweight="bold")
    ax1.set_title("[1] Đáp ứng Vận tốc (Target vs Actual Speed)", fontsize=11)
    ax1.legend(loc="upper right", ncol=2)
    ax1.grid(True, linestyle="--", alpha=0.6)

    # 2. PWM
    ax2 = axes[1]
    ax2.plot(time_s, pwm1, "r-", label="PWM Motor 1 (Phải)", linewidth=1.2)
    ax2.plot(time_s, pwm2, "b-", label="PWM Motor 2 (Trái)", linewidth=1.2)
    ax2.axhline(0, color="gray", linestyle=":")
    ax2.set_ylabel("PWM (-1200..1200)", fontweight="bold")
    ax2.set_title("[2] Tín hiệu Điều khiển PWM Động cơ", fontsize=11)
    ax2.legend(loc="upper right")
    ax2.grid(True, linestyle="--", alpha=0.6)

    # 3. Encoder Ticks
    ax3 = axes[2]
    ax3.plot(time_s, enc1, "r-", label="Hall Count Motor 1 (Phải)", linewidth=1.2)
    ax3.plot(time_s, enc2, "b-", label="Hall Count Motor 2 (Trái)", linewidth=1.2)
    ax3.set_ylabel("Xung Hall (Ticks)", fontweight="bold")
    ax3.set_title("[3] Quãng đường Tích lũy Cảm biến Hall (Encoder Ticks)", fontsize=11)
    ax3.legend(loc="upper left")
    ax3.grid(True, linestyle="--", alpha=0.6)

    # 4. Đồng bộ 2 bánh
    ax4 = axes[3]
    ax4_sub = ax4.twinx()
    l1 = ax4.plot(time_s, e_sync_mm, "m-", label="Sai số quãng đường e_sync (mm)", linewidth=1.5)
    l2 = ax4_sub.plot(time_s, v_corr, "g--", label="Lượng bù tốc v_corr (m/s)", linewidth=1.2)
    ax4.fill_between(time_s, min(e_sync_mm + [-5]), max(e_sync_mm + [5]),
                     where=[bool(s) for s in sync_act], color="yellow", alpha=0.15, label="Vòng đồng bộ Active")
    ax4.set_xlabel("Thời gian (giây)", fontweight="bold")
    ax4.set_ylabel("e_sync (mm)", color="m", fontweight="bold")
    ax4_sub.set_ylabel("v_corr (m/s)", color="g", fontweight="bold")
    ax4.set_title("[4] Vòng Tham chiếu Đồng bộ 2 Bánh (Relative Distance Sync)", fontsize=11)
    lines = l1 + l2
    labels = [l.get_label() for l in lines] + ["Vòng đồng bộ Active"]
    ax4.legend(lines, labels, loc="upper right")
    ax4.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    try:
        plt.savefig(out_img_path, dpi=200)
        print(f"[+] Đã lưu ảnh biểu đồ phân tích tại: {os.path.abspath(out_img_path)}")
    except Exception as e:
        print(f"[!] Lỗi khi lưu ảnh đồ thị: {e}")

    if show_plot:
        try:
            plt.show()
        except Exception:
            pass
    plt.close(fig)


# =============================================================================
# CÁC CHẾ ĐỘ THU THẬP LOG QUA ST-LINK
# =============================================================================
def save_records_to_csv(records, out_csv_path):
    """Lưu danh sách bản ghi Telemetry ra tệp CSV chuẩn"""
    with open(out_csv_path, "w", newline="", encoding="utf-8") as f:
        f.write(",".join(CSV_HEADER) + "\n")
        for entry in records:
            f.write(",".join(str(entry[k]) for k in CSV_HEADER) + "\n")
    print(f"[+] Đã tạo tệp CSV log thành công tại: {os.path.abspath(out_csv_path)}")


def run_stlink_dump(elf_path, out_csv_path, use_pyocd=False, plot=False, show_plot=False):
    """
    Trích xuất toàn bộ Ring Buffer 400 mẫu (~20 giây) từ RAM trong 1 giây qua ST-LINK
    """
    print("=" * 80)
    print("   CHẾ ĐỘ TRÍCH XUẤT LỊCH SỬ CHUYỂN ĐỘNG TỪ RAM QUA ST-LINK (DUMP RAM)")
    print("=" * 80)

    stlink = STLinkInterface(elf_path=elf_path, use_pyocd=use_pyocd)
    if not stlink.connect():
        sys.exit(1)

    try:
        records = stlink.read_ring_buffer()
    finally:
        stlink.close()

    if not records:
        print("[!] Không có dữ liệu hợp lệ trong RAM. Hãy cho robot chạy một đoạn rồi thử lại.")
        return

    print(f"[+] Đã trích xuất thành công {len(records)} mẫu chuyển động gần nhất từ RAM!")
    save_records_to_csv(records, out_csv_path)

    analyze_telemetry(records)
    if plot:
        img_path = out_csv_path.rsplit(".", 1)[0] + "_plot.png"
        generate_plots(records, out_img_path=img_path, show_plot=show_plot)


def run_stlink_live(elf_path, out_csv_path, max_duration=None, no_dashboard=False, use_pyocd=False, plot=False, show_plot=False):
    """
    Đọc biến g_telemetry_latest thời gian thực (20Hz) qua ST-LINK mà không dừng chip
    """
    print("=" * 80)
    print("   CHẾ ĐỘ THEO DÕI TRỰC TIẾP THỜI GIAN THỰC QUA ST-LINK (LIVE STREAM)")
    print("=" * 80)

    stlink = STLinkInterface(elf_path=elf_path, use_pyocd=use_pyocd)
    if not stlink.connect():
        sys.exit(1)

    print(f"[+] Đang ghi log vào: {out_csv_path}")
    print("[*] Nhấn Ctrl+C để dừng thu thập và xem báo cáo chẩn đoán.\n")

    records = []
    t0_ms = None
    last_time_ms = -1
    start_time = time.time()

    csv_file = open(out_csv_path, "w", newline="", encoding="utf-8")
    csv_file.write(",".join(CSV_HEADER) + "\n")
    csv_file.flush()

    if not no_dashboard:
        print_dashboard_header()

    try:
        while True:
            entry = stlink.read_latest()
            if entry and entry["time_ms"] != last_time_ms and entry["time_ms"] > 0:
                last_time_ms = entry["time_ms"]
                if t0_ms is None:
                    t0_ms = entry["time_ms"]

                records.append(entry)

                # Ghi ngay vào tệp CSV
                csv_line = ",".join(str(entry[k]) for k in CSV_HEADER) + "\n"
                csv_file.write(csv_line)
                csv_file.flush()

                if not no_dashboard:
                    print_dashboard_row(entry, t0_ms)

            if max_duration and (time.time() - start_time) >= max_duration:
                print(f"\n[*] Đã đạt giới hạn thời gian ghi ({max_duration}s).")
                break

            time.sleep(0.045)  # Polling ~20Hz

    except KeyboardInterrupt:
        print("\n[*] Đã nhận lệnh dừng từ bàn phím (Ctrl+C).")
    finally:
        stlink.close()
        csv_file.close()
        print(f"[+] Hoàn tất thu thập {len(records)} mẫu vào: {out_csv_path}")

    analyze_telemetry(records)
    if plot:
        img_path = out_csv_path.rsplit(".", 1)[0] + "_plot.png"
        generate_plots(records, out_img_path=img_path, show_plot=show_plot)


# =============================================================================
# CÁC CHẾ ĐỘ PHỤ (UART SERIAL, ĐỌC FILE, MÔ PHỎNG DEMO)
# =============================================================================
def run_serial_logger(port, baud, out_csv_path, max_duration=None, no_dashboard=False, plot=False, show_plot=False):
    if not HAS_SERIAL:
        print("[!] Thư viện pyserial chưa được cài đặt. Cài đặt bằng: pip install pyserial")
        sys.exit(1)

    print(f"[*] Đang kết nối tới cổng {port} ở tốc độ {baud} baud...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1.0)
    except Exception as e:
        print(f"[!] Không thể mở cổng {port}: {e}")
        sys.exit(1)

    print(f"[+] Kết nối thành công! Đang ghi dữ liệu vào: {out_csv_path}")
    print("[*] Nhấn Ctrl+C để dừng thu thập và bắt đầu phân tích.\n")

    records = []
    t0_ms = None
    start_time = time.time()

    csv_file = open(out_csv_path, "w", newline="", encoding="utf-8")
    csv_file.write(",".join(CSV_HEADER) + "\n")
    csv_file.flush()

    if not no_dashboard:
        print_dashboard_header()

    try:
        while True:
            raw_line = ser.readline().decode("latin-1", errors="ignore").strip()
            if not raw_line:
                continue

            entry = parse_log_line(raw_line)
            if entry:
                if t0_ms is None:
                    t0_ms = entry["time_ms"]

                records.append(entry)
                csv_line = ",".join(str(entry[k]) for k in CSV_HEADER) + "\n"
                csv_file.write(csv_line)
                csv_file.flush()

                if not no_dashboard:
                    print_dashboard_row(entry, t0_ms)

            if max_duration and (time.time() - start_time) >= max_duration:
                print(f"\n[*] Đã đạt giới hạn thời gian ghi ({max_duration}s).")
                break

    except KeyboardInterrupt:
        print("\n[*] Đã nhận lệnh dừng từ bàn phím (Ctrl+C).")
    finally:
        ser.close()
        csv_file.close()
        print(f"[+] Đã hoàn thành ghi {len(records)} mẫu vào: {out_csv_path}")

    analyze_telemetry(records)
    if plot:
        generate_plots(records, out_img_path=out_csv_path.rsplit(".", 1)[0] + "_plot.png", show_plot=show_plot)


def run_file_logger(input_file, out_csv_path=None, plot=False, show_plot=False):
    print(f"[*] Đang đọc dữ liệu từ tệp: {input_file}...")
    if not os.path.exists(input_file):
        print(f"[!] Tệp không tồn tại: {input_file}")
        sys.exit(1)

    records = []
    with open(input_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            entry = parse_log_line(line)
            if entry:
                records.append(entry)

    print(f"[+] Đã phân tích thành công {len(records)} mẫu log.")
    if out_csv_path:
        save_records_to_csv(records, out_csv_path)

    analyze_telemetry(records)
    if plot:
        img_path = (out_csv_path or input_file).rsplit(".", 1)[0] + "_plot.png"
        generate_plots(records, out_img_path=img_path, show_plot=show_plot)


def run_demo_simulation(out_csv_path="demo_telemetry.csv", plot=True, show_plot=False):
    print("[*] Chạy chế độ mô phỏng (Demo Simulation) tạo mẫu dữ liệu để kiểm tra công cụ...")
    import math

    records = []
    t_start = 1000
    enc1 = 0
    enc2 = 0

    for i in range(200):
        t_ms = t_start + i * 50
        v_cmd = 0.08 if 20 <= i < 160 else 0.0
        w_cmd = 0.0
        des_v1 = v_cmd
        des_v2 = v_cmd

        if v_cmd > 0:
            cur_v1 = 0.065 + 0.005 * math.sin(i * 0.2)
            cur_v2 = 0.080 + 0.004 * math.sin(i * 0.25)
            pwm1 = 380
            pwm2 = 320
            enc1 += int(cur_v1 * 0.05 / 0.00576)
            enc2 += int(cur_v2 * 0.05 / 0.00576)
            sync_act = 1
            e_sync_m = (enc1 - enc2) * 0.00576
            v_corr = -0.015
        else:
            cur_v1 = 0.0
            cur_v2 = 0.0
            pwm1 = 0
            pwm2 = 0
            sync_act = 0
            e_sync_m = 0.0
            v_corr = 0.0

        entry = {
            "time_ms":   t_ms,
            "v_cmd":     v_cmd,
            "w_cmd":     w_cmd,
            "des_v1":    des_v1,
            "cur_v1":    cur_v1,
            "pwm1":      pwm1,
            "enc1":      enc1,
            "des_v2":    des_v2,
            "cur_v2":    cur_v2,
            "pwm2":      pwm2,
            "enc2":      enc2,
            "e_sync_m":  e_sync_m,
            "v_corr":    v_corr,
            "sync_act":  sync_act
        }
        records.append(entry)

    save_records_to_csv(records, out_csv_path)
    analyze_telemetry(records)
    if plot:
        generate_plots(records, out_img_path="demo_analysis.png", show_plot=show_plot)


# =============================================================================
# HÀM MAIN
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Công cụ thu thập và phân tích Telemetry Log Động cơ Vi Sai STM32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
CÁC VÍ DỤ SỬ DỤNG CHÍNH:

1. ĐỌC TRỰC TIẾP QUA ST-LINK (KHÔNG CẦN DÂY UART):
   - Chế độ Live (Xem và ghi trực tiếp khi đang chạy robot):
     python extract_robot_log.py --stlink --plot

   - Chế độ Dump RAM (Trích xuất ngay 400 mẫu ~ 20 giây vừa chạy):
     python extract_robot_log.py --stlink --dump-ram --plot

2. ĐỌC QUA CỔNG NỐI TIẾP UART (PC10-TX):
   python extract_robot_log.py --port COM3 --plot

3. PHÂN TÍCH TỆP LOG ĐÃ LƯU TRƯỚC ĐÓ:
   python extract_robot_log.py --file saved_terminal.txt --plot
"""
    )

    # Nhóm ST-Link
    parser.add_argument("--stlink",   action="store_true", help="Đọc trực tiếp từ RAM vi điều khiển qua mạch ST-LINK")
    parser.add_argument("--dump-ram", action="store_true", help="Trích xuất toàn bộ Ring Buffer 400 mẫu từ RAM (dùng với --stlink)")
    parser.add_argument("--elf",      type=str, default=None, help="Đường dẫn tệp .elf (mặc định tự tìm trong Debug/*.elf)")
    parser.add_argument("--pyocd",    action="store_true", help="Dùng thư viện PyOCD thay cho STM32_Programmer_CLI")

    # Nhóm Cổng COM & Tệp tin
    parser.add_argument("--port",     type=str, help="Cổng nối tiếp UART (ví dụ: COM3, COM4 hoặc /dev/ttyUSB0)")
    parser.add_argument("--baud",     type=int, default=115200, help="Tốc độ Baud UART (mặc định: 115200)")
    parser.add_argument("--file",     type=str, help="Đường dẫn tệp log dạng văn bản để phân tích")
    parser.add_argument("--out",      type=str, default=None, help="Tên tệp CSV xuất ra (mặc định: telemetry_<timestamp>.csv)")
    parser.add_argument("--duration", type=float, default=None, help="Thời gian thu thập tối đa (giây)")

    # Hiển thị & Đồ họa
    parser.add_argument("--no-dashboard", action="store_true", help="Tắt bảng hiển thị thời gian thực trên console")
    parser.add_argument("--plot",     action="store_true", help="Tự động vẽ và lưu biểu đồ phân tích 4 khung hình (PNG)")
    parser.add_argument("--show",     action="store_true", help="Mở cửa sổ đồ họa hiển thị biểu đồ trên màn hình")
    parser.add_argument("--demo",     action="store_true", help="Chạy chế độ mô phỏng thử nghiệm công cụ")

    args = parser.parse_args()

    # 1. Chế độ Mô phỏng Demo
    if args.demo:
        run_demo_simulation(out_csv_path=args.out or "demo_telemetry.csv", plot=True, show_plot=args.show)
        return

    # 2. Chế độ Đọc trực tiếp qua ST-Link
    if args.stlink:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_csv = args.out or f"telemetry_stlink_{timestamp}.csv"

        if args.dump_ram:
            run_stlink_dump(elf_path=args.elf, out_csv_path=out_csv,
                            use_pyocd=args.pyocd, plot=args.plot, show_plot=args.show)
        else:
            run_stlink_live(elf_path=args.elf, out_csv_path=out_csv,
                            max_duration=args.duration,
                            no_dashboard=args.no_dashboard,
                            use_pyocd=args.pyocd,
                            plot=args.plot,
                            show_plot=args.show)
        return

    # 3. Chế độ Đọc từ tệp log có sẵn
    if args.file:
        out_csv = args.out or args.file.rsplit(".", 1)[0] + "_extracted.csv"
        run_file_logger(args.file, out_csv_path=out_csv, plot=args.plot, show_plot=args.show)
        return

    # 4. Chế độ Đọc qua Cổng nối tiếp UART
    if args.port:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_csv = args.out or f"telemetry_uart_{timestamp}.csv"
        run_serial_logger(args.port, args.baud, out_csv,
                          max_duration=args.duration,
                          no_dashboard=args.no_dashboard,
                          plot=args.plot,
                          show_plot=args.show)
        return

    parser.print_help()


if __name__ == "__main__":
    main()
