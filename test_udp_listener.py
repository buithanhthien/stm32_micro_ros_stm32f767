import socket
import datetime

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Cho phép tái sử dụng địa chỉ
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    sock.bind((UDP_IP, UDP_PORT))
    print("=" * 65)
    print(f"[*] Dang lang nghe UDP tren cong {UDP_PORT} (tat ca dia chi IP)...")
    print("[*] Bat nguon STM32 hoac bam nut RESET (Black button) tren board.")
    print("[*] Nhan Ctrl+C de dung.")
    print("=" * 65)
except Exception as e:
    print(f"[-] Loi bind socket tren port {UDP_PORT}: {e}")
    print("[-] Hay chac chan khong co ung dung khac (nhu micro_ros_agent) dang chiem port 8888.")
    exit(1)

packet_count = 0

while True:
    try:
        data, addr = sock.recvfrom(2048)
        packet_count += 1
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{now}] Gói #{packet_count:04d} tu {addr[0]}:{addr[1]} | Do dai: {len(data)} bytes")
        hex_data = " ".join(f"{b:02X}" for b in data[:32])
        if len(data) > 32:
            hex_data += " ..."
        print(f"    HEX: {hex_data}\n")
    except KeyboardInterrupt:
        print("\n[*] Da dung lang nghe.")
        break
    except Exception as e:
        print(f"[-] Loi: {e}")
        break

sock.close()

