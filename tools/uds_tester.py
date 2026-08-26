#!/usr/bin/env python3
"""UDS 诊断仪脚本（针对 Front_Light 车载前大灯项目）

依赖：pip install python-can
用法示例：
  python tools/uds_tester.py --interface pcan --channel PCAN_USBBUS1
  python tools/uds_tester.py --interface pcan --channel PCAN_USBBUS1 --send "02 10 02"
  python tools/uds_tester.py --interface socketcan --channel can0 --monitor
"""

import argparse
import sys
import time

import can

REQ_ID = 0x7DF
RESP_ID = 0x7E7

# 与固件 bsp_can.c 中一致的 CRC8（AUTOSAR SAE J1850 poly 0x1D）
def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def secure_frame(arb_id: int, payload: bytes, counter: int = 0) -> bytes:
    frame = bytearray(8)
    for i in range(min(len(payload), 6)):
        frame[i] = payload[i]
    frame[6] = counter & 0x0F
    frame[7] = crc8(bytes(frame[:7]))
    return bytes(frame)

def send(bus, arb_id, data, timeout=0.5):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg, timeout=timeout)

def recv_id(bus, arb_id, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = bus.recv(timeout=max(0.01, end - time.monotonic()))
        if msg is not None and msg.arbitration_id == arb_id:
            return msg.data
    return None

def tp_send(bus, req_id, resp_id, payload):
    """发送 UDS 请求（自动 SF/FF+CF 组包）"""
    if len(payload) <= 7:
        send(bus, req_id, bytes([0x00 | len(payload)]) + payload)
        return
    total = len(payload)
    send(bus, req_id, bytes([0x10 | (total >> 8) & 0x0F, total & 0xFF]) + payload[:6])
    fc = recv_id(bus, req_id, timeout=1.0)
    if fc is None or (fc[0] >> 4) != 0x3:
        print("[TP] 未收到流控帧(FC)，放弃发送")
        return
    idx = 6
    seq = 1
    while idx < total:
        chunk = payload[idx:idx + 7]
        send(bus, req_id, bytes([0x20 | (seq & 0x0F)]) + chunk)
        seq += 1
        idx += len(chunk)

def tp_receive(bus, req_id, resp_id, timeout=2.0):
    """接收 UDS 响应（自动 SF/FF+CF 组包）"""
    first = recv_id(bus, resp_id, timeout=timeout)
    if first is None:
        return None
    pci = first[0] >> 4
    if pci == 0x0:
        return bytes(first[1:1 + (first[0] & 0x0F)])
    if pci == 0x1:
        total = ((first[0] & 0x0F) << 8) | first[1]
        buf = bytearray(first[2:8])
        send(bus, req_id, b"\x30\x00\x00")
        seq = 1
        end = time.monotonic() + timeout
        while len(buf) < total and time.monotonic() < end:
            cf = recv_id(bus, resp_id, timeout=0.5)
            if cf is None:
                break
            if (cf[0] >> 4) != 0x2 or (cf[0] & 0x0F) != (seq & 0x0F):
                print("[TP] 连续帧序号错误，放弃")
                return None
            buf.extend(cf[1:])
            seq += 1
        return bytes(buf[:total])
    print(f"[TP] 意外的首帧 PCI: {first[0]:#x}")
    return None

def do_request(bus, payload):
    print(f">>> TX {payload.hex(' ').upper()}")
    tp_send(bus, REQ_ID, RESP_ID, payload)
    resp = tp_receive(bus, REQ_ID, RESP_ID)
    if resp is None:
        print("<<< 无响应 (超时)")
        return
    print(f"<<< RX {resp.hex(' ').upper()}")
    return resp

def read_did(bus, did):
    return do_request(bus, bytes([0x22, (did >> 8) & 0xFF, did & 0xFF]))

def parse_neg(resp):
    if len(resp) >= 3 and resp[0] == 0x7F:
        nrc = resp[2]
        names = {0x11: "serviceNotSupported", 0x12: "subFunctionNotSupported",
                 0x13: "incorrectMessageLength", 0x22: "conditionsNotCorrect",
                 0x31: "requestOutOfRange", 0x33: "securityAccessDenied",
                 0x35: "invalidKey", 0x37: "requiredTimeDelayNotExpired"}
        print(f"  NRC={nrc:#04x} ({names.get(nrc, 'unknown')})")

def menu(bus):
    while True:
        print("\n=== UDS 诊断测试菜单 ===")
        print(" 1. 读取软件版本 (22 F190)")
        print(" 2. 读取硬件版本 (22 F191)")
        print(" 3. 读取VIN码 (22 F193)")
        print(" 4. 读取灯光状态 (22 F194)")
        print(" 5. 读取CAN错误信息 (22 F195)")
        print(" 6. 读取网络管理状态 (22 F197)")
        print(" 7. 安全访问 (27 01/02)")
        print(" 8. 灯光自检 (31 01 1000)")
        print(" 9. 读取DTC数量 (19 01 FF)")
        print("10. 读取DTC列表 (19 02 FF)")
        print("11. 清除DTC (14 FF FF)")
        print("12. 切换编程会话 (10 02)")
        print("13. 回到默认会话 (10 01)")
        print("14. 保持连接 (3E 00)")
        print("15. 连续心跳监控")
        print(" 0. 退出")
        choice = input("请选择: ").strip()
        try:
            if choice == "1": read_did(bus, 0xF190)
            elif choice == "2": read_did(bus, 0xF191)
            elif choice == "3": read_did(bus, 0xF193)
            elif choice == "4":
                resp = read_did(bus, 0xF194)
                if resp and resp[0] == 0x62:
                    states = {0: "OFF", 1: "LOW_BEAM_ON", 2: "LOW_BEAM_OFF", 3: "HIGH_BEAM_ON",
                              4: "HIGH_BEAM_OFF", 5: "LEFT_TURN", 6: "RIGHT_TURN", 7: "HAZARD"}
                    print(f"  灯光状态 = {states.get(resp[3], resp[3])}")
            elif choice == "5":
                resp = read_did(bus, 0xF195)
                if resp and resp[0] == 0x62:
                    states = {0: "ACTIVE", 1: "WARNING", 2: "PASSIVE", 3: "BUS_OFF"}
                    print(f"  总线状态 = {states.get(resp[3])}  错误标志 = {resp[4]:#04x}  错误计数 = {(resp[5] << 8) | resp[6]}")
            elif choice == "6":
                resp = read_did(bus, 0xF197)
                if resp and resp[0] == 0x62:
                    uptime = (resp[4] << 8) | resp[5]
                    print(f"  对端在线 = {'是' if resp[3] == 1 else '否'}  对端运行时间 = {uptime}s")
            elif choice == "7":
                do_request(bus, bytes([0x27, 0x01]))
                seed = int(input("输入返回的种子(16进制, 如 1A2B): "), 16)
                do_request(bus, bytes([0x27, 0x02, (seed ^ 0x5A5A) >> 8, (seed ^ 0x5A5A) & 0xFF]))
            elif choice == "8":
                do_request(bus, bytes([0x31, 0x01, 0x10, 0x00]))
                time.sleep(2)
                do_request(bus, bytes([0x31, 0x03, 0x10, 0x00]))
            elif choice == "9": do_request(bus, bytes([0x19, 0x01, 0xFF]))
            elif choice == "10":
                resp = do_request(bus, bytes([0x19, 0x02, 0xFF]))
                if resp and resp[0] == 0x59:
                    i = 2
                    while i + 2 < len(resp):
                        code = (resp[i] << 8) | resp[i + 1]
                        status = resp[i + 2]
                        active = "活跃" if (status & 0x09) else "历史"
                        print(f"  DTC {code:04X} 状态={status:02X} ({active})")
                        i += 3
            elif choice == "11":
                do_request(bus, bytes([0x14, 0xFF, 0xFF]))
            elif choice == "12": do_request(bus, bytes([0x10, 0x02]))
            elif choice == "13": do_request(bus, bytes([0x10, 0x01]))
            elif choice == "14": do_request(bus, bytes([0x3E, 0x00]))
            elif choice == "15":
                print("监控总线 10 秒... (Ctrl+C 停止)")
                end = time.monotonic() + 10
                try:
                    while time.monotonic() < end:
                        msg = bus.recv(timeout=0.5)
                        if msg is not None:
                            print(f"{time.strftime('%H:%M:%S')}  ID={msg.arbitration_id:03X}  {msg.data.hex(' ').upper()}")
                except KeyboardInterrupt:
                    pass
            elif choice == "0":
                break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"错误: {e}")

def main():
    ap = argparse.ArgumentParser(description="Front_Light UDS 诊断仪")
    ap.add_argument("--interface", default="pcan", help="python-can 接口 (pcan/socketcan/vector/ixxat/ics)")
    ap.add_argument("--channel", default="PCAN_USBBUS1", help="通道名")
    ap.add_argument("--bitrate", type=int, default=250000, help="波特率 (默认 250000)")
    ap.add_argument("--send", help="一次性发送UDS请求，如 --send \"02 10 02\"")
    ap.add_argument("--monitor", action="store_true", help="监控模式：仅打印总线报文")
    ap.add_argument("--req-id", type=lambda x: int(x, 0), default=REQ_ID)
    ap.add_argument("--resp-id", type=lambda x: int(x, 0), default=RESP_ID)
    args = ap.parse_args()

    bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    print(f"已连接: {args.interface} / {args.channel} @ {args.bitrate}bps")

    try:
        if args.monitor:
            print("监控模式，Ctrl+C 退出")
            while True:
                msg = bus.recv(timeout=1.0)
                if msg is not None:
                    print(f"{time.strftime('%H:%M:%S')}  ID={msg.arbitration_id:03X}  DLC={msg.dlc}  {msg.data.hex(' ').upper()}")
        elif args.send:
            payload = bytes.fromhex(args.send.replace(" ", ""))
            do_request(bus, payload)
        else:
            menu(bus)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

if __name__ == "__main__":
    sys.exit(main())