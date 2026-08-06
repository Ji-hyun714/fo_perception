import can
import time

can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, interface='socketcan')

count = 0

def parse_can_message(message):
    global count
    count = count % 15

    if message.arbitration_id == 0x200:
        dx_raw = ((message.data[0] & 0xC0) >> 6) | (message.data[1] << 2) | ((message.data[2] & 0x07) << 10)
        dx = dx_raw * 0.0625 - 256  # LSB와 Offset 적용

        print(f"range : {dx:.2f} m")
        
        if dx <= 3.0:
            print("차선 변경 필요! 전방 차량이 빠르게 접근 중입니다.")
            message_Heading = can.Message(arbitration_id=0x102, data=[0x3c & 0xFF, 0x3c, 0x10, (1 << 7) | (0 << 6) | (1 << 5) | (0 << 4), 1, 0, 1, count], is_extended_id=False)
            time.sleep(0.09)
            bus.send(message_Heading)
            print(f"Tx_Long_cmd : ID=0x102, Packet={message_Heading}")
            count += 1
            return True
        else:
            print("차선 변경 불필요. 현재 전방 차량 상태 양호.")
            message_Heading = can.Message(arbitration_id=0x102, data=[0x3c & 0xFF, 0xCF, 0x10, (1 << 7) | (0 << 6) | (1 << 5) | (0 << 4), 0, 0, 1, count], is_extended_id=False)
            time.sleep(0.09)
            bus.send(message_Heading)
            print(f"Tx_Long_cmd : ID=0x102, Packet={message_Heading}")
            count += 1
            return False
    return None

try:
    print("CAN 메시지 수신 중...")
    while True:
        message = bus.recv()
        if message is not None:            
            lane_change_signal = parse_can_message(message)
            
except KeyboardInterrupt:
    print("프로그램 종료")
finally:
    bus.shutdown()
