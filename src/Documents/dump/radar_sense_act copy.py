import can

# CAN 인터페이스 설정 (여기서는 `can0`을 사용)
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, interface='socketcan')

def parse_can_message(message):
    # 메시지가 ID 200인지 확인
    if message.arbitration_id == 0x200:
        # dx와 vx 추출
        # dx는 start bit가 6이고, 13비트 길이
        dx_raw = (message.data[0] & 0xFC) << 5 | message.data[1]  # 비트 마스킹과 시프트로 추출
        dx = dx_raw * 0.0625 - 256  # 변환

        # vx는 start bit가 31이고, 10비트 길이
        vx_raw = (message.data[3] & 0xFF) << 2 | (message.data[4] >> 6)
        vx = vx_raw * 0.125 - 64  # 변환

        # 조건 설정: dx가 10m 이하이고, vx가 -2 m/s 이하이면 차선 변경 신호 반환
        if dx <= 4.0 and vx <= -2:
            print("차선 변경 필요! 전방 차량이 빠르게 접근 중입니다.")
            return True
        else:
            print("차선 변경 불필요. 현재 전방 차량 상태 양호.")
            return False
    return None

try:
    print("CAN 메시지 수신 중...")
    while True:
        # CAN 메시지 수신
        message = bus.recv()
        if message is not None:
            # print(f"수신된 메시지: ID={hex(message.arbitration_id)}, 데이터={message.data.hex()}")  # 디버깅 출력
            lane_change_signal = parse_can_message(message)
            if message.arbitration_id == 0x200:
                print(f"수신된 메시지: ID={hex(message.arbitration_id)}, 데이터={message.data.hex()}")  # 디버깅 출력
            if lane_change_signal:
                # 여기에 차선 변경 신호를 보내는 코드 추가
                pass
except KeyboardInterrupt:
    print("프로그램 종료")

