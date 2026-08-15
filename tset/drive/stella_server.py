import asyncio
import json
import websockets
from collections import deque

# -------------------------------------------------------------
# 1. 설정 및 파라미터
# -------------------------------------------------------------
PORT = 8765
TARGET_DISTANCE = 1.0  # 목표 유지 거리 (1미터)
MARGIN = 0.3          # 거리 오차 허용 범위 (±30cm)

# BLE 신호 튀는 현상을 잡기 위한 이동 평균 필터 (최근 5개 데이터 평균)
distance_history = deque(maxlen=5)
angle_history = deque(maxlen=5)

# -------------------------------------------------------------
# 2. 모터 제어 함수 (스텔라 하드웨어 제어부)
# -------------------------------------------------------------
def drive_stella(distance, angle):
    """
    필터링된 거리(m)와 각도(도)를 기반으로 모터를 제어합니다.
    """
    print(f"[제어 신호] 거리: {distance:.2f}m | 각도: {angle:.1f}°")

    # (1) 방향 제어 (각도 기반)
    if angle > 15:
        print(" -> 🔄 우회전 (Turn Right)")
        # TODO: 스텔라 우회전 모터 신호 (예: motor.turn_right())
    elif angle < -15:
        print(" -> 🔄 좌회전 (Turn Left)")
        # TODO: 스텔라 좌회전 모터 신호 (예: motor.turn_left())
    else:
        # (2) 전진/후진/정지 제어 (거리 기반)
        if distance > (TARGET_DISTANCE + MARGIN):
            print(" -> ⬆️ 전진 (Forward)")
            # TODO: 스텔라 전진 모터 신호 (예: motor.forward())
        elif distance < (TARGET_DISTANCE - MARGIN):
            print(" -> ⬇️ 후진 (Backward)")
            # TODO: 스텔라 후진 모터 신호 (예: motor.backward())
        else:
            print(" -> 🛑 정지 (Stop - 안전 거리 유지)")
            # TODO: 스텔라 정지 모터 신호 (예: motor.stop())

# -------------------------------------------------------------
# 3. 웹소켓 수신 루프
# -------------------------------------------------------------
async def handle_client(websocket):
    print("📱 스마트폰(Flutter App) 연결 성공!")
    try:
        async for message in websocket:
            data = json.loads(message)
            
            raw_dist = data.get("distance", 1.0)
            raw_angle = data.get("angle", 0.0)

            # 이동 평균 필터 적용 (데이터 평탄화)
            distance_history.append(raw_dist)
            angle_history.append(raw_angle)

            avg_distance = sum(distance_history) / len(distance_history)
            avg_angle = sum(angle_history) / len(angle_history)

            # 모터 제어 함수 호출
            drive_stella(avg_distance, avg_angle)

    except websockets.exceptions.ConnectionClosed:
        print("⚠️ 스마트폰 연결이 끊겼습니다! 비상 정지!")
        # TODO: motor.stop() 호출하여 비상 정지 처리

async def main():
    server = await websockets.serve(handle_client, "0.0.0.0", PORT)
    print(f"🚀 스텔라 수신 서버 실행 중... (Port: {PORT})")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
