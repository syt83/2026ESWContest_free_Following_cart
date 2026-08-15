#imu의 현재상태를 보여주는 코드
import asyncio
import json
import websockets
from smbus2 import SMBus

# ==========================================
# 1. IMU (MPU-6050) 초기화
# ==========================================
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ADDRESS = 0x68

bus = SMBus(1)
bus.write_byte_data(ADDRESS, PWR_MGMT_1, 0) # Sleep 해제

def read_raw_data(addr):
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def get_imu_data():
    # 가속도 (g)
    acc_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    acc_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    acc_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    
    # 자이로 (°/s)
    gyro_z = read_raw_data(GYRO_XOUT_H + 4) / 131.0

    return {
        "acc_x": round(acc_x, 2),
        "acc_y": round(acc_y, 2),
        "acc_z": round(acc_z, 2),
        "gyro_z": round(gyro_z, 2)
    }

# ==========================================
# 2. 웹소켓 핸들러
# ==========================================
connected_clients = set()

async def handler(websocket):
    print(f"📱 스마트폰 앱 연결됨: {websocket.remote_address}")
    connected_clients.add(websocket)
    
    try:
        # 스마트폰에서 들어오는 조종 신호 처리 루프
        async for message in websocket:
            print(f"[수신 메시지]: {message}")
            # 추후 조종 명령 수신 처리
            
    except websockets.exceptions.ConnectionClosed:
        print("📱 스마트폰 앱 연결 해제됨")
    finally:
        connected_clients.remove(websocket)

async def send_imu_loop():
    # 연결된 스마트폰으로 0.1초(10Hz)마다 IMU 데이터 전송
    while True:
        if connected_clients:
            imu_data = get_imu_data()
            # JSON 형태로 포맷팅
            payload = json.dumps({"type": "imu", "data": imu_data})
            
            # 모든 연결된 클라이언트에 전송
            for ws in list(connected_clients):
                try:
                    await ws.send(payload)
                except Exception:
                    pass
        await asyncio.sleep(0.1)

async def main():
    # 8765 포트로 웹소켓 서버 오픈
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("🚀 라즈베리 파이 웹소켓 서버 실행 중 (포트: 8765)...")
        await send_imu_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n서버가 종료되었습니다.")
        bus.close()
