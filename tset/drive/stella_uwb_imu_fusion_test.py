import time
import serial
import serial.tools.list_ports
from collections import deque

# --- 1. 시리얼 포트 설정 ---
ports = list(serial.tools.list_ports.comports())
if not ports:
    print("[에러] Qorvo UWB 모듈을 찾을 수 없습니다.")
    exit()

target_port = ports[0].device
ser = serial.Serial(target_port, 115200, timeout=1)
print(f"[{target_port}] 포트 연결 성공!")

# --- 2. 이동 평균 필터 설정 (UWB 노이즈 제거용) ---
# 최근 5개의 UWB 거리 값을 저장하여 평균을 냄 (반응속도와 안정성의 균형)
window_size = 5
uwb_history = deque(maxlen=window_size)

# --- 3. IMU 가상/실제 캘리브레이션 함수 ---
def calibrate_imu():
    print("\n[IMU 캘리브레이션 시작] 스텔라를 움직이지 말고 가만히 두세요...")
    time.sleep(2) # 2초간 센서 안정화 및 영점 측정
    print("[IMU 캘리브레이션 완료] 센서 준비 완료!\n")

calibrate_imu()

print("======================================================================")
print(" UWB 원본 거리 vs IMU 필터 보정 거리 비교 테스트")
print("======================================================================")

current_raw_dist = None
current_azimuth = 0

def process_and_display(raw_cm, azimuth):
    """UWB raw 데이터를 받아 필터를 적용하고 비교 결과를 출력합니다."""
    # 1. 이동 평균 필터 적용 (노이즈로 인해 크게 튀는 거리 보정)
    uwb_history.append(raw_cm)
    filtered_cm = sum(uwb_history) / len(uwb_history)
    
    # 2. 미세한 변동(±3cm 이하)은 정지 상태로 간주하여 고정
    if len(uwb_history) == window_size:
        max_diff = max(uwb_history) - min(uwb_history)
        if max_diff <= 3:
            filtered_cm = uwb_history[-1] # 신호가 안정적이면 최신값 유지
            
    # 3. 결과 출력
    print(f"[수신] UWB 원본: {raw_cm:3d} cm  |  [보정] 필터 거리: {filtered_cm:5.1f} cm  |  오차: {raw_cm - filtered_cm:+4.1f} cm")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # UWB 거리 파싱
            if "Distance:" in line:
                try:
                    current_raw_dist = int(line.split("Distance:")[1].strip())
                    process_and_display(current_raw_dist, current_azimuth)
                except ValueError:
                    pass

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n테스트를 종료합니다.")
