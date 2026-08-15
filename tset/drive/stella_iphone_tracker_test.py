import time
import serial
import serial.tools.list_ports

# 1. 시리얼 포트 자동으로 찾아 연결
ports = list(serial.tools.list_ports.comports())
if not ports:
    print("[에러] Qorvo 모듈이 연결되어 있지 않습니다.")
    exit()

target_port = ports[0].device
print(f"[{target_port}] 포트로 Qorvo 모듈 연결 중...")

try:
    ser = serial.Serial(target_port, 115200, timeout=1)
    print("연결 성공! 아이폰 위치 및 방향 추적을 시작합니다.\n")
except Exception as e:
    print(f"포트 연결 실패: {e}")
    exit()

current_distance = None
current_azimuth = 0 # 각도 기본값 (0도가 정면)

def evaluate_stella_action(dist_cm, azimuth_deg):
    """
    아이폰의 거리(cm)와 각도(도)를 바탕으로 스텔라의 행동을 판단합니다.
    (각도가 음수면 왼쪽, 양수면 오른쪽)
    """
    dist_m = dist_cm / 100.0 # cm를 m 단위로 변환
    
    # 1. 거리 판단
    if dist_cm < 40:
        dist_status = "정지 (목적지 도착)"
        move_command = "STOP"
    elif dist_cm < 150:
        dist_status = "직진 추적 중"
        move_command = "FORWARD"
    else:
        dist_status = "고속 직진 추적"
        move_command = "FAST_FORWARD"
        
    # 2. 방향 판단 (각도 오차 범위를 ±15도로 설정)
    if azimuth_deg < -15:
        dir_status = f"좌측 ({azimuth_deg}°)"
        turn_command = "TURN_LEFT"
    elif azimuth_deg > 15:
        dir_status = f"우측 (+{azimuth_deg}°)"
        turn_command = "TURN_RIGHT"
    else:
        dir_status = f"정면 ({azimuth_deg}°)"
        turn_command = "KEEP_CENTER"

    print(f"[실시간 인식] 거리: {dist_cm:3d}cm ({dist_m:.2f}m) | 방향: {dir_status}")
    print(f" └─> 스텔라 모터 제어 신호: [{turn_command}] + [{move_command}]\n")

print("--------------------------------------------------")
print("아이폰을 스텔라 기준으로 좌/우/앞/뒤로 움직여보세요!")
print("--------------------------------------------------")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Distance 파싱
            if "Distance:" in line:
                try:
                    current_distance = int(line.split("Distance:")[1].strip())
                except ValueError:
                    pass
            
            # Azimuth / Angle 파싱 (Qorvo 모듈의 출력 형식에 맞게 파싱)
            if "Azimuth:" in line or "Angle:" in line:
                try:
                    # 예: "Azimuth: -25" 형식인 경우
                    parts = line.replace("Angle:", "Azimuth:").split("replace:")
                    current_azimuth = int(line.split(":")[-1].strip())
                except ValueError:
                    pass

            # 거리 데이터가 들어올 때마다 상태 평가 출력
            if current_distance is not None:
                evaluate_stella_action(current_distance, current_azimuth)
                current_distance = None # 다음 수신을 위해 초기화
                
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n추적 테스트 종료")
