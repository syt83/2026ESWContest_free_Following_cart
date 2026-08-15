import os
import tkinter as tk

def check_display():
    print("====================================")
    print(" 🔍 라즈베리파이 디스플레이 진단 시작")
    print("====================================\n")

    # 1. 환경 변수 확인
    display_env = os.environ.get('DISPLAY')
    wayland_env = os.environ.get('WAYLAND_DISPLAY')

    print("[1단계] 환경 변수 체크")
    print(f" - DISPLAY 변수: {display_env}")
    print(f" - WAYLAND_DISPLAY 변수: {wayland_env}\n")

    if not display_env and not wayland_env:
        print(" ⚠️ 그래픽 환경 변수가 비어있습니다. (SSH 원격 접속 중이거나 바탕화면이 없는 OS일 수 있습니다.)")
        print(" 🛠️ 강제로 DISPLAY=':0' 환경 변수를 주입하여 테스트를 시도합니다...\n")
        os.environ['DISPLAY'] = ':0'

    # 2. GUI 연결 테스트
    print("[2단계] 그래픽 엔진(Tkinter) 연결 테스트")
    try:
        # 화면 객체 생성 시도
        root = tk.Tk()
        print(" ✅ 성공! 라즈베리파이가 모니터를 정상적으로 인식하고 그래픽을 띄울 준비가 되었습니다.")
        root.destroy()
        
    except tk.TclError as e:
        print(f" ❌ 실패! 화면 연결에 실패했습니다.")
        print(f" └─ 에러 내용: {e}\n")
        
        print("====================================")
        print(" 💡 [최종 진단 결과 및 해결책] 💡")
        print("====================================")
        print("1. 하드웨어 미인식: HDMI 선이 제대로 안 꽂혀 있거나, 라즈베리파이가 켜진 후 모니터를 늦게 켰을 수 있습니다. (라즈베리파이를 재부팅해 보세요)")
        print("2. OS 문제: 바탕화면이 없는 'Raspberry Pi OS Lite' 버전이 설치되어 있을 확률이 높습니다. 이 경우 Tkinter(GUI)를 사용할 수 없습니다.")
        print("3. 권한 문제: SSH로 접속 중이라면, 모니터에 화면을 띄울 권한이 막혀 있는 것입니다.")

if __name__ == '__main__':
    check_display()
