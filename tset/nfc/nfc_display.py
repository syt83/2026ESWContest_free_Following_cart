#nfc 테스트 
import time
import threading
import tkinter as tk
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

class RobotDisplayApp:
    def __init__(self, root):
        self.root = root
        # 💡 전체 화면(Full Screen) 설정
        self.root.attributes('-fullscreen', True)
        self.root.config(bg="black")  # 대기 상태는 까만 배경

        # 화면 정중앙에 텍스트 표시
        self.label = tk.Label(
            self.root, 
            text="[ STANDBY ]\n\n로봇을 깨우려면 NFC 카드를 태그하세요.", 
            fg="gray", 
            bg="black", 
            font=("Helvetica", 30, "bold")
        )
        self.label.pack(expand=True)

        # 화면 닫기 버튼 (테스트 중 종료를 위해 - 클릭하거나 터치하면 꺼짐)
        self.quit_btn = tk.Button(self.root, text="종료 (X)", command=self.close_app, bg="red", fg="white")
        self.quit_btn.place(x=10, y=10)

        # 💡 백그라운드에서 NFC를 기다리는 스레드 시작
        self.nfc_thread = threading.Thread(target=self.wait_for_nfc, daemon=True)
        self.nfc_thread.start()

    def wait_for_nfc(self):
        reader = SimpleMFRC522()
        try:
            print("NFC 리더기 대기 중...")
            # 카드가 태그될 때까지 이 스레드는 여기서 멈춰서 기다림
            id, text = reader.read() 
            print(f"✅ 카드 인식 성공! ID: {id}")
            
            # 카드가 인식되면 화면을 바꾸는 함수를 호출 (GUI 스레드 안전하게 호출)
            self.root.after(0, self.activate_screen, id)
        finally:
            GPIO.cleanup()

    def activate_screen(self, card_id):
        # 💡 [ACTIVE 상태 전환] 화면 배경을 파란색으로, 글씨를 흰색으로 변경
        self.root.config(bg="#0055ff")
        self.label.config(
            text=f"[ ACTIVE ]\n\n인증 완료!\n\n(Card ID: {card_id})\n\n로봇이 주행을 시작합니다 🚀",
            fg="white",
            bg="#0055ff",
            font=("Helvetica", 40, "bold")
        )
        # 여기서 로봇 얼굴(이미지)을 띄우거나, 주행 데이터를 보여주는 화면으로 넘어가도 됩니다!

    def close_app(self):
        self.root.destroy()
        GPIO.cleanup()

def main():
    # Tkinter 화면(GUI) 객체 생성
    root = tk.Tk()
    app = RobotDisplayApp(root)
    
    # 화면을 계속 띄워두는 무한 루프 (메인 스레드)
    root.mainloop()

if __name__ == '__main__':
    main()
