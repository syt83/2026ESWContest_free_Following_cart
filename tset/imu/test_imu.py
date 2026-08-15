import time

try:
    import smbus2 as smbus
except ImportError:
    import smbus

# MPU6050 I2C 주소 (AD0 핀 상태에 따라 0x68 또는 0x69)
MPU6050_ADDRS = [0x68, 0x69]
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


def scan_and_init():
    bus = smbus.SMBus(1)
    target_addr = None

    # 1. 센서 주소 스캔
    for addr in MPU6050_ADDRS:
        try:
            bus.read_byte(addr)
            target_addr = addr
            print(f"✅ IMU 센서를 감지했습니다! (I2C 주소: {hex(addr)})")
            break
        except OSError:
            continue

    if target_addr is None:
        print("❌ IMU 센서를 찾을 수 없습니다.")
        print(
            "👉 VCC/GND 전원 불빛이 들어와 있다면 SDA(Pin 3)와 SCL(Pin 5) 선을 교체하거나 꾹 눌러주세요."
        )
        bus.close()
        return None, None

    # 2. MPU6050 초기화 (Sleep 모드 해제)
    try:
        bus.write_byte_data(target_addr, PWR_MGMT_1, 0)
        time.sleep(0.1)
    except OSError as e:
        
        print(f"❌ 센서 초기화 명령 전송 실패: {e}")
        bus.close()
        return None, None

    return bus, target_addr


def read_raw_data(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value


def main():
    print("=" * 60)
    print(" 🧭 MPU6050 IMU 센서 통신 테스트")
    print("=" * 60)

    bus, addr = scan_and_init()
    if bus is None:
        return

    print("\n📊 데이터 수신 시작 (종료하려면 Ctrl+C 클릭)")
    print("-" * 60)

    try:
        while True:
            # 가속도 데이터 (X, Y, Z)
            acc_x = read_raw_data(bus, addr, ACCEL_XOUT_H)
            acc_y = read_raw_data(bus, addr, ACCEL_XOUT_H + 2)
            acc_z = read_raw_data(bus, addr, ACCEL_XOUT_H + 4)

            # 자이로 데이터 (X, Y, Z)
            gyro_x = read_raw_data(bus, addr, GYRO_XOUT_H)
            gyro_y = read_raw_data(bus, addr, GYRO_XOUT_H + 2)
            gyro_z = read_raw_data(bus, addr, GYRO_XOUT_H + 4)

            print(
                f"[가속도] X:{acc_x:6d} Y:{acc_y:6d} Z:{acc_z:6d} | "
                f"[자이로] X:{gyro_x:6d} Y:{gyro_y:6d} Z:{gyro_z:6d}"
            )
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n테스트를 종료합니다.")
    except OSError:
        print(
            "\n❌ 통신 중 연결이 끊겼습니다. 점퍼선 접촉 상태를 확인하세요."
        )
    finally:
        bus.close()


if __name__ == "__main__":
    main()