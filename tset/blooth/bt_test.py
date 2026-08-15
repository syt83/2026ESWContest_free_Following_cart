import asyncio
from bleak import BleakScanner

async def main():
    print("주변 블루투스 기기 스캔 중... (약 5초 소요)")
    
    # callback 방식을 사용하거나 discover의 return_adv=True를 사용하면 adv(advertising) 데이터에서 RSSI를 얻을 수 있습니다.
    devices = await BleakScanner.discover(return_adv=True, timeout=5.0)
    
    print("\n[스캔 결과]")
    for bd_addr, (device, adv_data) in devices.items():
        name = device.name if device.name else "Unknown"
        rssi = adv_data.rssi
        print(f"기기 이름: {name} | MAC 주소: {bd_addr} | RSSI(신호세기): {rssi} dBm")

if __name__ == "__main__":
    asyncio.run(main())
