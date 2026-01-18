import asyncio
from bleak import BleakScanner

async def scan_ble_devices():
    print("Scanning for BLE devices (10 seconds)...\n")
    
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    
    if not devices:
        print("No devices found!")
        return
    
    print(f"Found {len(devices)} device(s):\n")
    
    for address, (device, adv_data) in devices.items():
        print(f"Name: {device.name or 'Unknown'}")
        print(f"Address: {device.address}")
        print(f"RSSI: {adv_data.rssi} dBm")
        
        # Check for Nordic UART Service
        if adv_data.service_uuids:
            print(f"Services: {adv_data.service_uuids}")
            if "6e400001-b5a3-f393-e0a9-e50e24dcca9e" in adv_data.service_uuids:
                print("✓✓✓ HAS NORDIC UART SERVICE - THIS IS YOUR FPGA! ✓✓✓")
        
        print("-" * 60)

if __name__ == "__main__":
    asyncio.run(scan_ble_devices())