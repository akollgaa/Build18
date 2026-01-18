import asyncio
from bleak import BleakClient
from keyboard import is_pressed, add_hotkey

# ====== FPGA BLE CONFIG ======
FPGA_ADDRESS = "EF:5A:77:54:D7:CD"
CHAR_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"  # Nordic UART RX

# ====== Control State ======
pitch = 0
yaw = 0
initialize_mpu = 0
initialize_mpu_motor = 0

# PID constants
pitch_kP = 0
pitch_kI = 0
pitch_kD = 0
yaw_kP = 0
yaw_kI = 0
yaw_kD = 0

# ====== Helpers ======
def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def to_uint8(val):
    """Convert to unsigned 8-bit (0-255)"""
    return int(val) & 0xFF

def make_packet():
    """
    Create a packet matching the SV ble_vector_parser format:
    10 bytes + newline (0x0A) terminator
    
    Packet format:
    [0] initialize_mpu_motor
    [1] initialize_mpu
    [2] ble_set_pitch
    [3] ble_set_yaw
    [4] ble_pitch_kP
    [5] ble_pitch_kI
    [6] ble_pitch_kD
    [7] ble_yaw_kP
    [8] ble_yaw_kI
    [9] ble_yaw_kD
    [10] newline (0x0A)
    """
    return bytes([
        to_uint8(initialize_mpu_motor),
        to_uint8(initialize_mpu),
        to_uint8(pitch),      # ble_set_pitch
        to_uint8(yaw),        # ble_set_yaw
        to_uint8(pitch_kP),
        to_uint8(pitch_kI),
        to_uint8(pitch_kD),
        to_uint8(yaw_kP),
        to_uint8(yaw_kI),
        to_uint8(yaw_kD),
        0x0A  # newline terminator
    ])

# ====== Async BLE Loop ======
async def ble_loop():
    global pitch, yaw

    while True:
        try:
            async with BleakClient(FPGA_ADDRESS, timeout=20.0) as client:
                print(f"✓ Connected to {FPGA_ADDRESS}")
                print(f"✓ Ready to send packets (10 bytes + newline)")

                while True:
                    # --- read joystick keys ---
                    pitch = 0
                    yaw = 0

                    if is_pressed("w") or is_pressed("up"):
                        pitch += 10
                    elif is_pressed("s") or is_pressed("down"):
                        pitch -= 10

                    if is_pressed("d") or is_pressed("right"):
                        yaw += 10
                    elif is_pressed("a") or is_pressed("left"):
                        yaw -= 10

                    pitch = clamp(pitch, 0, 255)
                    yaw = clamp(yaw, 0, 255)

                    # --- send packet ---
                    packet = make_packet()
                    
                    await client.write_gatt_char(CHAR_UUID, packet, response=False)

                    # --- small delay ---
                    await asyncio.sleep(0.05)

        except KeyboardInterrupt:
            print("\n✓ Exiting...")
            break
        except Exception as e:
            print(f"✗ BLE connection lost or error: {e}")
            print("  Reconnecting in 2 seconds...")
            await asyncio.sleep(2.0)

# ====== Hotkeys for changing constants dynamically ======
def setup_hotkeys():
    add_hotkey("1", lambda: increment("pitch_kP", 1))
    add_hotkey("2", lambda: increment("pitch_kP", -1))
    add_hotkey("3", lambda: increment("pitch_kI", 1))
    add_hotkey("4", lambda: increment("pitch_kI", -1))
    add_hotkey("5", lambda: increment("pitch_kD", 1))
    add_hotkey("6", lambda: increment("pitch_kD", -1))
    add_hotkey("7", lambda: increment("yaw_kP", 1))
    add_hotkey("8", lambda: increment("yaw_kP", -1))
    add_hotkey("9", lambda: increment("yaw_kI", 1))
    add_hotkey("0", lambda: increment("yaw_kI", -1))
    add_hotkey("-", lambda: increment("yaw_kD", 1))
    add_hotkey("=", lambda: increment("yaw_kD", -1))
    add_hotkey("[", lambda: increment("initialize_mpu", 1))
    add_hotkey("]", lambda: increment("initialize_mpu", -1))
    add_hotkey(";", lambda: increment("initialize_mpu_motor", 1))
    add_hotkey("'", lambda: increment("initialize_mpu_motor", -1))

def increment(name, delta):
    global pitch_kP, pitch_kI, pitch_kD, yaw_kP, yaw_kI, yaw_kD
    global initialize_mpu, initialize_mpu_motor
    
    if name == "pitch_kP":
        pitch_kP = clamp(pitch_kP + delta, 0, 255)
        print(f"pitch_kP = {pitch_kP}")
    elif name == "pitch_kI":
        pitch_kI = clamp(pitch_kI + delta, 0, 255)
        print(f"pitch_kI = {pitch_kI}")
    elif name == "pitch_kD":
        pitch_kD = clamp(pitch_kD + delta, 0, 255)
        print(f"pitch_kD = {pitch_kD}")
    elif name == "yaw_kP":
        yaw_kP = clamp(yaw_kP + delta, 0, 255)
        print(f"yaw_kP = {yaw_kP}")
    elif name == "yaw_kI":
        yaw_kI = clamp(yaw_kI + delta, 0, 255)
        print(f"yaw_kI = {yaw_kI}")
    elif name == "yaw_kD":
        yaw_kD = clamp(yaw_kD + delta, 0, 255)
        print(f"yaw_kD = {yaw_kD}")
    elif name == "initialize_mpu":
        initialize_mpu = clamp(initialize_mpu + delta, 0, 255)
        print(f"initialize_mpu = {initialize_mpu}")
    elif name == "initialize_mpu_motor":
        initialize_mpu_motor = clamp(initialize_mpu_motor + delta, 0, 255)
        print(f"initialize_mpu_motor = {initialize_mpu_motor}")

# ====== Main ======
if __name__ == "__main__":
    print("=== FPGA BLE Controller ===")
    print("\nJoystick Controls:")
    print("  W/↑ : Pitch +10")
    print("  S/↓ : Pitch -10")
    print("  D/→ : Yaw +10")
    print("  A/← : Yaw -10")
    print("\nPID Tuning Hotkeys:")
    print("  1/2: pitch_kP ±1")
    print("  3/4: pitch_kI ±1")
    print("  5/6: pitch_kD ±1")
    print("  7/8: yaw_kP ±1")
    print("  9/0: yaw_kI ±1")
    print("  -/=: yaw_kD ±1")
    print("\nInitialization Hotkeys:")
    print("  [/]: initialize_mpu ±1")
    print("  ;/': initialize_mpu_motor ±1")
    print("\nPress Ctrl+C to exit\n")
    
    setup_hotkeys()
    try:
        asyncio.run(ble_loop())
    except KeyboardInterrupt:
        print("\n✓ Shutdown complete")