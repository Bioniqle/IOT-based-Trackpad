import serial
import pyautogui
import time

# Disable PyAutoGUI pauses for real-time cursor response
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False

# Update COM port (e.g., 'COM3' on Windows, '/dev/ttyACM0' or '/dev/ttyUSB0' on Linux/Mac)
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

# Physical plane dimensions (cm)
PLANE_WIDTH = 32.0
PLANE_HEIGHT = 18.0

# Monitor dimensions
SCREEN_W, SCREEN_H = pyautogui.size()

# Smoothing state
smooth_x, smooth_y = SCREEN_W / 2, SCREEN_H / 2
ALPHA = 0.25  # Smoothing factor (0.1 = very smooth/laggy, 0.8 = snappy/jittery)

def map_range(val, in_min, in_max, out_min, out_max):
    val = max(in_min, min(in_max, val))
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main():
    global smooth_x, smooth_y
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(2)  # Wait for Arduino reset
        print(f"Connected! Mapping 32x18cm plane to {SCREEN_W}x{SCREEN_H} screen.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) == 3 and parts[0] == "LOCKED":
                x_cm = float(parts[1])
                y_cm = float(parts[2])

                # Map X (0 -> 32 cm) to Screen X (0 -> SCREEN_W)
                target_px_x = map_range(x_cm, 0.0, PLANE_WIDTH, 0, SCREEN_W)
                
                # Map Y (0 -> 18 cm) to Screen Y (SCREEN_H -> 0) [Inverted for screen top-left origin]
                target_px_y = map_range(y_cm, 0.0, PLANE_HEIGHT, SCREEN_H, 0)

                # Apply exponential smoothing to eliminate ultrasonic jitter
                smooth_x = (1.0 - ALPHA) * smooth_x + ALPHA * target_px_x
                smooth_y = (1.0 - ALPHA) * smooth_y + ALPHA * target_px_y

                pyautogui.moveTo(int(smooth_x), int(smooth_y))

        except KeyboardInterrupt:
            print("\nExiting mouse controller.")
            break
        except ValueError:
            pass

if __name__ == "__main__":
    main()