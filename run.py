import serial
import pyautogui
import time
import math

# --- CONFIGURATION ---
PORT = 'COM7'         
BAUD = 9600         
PLANE_WIDTH = 32.0    
PLANE_HEIGHT = 18.0   

# Crops 1.0 cm off the virtual edges so your reachable physical area 
# stretches all the way to the very edges of your monitor
ACTIVE_MARGIN_X = 1.0  # Maps X: 1.0cm -> 31.0cm to 0 -> 100% screen width
ACTIVE_MARGIN_Y = 1.0  # Maps Y: 1.0cm -> 17.0cm to 0 -> 100% screen height

SCREEN_W, SCREEN_H = pyautogui.size()

# Calculate exact multipliers (Pixels per cm)
SCALE_X = SCREEN_W / (PLANE_WIDTH - (2 * ACTIVE_MARGIN_X))
SCALE_Y = SCREEN_H / (PLANE_HEIGHT - (2 * ACTIVE_MARGIN_Y))

pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.005 

DWELL_TIME_SEC = 1.2        
DWELL_RADIUS_CM = 1.5       
last_click_time = 0
dwell_start_time = 0
dwell_anchor_x = None
dwell_anchor_y = None

print(f"Monitor Detected: {SCREEN_W}x{SCREEN_H} (Ratio: {SCREEN_W/SCREEN_H:.2f})")
print(f"Board Size: {PLANE_WIDTH}x{PLANE_HEIGHT} cm (Ratio: {PLANE_WIDTH/PLANE_HEIGHT:.2f})")
print(f"Multiplier: 1 cm = {SCALE_X:.1f} px (X) | {SCALE_Y:.1f} px (Y)")

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2)
    print("Trackpad Active! Move your hand over the board.")
except Exception as e:
    print(f"Connection Failed: {e}")
    exit()

was_out = True

while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            
            # Ignore debug lines if any are active
            if line.startswith("#"):
                continue

            if line == "OUT":
                if not was_out:
                    print("Hand lost. Returning to idle sweep...")
                    was_out = True
                dwell_start_time = 0 
                dwell_anchor_x = None
                continue
                
            if "," in line:
                if was_out:
                    print("TARGET LOCKED! Tracking cursor...")
                    was_out = False
                    
                parts = line.split(",")
                cmX = float(parts[0])
                cmY = float(parts[1])
                
                # Invert Y axis (Board Y=0 is bottom wall, Screen Y=0 is top edge)
                cmY_inverted = PLANE_HEIGHT - cmY 
                
                # Shift by active margin and multiply by the screen scale factor
                pixelX = int((cmX - ACTIVE_MARGIN_X) * SCALE_X)
                pixelY = int((cmY_inverted - ACTIVE_MARGIN_Y) * SCALE_Y)
                
                # Clamp 5 pixels inside the monitor corners so PyAutoGUI FailSafe never crashes
                pixelX = max(5, min(pixelX, SCREEN_W - 5))
                pixelY = max(5, min(pixelY, SCREEN_H - 5))
                
                pyautogui.moveTo(pixelX, pixelY, _pause=False)
                
                # Dwell Click Logic
                current_time = time.time()
                if dwell_anchor_x is None:
                    dwell_anchor_x = cmX
                    dwell_anchor_y = cmY
                    dwell_start_time = current_time
                else:
                    drift = math.sqrt((cmX - dwell_anchor_x)**2 + (cmY - dwell_anchor_y)**2)
                    
                    if drift > DWELL_RADIUS_CM:
                        dwell_anchor_x = cmX
                        dwell_anchor_y = cmY
                        dwell_start_time = current_time
                    else:
                        time_held = current_time - dwell_start_time
                        if time_held >= DWELL_TIME_SEC and (current_time - last_click_time) > 2.0:
                            print(f"CLICK triggered at ({cmX:.1f}cm, {cmY:.1f}cm) -> ({pixelX}px, {pixelY}px)")
                            pyautogui.click()
                            last_click_time = current_time
                            dwell_start_time = current_time 

    except KeyboardInterrupt:
        print("\nTrackpad Offline.")
        ser.close()
        break
    except ValueError:
        pass