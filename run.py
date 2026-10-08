import serial
import pyautogui
import time
import math

# --- CONFIGURATION ---
PORT = 'COM7'         
BAUD = 9600         
PLANE_WIDTH = 32.0    
PLANE_HEIGHT = 18.0   

SCREEN_W, SCREEN_H = pyautogui.size()

# Keep Failsafe ON, but we will mathematically avoid the corners
pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.005 

DWELL_TIME_SEC = 1.2        
DWELL_RADIUS_CM = 1.5       
last_click_time = 0
dwell_start_time = 0
dwell_anchor_x = None
dwell_anchor_y = None

print(f"Connecting Trackpad on {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2)
    print("Trackpad Active! Motors should be sweeping.")
    print("Place your hand broadly across the board to let the sweeping radar catch it.")
except Exception as e:
    print(f"Connection Failed: {e}")
    exit()

was_out = True

while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            
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
                
                cmY_inverted = PLANE_HEIGHT - cmY 
                
                pixelX = int((cmX / PLANE_WIDTH) * SCREEN_W)
                pixelY = int((cmY_inverted / PLANE_HEIGHT) * SCREEN_H)
                
                # ANTI-CRASH CLAMP: Keep mouse 5 pixels away from the literal screen edges
                # This prevents the PyAutoGUI FailSafeException from ever triggering.
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
                            print(f"CLICK triggered at {cmX:.1f}, {cmY:.1f}")
                            pyautogui.click()
                            last_click_time = current_time
                            dwell_start_time = current_time 

    except KeyboardInterrupt:
        print("\nTrackpad Offline. Good job!")
        ser.close()
        break
    except ValueError:
        pass # Ignore garbled serial lines