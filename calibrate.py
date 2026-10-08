import tkinter as tk
import serial
import math

# --- CONFIGURATION ---
PORT = 'COM7'         
BAUD = 9600         
PLANE_WIDTH = 32.0    
PLANE_HEIGHT = 18.0   
SCALE = 25            # Pixels per cm for the visual UI

WINDOW_W = int(PLANE_WIDTH * SCALE)
WINDOW_H = int(PLANE_HEIGHT * SCALE)

class CalibrationTool:
    def __init__(self, root):
        self.root = root
        self.root.title("Radar Trackpad Visualizer")
        self.root.configure(bg="#1e1e1e")
        
        self.coord_lbl = tk.Label(root, text="Waiting for hand lock-on...", font=("Consolas", 14), bg="#1e1e1e", fg="#00ff00")
        self.coord_lbl.pack(pady=10)

        self.canvas = tk.Canvas(root, width=WINDOW_W, height=WINDOW_H, bg="#0d0d0d", highlightthickness=1)
        self.canvas.pack(padx=20, pady=10)

        # Draw 32x18 Board Grid
        for x in range(0, int(PLANE_WIDTH) + 1, 4):
            self.canvas.create_line(x * SCALE, 0, x * SCALE, WINDOW_H, fill="#333333", dash=(2, 4))
        for y in range(0, int(PLANE_HEIGHT) + 1, 3):
            self.canvas.create_line(0, y * SCALE, WINDOW_W, y * SCALE, fill="#333333", dash=(2, 4))

        # Draw Physical Servo Locations based on Math
        self.draw_servo_marker(0, 9.0, "Servo 1 (0, 9)")
        self.draw_servo_marker(16.0, 0, "Servo 2 (16, 0)")

        # The Cursor
        self.cursor_dot = self.canvas.create_oval(-20, -20, -20, -20, fill="#00ff00", outline="white", width=2)
        self.trail = []

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.01)
            self.root.after(50, self.read_serial)
        except Exception as e:
            self.coord_lbl.config(text=f"ERROR: Could not open {PORT}. Close other scripts!", fg="#ff0000")

    def draw_servo_marker(self, cm_x, cm_y, label):
        px = cm_x * SCALE
        py = (PLANE_HEIGHT - cm_y) * SCALE
        r = 6
        self.canvas.create_rectangle(px-r, py-r, px+r, py+r, fill="#ffaa00", outline="white")
        
        # Adjust text offset depending on wall
        text_x = px + 40 if cm_x == 0 else px
        text_y = py - 20 if cm_x != 0 else py
        self.canvas.create_text(text_x, text_y, text=label, fill="#ffaa00", font=("Arial", 9, "bold"))

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line == "OUT":
                    self.coord_lbl.config(text="Status: Idle Sweeping (No Target)")
                    self.canvas.coords(self.cursor_dot, -20, -20, -20, -20)
                    self.clear_trail()
                    
                elif "," in line:
                    parts = line.split(",")
                    cmX = float(parts[0])
                    cmY = float(parts[1])
                    
                    self.coord_lbl.config(text=f"Target Locked -> X: {cmX:.1f} cm | Y: {cmY:.1f} cm")
                    
                    pixelX = cmX * SCALE
                    pixelY = (PLANE_HEIGHT - cmY) * SCALE
                    
                    r = 8 
                    self.canvas.coords(self.cursor_dot, pixelX - r, pixelY - r, pixelX + r, pixelY + r)
                    
                    # Draw fading trail to visualize movement lag
                    trail_dot = self.canvas.create_oval(pixelX-2, pixelY-2, pixelX+2, pixelY+2, fill="#006600", outline="")
                    self.trail.append(trail_dot)
                    if len(self.trail) > 10:
                        old_dot = self.trail.pop(0)
                        self.canvas.delete(old_dot)
        except:
            pass 
            
        self.root.after(15, self.read_serial)

    def clear_trail(self):
        for dot in self.trail:
            self.canvas.delete(dot)
        self.trail.clear()

if __name__ == "__main__":
    root = tk.Tk()
    app = CalibrationTool(root)
    root.mainloop()