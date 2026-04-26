import cv2, serial
import numpy as np
from scripts.camera import Camera, IS_RASPBERRY, HEIGHT, WIDTH
from scripts.pillar import Pillar

# --- FIX 1: Tighter HSV Ranges ---
# Increased the minimum Saturation (the second value) to 150.
# This prevents "washed out" colors (like orange) from being detected as Red.
lower_green = np.array([35, 100, 50])  # Slightly tighter green
upper_green = np.array([90, 255, 255])

lower_red1 = np.array([0, 150, 120])  # Increased Sat (150) and Value (120)
upper_red1 = np.array([5, 255, 255])  # Narrowed Hue range slightly
lower_red2 = np.array([170, 150, 120]) # Increased Sat (150) and Value (120)
upper_red2 = np.array([180, 255, 255])

# --- FIX 2: Calculate Crop Parameters ---
# Limit 210 degree view to 120 degrees
# New width = Original Width * (120/210)
CROP_RATIO = 210 / 210
PROC_WIDTH = int(WIDTH * CROP_RATIO)
START_X = (WIDTH - PROC_WIDTH) // 2

video = Camera()
# Note: Writer will record the full view, but you are processing the crop
# Change these two lines
video_name = './output.avi' # Change to .avi
writer = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'XVID'), 15, (WIDTH, HEIGHT))

FOCAL_LENGTH = (130 * 30) / 4.5
largest_pillar = Pillar(0, 0, 0, 0, 0, 0, 0, 0)

def get_all_objects(mask, type):
    global largest_pillar
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > largest_pillar.area:
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2
            largest_pillar = Pillar(x, y, w, h, cx, cy, area, type)

arduino_serial = serial.Serial('/dev/ttyS0', 9600, timeout=0.1)
arduino_serial.flush()

while True:
    largest_pillar = Pillar(0, 0, 0, 0, 0, 0, 0, 0)
    img = video.get_output()
    if img is None:
        continue

    # --- APPLY CROP ---
    # Slice the image: [height, start_x : end_x]
    img_processed = img[:, START_X : START_X + PROC_WIDTH]

    hsv = cv2.cvtColor(img_processed, cv2.COLOR_BGR2HSV)

    # Red masks
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Green mask
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    get_all_objects(mask_red, 0)
    get_all_objects(mask_green, 1)

    if (largest_pillar.area > 40):
        # Draw on the processed image (or map back to original if preferred)
        cv2.rectangle(img_processed, 
            (largest_pillar.x, largest_pillar.y), 
            (largest_pillar.x + largest_pillar.w, largest_pillar.y + largest_pillar.h), 
            (0, 255, 0) if largest_pillar.color == 1 else (0, 0, 255), 3)
    
        # Use PROC_WIDTH for the math to ensure accuracy in the cropped view
        offset_x = largest_pillar.cx - PROC_WIDTH / 2
        distance_y = (45 * FOCAL_LENGTH) / (largest_pillar.w or 1)
        distance_x = offset_x * distance_y / (FOCAL_LENGTH or 1)
        output = f"{largest_pillar.color == 1},{round(distance_x)}"
        print(output)
        arduino_serial.write((f"{output}\n").encode())
    else:
        arduino_serial.write("no_target\n".encode())

    # Visual alignment line for the cropped view
    cv2.line(img_processed, (PROC_WIDTH // 2, 0), (PROC_WIDTH // 2, HEIGHT), (255, 255, 255), 3)

    if not IS_RASPBERRY:
        cv2.imshow("Camera", img_processed)
    
    writer.write(img) # Writes original full-size to file
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

video.release()
writer.release()
cv2.destroyAllWindows()