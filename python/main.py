import cv2, serial
import numpy as np
from scripts.camera import Camera, IS_RASPBERRY, HEIGHT, WIDTH
from scripts.pillar import Pillar

# HSV color ranges 
lower_green = np.array([35, 60, 30])
upper_green = np.array([90, 255, 255])

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([8, 255, 255])
lower_red2 = np.array([168, 100, 100])
upper_red2 = np.array([180, 255, 255])

video = Camera()
video_name = 'output.mp4'
writer = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M','J','P','G'), 15, (WIDTH, HEIGHT))

# Focal length
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

frame_count = 0
arduino_serial = serial.Serial('/dev/ttyS0', 9600, timeout= 0.1)
arduino_serial.flush()

while True:
    largest_pillar = Pillar(0, 0, 0, 0, 0, 0, 0, 0)
    img = video.get_output()
    if img is None:
        print('waiting output...')
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red masks
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Green mask
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    get_all_objects(mask_red, 0)
    get_all_objects(mask_green, 1)

    if (largest_pillar.area > 100):
        cv2.rectangle(img, 
            (largest_pillar.x, largest_pillar.y), 
            (largest_pillar.x + largest_pillar.w, largest_pillar.y + largest_pillar.h), 
            (0, 255, 0) if largest_pillar.color == 1 else (0, 0, 255), 3)
    
        # only x coordinate is important to us
        offset_x = largest_pillar.cx - WIDTH / 2
        distance_y = (4.5 * FOCAL_LENGTH) / (largest_pillar.w or 1)
        distance_x = offset_x * distance_y / (FOCAL_LENGTH or 1)
        output = f"{largest_pillar.color == 1},{distance_x}"
        print(output)
        arduino_serial.write((f"{output}\n").encode())

    
    cv2.line(img, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), (255, 255, 255), 3)

    if not IS_RASPBERRY:
        cv2.imshow("Camera", img)
    
    writer.write(img)
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


video.release()
writer.release()
cv2.destroyAllWindows()