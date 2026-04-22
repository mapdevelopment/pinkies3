import platform
import cv2

# let's check if the device is raspberry pi
machine = platform.uname().node.lower()
IS_RASPBERRY = "raspberrypi" in machine
if IS_RASPBERRY:
    from picamera2 import Picamera2

WIDTH = 640
HEIGHT = 360

class Camera:
    def __init__(self):
        if IS_RASPBERRY: 
            self.camera = Picamera2()
            self.camera.configure(
                self.camera.create_video_configuration(main=
                    {"format": "RGB888", "size": (1640, 922)}
                )
            )

            self.camera.set_controls({"FrameRate": 15})
            self.camera.start()
        else:
            self.camera = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

    def get_output(self):
        if not IS_RASPBERRY:
            success, frame = self.camera.read()
            if not success:
                return None
        else:
            frame = self.camera.capture_array()

        frame_resized = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        return frame_resized
    
    def release(self):
        if IS_RASPBERRY:
            return
        
        self.camera.release()