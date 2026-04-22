class Pillar:
    """
    Represents a detected pillar in the frame.
    color: 0 = red, 1 = green
    """
    def __init__(self, x=0, y=0, w=0, h=0, cx=0, cy=0, area=0, color=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.cx = cx
        self.cy = cy
        self.area = area
        self.color = color

    def is_valid(self):
        return self.area > 0