import cv2
import numpy

class LineTracker:
    def __init__(self):
        self._delta = 0.0

    def process(self, img: numpy.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape
        search_top = int(h / 3)

        mask[0:search_top, 0:w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cy - 3*h/4
            self._delta = err
            # END CONTROL
        cv2.imshow("window", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)

        @property
        def _delta(self):
            return self._delta

def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/sample.jpg')
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
