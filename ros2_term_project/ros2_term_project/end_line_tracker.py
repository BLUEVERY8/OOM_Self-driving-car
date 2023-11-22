import cv2
import numpy

class EndLineTracker:
    def __init__(self):
        self._delta = None

    def process(self, img: numpy.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 250])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape
        search_top = int(11 * h / 100)
        search_bot = int(h / 3)


        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:int(w / 4)] = 0
        mask[0:h, int(2 * w / 3):w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (255, 0, 0), -1)
            # BEGIN CONTROL
            err = abs(cx - w/2)
            self._delta = err
            # END CONTROL

        cv2.imshow("front_window2", img)
        cv2.imshow("front_mask2", mask)
        cv2.waitKey(3)

        @property
        def _delta(self):
            return self._delta

def main():
    tracker = EndLineTracker()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/sample.jpg')
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
