import cv2
import numpy

class StopLineTracker:
    def __init__(self):
        self._delta = None

    def process(self, img: numpy.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 255, 255])
        self._delta = None

        # 흰색 식별
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape
        search_top = int(13*h / 40)
        search_bot = int(25*h/40)

        # 마스킹
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:int(2 * w / 5)] = 0
        mask[0:h, int(3 * w / 5):w] = 0

        # 정지선 검출
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = abs(cx - w / 2)
            self._delta = err
            # END CONTROL

        # 카메라에서 오는 이미지 정보 띄워줌
        # cv2.imshow("stop_window", img)
        # cv2.imshow("stop_mask", mask)
        # cv2.waitKey(3)

        # Decorator
        @property
        def _delta(self):
            return self._delta
# 테스트 코드
def main():
    tracker = StopLineTracker()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/sample.jpg')
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
