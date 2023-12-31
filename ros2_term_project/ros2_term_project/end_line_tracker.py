import cv2
import numpy

class EndLineTracker:
    def __init__(self):
        self._delta = None

    def process(self, img: numpy.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 노란색 종료 지점 마
        lower_yellow = numpy.array([20, 200, 200])
        upper_yellow = numpy.array([30, 255, 250])

        # 노란색 식별
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape
        search_bot = int(4*h / 7)

        # 마스킹
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:int(w / 4)] = 0
        mask[0:h, int(2 * w / 3):w] = 0

        # 종료선 검출
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (255, 0, 0), -1)
            # BEGIN CONTROL
            err = abs(cx - w/2)
            self._delta = err
            # END CONTROL

        # 카메라에서 오는 이미지 정보 띄워줌
        # cv2.imshow("front_window2", img)
        # cv2.imshow("front_mask2", mask)
        # cv2.waitKey(3)

        # Decorator
        @property
        def _delta(self):
            return self._delta

# 테스트 코드
def main():
    tracker = EndLineTracker()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/end_line.jpg')
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
