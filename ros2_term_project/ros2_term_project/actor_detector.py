import cv2
import numpy as np

class ActorDetector:
    def __init__(self):
        self._delta = None

    def process(self, img: np.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_beige = np.array([0, 48, 80])
        upper_beige = np.array([20, 255, 255])

        self._delta = None

        # actor 식별
        mask = cv2.inRange(hsv, lower_beige, upper_beige)

        h, w, d = img.shape
        search_top = int(40*h / 120)
        # search_top = int(20 * h / 120)


        # 마스킹
        mask[0:search_top, 0:w] = 0
        # mask[0:h, 0:int(w / 8)] = 0
        # mask[0:h, int(7 * w / 8):w] = 0
        mask[0:h, 0:int(w / 7)] = 0
        mask[0:h, int(9 * w / 10):w] = 0


        # 정지선 검출
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 0), -1)
            # BEGIN CONTROL
            err = abs(cx - w / 2)
            self._delta = err
            # END CONTROL

        # 카메라에서 오는 이미지 정보 띄워줌
        # cv2.imshow("actor_window", img)
        # cv2.imshow("actor_mask", mask)
        # cv2.waitKey(3)

        # Decorator
        @property
        def _delta(self):
            return self._delta
# 테스트 코드
def main():
    tracker = ActorDetector()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/actor.jpg')
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
