import cv2
from vision.vision import RealSenseColorDetector

def main():
    # ROI는 네가 쓰던 값 그대로
    vision = RealSenseColorDetector(
        roi_area=(80, 240, 270, 320)
    )

    print("[INFO] Press 'q' to quit")

    try:
        while True:
            view, detected_color, (u, v, depth) = vision.detect_one_frame()

            if view is None:
                continue

            # 좌표가 정상적으로 검출되었을 때만 출력
            if u is not None and v is not None and depth is not None:
                text1 = f"u: {u}, v: {v}"
                text2 = f"depth: {depth:.1f} mm"

                cv2.putText(
                    view,
                    text1,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    view,
                    text2,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            if detected_color:
                cv2.putText(
                    view,
                    f"Color: {detected_color}",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2
                )

            cv2.imshow("D435i Debug View", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        vision.stop()
        print("[INFO] Camera stopped")

if __name__ == "__main__":
    main()