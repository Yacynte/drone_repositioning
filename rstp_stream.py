import cv2

RTSP_URL = "rtsp://10.116.88.38:8554/mystream"

def main():
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Failed to open stream")
        return
    x = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed")
            break

        # Just show frame shape (no GUI)
        print(f"Frame {x}: {frame.shape}")
        x += 1

if __name__ == "__main__":
    main()
