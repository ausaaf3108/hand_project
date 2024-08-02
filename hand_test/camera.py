import cv2

def vid_process(cap):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break
        yield frame

def display():
    cap = cv2.VideoCapture("/dev/video0")
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    frame_generator = vid_process(cap)
    while True:
        frame = next(frame_generator)
        cv2.imshow('Webcam', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

display()
