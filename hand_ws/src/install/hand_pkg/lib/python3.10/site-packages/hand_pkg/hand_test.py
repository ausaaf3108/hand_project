import mediapipe as mp
import cv2
from math import dist

class HandDetect:
    def __init__(self):
        self.hands = mp.solutions.hands.Hands()
        self.cap = cv2.VideoCapture("/dev/video0")
        self.x_max = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.y_max = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if not self.cap.isOpened():
            raise Exception("Error: Could not open webcam.")
    
    def process_pose_landmarks(self, hand_landmarks):
        hand = []
        if hand_landmarks:
            for hand_landmark in hand_landmarks:
                for landmark in hand_landmark.landmark:
                    x_pixel = int(landmark.x * self.x_max)
                    y_pixel = int(landmark.y * self.y_max)
                    hand.append((x_pixel, y_pixel))
        return hand

    def draw_pose_landmarks(self, frame, landmarks, color=(0, 255, 0), radius=5, thickness=2):
        for landmark in landmarks:
            cv2.circle(frame, landmark, radius, color, thickness)

    def draw_text(self, frame, hand, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=1, font_color=(255, 255, 255), line_thickness=2):
        t = 1
        for position in hand:
            p = (abs(self.x_max - position[0]), position[1])
            t1 = t % 21
            text = str(t1)
            cv2.putText(frame, text, p, font, font_scale, font_color, line_thickness)
            t += 1

    def draw_pose_lines(self, frame, landmarks, connections, color=(0, 255, 0), thickness=2):
        for connection in connections:
            cv2.line(frame, landmarks[connection[0]], landmarks[connection[1]], color, thickness)

    def gesture(self, frame, finger_tips, palm_avg):
        distance = []
        gest = []
        for finger_tip in finger_tips:
            Dist = int(dist(finger_tip, palm_avg))
            distance.append(Dist)
            if Dist > 55:
                gest.append(1)
            else:
                gest.append(0)
        return gest

    def vid_process(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture frame.")
                continue

            try:
                RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(RGB)
                hand_landmarks = results.multi_hand_landmarks
                hand = self.process_pose_landmarks(hand_landmarks)

                if len(hand) < 21:
                    continue

                thumb = [hand[2], hand[3], hand[4]]
                index = [hand[6], hand[7], hand[8]]
                middle = [hand[10], hand[11], hand[12]]
                ring = [hand[14], hand[15], hand[16]]
                pinky = [hand[18], hand[19], hand[20]]
                palm = [hand[0], hand[1], hand[5], hand[9], hand[13], hand[17]]
                finger_tips = [hand[4], hand[8], hand[12], hand[16], hand[20]]

                self.draw_pose_landmarks(frame, thumb, color=(255, 0, 0))
                self.draw_pose_lines(frame, thumb, [(0, 1), (1, 2)], color=(255, 0, 0))
                self.draw_pose_landmarks(frame, index, color=(0, 0, 255))
                self.draw_pose_lines(frame, index, [(0, 1), (1, 2)], color=(0, 0, 255))
                self.draw_pose_landmarks(frame, middle, color=(0, 255, 0))
                self.draw_pose_lines(frame, middle, [(0, 1), (1, 2)], color=(0, 255, 0))
                self.draw_pose_landmarks(frame, ring, color=(0, 255, 255))
                self.draw_pose_lines(frame, ring, [(0, 1), (1, 2)], color=(0, 255, 255))
                self.draw_pose_landmarks(frame, pinky, color=(255, 0, 255))
                self.draw_pose_lines(frame, pinky, [(0, 1), (1, 2)], color=(255, 0, 255))
                self.draw_pose_landmarks(frame, palm, color=(255, 255, 0))
                self.draw_pose_lines(frame, palm, [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)], color=(255, 255, 0))
                self.draw_pose_landmarks(frame, finger_tips, color=(255, 255, 0))

                x = sum(pt[0] for pt in palm) // len(palm)
                y = sum(pt[1] for pt in palm) // len(palm)
                palm_avg = (x, y)
                cv2.circle(frame, palm_avg, 55, (255, 255, 0), 2)

                self.state = self.gesture(frame, finger_tips, palm_avg)
                num = sum(self.state)
                text = str(num)
                frame = cv2.flip(frame, 1)
                cv2.putText(frame, text, (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 7)

            except Exception as e:
                print(f"Error processing frame: {e}")
                frame = cv2.flip(frame, 1)

            return frame

    def display(self):
        frame_generator = self.vid_process()
        while True:
            frame = next(frame_generator)
            cv2.imshow('Webcam', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    hd = HandDetect()
    hd.display()
