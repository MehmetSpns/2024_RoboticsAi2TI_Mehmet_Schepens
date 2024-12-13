import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp

class GestureControl(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        self.publisher_ = self.create_publisher(String, 'hand_gesture', 10)
        self.timer = self.create_timer(0.1, self.detect_hand_gesture)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Camera not accessible.")
            rclpy.shutdown()

    def detect_hand_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame.")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        gesture_msg = String()
        gesture_msg.data = "stop" 

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                landmarks = hand_landmarks.landmark
                thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
                index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
                index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
                pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]

                # Detect "thumb up"
                if thumb_tip.y < thumb_ip.y and thumb_tip.y < index_mcp.y:
                    gesture_msg.data = "thumb_up"
                else:
                    # Detect "start" (hand open)
                    is_hand_open = (
                        index_tip.y < index_mcp.y and
                        middle_tip.y < middle_mcp.y and
                        ring_tip.y < middle_mcp.y and
                        pinky_tip.y < middle_mcp.y
                    )
                    if is_hand_open:
                        gesture_msg.data = "start"
                    else:
                        gesture_msg.data = "stop"
        else:
            gesture_msg.data = "stop"

        self.publisher_.publish(gesture_msg)

        frame_resized = cv2.resize(frame, (320, 240))
        cv2.imshow("Hand Gesture Control", frame_resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.shutdown()

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    gesture_control_node = GestureControl()
    rclpy.spin(gesture_control_node)
    gesture_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
