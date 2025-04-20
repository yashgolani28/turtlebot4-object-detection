import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
import cv2

# Allowed object classes with known widths (underscores removed and names normalized)
KNOWN_WIDTHS_CUSTOM = {
    "air compressor": 50, "bag": 40, "bambulab 3d printer": 45, "black marker": 2,
    "blue robotarm": 30, "book": 15, "bottle": 7, "bnr toolkit": 35,
    "chair": 50, "dobot magician": 40, "dustbin": 45,
    "elegoo mercury xs cure station": 50, "elegoo mercury xs wash station": 50,
    "elgo infinity 3d printer green": 50, "endermax 3d printer": 50,
    "ender 3d printer": 50, "fire extinguisher": 25, "flashforge 3d printer": 55,
    "hp monitor": 50, "janatics modular manufacturing system": 60, "laptop": 30,
    "marker": 2, "mobile phone": 7, "notice board": 150, "podium stand": 100,
    "projector remote": 10, "smart monitor": 55, "student table": 120,
    "turtlebot mini": 40, "watch": 5, "water purifier": 60,
    "x-plus qidi 3d printer": 55
}

KNOWN_WIDTHS_BASE = {
    "person": 40, "backpack": 30, "handbag": 25, "wallet": 10,
    "book": 15, "notebook": 18, "pen": 1.5, "pencil": 1, "eraser": 4,
    "ruler": 30, "scissors": 12, "calculator": 8, "cell phone": 7,
    "tablet": 20, "laptop": 30, "computer monitor": 50, "mouse": 6,
    "keyboard": 45, "projector": 35, "headphones": 15, "speaker": 10,
    "microphone": 8, "desk": 120, "chair": 45, "whiteboard": 150,
    "blackboard": 180, "table": 120, "cupboard": 90, "bookshelf": 100,
    "bottle": 7, "mug": 8, "plate": 25, "spoon": 4, "fork": 4, "knife": 3,
    "lamp": 20, "fan": 50, "clock": 35, "pillow": 40, "blanket": 150, "bed": 160,
    "whiteboard marker": 2, "chalk": 1, "school bag": 35, "globe": 30, "trophy": 25,
    "fire extinguisher": 20, "trash bin": 40, "window": 150, "door": 90,
    "board eraser": 8, "remote control": 15, "shoe": 10
}

# Combine all known classes and normalize them (lowercase, no underscore)
ALLOWED_CLASSES = set(KNOWN_WIDTHS_CUSTOM.keys()) | set(KNOWN_WIDTHS_BASE.keys())

class YoloNode(Node):
    def _init_(self):
        super()._init_('yolo_node')
        self.model = YOLO("yolov8n.pt")  # You can change to best.pt if needed
        self.subscription = self.create_subscription(
            Image,
            "/oakd/rgb/preview/image_raw",
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the raw image bytes to a NumPy array (assuming RGB8)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV display
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        results = self.model.predict(frame, conf=0.5, verbose=False)

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id].replace("_", " ").lower()

                # Filter out detections not in our known list
                if label in ALLOWED_CLASSES and conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    text = f"{label} {conf:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self.get_logger().info(f"Detected: {text} at [{x1},{y1},{x2},{y2}]")

        cv2.imshow("YOLOv8 Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if _name_ == "_main_":
    main()