#!/usr/bin/env python3
import argparse
import sys
import cv2
import numpy as np
import onnxruntime as ort
import yaml
import time
import threading
import traceback

# ROS2 库
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from yolo11_msgs.msg import Coordinate, CoordinateArray  # ROS2 消息

# 你的坐标转换库（保持不变）
from Coordinate_Transformation import pixel_to_world


class YOLO11ROS2(Node):
    def __init__(self, onnx_model, yaml_file, confidence_thres, iou_thres, show_display):
        super().__init__("yolo11_node")
        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres
        self.show_display = show_display

        # 加载类别
        with open(yaml_file, "r", encoding="utf-8") as f:
            self.classes = yaml.safe_load(f)["names"]

        # 随机颜色
        self.color_palette = np.random.randint(0, 255, size=(len(self.classes), 3), dtype=np.uint8)

        # ONNX 模型
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        # 尝试使用 CPU（避免 GPU 探测失败问题）
        try:
            self.session = ort.InferenceSession(
                onnx_model,
                sess_options=session_options,
                providers=["CPUExecutionProvider"]  # 先用 CPU，稳定后再尝试 GPU
            )
            self.get_logger().info("使用 CPU 进行推理")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            raise

        # 输入尺寸
        model_inputs = self.session.get_inputs()
        self.input_name = model_inputs[0].name
        self.input_shape = model_inputs[0].shape
        self.input_width = self.input_shape[2]
        self.input_height = self.input_shape[3]
        self.get_logger().info(f"模型输入尺寸: {self.input_width}x{self.input_height}")

        # ROS2 发布/订阅
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(CoordinateArray, "yolo11_data", 30)
        self.subscription = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",  # 相机话题
            self.image_callback,
            10
        )

        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # FPS
        self.frame_count = 0
        self.start_time = time.time()

        self.get_logger().info("YOLO11 节点初始化完成")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                with self.frame_lock:
                    self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Image error: {str(e)}")

    def draw_detections(self, img, box, score, class_id):
        x1, y1, w, h = box
        color = self.color_palette[class_id].tolist()
        label = f"{self.classes[class_id]}: {score:.2f}"
        cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), color, 2)
        # 可选：添加标签文本
        cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self.input_width, self.input_height))
        # 使用 float32 而不是 float16（更稳定）
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)
        return np.expand_dims(img, axis=0)

    def postprocess(self, input_image, output):
        outputs = np.squeeze(output[0]).T
        img_h, img_w = input_image.shape[:2]
        scale_w = img_w / self.input_width
        scale_h = img_h / self.input_height

        scores = np.max(outputs[:, 4:], axis=1)
        valid_idx = np.where(scores >= self.confidence_thres)[0]
        
        if len(valid_idx) == 0:
            return input_image

        class_ids = np.argmax(outputs[valid_idx, 4:], axis=1)
        boxes = outputs[valid_idx, :4]

        boxes[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2) * scale_w
        boxes[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2) * scale_h
        boxes[:, 2] *= scale_w
        boxes[:, 3] *= scale_h
        boxes = boxes.astype(int)

        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores[valid_idx].tolist(), 
                                    self.confidence_thres, self.iou_thres)

        # 发布消息
        msg = CoordinateArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 处理 NMS 结果
        if len(indices) > 0:
            # 兼容不同 OpenCV 版本返回的 indices 格式
            if isinstance(indices, tuple):
                indices = indices[0]
            elif hasattr(indices, 'flatten'):
                indices = indices.flatten()
            
            for i in indices:
                # 确保索引有效
                if i >= len(boxes):
                    continue
                    
                self.draw_detections(input_image, boxes[i], scores[valid_idx[i]], class_ids[i])
                cx = int(boxes[i][0] + boxes[i][2] / 2)
                cy = int(boxes[i][1] + boxes[i][3])
                
                # 坐标转换（带容错）
                try:
                    wx, wy = pixel_to_world(cx, cy)
                    coord = Coordinate()
                    coord.x = float(wx)
                    coord.y = float(wy)
                    msg.array.append(coord)
                    self.get_logger().debug(f"检测到目标: ({wx:.2f}, {wy:.2f})")
                except Exception as e:
                    self.get_logger().error(f"坐标转换失败: {str(e)}")
                    continue

        # 发布消息（即使为空也发布，便于下游节点知道无检测）
        self.publisher.publish(msg)
        
        # 打印 FPS（每 30 帧）
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            self.get_logger().info(f"FPS: {fps:.2f}")
        
        return input_image

    def run(self):
        if self.show_display:
            cv2.namedWindow("YOLO11 ROS2", cv2.WINDOW_NORMAL)

        while rclpy.ok() and self.running:
            with self.frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is not None:
                try:
                    img_data = self.preprocess(frame)
                    output = self.session.run(None, {self.input_name: img_data})
                    processed = self.postprocess(frame, output)

                    if self.show_display:
                        cv2.imshow("YOLO11 ROS2", processed)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                except Exception as e:
                    self.get_logger().error(f"推理错误: {str(e)}")
                    self.get_logger().error(traceback.format_exc())
            else:
                # 没有新帧时短暂休眠，避免 CPU 空转
                time.sleep(0.01)

        cv2.destroyAllWindows()


def main(args=None):
    # 使用 parse_known_args 忽略 ROS2 launch 传递的额外参数
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--yaml", type=str, required=True)
    parser.add_argument("--conf-thres", type=float, default=0.8)
    parser.add_argument("--iou-thres", type=float, default=0.7)
    parser.add_argument("--show", action="store_true")
    
    # 只解析已知参数，忽略 --ros-args 及其后续参数
    known_args, unknown_args = parser.parse_known_args()
    
    # 打印忽略的参数（调试用）
    if unknown_args:
        print(f"Ignoring unknown arguments: {unknown_args}")

    rclpy.init(args=sys.argv)
    node = YOLO11ROS2(known_args.model, known_args.yaml, 
                      known_args.conf_thres, known_args.iou_thres, 
                      known_args.show)
    
    # 并行运行 ROS2 回调 + 推理主循环
    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()