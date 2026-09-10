#!/usr/bin/env python3
"""
depth_estimator_node.py
ROS2 Humble wrapper que recebe imagens de câmera e publica depth estimado.

Modelo padrão: Depth Anything V2 (Small) via transformers/torch.
Pode ser substituído por qualquer outro backbone alterando DepthBackend.

Tópicos:
  Subscribe : /camera/image_raw        (sensor_msgs/Image)
              /camera/camera_info      (sensor_msgs/CameraInfo)  [opcional]
  Publish   : /depth/image_raw         (sensor_msgs/Image, encoding=32FC1, metros)
              /depth/image_visual      (sensor_msgs/Image, encoding=rgb8, colormap)
              /depth/camera_info       (sensor_msgs/CameraInfo)  [repassa camera_info]

Parâmetros (ros2 param):
  model_name      (str)   "depth-anything/Depth-Anything-V2-Small-hf"
  device          (str)   "cuda" | "cpu"
  input_topic     (str)   "/camera/image_raw"
  output_topic    (str)   "/depth/image_raw"
  publish_visual  (bool)  True
  queue_size      (int)   5
  min_depth       (float) 0.1   # metros (para normalização visual)
  max_depth       (float) 10.0  # metros (para normalização visual)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import torch
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber


# ---------------------------------------------------------------------------
# Backend de estimativa de profundidade
# ---------------------------------------------------------------------------

class DepthBackend:
    """
    Wrapper fino sobre Depth Anything V2 (HuggingFace transformers).
    Troca o model_name para usar outro checkpoint sem mexer no nó ROS.
    """

    def __init__(self, model_name: str, device: str):
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        import torch

        self.device = torch.device(device if torch.cuda.is_available() or device == "cpu" else "cpu")
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_name)
        self.model.to(self.device)
        self.model.eval()

    @torch.no_grad()
    def predict(self, bgr_image: np.ndarray) -> np.ndarray:
        from PIL import Image as PILImage

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)

        inputs = self.processor(images=pil_img, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        outputs = self.model(**inputs)
        depth = outputs.predicted_depth.squeeze().cpu().numpy()

        # Converte metros → milímetros
        depth = depth * 1000.0

        h, w = bgr_image.shape[:2]
        if depth.shape != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

        return depth.astype(np.float32)


# ---------------------------------------------------------------------------
# Nó ROS2
# ---------------------------------------------------------------------------

class DepthEstimatorNode(Node):

    def __init__(self):
        super().__init__("depth_estimator_node")

        # --- Declaração de parâmetros ---
        self.declare_parameter("model_name", "depth-anything/Depth-Anything-V2-Small-hf")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_topic", "/depth/image_raw")
        self.declare_parameter("visual_topic", "/depth/image_visual")
        self.declare_parameter("publish_visual", True)
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("min_depth", 100.0)
        self.declare_parameter("max_depth", 10000.0)

        model_name     = self.get_parameter("model_name").value
        device         = self.get_parameter("device").value
        input_topic    = self.get_parameter("input_topic").value
        info_topic     = self.get_parameter("camera_info_topic").value
        output_topic   = self.get_parameter("output_topic").value
        visual_topic   = self.get_parameter("visual_topic").value
        self.pub_vis   = self.get_parameter("publish_visual").value
        self.q_size    = self.get_parameter("queue_size").value
        self.min_d     = self.get_parameter("min_depth").value
        self.max_d     = self.get_parameter("max_depth").value

        # --- Carrega modelo ---
        self.get_logger().info(f"Carregando modelo '{model_name}' no device '{device}'...")
        self.backend = DepthBackend(model_name, device)
        self.get_logger().info("Modelo carregado com sucesso.")

        self.bridge = CvBridge()
        self._last_camera_info: CameraInfo | None = None

        # QoS sensor-like (Best Effort, para câmeras reais)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.q_size,
        )

        # --- Publishers ---
        self.pub_depth = self.create_publisher(Image, output_topic, self.q_size)
        self.pub_info  = self.create_publisher(CameraInfo, "/depth/camera_info", self.q_size)
        if self.pub_vis:
            self.pub_visual = self.create_publisher(Image, visual_topic, self.q_size)

        # --- Subscribers ---
        self.sub_image = self.create_subscription(
            Image, input_topic, self._image_callback, sensor_qos
        )
        self.sub_info = self.create_subscription(
            CameraInfo, info_topic, self._info_callback, sensor_qos
        )

        self.get_logger().info(
            f"DepthEstimatorNode pronto.\n"
            f"  Input : {input_topic}\n"
            f"  Output: {output_topic}\n"
            f"  Visual: {visual_topic if self.pub_vis else 'desabilitado'}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _info_callback(self, msg: CameraInfo):
        self._last_camera_info = msg

    def _image_callback(self, msg: Image):
        try:
            # Converte para BGR (OpenCV padrão)
            encoding = msg.encoding.lower()
            if encoding in ("rgb8", "bgr8", "mono8", "8uc1", "8uc3"):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                # Tenta conversão genérica
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # --- Inferência ---
        try:
            depth_map = self.backend.predict(cv_img)   # float32, HxW
        except Exception as e:
            self.get_logger().error(f"Erro na inferência: {e}")
            return

        stamp  = msg.header.stamp
        frame  = msg.header.frame_id

        # --- Publica depth (32FC1, metros / unidades relativas) ---
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        depth_msg.header.stamp    = stamp
        depth_msg.header.frame_id = frame
        self.pub_depth.publish(depth_msg)

        # --- Repassa CameraInfo ---
        if self._last_camera_info is not None:
            info_out = CameraInfo()
            info_out.header.stamp    = stamp
            info_out.header.frame_id = frame
            # Copia campos relevantes
            info_out.width    = self._last_camera_info.width
            info_out.height   = self._last_camera_info.height
            info_out.k        = self._last_camera_info.k
            info_out.d        = self._last_camera_info.d
            info_out.r        = self._last_camera_info.r
            info_out.p        = self._last_camera_info.p
            info_out.distortion_model = self._last_camera_info.distortion_model
            self.pub_info.publish(info_out)

        # --- Publica visualização colorida (opcional) ---
        if self.pub_vis:
            visual = self._depth_to_colormap(depth_map)
            vis_msg = self.bridge.cv2_to_imgmsg(visual, encoding="rgb8")
            vis_msg.header.stamp    = stamp
            vis_msg.header.frame_id = frame
            self.pub_visual.publish(vis_msg)

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    def _depth_to_colormap(self, depth: np.ndarray) -> np.ndarray:
        """Normaliza depth para [0,255] e aplica colormap INFERNO (RGB)."""
        d_min = float(np.percentile(depth, 2))
        d_max = float(np.percentile(depth, 98))
        if d_max - d_min < 1e-6:
            d_max = d_min + 1.0

        norm = np.clip((depth - d_min) / (d_max - d_min), 0.0, 1.0)
        uint8 = (norm * 255).astype(np.uint8)
        colored_bgr = cv2.applyColorMap(uint8, cv2.COLORMAP_INFERNO)
        return cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB)


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
