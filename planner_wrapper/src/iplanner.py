#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from iplanner_agent import IPlannerAgent
from visualization_utils import VisualizationManager
from basic_utils import draw_box_with_text

from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from planner.msg import Trajectory2D

import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge

import numpy as np
import threading
import imageio
import cv2

class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('intrinsic', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('config', '')
        self.declare_parameter('speed', 0.0)
        self.declare_parameter('goal_range', 0.0)
        self.declare_parameter('planner_max_frequency', 60.0)
        self.declare_parameter('depth_height', 0)
        self.declare_parameter('depth_width', 0)
        self.declare_parameter('trajectories_folder', '')
        self.declare_parameter('goal_target_frame', '')

        self.depth_height = self.get_parameter('depth_height').get_parameter_value().integer_value
        self.depth_width = self.get_parameter('depth_width').get_parameter_value().integer_value

        self.intrinsic = np.array(self.get_parameter('intrinsic').get_parameter_value().double_array_value)
        self.intrinsic = np.array(self.intrinsic).reshape(3, 3)
        checkpoint = self.get_parameter('checkpoint').get_parameter_value().string_value
        config = self.get_parameter('config').get_parameter_value().string_value
        self.intrinsic[0, 2], self.intrinsic[1, 2] = self.depth_width // 2, self.depth_height // 2

        self.planner = IPlannerAgent(self.intrinsic,
                                    model_path=checkpoint,
                                    model_config_path=config,
                                    device='cuda:0')

        self.mutex_odom = threading.Lock() # Odometry value mutex
        self.mutex_depth = threading.Lock() # Depth mutex
        self.mutex_goal = threading.Lock() # Current goal mutex
        self.mutex_image = threading.Lock() # Current image mutex
        self.freq_planner = self.get_parameter('planner_max_frequency').get_parameter_value().double_value # Maximum planner frequency in hz
        self.planner_group = MutuallyExclusiveCallbackGroup()
        self.odometry_group = MutuallyExclusiveCallbackGroup()
        self.depth_group = MutuallyExclusiveCallbackGroup()
        self.goal_group = MutuallyExclusiveCallbackGroup()
        self.image_group = MutuallyExclusiveCallbackGroup()

        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.current_goal = None
        self.current_odom = None
        self.current_depth = None
        self.current_image = None
        self.vis_depth = None
        self.vis_image = None
        self.vis_goal = None
        self.goal_range = self.get_parameter('goal_range').get_parameter_value().double_value
        self.goal_target_frame = self.get_parameter('goal_target_frame').get_parameter_value().string_value

        save_dir = self.get_parameter('trajectories_folder').get_parameter_value().string_value
        self.fps_writer = imageio.get_writer(save_dir + "fps.mp4", fps=10)
        self.vis_manager = VisualizationManager(history_size=5)

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.trajectory_pub = self.create_publisher(
            Trajectory2D,
            '/trajectory',
            10
        )

        self.vis_image_pub = self.create_publisher(
            Image,
            '/planner/visualization',
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.update_image,
            self.qos,
            callback_group=self.image_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.update_odometry,
            self.qos,
            callback_group=self.odometry_group
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth',
            self.update_depth,
            self.qos,
            callback_group = self.depth_group
        )

        self.goal_sub = self.create_subscription(
            PointStamped,
            '/waypoint',
            self.update_goal,
            self.qos,
            callback_group=self.goal_group
        )

        self.planner_loop = self.create_timer(
            1.0 / self.freq_planner,
            self.planning,
            callback_group=self.planner_group  
        )

        self.get_logger().info("Planner node started.")

    def planning(self):
        try:
            with self.mutex_odom:
                odom = self.current_odom

            if odom is None:
                self.get_logger().warn('(Planner): No odometry detected for planning.')
                return

            with self.mutex_depth:
                depth = self.current_depth
             
            if depth is None:
                self.get_logger().warn('(Planner): No depth detected for planning.')
                return
            depth = depth.copy()

            with self.mutex_goal:
                goal_msg = self.current_goal

            if goal_msg is None:
                self.get_logger().warn('(Planner): No goal available.')
                return
            
            with self.mutex_image:
                image = self.current_image
            
            if image is None:
                self.get_logger().warn('(Planner): No image detected for trajectory visualization.')
                return
            image = image.copy()

            pos = odom.pose.pose.position
            quat = odom.pose.pose.orientation
            camera_rot = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            camera_pos = np.array([pos.x, pos.y, pos.z])
            robot_vel = np.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)
            robot_ang_vel = odom.twist.twist.angular.z

            target_frame = self.goal_target_frame or odom.header.frame_id
            goal_world = self.transform_goal(goal_msg, target_frame)
            if goal_world is None:
                return

            goal_relative = camera_rot.T @ (goal_world - camera_pos)

            goal_x = np.array([goal_relative[0]], dtype=np.float32)
            goal_y = np.array([goal_relative[1]], dtype=np.float32)
            goal = np.stack((goal_x, goal_y, np.zeros_like(goal_x)), axis=1)
            goal_for_vis = goal[0]
            goal = np.clip(goal, -self.goal_range, self.goal_range)
            batch_size = goal.shape[0]

            depth_for_vis = depth.copy()
            depth = depth.reshape((batch_size, depth.shape[0], depth.shape[1], 1))

            # 🔹 planner_output
            _, trajectory, fear = self.planner.step_pointgoal(depth, goal)

            trajectory_points_camera = trajectory.cpu().numpy()
            all_trajectories_camera = trajectory.cpu().numpy()[None, :, :, :]
            all_values_camera = fear.cpu().numpy()
            
            # Transform trajectory from camera frame to world frame
            trajectory_points_camera_single = trajectory_points_camera[0]            
            trajectory_points_world = []
            for point in trajectory_points_camera_single:
                point_local = np.array([point[0], point[1], 0.0])
                point_world = camera_pos + camera_rot @ point_local
                trajectory_points_world.append(point_world[:2])
            trajectory_points_world = np.array(trajectory_points_world)
            
            all_trajectories_camera_single = all_trajectories_camera[0]

            all_trajectories_world = []
            for traj_camera in all_trajectories_camera_single:
                traj_world = []
                for point in traj_camera:
                    point_local = np.array([point[0], point[1], 0.0])
                    point_world = camera_pos + camera_rot @ point_local
                    traj_world.append(point_world[:2])
                all_trajectories_world.append(np.array(traj_world))
            
            msg = Trajectory2D()
            msg.data = trajectory_points_world.astype(np.float32).flatten().tolist()
            msg.rows, msg.cols = trajectory_points_world.shape

            self.trajectory_pub.publish(msg)

            current_trajectory = trajectory_points_world
            current_all_trajectories = np.array(all_trajectories_world)
            current_all_values = all_values_camera

            x0 = np.array([camera_pos[0], camera_pos[1], np.arctan2(camera_rot[1,0], camera_rot[0,0]), robot_vel, robot_ang_vel])
            depth_for_vis = np.nan_to_num(depth_for_vis, nan=0.0, posinf=0.0, neginf=0.0)
            depth_for_vis[depth_for_vis <= 0] = 0.0

            try:
                vis_image = self.vis_manager.visualize_trajectory(
                    image, depth_for_vis[:,:,None], self.intrinsic,
                    current_trajectory,
                    robot_pose=x0,
                    all_trajectories_points=current_all_trajectories,
                    all_trajectories_values=current_all_values
                )
                # Visualization
                vis_image = draw_box_with_text(vis_image,0,50,430,50,"actual lin.:%.2f ang.:%.2f"%(robot_vel,robot_ang_vel))
                if current_all_values is not None:
                    vis_image = draw_box_with_text(vis_image,0,770,430,50,"critic max:%.2f min:%.2f"%(np.max(current_all_values), np.min(current_all_values)))
                vis_image = draw_box_with_text(vis_image,0,820,430,50,"point goal:(%.2f, %.2f)"%(goal_for_vis[0],goal_for_vis[1]))
                self.fps_writer.append_data(vis_image)
                ros_image = self.cv_bridge.cv2_to_imgmsg(
                cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR),
                encoding='bgr8'
                )
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'base_link'
                self.vis_image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().warn(f'(Planner): Visualization failed: {e}')

        except Exception as e:
            self.get_logger().error(f"(Planner) Error: {e}")


    def update_odometry(self, odom):
        with self.mutex_odom:
            self.current_odom = odom

    def update_image(self, msg: Image):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        image = self.smart_resize(image, (self.depth_width, self.depth_height))
        with self.mutex_image:
            self.current_image = image

    def update_depth(self, msg: Image):
        #depth = self.pointcloud2_to_depth(pointcloud, self.depth_width, self.depth_height)
        depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32) / 1000.0
        depth = self.resize_depth(depth, (self.depth_width, self.depth_height))
        with self.mutex_depth:
            self.current_depth = depth 

    def update_goal(self, msg: PointStamped):
        with self.mutex_goal:
            self.current_goal = msg

    def transform_goal(self, msg: PointStamped, target_frame: str):
        source_frame = msg.header.frame_id
        try:
            if not target_frame or not source_frame or source_frame == target_frame:
                point = msg.point
            else:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                point = tf2_geometry_msgs.do_transform_point(msg, transform).point

            return np.array([point.x, point.y, point.z], dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(
                f'(Goal): Failed to transform goal from {source_frame!r} to {target_frame!r}: {e}'
            )
            return None

    def smart_resize(self, img, size):
        h, w = img.shape[:2]
        new_w, new_h = size

        if new_w < w or new_h < h:
            interp = cv2.INTER_AREA      # downscale
        else:
            interp = cv2.INTER_LINEAR    # upscale
        
        return cv2.resize(img, size, interpolation=interp)

    def resize_depth(self, img, size):
        h, w = img.shape[:2]
        new_w, new_h = size

        if new_w < w or new_h < h:
            interp = cv2.INTER_NEAREST   # nearest neighbor interpolation for depth
        else:
            interp = cv2.INTER_NEAREST  
        
        return cv2.resize(img, size, interpolation=interp)

    def pointcloud2_to_depth(self, msg: PointCloud2, width: int, height: int) -> np.ndarray:
        intrinsic = self.intrinsic  # intrinsic matrix

        points_raw = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array(list(points_raw), dtype=np.float32)

        if points.shape[0] == 0:
            return np.zeros((height, width), dtype=np.float32)

        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]

        valid = points[:, 2] > 0
        points = points[valid]

        u = (fx * points[:, 0] / points[:, 2] + cx).astype(int)
        v = (fy * points[:, 1] / points[:, 2] + cy).astype(int)

        mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, z = u[mask], v[mask], points[mask, 2]

        depth_image = np.full((height, width), np.inf, dtype=np.float32)
        idx = np.argsort(z)
        depth_image[v[idx], u[idx]] = z[idx]
        depth_image[depth_image == np.inf] = 0.0

        return depth_image

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.fps_writer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
