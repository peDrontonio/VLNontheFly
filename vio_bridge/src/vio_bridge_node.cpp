#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class VioBridgeNode : public rclcpp::Node
{
public:
    VioBridgeNode() : Node("vio_bridge_node")
    {
        this->declare_parameter("camera_pitch_deg",    -7.0);
        this->declare_parameter("slam_scale",           1.0);
        this->declare_parameter("body_offset_x",       0.0);
        this->declare_parameter("body_offset_y",       0.0);
        this->declare_parameter("body_offset_z",       0.0);
        // Frames to wait after VIBA 2 before locking alignment.
        // Gives the SLAM gravity-alignment a moment to settle (~30 frames ≈ 1 s at 30 Hz).
        this->declare_parameter("inertial_settle_frames", 30);

        const double pitch_deg    = this->get_parameter("camera_pitch_deg").as_double();
        slam_scale_               = this->get_parameter("slam_scale").as_double();
        settle_frames_            = this->get_parameter("inertial_settle_frames").as_int();
        t_offset_ = {
            this->get_parameter("body_offset_x").as_double(),
            this->get_parameter("body_offset_y").as_double(),
            this->get_parameter("body_offset_z").as_double(),
        };

        // FRD body frame: X=Forward, Y=Right, Z=Down
        // Camera frame:   X=Right,   Y=Down,   Z=Forward
        // R_slam2frd maps a vector from camera frame to FRD body frame.
        tf2::Matrix3x3 R_slam2frd(
             0,  0, 1,
             1,  0, 0,
             0,  1, 0);

        // q_cam2body = T_{body←cam}: rotation that takes camera-frame vectors to body-frame.
        // Pitch correction accounts for camera tilt relative to body X axis.
        tf2::Matrix3x3 R_pitch;
        R_pitch.setRPY(0.0, pitch_deg * M_PI / 180.0, 0.0);
        (R_pitch * R_slam2frd).getRotation(q_cam2body_);
        q_cam2body_.normalize();

        RCLCPP_INFO(get_logger(),
            "VIO Bridge started | camera_pitch=%.1f deg | slam_scale=%.3f | waiting for first SLAM pose...",
            pitch_deg, slam_scale_);

        att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            rclcpp::SensorDataQoS(),
            std::bind(&VioBridgeNode::onAttitude, this, _1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose", 10,
            std::bind(&VioBridgeNode::onPose, this, _1));

        // Match the transient_local QoS used by the SLAM node publisher.
        rclcpp::QoS ready_qos(1);
        ready_qos.transient_local().reliable();
        ready_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/orb_slam3/inertial_ready", ready_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool was_ready = inertial_ready_;
                inertial_ready_ = msg->data;
                if (!was_ready && msg->data) {
                    // VIBA 2 just completed — reset alignment so we re-snapshot
                    // after the settle dwell.
                    initialized_        = false;
                    frames_since_ready_ = 0;
                    RCLCPP_INFO(get_logger(),
                        "VIBA2 complete — settling %d frames before locking alignment.",
                        settle_frames_);
                } else if (was_ready && !msg->data) {
                    // New map started, VIBA 2 reset.
                    initialized_        = false;
                    frames_since_ready_ = 0;
                    RCLCPP_WARN(get_logger(), "Inertial ready lost — waiting for new VIBA2.");
                }
            });

        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // PX4 quaternion layout: [w, x, y, z]
        std::lock_guard<std::mutex> lk(att_mutex_);
        q_imu_.setValue(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        imu_received_ = true;
    }

    void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!imu_received_) {
            return;
        }

        // ORB-SLAM3 publishes Tcw (world→camera). The message translation is t_cw
        // (world origin in camera frame), NOT the camera position in world.
        // Camera-in-world: p_wc = -R_wc · t_cw  with  R_wc = q_cw⁻¹.
        tf2::Quaternion q_cw;
        tf2::fromMsg(msg->pose.orientation, q_cw);
        const tf2::Vector3 t_cw(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const tf2::Vector3 p_wc = -tf2::quatRotate(q_cw.inverse(), t_cw);

        // Broadcast map → base_link in SLAM world coordinates so the full chain
        // (map → camera_link → … and map → base_link) is visible in rqt_tf_tree.
        {
            const tf2::Quaternion q_wc = q_cw.inverse();
            // R_{world←body} = R_{world←cam} · R_{cam←body}
            const tf2::Quaternion q_w_body = (q_wc * q_cam2body_.inverse()).normalized();
            const tf2::Vector3 lever_body(t_offset_[0], t_offset_[1], t_offset_[2]);
            const tf2::Vector3 p_w_body = p_wc - tf2::quatRotate(q_w_body, lever_body);

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id  = "base_link";
            tf_msg.transform.translation.x = p_w_body.x();
            tf_msg.transform.translation.y = p_w_body.y();
            tf_msg.transform.translation.z = p_w_body.z();
            tf_msg.transform.rotation.x = q_w_body.x();
            tf_msg.transform.rotation.y = q_w_body.y();
            tf_msg.transform.rotation.z = q_w_body.z();
            tf_msg.transform.rotation.w = q_w_body.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // On first pose after VIBA 2 completes: snapshot IMU attitude, SLAM pose,
        // camera-in-world, and the constant SLAM-world → NED rotation.
        //   T_{NED ← world_slam} = T_{NED ← body_init} · T_{body ← cam} · T_{cam_init ← world_slam}
        //                        = q_imu_init · q_cam2body · q_slam_init
        if (!initialized_) {
            if (!inertial_ready_) return;
            if (++frames_since_ready_ < settle_frames_) return;  // let gravity alignment settle
            {
                std::lock_guard<std::mutex> lk(att_mutex_);
                q_imu_init_ = q_imu_;
            }
            q_slam_init_ = q_cw;
            p_wc_init_   = p_wc;
            q_ned_world_ = (q_imu_init_ * q_cam2body_ * q_slam_init_).normalized();
            initialized_ = true;
            RCLCPP_INFO(get_logger(), "Alignment locked. IMU att: q=[%.3f %.3f %.3f %.3f]",
                q_imu_init_.w(), q_imu_init_.x(), q_imu_init_.y(), q_imu_init_.z());
        }

        const uint64_t ts = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000ULL
                          + msg->header.stamp.nanosec / 1000ULL;

        // Orientation: T_{NED←body_t} = q_imu_init · q_cam2body · (q_slam_init · q_slam_t⁻¹) · q_cam2body⁻¹
        const tf2::Quaternion q_slam_delta = q_slam_init_ * q_cw.inverse();
        const tf2::Quaternion q_body_delta = q_cam2body_ * q_slam_delta * q_cam2body_.inverse();
        const tf2::Quaternion q = (q_imu_init_ * q_body_delta).normalized();

        // Position of the camera in NED (relative to body_init origin).
        // slam_scale_ converts SLAM-internal units to real-world metric.
        //   p_cam_ned = R_{NED←world_slam} · scale · (p_wc(t) − p_wc(init))
        const tf2::Vector3 p_cam_ned = tf2::quatRotate(q_ned_world_, slam_scale_ * (p_wc - p_wc_init_));

        // Body (FC) position = camera position − R_{NED←body} · lever_arm_body.
        // body_offset_* describes where the camera sits in body frame (camera ahead of FC),
        // so the FC is behind the camera by that vector once rotated into NED.
        const tf2::Vector3 lever_body(t_offset_[0], t_offset_[1], t_offset_[2]);
        const tf2::Vector3 p_ned = p_cam_ned - tf2::quatRotate(q, lever_body);

        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp        = ts;
        out.timestamp_sample = ts;
        out.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        out.position[0] = static_cast<float>(p_ned.x());
        out.position[1] = static_cast<float>(p_ned.y());
        out.position[2] = static_cast<float>(p_ned.z());

        // PX4 quaternion order: [w, x, y, z]
        out.q[0] = static_cast<float>(q.w());
        out.q[1] = static_cast<float>(q.x());
        out.q[2] = static_cast<float>(q.y());
        out.q[3] = static_cast<float>(q.z());

        out.velocity_frame          = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        out.velocity[0]             = out.velocity[1]             = out.velocity[2]             = NAN;
        out.angular_velocity[0]     = out.angular_velocity[1]     = out.angular_velocity[2]     = NAN;
        out.position_variance[0]    = out.position_variance[1]    = out.position_variance[2]    = NAN;
        out.orientation_variance[0] = out.orientation_variance[1] = out.orientation_variance[2] = NAN;
        out.velocity_variance[0]    = out.velocity_variance[1]    = out.velocity_variance[2]    = NAN;

        odom_pub_->publish(out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr   att_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              ready_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr      odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                    tf_broadcaster_;

    tf2::Quaternion      q_cam2body_{0, 0, 0, 1};
    tf2::Quaternion      q_imu_{0, 0, 0, 1};
    tf2::Quaternion      q_imu_init_{0, 0, 0, 1};
    tf2::Quaternion      q_slam_init_{0, 0, 0, 1};
    tf2::Quaternion      q_ned_world_{0, 0, 0, 1};
    tf2::Vector3         p_wc_init_{0, 0, 0};
    std::mutex           att_mutex_;
    std::array<double,3> t_offset_{};

    double       slam_scale_          = 1.0;
    bool         initialized_        = false;
    bool         imu_received_       = false;
    bool         inertial_ready_     = false;
    int          frames_since_ready_ = 0;
    int          settle_frames_      = 30;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioBridgeNode>());
    rclcpp::shutdown();
}