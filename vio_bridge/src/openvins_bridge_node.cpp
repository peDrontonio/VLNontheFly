// OpenVINS -> PX4 visual-odometry bridge.
//
// OpenVINS publishes nav_msgs/Odometry on (default) /ov_msckf/odomimu:
//   * header.frame_id = global gravity-aligned frame G (Z points UP),
//     with an ARBITRARY initial yaw.
//   * pose = IMU pose in G:  position p_GI,  orientation q_GtoI (Hamilton q_ItoG).
//   * twist.linear = IMU velocity expressed in the local IMU frame.
//   * The IMU frame is the RealSense optical convention: X-right, Y-down, Z-forward.
//
// PX4 /fmu/in/vehicle_visual_odometry expects px4_msgs/VehicleOdometry in NED:
//   X-North, Y-East, Z-Down, body = FRD (X-forward, Y-right, Z-down).
//
// This node anchors a local NED frame on the first message so the output starts
// at position (0,0,0) with ZERO yaw while keeping OpenVINS's gravity-aligned
// roll/pitch, then applies the optical->FRD body rotation and a lever-arm
// correction from the IMU to the flight-controller centre.  Because OpenVINS is
// already gravity-aligned and metric, no PX4 attitude, VIBA, or scale is needed
// (contrast vio_bridge_node.cpp, the ORB-SLAM3 bridge).

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OpenVinsBridgeNode : public rclcpp::Node
{
public:
    OpenVinsBridgeNode() : Node("openvins_bridge_node")
    {
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/ov_msckf/odomimu");
        // IMU/camera position in the FRD body frame (sensor is ahead of the FC).
        body_offset_ = {
            declare_parameter<double>("body_offset_x", 0.11),
            declare_parameter<double>("body_offset_y", 0.0),
            declare_parameter<double>("body_offset_z", 0.0),
        };
        publish_velocity_ = declare_parameter<bool>("publish_velocity", true);
        publish_tf_       = declare_parameter<bool>("publish_tf", true);

        // Optical (X-right,Y-down,Z-forward) -> FRD (X-forward,Y-right,Z-down):
        //   x_FRD = z_opt, y_FRD = x_opt, z_FRD = y_opt
        tf2::Matrix3x3 R_opt2frd(
            0, 0, 1,
            1, 0, 0,
            0, 1, 0);
        R_opt2frd.getRotation(q_opt2frd_);
        q_opt2frd_.normalize();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&OpenVinsBridgeNode::onOdom, this, _1));
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(),
            "OpenVINS->PX4 bridge | odom_topic=%s | lever(FRD)=[%.3f %.3f %.3f] | "
            "waiting for first odometry...",
            odom_topic_.c_str(), body_offset_[0], body_offset_[1], body_offset_[2]);
    }

private:
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const tf2::Vector3 p_GI(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        tf2::Quaternion q_GI;
        tf2::fromMsg(msg->pose.pose.orientation, q_GI);
        q_GI.normalize();

        // ---- anchor a zero-yaw local NED frame on the first message ----------
        if (!initialized_) {
            p0_ = p_GI;
            // FRD forward axis = optical Z; its heading in G defines NED north.
            const tf2::Vector3 fwd_G = tf2::quatRotate(q_GI, tf2::Vector3(0, 0, 1));
            tf2::Vector3 fwd_h(fwd_G.x(), fwd_G.y(), 0.0);
            if (fwd_h.length() < 1e-6) {
                // Looking straight up/down: fall back to the optical X axis heading.
                const tf2::Vector3 alt = tf2::quatRotate(q_GI, tf2::Vector3(1, 0, 0));
                fwd_h = tf2::Vector3(alt.x(), alt.y(), 0.0);
            }
            fwd_h.normalize();
            const tf2::Vector3 xN_G = fwd_h;                 // North  (in G)
            const tf2::Vector3 zN_G(0, 0, -1);               // Down   (gravity)
            const tf2::Vector3 yN_G = zN_G.cross(xN_G);      // East
            // R_N_G rows are the NED axes expressed in G (maps G vectors -> NED).
            R_N_G_.setValue(xN_G.x(), xN_G.y(), xN_G.z(),
                            yN_G.x(), yN_G.y(), yN_G.z(),
                            zN_G.x(), zN_G.y(), zN_G.z());
            R_N_G_.getRotation(q_N_G_);
            q_N_G_.normalize();
            initialized_ = true;
            RCLCPP_INFO(get_logger(),
                "Anchored NED frame: north-heading=%.1f deg in OpenVINS global.",
                std::atan2(fwd_h.y(), fwd_h.x()) * 180.0 / M_PI);
        }

        // ---- pose of the IMU in the local NED frame --------------------------
        const tf2::Vector3 p_imu_N = R_N_G_ * (p_GI - p0_);
        // Orientation of the FRD body in NED.
        const tf2::Quaternion q_ned_frd =
            (q_N_G_ * q_GI * q_opt2frd_.inverse()).normalized();

        // Lever arm: FC centre = IMU - R_{NED<-FRD} * (IMU offset in FRD).
        const tf2::Vector3 lever(body_offset_[0], body_offset_[1], body_offset_[2]);
        const tf2::Vector3 p_fc_N = p_imu_N - tf2::quatRotate(q_ned_frd, lever);

        const uint64_t ts = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000ULL
                          + msg->header.stamp.nanosec / 1000ULL;

        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp        = ts;
        out.timestamp_sample = ts;
        out.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.position[0] = static_cast<float>(p_fc_N.x());
        out.position[1] = static_cast<float>(p_fc_N.y());
        out.position[2] = static_cast<float>(p_fc_N.z());
        out.q[0] = static_cast<float>(q_ned_frd.w());   // PX4 order [w,x,y,z]
        out.q[1] = static_cast<float>(q_ned_frd.x());
        out.q[2] = static_cast<float>(q_ned_frd.y());
        out.q[3] = static_cast<float>(q_ned_frd.z());

        if (publish_velocity_) {
            // twist.linear is IMU velocity in the local IMU(optical) frame.
            const tf2::Vector3 v_local(
                msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
            const tf2::Vector3 v_N = tf2::quatRotate((q_N_G_ * q_GI).normalized(), v_local);
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
            out.velocity[0] = static_cast<float>(v_N.x());
            out.velocity[1] = static_cast<float>(v_N.y());
            out.velocity[2] = static_cast<float>(v_N.z());
        } else {
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
            out.velocity[0] = out.velocity[1] = out.velocity[2] = NAN;
        }
        out.angular_velocity[0] = out.angular_velocity[1] = out.angular_velocity[2] = NAN;

        // Pass through OpenVINS pose covariance diagonal (x,y,z / roll,pitch,yaw).
        out.position_variance[0]    = static_cast<float>(msg->pose.covariance[0]);
        out.position_variance[1]    = static_cast<float>(msg->pose.covariance[7]);
        out.position_variance[2]    = static_cast<float>(msg->pose.covariance[14]);
        out.orientation_variance[0] = static_cast<float>(msg->pose.covariance[21]);
        out.orientation_variance[1] = static_cast<float>(msg->pose.covariance[28]);
        out.orientation_variance[2] = static_cast<float>(msg->pose.covariance[35]);
        out.velocity_variance[0] = out.velocity_variance[1] = out.velocity_variance[2] = NAN;

        odom_pub_->publish(out);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = "odom_ned";
            tf_msg.child_frame_id  = "base_link_frd";
            tf_msg.transform.translation.x = p_fc_N.x();
            tf_msg.transform.translation.y = p_fc_N.y();
            tf_msg.transform.translation.z = p_fc_N.z();
            tf_msg.transform.rotation = tf2::toMsg(q_ned_frd);
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr  odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;

    std::string          odom_topic_;
    std::array<double,3> body_offset_{};
    bool                 publish_velocity_ = true;
    bool                 publish_tf_       = true;

    tf2::Quaternion q_opt2frd_{0, 0, 0, 1};
    tf2::Quaternion q_N_G_{0, 0, 0, 1};
    tf2::Matrix3x3  R_N_G_{tf2::Matrix3x3::getIdentity()};
    tf2::Vector3    p0_{0, 0, 0};
    bool            initialized_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenVinsBridgeNode>());
    rclcpp::shutdown();
}
