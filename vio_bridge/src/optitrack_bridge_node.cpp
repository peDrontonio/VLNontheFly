// OptiTrack -> PX4 visual-odometry bridge.
//
// Subscribes to the tracked rigid-body pose on (default) /drone/pose, published
// by the NatNet/OptiTrack driver. The PoseStamped header names the OptiTrack
// reference frame. The Motive asset is deliberately NOT named base_link, so the
// driver's TF (map -> drone) cannot collide with the odometry converter's
// map -> base_link.
// Conventions for THIS setup:
//   * World frame: X-left, Y-back, Z-up (right-handed, Z-up). The absolute
//     direction of world X/Y is irrelevant because we anchor a local NED frame
//     on the first message (see below); only "Z is up" and right-handedness
//     are relied upon.
//   * Rigid-body frame (as defined in Motive): X-left, Y-back, Z-up, with the
//     drone facing -Y (forward = -Y_body). This is NOT FLU, so it is remapped
//     to FLU (X-forward, Y-left, Z-up) on the body side before processing:
//     a -90 deg yaw about body Z.  After the remap, the rest of the pipeline
//     uses the standard FLU drone convention.
//
// PX4 /fmu/in/vehicle_visual_odometry expects px4_msgs/VehicleOdometry in NED:
//   X-North, Y-East, Z-Down, body = FRD (X-forward, Y-right, Z-down).
//
// This node anchors a local NED frame on the first message (zero position,
// zero yaw) while keeping gravity-aligned roll/pitch, applies the FLU->FRD
// body rotation, and estimates NED velocity by finite-differencing positions.

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OptitrackBridgeNode : public rclcpp::Node
{
public:
    OptitrackBridgeNode() : Node("optitrack_bridge_node")
    {
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/drone/pose");
        // Rigid-body origin offset relative to the FC centre, in FRD body frame.
        body_offset_ = {
            declare_parameter<double>("body_offset_x", 0.0),
            declare_parameter<double>("body_offset_y", 0.0),
            declare_parameter<double>("body_offset_z", 0.0),
        };
        publish_velocity_ = declare_parameter<bool>("publish_velocity", true);
        publish_tf_       = declare_parameter<bool>("publish_tf", true);

        // Rigid-body frame is X-left, Y-back, Z-up (forward = -Y_body).
        // Remap it to FLU (X-forward, Y-left, Z-up): -90 deg yaw about body Z.
        // Applied to the incoming pose as q_W_B = q_W_B * q_body2flu_, which
        // turns the reported orientation into a genuine FLU->world quaternion so
        // the downstream heading anchor and FLU->FRD step are correct.
        q_body2flu_.setRPY(0.0, 0.0, -M_PI / 2);
        q_body2flu_.normalize();

        // FLU (X-forward, Y-left, Z-up) -> FRD (X-forward, Y-right, Z-down):
        // 180° rotation around the X axis.
        q_flu2frd_.setRPY(M_PI, 0.0, 0.0);
        q_flu2frd_.normalize();

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&OptitrackBridgeNode::onPose, this, _1)); //passando para BestAfford
            // std::bind(&OptitrackBridgeNode::onPose, this, _1));
        // PX4 uXRCE-DDS agent subscribes to /fmu/in/* as BEST_EFFORT, VOLATILE,
        // KEEP_LAST. Match that here; a RELIABLE publisher will back-pressure
        // and stall on every publish() when the uXRCE link or a co-running
        // rosbag2 recorder cannot keep up, collapsing the effective rate.
        rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
        px4_qos.best_effort().durability_volatile();
        odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", px4_qos);
        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(),
            "OptiTrack->PX4 bridge | pose_topic=%s | body=X-left/Y-back/Z-up (remapped to FLU) | "
            "lever(FRD)=[%.3f %.3f %.3f] | waiting for first pose...",
            pose_topic_.c_str(), body_offset_[0], body_offset_[1], body_offset_[2]);
    }

private:
    void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const tf2::Vector3 p_W(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        tf2::Quaternion q_W_B;
        tf2::fromMsg(msg->pose.orientation, q_W_B);
        q_W_B.normalize();

        // Remap the rigid-body frame (X-left, Y-back, Z-up) to FLU so that the
        // rest of the node can assume the standard FLU convention. Right-multiply,
        // because q_W_B maps body->world and we are correcting the body side.
        q_W_B = (q_W_B * q_body2flu_).normalized();

        // ---- anchor a zero-yaw local NED frame on the first message ----------
        if (!initialized_) {
            p0_ = p_W;
            // FLU forward axis (body X) projected onto the horizontal world plane.
            const tf2::Vector3 fwd_W = tf2::quatRotate(q_W_B, tf2::Vector3(1, 0, 0));
            tf2::Vector3 fwd_h(fwd_W.x(), fwd_W.y(), 0.0);
            if (fwd_h.length() < 1e-6) {
                // Looking straight up/down: fall back to world X.
                fwd_h = tf2::Vector3(1.0, 0.0, 0.0);
            }
            fwd_h.normalize();

            // Build NED axes in world (Z-up) coordinates.
            const tf2::Vector3 xN_W = fwd_h;              // North
            const tf2::Vector3 zN_W(0, 0, -1);            // Down (world is Z-up)
            const tf2::Vector3 yN_W = zN_W.cross(xN_W);   // East

            R_N_W_.setValue(xN_W.x(), xN_W.y(), xN_W.z(),
                            yN_W.x(), yN_W.y(), yN_W.z(),
                            zN_W.x(), zN_W.y(), zN_W.z());
            R_N_W_.getRotation(q_N_W_);
            q_N_W_.normalize();
            initialized_ = true;
            RCLCPP_INFO(get_logger(),
                "Anchored NED frame: north-heading=%.1f deg in OptiTrack world.",
                std::atan2(fwd_h.y(), fwd_h.x()) * 180.0 / M_PI);
        }

        // ---- pose of the rigid body in the local NED frame -------------------
        const tf2::Vector3 p_N = R_N_W_ * (p_W - p0_);

        // Orientation of the FRD body in NED:
        //   q_NED_FRD = q_{N from W} * q_{W from FLU} * q_{FLU from FRD}
        //             = q_N_W * q_W_B * q_flu2frd_^{-1}
        const tf2::Quaternion q_ned_frd =
            (q_N_W_ * q_W_B * q_flu2frd_.inverse()).normalized();

        // Lever arm: FC centre = rigid-body origin - R_{NED<-FRD} * (offset in FRD).
        const tf2::Vector3 lever(body_offset_[0], body_offset_[1], body_offset_[2]);
        const tf2::Vector3 p_fc_N = p_N - tf2::quatRotate(q_ned_frd, lever);

        const uint64_t ts_sample = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000ULL
                                 + msg->header.stamp.nanosec / 1000ULL;
        const uint64_t ts = this->get_clock()->now().nanoseconds() / 1000ULL;

        // ---- finite-difference NED velocity ----------------------------------
        tf2::Vector3 v_N(0.0, 0.0, 0.0);
        bool velocity_valid = false;
        if (publish_velocity_ && prev_ts_ > 0) {
            const double dt = static_cast<double>(ts_sample - prev_ts_) * 1e-6;
            if (dt > 1e-4 && dt < 0.1) {
                v_N = (p_fc_N - prev_p_fc_N_) * (1.0 / dt);
                velocity_valid = true;
            }
        }

        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp        = ts;
        out.timestamp_sample = ts_sample;
        // Heading is anchored on the first sample (not aligned with True North),
        // so this is FRD per PX4's POSE_FRAME definition.
        out.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
        out.position[0] = static_cast<float>(p_fc_N.x());
        out.position[1] = static_cast<float>(p_fc_N.y());
        out.position[2] = static_cast<float>(p_fc_N.z());
        out.q[0] = static_cast<float>(q_ned_frd.w());   // PX4 order [w,x,y,z]
        out.q[1] = static_cast<float>(q_ned_frd.x());
        out.q[2] = static_cast<float>(q_ned_frd.y());
        out.q[3] = static_cast<float>(q_ned_frd.z());

        if (velocity_valid) {
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
            out.velocity[0] = static_cast<float>(v_N.x());
            out.velocity[1] = static_cast<float>(v_N.y());
            out.velocity[2] = static_cast<float>(v_N.z());
        } else {
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
            out.velocity[0] = out.velocity[1] = out.velocity[2] = NAN;
        }
        out.angular_velocity[0] = out.angular_velocity[1] = out.angular_velocity[2] = NAN;

        // OptiTrack sub-millimeter accuracy: use small fixed variances (m^2, rad^2).
        out.position_variance[0]    = 1e-4f;
        out.position_variance[1]    = 1e-4f;
        out.position_variance[2]    = 1e-4f;
        out.orientation_variance[0] = 1e-4f;
        out.orientation_variance[1] = 1e-4f;
        out.orientation_variance[2] = 1e-4f;
        out.velocity_variance[0] = out.velocity_variance[1] = out.velocity_variance[2] = NAN;

        prev_ts_      = ts_sample;
        prev_p_fc_N_  = p_fc_N;

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

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr     odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                   tf_broadcaster_;

    std::string          pose_topic_;
    std::array<double,3> body_offset_{};
    bool                 publish_velocity_ = true;
    bool                 publish_tf_       = true;

    tf2::Quaternion q_body2flu_{0, 0, 0, 1};   // rigid-body (X-left/Y-back/Z-up) -> FLU
    tf2::Quaternion q_flu2frd_{0, 0, 0, 1};
    tf2::Quaternion q_N_W_{0, 0, 0, 1};
    tf2::Matrix3x3  R_N_W_{tf2::Matrix3x3::getIdentity()};
    tf2::Vector3    p0_{0, 0, 0};
    bool            initialized_ = false;

    uint64_t     prev_ts_     = 0;
    tf2::Vector3 prev_p_fc_N_{0, 0, 0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackBridgeNode>());
    rclcpp::shutdown();
}
