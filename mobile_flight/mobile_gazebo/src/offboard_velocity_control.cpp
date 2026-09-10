#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <std_srvs/srv/trigger.hpp>

// VIO liveness is read from /orb_slam3/pose directly so we don't cross-subscribe
// to an /fmu/in/* topic that vio_bridge is the sole writer of.
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mobile_msgs/srv/set_velocity.hpp>
#include <mobile_msgs/srv/takeoff.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class OffboardVelocityControl : public rclcpp::Node {
public:
    OffboardVelocityControl() : Node("offboard_velocity_control") {
        // --- Parameters ---
        // bench_mode bypasses estimator/battery guards for ground testing.
        // MUST be false for any real flight.
        this->declare_parameter<bool>("bench_mode", false);
        bench_mode_ = this->get_parameter("bench_mode").as_bool();
        if (bench_mode_) {
            RCLCPP_WARN(this->get_logger(),
                "BENCH MODE active — estimator and battery guards are DISABLED. "
                "Do not fly in this configuration.");
        }

        // --- Publishers ---
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        // NOTE: We do NOT publish VehicleOdometry here.
        // The vio_bridge node is the sole publisher to /fmu/in/vehicle_visual_odometry.

        // --- PX4 Subscriptions ---
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::local_position_callback, this, std::placeholders::_1));

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::vehicle_status_callback, this, std::placeholders::_1));

        timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status", 
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::timesync_callback, this, std::placeholders::_1));

        battery_status_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::battery_status_callback, this, std::placeholders::_1));

        sensor_gps_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
            "/fmu/out/vehicle_gps_position",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::sensor_gps_callback, this, std::placeholders::_1));

        // --- VIO: Health-monitoring subscription (reads ORB-SLAM3 directly) ---
        // We listen to /orb_slam3/pose instead of /fmu/in/vehicle_visual_odometry so
        // the FC-bound flow stays one-way: vio_bridge is the sole writer to PX4.
        vio_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose", 10,
            std::bind(&OffboardVelocityControl::vio_pose_callback, this, std::placeholders::_1));

        control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::control_mode_callback, this, std::placeholders::_1));

        failsafe_flags_sub_ = this->create_subscription<px4_msgs::msg::FailsafeFlags>(
            "/fmu/out/failsafe_flags",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::failsafe_flags_callback, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::attitude_callback, this, std::placeholders::_1));

        ekf_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::ekf_odom_callback, this, std::placeholders::_1));

        sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardVelocityControl::sensor_combined_callback, this, std::placeholders::_1));

        // --- /cmd_vel Subscription ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&OffboardVelocityControl::cmd_vel_callback, this, std::placeholders::_1));

        // --- Services ---
        arm_srv_ = this->create_service<std_srvs::srv::Trigger>("arm", std::bind(&OffboardVelocityControl::arm_callback, this, std::placeholders::_1, std::placeholders::_2));
        land_srv_ = this->create_service<std_srvs::srv::Trigger>("land", std::bind(&OffboardVelocityControl::land_callback, this, std::placeholders::_1, std::placeholders::_2));
        estop_srv_ = this->create_service<std_srvs::srv::Trigger>("estop", std::bind(&OffboardVelocityControl::estop_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        last_command_time_ = this->get_clock()->now();
        velocity_end_time_ = last_command_time_;  // safe default; INIT/TAKEOFF don't read it
        takeoff_srv_ = this->create_service<mobile_msgs::srv::Takeoff>("takeoff", std::bind(&OffboardVelocityControl::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        set_velocity_srv_ = this->create_service<mobile_msgs::srv::SetVelocity>(
            "set_velocity", std::bind(&OffboardVelocityControl::set_velocity_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardVelocityControl::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Offboard Velocity Control Node Started (VIO health monitor active). Waiting for commands...");
    }

private:
    enum class State { INIT, TAKEOFF, VELOCITY };
    State current_state_ = State::INIT;

    rclcpp::TimerBase::SharedPtr timer_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // --- PX4 Subscriptions ---
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sensor_gps_sub_;

    // --- VIO Health Monitor + cmd_vel ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vio_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // --- Additional FC Telemetry ---
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_flags_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr ekf_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;

    // --- Telemetry state ---
    uint8_t nav_state_ = 0;
    uint8_t arming_state_ = 0;
    float battery_voltage_ = 0.0;
    float battery_remaining_ = 0.0;
    bool estimator_good_ = false;
    uint8_t gps_fix_type_ = 0;
    uint8_t gps_satellites_ = 0;

    // --- Control mode state ---
    bool offboard_mode_engaged_ = false;
    bool flag_armed_fc_ = false;

    // --- Failsafe state (critical blockers for arming/takeoff) ---
    bool critical_failsafe_active_ = false;
    std::string last_failsafe_reason_;

    // --- Attitude (quaternion from FC, NED earth → FRD body) ---
    std::array<float, 4> attitude_q_ = {1.0f, 0.0f, 0.0f, 0.0f};

    // --- IMU snapshot (sensor_combined) ---
    std::array<float, 3> imu_gyro_rad_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> imu_accel_m_s2_ = {0.0f, 0.0f, 0.0f};
    uint8_t imu_accel_clipping_ = 0;

    // --- Services ---
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
    
    rclcpp::Time last_command_time_;
    rclcpp::Time last_timesync_time_;
    bool estop_active_ = false;

    // --- Safety constraints ---
    const float MAX_VELOCITY = 5.0; // m/s
    const float MAX_Z_VELOCITY = 2.0; // m/s
    const float MAX_ALTITUDE = -50.0; // NED frame, negative is up
    const float MIN_BATTERY_FOR_TAKEOFF = 0.20f; // 20% — reject below this
    bool bench_mode_ = false;

    rclcpp::Service<mobile_msgs::srv::Takeoff>::SharedPtr takeoff_srv_;
    rclcpp::Service<mobile_msgs::srv::SetVelocity>::SharedPtr set_velocity_srv_;

    float current_yaw_ = 0.0;
    float current_x_ = 0.0;                // NED, from PX4 local_position
    float current_y_ = 0.0;
    float takeoff_hold_x_ = 0.0;           // captured at takeoff entry
    float takeoff_hold_y_ = 0.0;
    float target_takeoff_altitude_ = -5.0; // NED frame, negative is up
    
    // --- Velocity command state ---
    // Exactly one of {target_yaw_, target_yawspeed_} should be a real number;
    // the other should be NaN. PX4 ignores NaN fields in TrajectorySetpoint.
    float target_vx_ = 0.0;
    float target_vy_ = 0.0;
    float target_vz_ = 0.0;
    float target_yaw_ = std::numeric_limits<float>::quiet_NaN();      // absolute heading (rad, NED)
    float target_yawspeed_ = 0.0f;                                    // rad/s
    // Contract: /cmd_vel is ALWAYS body-frame (FRD). The set_velocity service may
    // override this per-call to "ned" for direct world-frame scripting.
    std::string frame_id_ = "body";
    rclcpp::Time velocity_end_time_;

    // --- VIO state (health monitoring only) ---
    bool vio_active_ = false;
    rclcpp::Time last_vio_time_;

    // ========================================================================
    // VIO Health Monitor Callback
    // Tracks ORB-SLAM3 liveness only. Does NOT touch estimator_good_ —
    // that flag is owned solely by local_position_callback (PX4 EKF truth).
    // ========================================================================
    void vio_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        (void)msg;
        last_vio_time_ = this->get_clock()->now();
        vio_active_ = true;
    }

    // ========================================================================
    // /cmd_vel Callback — standard velocity interface (body frame)
    // ========================================================================
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (estop_active_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cmd_vel ignored: ESTOP active!");
            return;
        }

        last_command_time_ = this->get_clock()->now();
        current_state_ = State::VELOCITY;
        frame_id_ = "body";

        // Map Twist linear to velocity targets (body frame)
        target_vx_ = std::clamp(static_cast<float>(msg->linear.x), -MAX_VELOCITY, MAX_VELOCITY);
        target_vy_ = std::clamp(static_cast<float>(msg->linear.y), -MAX_VELOCITY, MAX_VELOCITY);
        target_vz_ = std::clamp(static_cast<float>(msg->linear.z), -MAX_Z_VELOCITY, MAX_Z_VELOCITY);

        // /cmd_vel.angular.z is a yaw RATE. Hand it to PX4 via yawspeed so PX4
        // integrates with its own loop period instead of us guessing dt.
        target_yawspeed_ = static_cast<float>(msg->angular.z);
        target_yaw_ = std::numeric_limits<float>::quiet_NaN();

        // cmd_vel is continuous — set a generous timeout (500ms without a new msg → hover)
        velocity_end_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(0.5);
    }

    // ========================================================================
    // PX4 Topic Callbacks (unchanged)
    // ========================================================================
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_yaw_ = msg->heading;
        current_x_ = msg->x;
        current_y_ = msg->y;
        // Single source of truth: PX4 EKF flags. If VIO is feeding EKF correctly,
        // these go valid; if VIO drops, PX4 reports it here. Don't second-guess it.
        estimator_good_ = msg->xy_valid && msg->v_xy_valid && msg->z_valid;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
    }

    void battery_status_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
        battery_voltage_ = msg->voltage_v;
        battery_remaining_ = msg->remaining;
    }

    void sensor_gps_callback(const px4_msgs::msg::SensorGps::SharedPtr msg) {
        gps_fix_type_ = msg->fix_type;
        gps_satellites_ = msg->satellites_used;
    }

    void timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
        (void)msg;
        last_timesync_time_ = this->get_clock()->now();
    }

    void control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
        bool was_engaged = offboard_mode_engaged_;
        offboard_mode_engaged_ = msg->flag_control_offboard_enabled;
        flag_armed_fc_ = msg->flag_armed;
        if (offboard_mode_engaged_ != was_engaged) {
            RCLCPP_INFO(this->get_logger(), "Offboard mode %s",
                        offboard_mode_engaged_ ? "ENGAGED" : "DISENGAGED");
        }
    }

    void failsafe_flags_callback(const px4_msgs::msg::FailsafeFlags::SharedPtr msg) {
        std::string reason;
        if (msg->fd_critical_failure)           reason = "fd_critical_failure";
        // else if (msg->battery_unhealthy)        reason = "battery_unhealthy";
        // else if (msg->battery_low_remaining_time) reason = "battery_low_remaining_time";
        else if (msg->offboard_control_signal_lost && offboard_mode_engaged_) reason = "offboard_control_signal_lost";
        else if (msg->local_position_invalid && msg->local_position_invalid_relaxed)
                                                reason = "local_position_invalid";
        else if (msg->attitude_invalid)         reason = "attitude_invalid";

        bool active = !reason.empty();
        if (active && !critical_failsafe_active_) {
            RCLCPP_WARN(this->get_logger(), "Critical failsafe ACTIVE: %s", reason.c_str());
        } else if (!active && critical_failsafe_active_) {
            RCLCPP_INFO(this->get_logger(), "Critical failsafe cleared (was: %s)", last_failsafe_reason_.c_str());
        }
        critical_failsafe_active_ = active;
        last_failsafe_reason_ = reason;
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        attitude_q_ = msg->q;
    }

    void ekf_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        (void)msg;
        // Available for cross-check against VIO odometry. No action required.
    }

    void sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        imu_gyro_rad_ = msg->gyro_rad;
        imu_accel_m_s2_ = msg->accelerometer_m_s2;
        imu_accel_clipping_ = msg->accelerometer_clipping;
        if (imu_accel_clipping_ != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Accelerometer clipping detected (bitfield: 0x%02X)", imu_accel_clipping_);
        }
    }

    // ========================================================================
    // Service Callbacks (preserved from previous implementation)
    // ========================================================================
    void arm_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        if (estop_active_) {
            response->success = false;
            response->message = "Cannot arm, ESTOP active!";
            return;
        }
        
        if (!estimator_good_) {
            if (bench_mode_) {
                RCLCPP_WARN(this->get_logger(), "Estimator not ready — bench_mode override, allowing arm.");
            } else {
                response->success = false;
                response->message = "Estimator not ready (PX4 EKF reports xy/v_xy/z invalid).";
                RCLCPP_ERROR(this->get_logger(), "Arming rejected: estimator not ready.");
                return;
            }
        }
        if (critical_failsafe_active_) {
            response->success = false;
            response->message = "Critical failsafe active: " + last_failsafe_reason_;
            RCLCPP_ERROR(this->get_logger(), "Arming rejected: %s", last_failsafe_reason_.c_str());
            return;
        }
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Mode: Offboard
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        response->success = true;
        response->message = "Switched to Offboard mode and sent Arm command";
        RCLCPP_INFO(this->get_logger(), "Switched to Offboard mode and sent Arm command");
    }

    void takeoff_callback(const std::shared_ptr<mobile_msgs::srv::Takeoff::Request> request, std::shared_ptr<mobile_msgs::srv::Takeoff::Response> response) {
        if (estop_active_) {
            response->success = false;
            response->message = "Cannot takeoff, ESTOP active";
            return;
        }

        if (!estimator_good_) {
            if (bench_mode_) {
                RCLCPP_WARN(this->get_logger(), "Estimator not ready — bench_mode override, allowing takeoff.");
            } else {
                response->success = false;
                response->message = "Estimator not ready (PX4 EKF reports xy/v_xy/z invalid).";
                RCLCPP_ERROR(this->get_logger(), "Takeoff rejected: estimator not ready.");
                return;
            }
        }

        // Battery gate: reject below threshold unless bench_mode or no reading yet.
        if (battery_remaining_ > 0.0f && battery_remaining_ < MIN_BATTERY_FOR_TAKEOFF) {
            if (bench_mode_) {
                RCLCPP_WARN(this->get_logger(), "Battery at %.0f%% — bench_mode override, allowing takeoff.",
                            battery_remaining_ * 100.0f);
            } else {
                response->success = false;
                response->message = "Battery below " +
                    std::to_string(static_cast<int>(MIN_BATTERY_FOR_TAKEOFF * 100)) + "%.";
                RCLCPP_ERROR(this->get_logger(), "Takeoff rejected: battery at %.1f%%.",
                             battery_remaining_ * 100.0f);
                return;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Telemetry Check passed. ArmState: %d, NavState: %d, GPS Fix: %d, Sats: %d, VIO: %s", 
                    arming_state_, nav_state_, gps_fix_type_, gps_satellites_, vio_active_ ? "ACTIVE" : "INACTIVE");

        last_command_time_ = this->get_clock()->now();
        current_state_ = State::TAKEOFF;

        // Snapshot current XY so takeoff climbs straight up instead of translating
        // back to the EKF origin {0,0}. Falls back to 0 only if EKF hasn't published.
        takeoff_hold_x_ = current_x_;
        takeoff_hold_y_ = current_y_;

        // Convert positive altitude to NED Frame (negative z)
        target_takeoff_altitude_ = -std::abs(request->altitude);
        if (target_takeoff_altitude_ == 0.0) {
            target_takeoff_altitude_ = -5.0; // Default
        }
        
        // Altitude limit clamp
        if (target_takeoff_altitude_ < MAX_ALTITUDE) {
            target_takeoff_altitude_ = MAX_ALTITUDE;
        }
        
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Mode: Offboard
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        response->success = true;
    
        response->message = "Arming and Takeoff initiated";
        RCLCPP_INFO(this->get_logger(), "Takeoff initiated (Offboard mode target: %.2f m altitude)", request->altitude);
    }

    void land_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        current_state_ = State::INIT;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        response->success = true;
        response->message = "Landing commanded";
        RCLCPP_INFO(this->get_logger(), "Land command sent via VehicleCommand.");
    }

    void estop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        estop_active_ = true;
        current_state_ = State::INIT;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); // Emergency land
        response->success = true;
        response->message = "ESTOP EXECUTED!";
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP!");
    }

    void set_velocity_callback(const std::shared_ptr<mobile_msgs::srv::SetVelocity::Request> request, std::shared_ptr<mobile_msgs::srv::SetVelocity::Response> response) {
        if (estop_active_) {
            response->success = false;
            response->message = "Cannot set velocity, ESTOP active";
            return;
        }
        last_command_time_ = this->get_clock()->now();
        current_state_ = State::VELOCITY;
        
        // Clamp velocities
        target_vx_ = std::clamp(request->vx, -MAX_VELOCITY, MAX_VELOCITY);
        target_vy_ = std::clamp(request->vy, -MAX_VELOCITY, MAX_VELOCITY);
        target_vz_ = std::clamp(request->vz, -MAX_Z_VELOCITY, MAX_Z_VELOCITY);
        target_yaw_ = request->yaw;
        target_yawspeed_ = std::numeric_limits<float>::quiet_NaN();
        frame_id_ = request->frame_id;
        
        if (request->duration > 0.0) {
            velocity_end_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(request->duration);
        } else {
            // If duration <= 0, consider it infinite (we set a very far future time)
            velocity_end_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(999999.0);
        }

        if (request->auto_arm) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "Auto-armed drone.");
        }

        response->success = true;
        response->message = "Velocity command applied.";
        RCLCPP_INFO(this->get_logger(), "Velocity command received. Vx: %.2f Vy: %.2f Vz: %.2f Frame: %s Duration: %.2f", 
            target_vx_, target_vy_, target_vz_, frame_id_.c_str(), request->duration);
    }

    // ========================================================================
    // Timer Callback — publishes offboard heartbeat + setpoints at 10Hz
    // ========================================================================
    void timer_callback() {
        if (estop_active_) return; 
        
        auto now = this->get_clock()->now();
        
        // Time sync heartbeat
        if (last_timesync_time_.nanoseconds() > 0 && (now - last_timesync_time_).seconds() > 2.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "XRCE-DDS connection lost! No timesync for >2s");
        }

        // VIO liveness check — mark inactive if no pose for >500ms.
        // estimator_good_ is NOT touched here: PX4's local_position flags will
        // flip xy_valid/z_valid on their own if the EKF stops trusting VIO.
        if (vio_active_ && last_vio_time_.nanoseconds() > 0 && (now - last_vio_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "VIO tracking lost! No ORB-SLAM3 pose for >500ms");
            vio_active_ = false;
        }

        // Companion computer heartbeat monitoring
        if (current_state_ == State::VELOCITY && (now - last_command_time_).seconds() > 5.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No companion commands for 5s. Auto-hovering (Heartbeat failsafe).");
            velocity_end_time_ = now; // Force hover in place by invalidating duration
        }

        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    }

    // ========================================================================
    // Publish helpers
    // ========================================================================
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = (current_state_ == State::TAKEOFF);
        msg.velocity = (current_state_ == State::VELOCITY || current_state_ == State::INIT);
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        if (current_state_ == State::TAKEOFF) {
            // Climb straight up from the XY captured at takeoff entry.
            msg.position = {takeoff_hold_x_, takeoff_hold_y_, target_takeoff_altitude_};
            msg.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
            msg.yaw = current_yaw_;
        } else if (current_state_ == State::VELOCITY) {
            if (this->get_clock()->now() < velocity_end_time_) {
                float out_vx = target_vx_;
                float out_vy = target_vy_;
                float out_vz = target_vz_;

                // Default: body→NED rotation using PX4 heading.
                // Only "ned" (or "world") bypasses the rotation and sends velocity raw.
                const bool already_ned = (frame_id_ == "ned" || frame_id_ == "world");
                if (!already_ned) {
                    out_vx = target_vx_ * std::cos(current_yaw_) - target_vy_ * std::sin(current_yaw_);
                    out_vy = target_vx_ * std::sin(current_yaw_) + target_vy_ * std::cos(current_yaw_);
                }

                msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
                msg.velocity = {out_vx, out_vy, out_vz};
                msg.yaw = target_yaw_;
                msg.yawspeed = target_yawspeed_;
            } else {
                // Time is up, hover in place. Drop the yaw rate so we don't keep spinning.
                msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
                msg.velocity = {0.0, 0.0, 0.0};
                msg.yaw = std::numeric_limits<float>::quiet_NaN();
                msg.yawspeed = 0.0f;
            }
        } else { // INIT
            msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
            msg.velocity = {0.0, 0.0, 0.0};
            msg.yaw = std::numeric_limits<float>::quiet_NaN();
            msg.yawspeed = 0.0f;
        }
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardVelocityControl>());
    rclcpp::shutdown();
    return 0;
}
