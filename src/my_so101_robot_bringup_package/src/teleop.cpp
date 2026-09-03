#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class SubscriberArmCommandRelay : public rclcpp::Node
{
public:
    SubscriberArmCommandRelay() : Node("publisher_subscriber_relay") {
        RCLCPP_INFO(get_logger(), "Initializing SubscriberArmCommandRelay...");

        this->declare_parameter<bool>("activate_trajectory_controller", "false");
        this->get_parameter("activate_trajectory_controller", activate_trajectory_controller_);

        this->declare_parameter<std::string>("pub_arm_joint_topic", "/leader/joint_states");
        this->get_parameter("pub_arm_joint_topic", pub_arm_joint_topic_);

        this->declare_parameter<std::string>("sub_arm_joint_topic", "/follower/joint_states");
        this->get_parameter("sub_arm_joint_topic", sub_arm_joint_topic_);

        this->declare_parameter<std::string>("sub_arm_jtc_topic", "trajectory_controller/joint_trajectory");
        this->get_parameter("sub_arm_jtc_topic", sub_arm_jtc_topic_);

        this->declare_parameter<std::string>("sub_arm_fcc_topic", "forward_controller/commands");
        this->get_parameter("sub_arm_fcc_topic", sub_arm_fcc_topic_);

        this->declare_parameter<double>("publish_rate_hz", 50.0);
        this->get_parameter("publish_rate_hz", publish_rate_hz_);

        this->declare_parameter<double>("stale_timeout_s", 0.25);
        this->get_parameter("stale_timeout_s", stale_timeout_s_);

        this->declare_parameter<double>("point_dt_s", 0.02);
        this->get_parameter("point_dt_s", point_dt_s_);

        this->declare_parameter<std::vector<std::string>>("arm_joints", 
            std::vector<std::string>{"soulder_pan", "shoulder_lift", "elbow_flex",
                                      "wrist_flex", "wrist_roll", "gripper"});
        this->get_parameter("arm_joints", arm_joints_);

        RCLCPP_INFO(get_logger(), "Publisher Arm Joint States Topic: %s", pub_arm_joint_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Subscriber Arm Joint Trajectory Controller Topic %s", sub_arm_jtc_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Rate: %1.f Hz, Arm joints: %zu", publish_rate_hz_, arm_joints_.size());
        
        // ROS interfaces
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            pub_arm_joint_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SubscriberArmCommandRelay::joint_state_callback, this, std::placeholders::_1));
        
        trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            sub_arm_jtc_topic_,  rclcpp::QoS(10).reliable());

        forward_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            sub_arm_fcc_topic_, rclcpp::QoS(10).reliable());

        // timer_ = create_wall_timer(
        //     std::chrono::duration<double>(1.0 / publish_rate_hz_),
        //     std::bind(&SubscriberArmCommandRelay::control_loop, this)
        // );

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            std::bind(&SubscriberArmCommandRelay::control_loop, this)
        );

        raw_arm_.resize(arm_joints_.size(), 0.0);

        RCLCPP_INFO(get_logger(), "SubscriberArmCommandRelay initialized.");
    }

private:
    // Parameters
    bool activate_trajectory_controller_;
    std::string pub_arm_joint_topic_;
    std::string sub_arm_joint_topic_;
    std::string sub_arm_jtc_topic_;
    std::string sub_arm_fcc_topic_;
    double publish_rate_hz_;
    double stale_timeout_s_{0.25};
    double point_dt_s_{0.02};
    std::vector<std::string> arm_joints_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    bool initialized_{false};
    std::vector<int> arm_idx_;
    std::vector<double> raw_arm_;
    rclcpp::Time last_pub_arm_stamp_{0, 0, RCL_ROS_TIME};

    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg) {

        if (!initialized_ && !initialize_indices(*msg)) { 
            return;
        }

        last_pub_arm_stamp_ = this->now();

        for (size_t i = 0; i < arm_joints_.size(); ++i) {
            raw_arm_[i] = msg->position[arm_idx_[i]];
        }   
    }

    bool initialize_indices(const sensor_msgs::msg::JointState &msg) {
        // Build name -> index map
        std::unordered_map<std::string, int> idx_map;
        idx_map.reserve(msg.name.size());
        for (size_t i = 0; i < msg.name.size() ; i++)
            idx_map[msg.name[i]] = static_cast<int>(i);
        
        // Arm indexes
        arm_idx_.assign(arm_joints_.size(), -1);
        for (size_t i = 0; i < arm_joints_.size(); ++i) {
            auto it = idx_map.find(arm_joints_[i]);
            if (it == idx_map.end()) {
                RCLCPP_ERROR(this->get_logger(), "Publisher arm joint '%s' not found", arm_joints_[i].c_str());
                return false;
            }
            arm_idx_[i] = it->second;
        }

        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Initialized: %zu arm joints", arm_joints_.size());
        return true;
    }

    void control_loop() {

        if (!initialized_) return;

        const auto now = this->now();

        if ((now - last_pub_arm_stamp_).seconds() > stale_timeout_s_) return;

        publish_arm(now);
    }

    void publish_arm(const rclcpp::Time &time) {
        if (activate_trajectory_controller_) {
            trajectory_msgs::msg::JointTrajectory jt;
            jt.header.stamp = time;
            jt.joint_names = arm_joints_;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = raw_arm_;

            const int sec = static_cast<int>(point_dt_s_);
            const int nsec = static_cast<int>((point_dt_s_ - sec) * 1e9);
            pt.time_from_start.sec = sec;
            pt.time_from_start.nanosec = nsec;

            jt.points.push_back(pt);
            trajectory_pub_->publish(jt);

        } else {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data = raw_arm_;
            forward_pub_->publish(cmd);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberArmCommandRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}