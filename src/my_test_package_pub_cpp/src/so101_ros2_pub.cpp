#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class LeRobotJointStatePublisher : public rclcpp::Node {
public:
  LeRobotJointStatePublisher()
  : Node("lerobot_joint_state_publisher")
  {
    // Params
    this->declare_parameter<std::string>("robot_name", "so101_leader");
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<bool>("recalibrate", false);

    robot_name_ = this->get_parameter("robot_name").as_string();
    port_ = this->get_parameter("port").as_string();
    recalibrate_ = this->get_parameter("recalibrate").as_bool();

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);

    // Joint names mapping
    motor_key_to_joint_name_ = {
      {0, "shoulder_pan"},
      {1, "shoulder_lift"},
      {2, "elbow_flex"},
      {3, "wrist_flex"},
      {4, "wrist_roll"},
      {5, "gripper"},
    };
    for (const auto & kv : motor_key_to_joint_name_) {
      joint_names_.push_back(kv.second);
    }

    RCLCPP_INFO(this->get_logger(), "Node started. robot_name=%s port=%s recalibrate=%s",
                robot_name_.c_str(), port_.c_str(), recalibrate_ ? "true" : "false");

    // TODO: connecter le bras en C++
    // connect_robot(port_, robot_name_, recalibrate_);

    timer_ = this->create_wall_timer(100ms, std::bind(&LeRobotJointStatePublisher::publish_joint_states, this));
  }

private:
  void publish_joint_states() {
    // TODO: remplacer par une vraie lecture du bras
    // Exemple attendu: positions en degrés par moteur (0..5)
    std::map<int, double> joint_positions_deg;
    bool ok = get_device_state_deg(joint_positions_deg);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Robot not ready / read failed. Skipping publish.");
      return;
    }

    // Convert deg -> rad et respecter l'ordre 0..5
    std::vector<double> positions_rad;
    positions_rad.reserve(motor_key_to_joint_name_.size());

    for (const auto & kv : motor_key_to_joint_name_) {
      int key = kv.first;
      auto it = joint_positions_deg.find(key);
      if (it == joint_positions_deg.end()) {
        RCLCPP_WARN(this->get_logger(), "Missing motor key %d", key);
        positions_rad.push_back(0.0);
        continue;
      }
      positions_rad.push_back(it->second * M_PI / 180.0);
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names_;
    msg.position = positions_rad;

    pub_->publish(msg);
    // évite le spam: debug plutôt que info
    RCLCPP_DEBUG(this->get_logger(), "Published JointState (%zu joints)", msg.position.size());
  }

  // ====== A IMPLEMENTER ======
  bool get_device_state_deg(std::map<int, double> & out_deg) {
    // Ici tu dois lire /dev/ttyACM0 et décoder le protocole SO101
    // out_deg[0] = ...
    return false; // tant que pas implémenté
  }

  std::string robot_name_;
  std::string port_;
  bool recalibrate_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<int, std::string> motor_key_to_joint_name_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeRobotJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}