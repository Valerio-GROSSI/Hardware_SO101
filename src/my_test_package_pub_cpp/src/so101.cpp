#include "my_test_package_pub_cpp/so101.hpp"
#include <utility>
// #include <stdexcept>
// #include <fstream>
// #include <nlohmann/json.hpp>
// #include <map>

SO101::SO101(std::string port, std::string name, bool recalibrate)
: // Device() ,
  calibration_path_(std::filesystem::path(__FILE__).parent_path() / ".cache" / (name_ + ".json")) 
, port_(std::move(port))
, name_(std::move(name))
, _motor_limits_(SO101_FOLLOWER_MOTOR_LIMITS)
{
    // if (!std::filesystem::exists(calibration_path_) || recalibrate) { 
    //     calibrate();
    // }

    // std::unordered_map<std::string, MotorCalibration> calibration = load_calibration();

//     bus_ = std::make_unique<FeetcheMotorsBus>(
//     port_,
//     std::unordered_map<std::string, Motor>{
//         {"shoulder_pan",  Motor{1, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"shoulder_lift", Motor{2, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"elbow_flex",    Motor{3, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"wrist_flex",    Motor{4, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"wrist_roll",    Motor{5, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"gripper",       Motor{6, "sts3215", MotorNormMode::RANGE_0_100}},
//     },
//     /* calibration = */ std::unordered_map<std::string, MotorCalibration> calibration
//   )
}

// void SO101::calibrate()
// {
//     _bus_ = std::make_unique<FeetechMotorsBus>(
//     port_,
//     std::unordered_map<std::string, Motor>{
//         {"shoulder_pan",  Motor{1, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"shoulder_lift", Motor{2, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"elbow_flex",    Motor{3, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"wrist_flex",    Motor{4, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"wrist_roll",    Motor{5, "sts3215", MotorNormMode::RANGE_M100_100}},
//         {"gripper",       Motor{6, "sts3215", MotorNormMode::RANGE_0_100}},
//         },
//     )

//     connect();

//     std::cout << "\n Running calibration of SO101-Leader\n";
//     _bus->disable_torque();

//     for (const auto& kv : _bus->motors()) {
//         const std::string& motorName = kv.first;
//         _bus->write("Operating_Mode", motorName,
//         static_cast<int>(OperatingMode::POSITION));
//     }

//     wait_enter("Move SO101-Leader to the middle of its range of motion and press ENTER...");
//     auto homing_offset = _bus->set_half_turn_homings();

//     std::unordered_map<std::string, MotorCalibration> calibration;
//     calibration.reserve(_bus->motors().size());

//     for (const auto& kv : _bus->motors()) {
//         const std::string& motorName = kv.first;
//         const Motor& m = kv.second;

//         MotorCalibration mc;
//         mc.id = m.id;
//         mc.drive_mode = 0;
//         mc.homing_offset = homing_offset.at(motorName);
//         mc.range_min = range_mins.at(motorName);
//         mc.range_max = range_maxes.at(motorName);
        
//         calibration.emplace(motorName, mc);
//     }

//     _bus->write_calibration(calibration);
//     save_calibrationo(calibration);

//     std::cout << "Calibration saved to " << calibration_path_ << "\n";

//     disconnect();
// }

// std::unordered_map<std::string, MotorCalibration>
// _load_calibration() //const std::filesystem::path& calibration_path_
// {
//     std::ifstream file(calibration_path);
//     if (!file.is_open()) {
//         throw std::runtime_error("Failed to open calibration file: " + calibration_path.string());
//     }
//     json json_data;
//     file >> json_data;

//     std::unordered_map<std::string, MotorCalibration> calibration;

//     for (const auto& [motor_name, motor_data] : json_data.items()) {
//         calibration.emplace(
//             motor_name,
//             MotorCalibration{
//                 motor_data.at("id").get<int>(),
//                 motor_data.at("drive_mode").get<int>(),
//                 motor_data.at("homing_offset").get<int>();
//                 motor_data.at("range_min").get<int>();
//                 motor_data.at("range_max").get<int>()
//             }
//         );
//     }

//     return calibration;
// }


// MotorLimitsMap SO101::_motor_limits() const {
//     return _motor_limits_;
// };

// void SO101::add_callback(const std::string& key, std::function<void()> func) {
//     _additional_callbacks[key] = std::move(func);
// }

// int SO101::get_device_state() const {
//     return _bus->sync_read("Present_Position");
// }

// void SO101::disconnect() {
//     if (!is_connected()) {
//         throw DeviceNotConnectedError("SO101-Leader is not connected.");
//     }
//     _bus_->disconnected();
//     std::cout << "SO101-Leader disconnected" << "\n";
// }

// void SO101::connect() {
//     if (is_connected()) {
//         throw DeviceAlreadyConnectedError("SO101-Leader is already connected.");
//     }
//     _bus_->connect();
//     configure();
//     std::cout << "SO101 Arm initialized with: Port=" << port_ << ", Name=" << name_ << "\n";
// }

// std::string SO101::to_string() const {
//     return
//         "SO101-Leader device for SE(3) control.\n"
//         "\t----------------------------------------------\n"
//         "\tMove SO101-Leader to control SO101-Follower\n"
//         "\tIf SO101-Follower can't synchronize with SO101-Leader, "
//         "please add --recalibrate and rerun to recalibrate SO101-Leader.\n";
// }

// void SO101::configure() {
//     _bus_->disable_torque();
//     _bus_->configure_motors();
//     for (const auto& kv : _bus_->motors) {
//         const std::string& motor_name = kv.first;
//         bus_->write("Operating_Mode", motor_name, OperatingMode::POSITION.value);
//     }
// }

// void _save_calibration(std::unordered_map<std::string, MotorCalibration>& calibration) {
//     json save_calibration;

//     for (const auto& [key, value] : calibration) {
//         save_calibration[key] = {
//             {"id", value.id},
//             {"drive_mode", value.drive_mode},
//             {"homing_offset", value.homing_offset},
//             {"range_min", value.range_min},
//             {"range_max", value.range_max}
//         };
//     }

//     std::filesystem::path(calibration_path);
//     if (path.has_parent_path() && !std::filesystem::exists(path.parent_path())) {
//         std::filesystem::create_directories(path.parent_path());
//     }

//     std::ofstream file(calibration_path);
//     file << save_calibration.dump(4);
// }