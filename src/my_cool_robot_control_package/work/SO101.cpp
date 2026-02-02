#include "SO101.hpp"

#include <stdexcept>
#include <utility>
#include <unordered_map>

// JSON (si tu veux)
// #include <nlohmann/json.hpp>
// #include <fstream>

using namespace std;

SO101::SO101(std::string port, std::string name, bool recalibrate)
: Device()
, port_(std::move(port))
, name_(std::move(name))
, calibration_path_(
      std::filesystem::path(__FILE__).parent_path() / ".cache" / (name_ + ".json")
  )
, bus_(
      port_,
      std::unordered_map<std::string, Motor>{
          {"shoulder_pan",  Motor{1, "sts3215", MotorNormMode::RANGE_M100_100}},
          {"shoulder_lift", Motor{2, "sts3215", MotorNormMode::RANGE_M100_100}},
          {"elbow_flex",    Motor{3, "sts3215", MotorNormMode::RANGE_M100_100}},
          {"wrist_flex",    Motor{4, "sts3215", MotorNormMode::RANGE_M100_100}},
          {"wrist_roll",    Motor{5, "sts3215", MotorNormMode::RANGE_M100_100}},
          {"gripper",       Motor{6, "sts3215", MotorNormMode::RANGE_0_100}},
      },
      /* calibration = */ std::unordered_map<std::string, MotorCalibration>{} // remplacé juste après
  )
, motor_limits_(SO101_FOLLOWER_MOTOR_LIMITS)
{
    // calibration: créer le dossier .cache si besoin
    std::filesystem::create_directories(calibration_path_.parent_path());

    // si pas de fichier ou recalibrate -> calibrate()
    if (!std::filesystem::exists(calibration_path_) || recalibrate) {
        calibrate();
    }

    // charger calibration et l’injecter dans le bus
    auto calibration = load_calibration();
    bus_.set_calibration(std::move(calibration)); // à toi de fournir cette méthode, ou passer au ctor

    // connect optionnel (comme ton code commenté)
    // connect();

    // flags/callbacks déjà init via valeurs par défaut
}

void SO101::calibrate()
{
    // équivalent self.calibrate()
    // - lire les valeurs
    // - écrire le fichier calibration_path_
    // Ici: placeholder
    // throw si ça échoue
}

std::unordered_map<std::string, MotorCalibration> SO101::load_calibration() const
{
    // Version placeholder sans lib JSON
    // -> retourne une map vide si tu n’as pas encore le parser

    // Si tu veux une vraie version JSON avec nlohmann::json, dis-moi et je te la mets.
    return {};
}
