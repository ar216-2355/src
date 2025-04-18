// #include <cstdint>
// #include <robomas_plugins/msg/frame.hpp>

// namespace can_conection::solenoid_utils{
//     inline auto to_can_mode(uint32_t solenoid_id, uint8_t solenoid_data) -> robomas_plugins::msg::Frame{
//     robomas_plugins::msg::Frame message{};
//     message.id = solenoid_id;
//     message.is_rtr = false;
//     message.is_extended = false;
//     message.is_error = false;
//     message.dlc = 8;
//     message.data[0] = solenoid_data;
//     return message;
//     }
// }
#include <cstdint>
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>

namespace harurobo2::robomas_utils 
{
    /// @brief C620を速度制御するモードに遷移するRobomasFrameメッセージを作成
    /// @param moter_index 対象のロボマスモーターのID
    /// @return RobomasFrameメッセージ

    inline auto to_velocity_mode(std::uint16_t motor_index) -> robomas_plugins::msg::RobomasFrame 
    {
        robomas_plugins::msg::RobomasFrame message{};
        message.motor = motor_index;
        message.c620 = true;
        message.temp = 50;
        message.mode = 1;
        message.velkp = 0.15;
        message.velki = 9.0;
        message.poskp = 0.5;
        message.tyoku_vel_target = 0;
        message.tyoku_pos_target = 0;
        message.stable_pos_limit_vel = 0;
        return message;
    }

    /// @brief C620を停止させるモードに遷移するRobomasFrameメッセージを作成
    /// @param moter_index 対象のロボマスモーターのID
    /// @return RobomasFrameメッセージ

    inline auto to_disable_mode(std::uint16_t motor_index) -> robomas_plugins::msg::RobomasFrame 
    {
        robomas_plugins::msg::RobomasFrame message{};
        message.motor = motor_index;
        message.c620 = true;
        message.temp = 50;
        message.mode = 0;
        message.velkp = 0.15;
        message.velki = 9.0;
        message.poskp = 0.5;
        message.tyoku_vel_target = 0;
        message.tyoku_pos_target = 0;
        message.stable_pos_limit_vel = 0;
        return message;
    }

    /// @brief 目標値を設定するRobomasTargetメッセージを作成
    /// @param x 目標値
    /// @return RobomasTargetメッセージ
    
    inline auto make_target(double x) -> robomas_plugins::msg::RobomasTarget 
    {
        robomas_plugins::msg::RobomasTarget message;
        message.target = x;
        return message;
    }
}