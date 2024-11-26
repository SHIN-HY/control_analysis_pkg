#include "control_analysis_pkg/control_analysis_component.hpp"

namespace control_analysis_component_ns
{
ControlAnalysis::ControlAnalysis(const rclcpp::NodeOptions & node_options) : Node("control_analysis_component", node_options)
{
    temp_param_num_ = this->declare_parameter("temp_param_num", false);

    steering_status_sub = this->create_subscription<std_msgs::msg::Float64>(
        "/CarMaker/status/steering_angle", rclcpp::QoS(1), std::bind(&ControlAnalysis::statusCallback, this,std::placeholders::_1));

    pub_can_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", rclcpp::QoS(1));
    pub_steering_command_ = this->create_publisher<std_msgs::msg::Float64>("/steering_cmd_debug", rclcpp::QoS(1));
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControlAnalysis::timerCallback, this));
}

void ControlAnalysis::statusCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    status_.steering_status = msg->data;
}

void ControlAnalysis::timerCallback()
{
    rclcpp::Time current_time = this->now();

    const float time_elapsed = (current_time - start_time_).seconds();

    switch(current_command_type_)
    {
        case CommandType::STEP:
            if(!is_command_finished_)
            {
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"), "\033[1;32m[Start step command]");
                is_command_finished_ = startStepCommand(time_elapsed, current_time);
            }
            else if(is_command_finished_)
            {
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"\033[1;32m[Start analysis]");
                PerformanceData performance_data = startAnalysis(status_.steering_command_vec, status_.steering_status_vec);

                status_.steering_command_vec.clear();
                status_.steering_status_vec.clear();

                std::vector<float> temp_vec1(status_.steering_command_vec);
                std::vector<float> temp_vec2(status_.steering_status_vec);
                temp_vec1.swap(status_.steering_command_vec);
                temp_vec2.swap(status_.steering_status_vec);

                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"--------------------");
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  MAE: %f   |", performance_data.MAE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  MSE: %f   |", performance_data.MSE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  RMSE: %f  |", performance_data.RMSE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"--------------------");

                current_command_type_ = CommandType::SINEWAVE;
                is_command_finished_ = false;
            }
            break;

        case CommandType::SINEWAVE:
            if(!is_command_finished_)
            {
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"\033[1;32m[Start sinewave command]");
                is_command_finished_ = startSinewaveCommand(time_elapsed);
            }
            else if(is_command_finished_)
            {
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"\033[1;32m[Start analysis]");
                PerformanceData performance_data = startAnalysis(status_.steering_command_vec, status_.steering_status_vec);

                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"--------------------");
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  MAE: %f   |", performance_data.MAE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  MSE: %f   |", performance_data.MSE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"|  RMSE: %f  |", performance_data.RMSE);
                RCLCPP_INFO_ONCE(rclcpp::get_logger("control_analysis_component"),"--------------------");
                rclcpp::shutdown();
            }
            break;

        default:
            RCLCPP_INFO(rclcpp::get_logger("control_analysis_component"),"Error");
            break;
    }
}

bool ControlAnalysis::startStepCommand(const float time_elapsed, const rclcpp::Time current_time)
{
    float steering_cmd;
    status_.steering_status_vec.push_back(status_.steering_status);

    if(time_elapsed < 4.0f)
    {
        steering_cmd = 0.0;
    }
    else if(time_elapsed >= 4.0f)
    {
        steering_cmd = 0.3f;
    }

    status_.steering_command_vec.push_back(steering_cmd);
    // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),2000,"On step command...");

    if(time_elapsed > 8.0f)
    {
        RCLCPP_INFO(rclcpp::get_logger("control_analysis_component"),"\033[1;32m[Step command finished]");
        start_time_ = current_time;
        return true;
    }

    const uint8_t steer_can = static_cast<uint8_t>(std::clamp((-steering_cmd * STEERCMD2SIG) + STEERCMD_OFFSET, 0.0, 255.0));

    can_msgs::msg::Frame can_data;
    can_data.id = 320;
    can_data.dlc = 4;
    can_data.data[1] = steer_can;
    can_data.data[3] = 1;

    pub_can_->publish(can_data);
    std_msgs::msg::Float64 debug_msg;
    debug_msg.data = steering_cmd;
    pub_steering_command_->publish(debug_msg);
    return false;
}

bool ControlAnalysis::startSinewaveCommand(const float time_elapsed)
{
    float steering_cmd;
    status_.steering_status_vec.push_back(status_.steering_status);

    if(time_elapsed < 3.0f)
    {
        steering_cmd = 0.0;
    }
    else if(time_elapsed > 3.0f)
    {
        sinewave_count_ += 0.01f;
        steering_cmd = std::sin(sinewave_count_);
        steering_cmd *= 0.1f;
    }

    status_.steering_command_vec.push_back(steering_cmd);
    // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),2000,"On sinewave command...");
    
    if(time_elapsed >= 20.0f)
    {
        RCLCPP_INFO(rclcpp::get_logger("control_analysis_component"),"\033[1;32m[Sinewave command finished]");
        return true;
    }

    const uint8_t steer_can = static_cast<uint8_t>(std::clamp((-steering_cmd * STEERCMD2SIG) + STEERCMD_OFFSET, 0.0, 255.0));

    can_msgs::msg::Frame can_data;
    can_data.id = 320;
    can_data.dlc = 4;
    can_data.data[1] = steer_can;
    can_data.data[3] = 1;
    
    pub_can_->publish(can_data);
    std_msgs::msg::Float64 debug_msg;
    debug_msg.data = steering_cmd;
    pub_steering_command_->publish(debug_msg);
    return false;
}

ControlAnalysis::PerformanceData ControlAnalysis::startAnalysis(
    const std::vector<float> & steering_command_vec, const std::vector<float> & steering_status_vec)
{
    float absolute_sum = 0.0;
    float squared_sum = 0.0;
    PerformanceData performance_data;

    for(unsigned int i = 0; i < steering_status_vec.size(); i++)
    {
        absolute_sum += std::fabs(steering_command_vec[i] - steering_status_vec[i]);
        squared_sum += std::pow(steering_command_vec[i] - steering_status_vec[i], 2);
    }

    performance_data.MAE = absolute_sum / steering_status_vec.size();
    performance_data.MSE = squared_sum / steering_status_vec.size();
    performance_data.RMSE = std::sqrt(performance_data.MSE);

    return performance_data;
}
}// namespace roscco_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_analysis_component_ns::ControlAnalysis)
