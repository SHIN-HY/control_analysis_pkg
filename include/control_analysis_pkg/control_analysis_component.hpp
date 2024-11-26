#ifndef CONTROL_ANALYSIS_PKG__CONTROL_ANALYSIS_COMPONENT_HPP_
#define CONTROL_ANALYSIS_PKG__CONTROL_ANALYSIS_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

namespace control_analysis_component_ns
{
class ControlAnalysis : public rclcpp::Node
{
    public:
        explicit ControlAnalysis(const rclcpp::NodeOptions & node_options);

    private:
        struct Status
        {
            std::vector<float> steering_status_vec;
            std::vector<float> steering_command_vec;
            float steering_status{0.0};
        };

        struct PerformanceData
        {
            float MAE{0.0};
            float MSE{0.0};
            float RMSE{0.0};
        };

        enum class CommandType 
        {
            STEP,
            SINEWAVE
        };

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_status_sub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_command_pub;

        rclcpp::TimerBase::SharedPtr timer_;

        void statusCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void timerCallback();
        bool startStepCommand(const float time_elapsed, const rclcpp::Time current_time);
        bool startSinewaveCommand(const float time_elapsed);
        PerformanceData startAnalysis(
            const std::vector<float> & steering_command_vec, const std::vector<float> & steering_status_vec);

        Status status_;
        CommandType current_command_type_{CommandType::STEP};        
        bool is_command_finished_{false};
        float sinewave_count_{0.0};
        rclcpp::Time start_time_{this->now()};

        /** 
         * @param
        */ 
        bool temp_param_num_;
};
}
#endif  //CONTROL_ANALYSIS_PKG__CONTROL_ANALYSIS_COMPONENT_HPP_
