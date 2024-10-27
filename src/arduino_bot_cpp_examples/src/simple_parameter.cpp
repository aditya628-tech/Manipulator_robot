#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <memory>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_param", 28);//Declaring a new parameter as the type int and giving default value as 28
        declare_parameter<std::string>("simple_string_param", "place_holder_param");

        param_callback_handle = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1)); // Helps to manipulate the value of the parameter during node execution
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param : parameters){
            if (param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Pram simple_int_param changed. New value: " << param.as_int());
                result.successful = true;
            }
            if (param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Pram simple_string_param changed. New value: " << param.as_string());
                result.successful = true;
            }
        }

        return result;
    }
};

int main(int argc, char*argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}