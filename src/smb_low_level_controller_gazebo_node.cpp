#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class LowLevelControllerGazebo : public rclcpp::Node
{
public:
    LowLevelControllerGazebo()
    : Node("smb_low_level_controller_gazebo")
    {
        // Declare parameters for the joint names
        this->declare_parameter<std::string>("left_wheel_joint_name", "left_wheel_joint_default");
        this->declare_parameter<std::string>("right_wheel_joint_name", "right_wheel_joint_default");

        // Get the parameters
        this->get_parameter("left_wheel_joint_name", left_wheel_joint_);
        this->get_parameter("right_wheel_joint_name", right_wheel_joint_);

        RCLCPP_INFO(this->get_logger(), "Left wheel joint: %s", left_wheel_joint_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right wheel joint: %s", right_wheel_joint_.c_str());

        // Initialize subscriber to receive wheel velocities
        wheel_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_joint_commands", 10,
            std::bind(&LowLevelControllerGazebo::wheelVelocityCallback, this, std::placeholders::_1));

        // Initialize publishers for left and right wheel velocity commands to Gazebo
        left_joint_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_wheel_joint_velocity_gazebo", 10);
        right_joint_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_wheel_joint_velocity_gazebo", 10);
    }

private:
    void wheelVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 2)
        {
            double left_vel = msg->data[0];
            double right_vel = msg->data[1];
            publishWheelVelocities(left_vel, right_vel);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received wheel velocities message with incorrect size");
        }
    }

    void publishWheelVelocities(double left_vel, double right_vel)
    {
        // Publish left wheel velocity
        auto left_msg = std::make_shared<std_msgs::msg::Float64>();
        left_msg->data = left_vel;
        left_joint_pub_->publish(*left_msg);

        // Publish right wheel velocity
        auto right_msg = std::make_shared<std_msgs::msg::Float64>();
        right_msg->data = right_vel;
        right_joint_pub_->publish(*right_msg);
    }

    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_velocity_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowLevelControllerGazebo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
