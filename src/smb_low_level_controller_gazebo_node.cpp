#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


class LowLevelControllerGazebo : public rclcpp::Node
{
public:
    LowLevelControllerGazebo()
    : Node("smb_low_level_controller_gazebo")
    {
        // Declare parameters for the joint names
        this->declare_parameter<std::string>("LH_joint_name", "LH_joint_default");
        this->declare_parameter<std::string>("RH_joint_name", "RH_joint_default");
        this->declare_parameter<std::string>("RF_joint_name", "RF_joint_default");
        this->declare_parameter<std::string>("LF_joint_name", "LF_joint_default");

        // Get the parameters
        this->get_parameter("LH_joint_name", LH_joint_);
        this->get_parameter("RH_joint_name", RH_joint_);
        this->get_parameter("RF_joint_name", RF_joint_);
        this->get_parameter("LF_joint_name", LF_joint_);

        RCLCPP_INFO(this->get_logger(), "LH joint: %s", LH_joint_.c_str());
        RCLCPP_INFO(this->get_logger(), "RH joint: %s", RH_joint_.c_str());
        RCLCPP_INFO(this->get_logger(), "RF joint: %s", RF_joint_.c_str());
        RCLCPP_INFO(this->get_logger(), "LF joint: %s", LF_joint_.c_str());

        // Initialize subscriber to receive joint velocities
        wheel_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_joint_commands", 10,
            std::bind(&LowLevelControllerGazebo::wheelVelocityCallback, this, std::placeholders::_1));

        // Initialize publishers for each joint velocity command to Gazebo
        LH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/LH_WHEEL_JOINT/cmd_vel", 10);
        RH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/RH_WHEEL_JOINT/cmd_vel", 10);
        RF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/RF_WHEEL_JOINT/cmd_vel", 10);
        LF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/LF_WHEEL_JOINT/cmd_vel", 10);
    }

private:
    void wheelVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received wheel velocities: [%f, %f]", msg->data[0], msg->data[1]);
        if (msg->data.size() == 2)
        {
            double LH_vel = msg->data[0];
            double RH_vel = msg->data[1];
            double RF_vel = msg->data[1];
            double LF_vel = msg->data[0];
            publishWheelVelocities(LH_vel, RH_vel, RF_vel, LF_vel);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received wheel velocities message with incorrect size");
        }
    }

    void publishWheelVelocities(double LH_vel, double RH_vel, double RF_vel, double LF_vel)
    {
        // Publish LH joint velocity
        auto LH_msg = std::make_shared<std_msgs::msg::Float64>();
        LH_msg->data = LH_vel;
        LH_joint_velocity_pub_->publish(*LH_msg);

        // Publish RH joint velocity
        auto RH_msg = std::make_shared<std_msgs::msg::Float64>();
        RH_msg->data = RH_vel;
        RH_joint_velocity_pub_->publish(*RH_msg);

        // Publish RF joint velocity
        auto RF_msg = std::make_shared<std_msgs::msg::Float64>();
        RF_msg->data = RF_vel;
        RF_joint_velocity_pub_->publish(*RF_msg);

        // Publish LF joint velocity
        auto LF_msg = std::make_shared<std_msgs::msg::Float64>();
        LF_msg->data = LF_vel;
        LF_joint_velocity_pub_->publish(*LF_msg);
    }

    std::string LH_joint_;
    std::string RH_joint_;
    std::string RF_joint_;
    std::string LF_joint_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr LH_joint_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr RH_joint_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr RF_joint_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr LF_joint_velocity_pub_;
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
