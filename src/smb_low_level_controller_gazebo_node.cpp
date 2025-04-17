#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LowLevelControllerGazebo : public rclcpp::Node
{
public:
    LowLevelControllerGazebo()
    : Node("smb_low_level_controller_gazebo_ros2"), last_time_(0.0)
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

        // Initialize subscribers and publishers
        wheel_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_joint_commands", 10,
            std::bind(&LowLevelControllerGazebo::wheelVelocityCallback, this, std::placeholders::_1));

        LH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/LH_WHEEL_JOINT_velocity_cmd", 10);
        RH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/RH_WHEEL_JOINT_velocity_cmd", 10);
        RF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/RF_WHEEL_JOINT_velocity_cmd", 10);
        LF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/LF_WHEEL_JOINT_velocity_cmd", 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);

        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("state_estimation", 10, std::bind(&DifferentialDriveController::odomCallback, this, std::placeholders::_1));
        // Initialize Gazebo transport node
        gz_node_ = std::make_shared<gz::transport::Node>();
    }

private:

    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     odom_ = *msg;
    //     last_odom_time_ = this->now();
    //     received_odom_ = true;
    // }

    void wheelVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
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
        auto LH_msg = std::make_shared<std_msgs::msg::Float64>();
        LH_msg->data = LH_vel;
        LH_joint_velocity_pub_->publish(*LH_msg);

        auto RH_msg = std::make_shared<std_msgs::msg::Float64>();
        RH_msg->data = RH_vel;
        RH_joint_velocity_pub_->publish(*RH_msg);

        auto RF_msg = std::make_shared<std_msgs::msg::Float64>();
        RF_msg->data = RF_vel;
        RF_joint_velocity_pub_->publish(*RF_msg);

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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    std::shared_ptr<gz::transport::Node> gz_node_;
    gz::msgs::Pose last_pose_;
    nav_msgs::msg::Odometry odom_;
    double last_time_{0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowLevelControllerGazebo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
