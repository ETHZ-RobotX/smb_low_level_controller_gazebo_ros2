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
    : Node("smb_low_level_controller_gazebo"), last_time_(0.0)
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

        scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10,
            std::bind(&LowLevelControllerGazebo::scanHandler, this, std::placeholders::_1));

        LH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/LH_WHEEL_JOINT/cmd_vel", 10);
        RH_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/RH_WHEEL_JOINT/cmd_vel", 10);
        RF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/RF_WHEEL_JOINT/cmd_vel", 10);
        LF_joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/smb/joint/LF_WHEEL_JOINT/cmd_vel", 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);
        registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 10);

        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("state_estimation", 10, std::bind(&DifferentialDriveController::odomCallback, this, std::placeholders::_1));
        scanData = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // Initialize Gazebo transport node
        gz_node_ = std::make_shared<gz::transport::Node>();

        // Subscribe to dynamic pose topic
        if (!gz_node_->Subscribe("/world/empty/dynamic_pose/info",
                                  &LowLevelControllerGazebo::dynamicPoseCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to /world/empty/dynamic_pose/info");
        }
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

    void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
    {
        // if (!systemInited) {
        //     systemInitCount++;
        //     if (systemInitCount > systemDelay) {
        //         systemInited = true;
        //     }
        //     return;
        // }

        // Assume odometry is directly available from /state_estimation
        // float vehicleRecX = odom_.position.x;
        // float vehicleRecY = odom_.position.y;
        // float vehicleRecZ = odom_.position.z;

        float vehicleRecX = odom_.pose.pose.position.x;
        float vehicleRecY = odom_.pose.pose.position.y;
        float vehicleRecZ = odom_.pose.pose.position.z;

        // Transform point cloud data
        scanData->clear();
        pcl::fromROSMsg(*scanIn, *scanData);
        std::vector<int> scanInd;
        pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);
        // pcl::removeNaNFromPointCloud(*scanData, *scanData, std::vector<int>());

        int scanDataSize = scanData->points.size();
        for (int i = 0; i < scanDataSize; i++)
        {
            // Direct translation using vehicle odometry
            scanData->points[i].x += vehicleRecX;
            scanData->points[i].y += vehicleRecY;
            scanData->points[i].z += vehicleRecZ;
        }

        // ROS_INFO("Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);
        RCLCPP_ERROR(this->get_logger(), "Received scan data with %d points", scanDataSize);

        // Publish registered scan message
        sensor_msgs::msg::PointCloud2 scanData2;
        pcl::toROSMsg(*scanData, scanData2);
        scanData2.header.stamp = scanIn->header.stamp; // Use the original scan timestamp
        scanData2.header.frame_id = "map";
        registered_scan_pub_->publish(scanData2);
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

    void dynamicPoseCallback(const gz::msgs::Pose_V &msg)
    {
        for (const auto &pose : msg.pose())
        {
            if (pose.name() == "smb")
            {
                auto current_time = this->get_clock()->now();
                double current_time_secs = current_time.seconds();

                // Skip the first callback
                if (last_time_ == 0.0)
                {
                    last_time_ = current_time_secs;
                    last_pose_ = pose;
                    return;
                }

                double dt = current_time_secs - last_time_;

                // printf("dt: %f\n", dt);
                RCLCPP_INFO(this->get_logger(), "dt: %f", dt);

                // Calculate velocities
                double dx = pose.position().x() - last_pose_.position().x();
                double dy = pose.position().y() - last_pose_.position().y();
                double dz = pose.position().z() - last_pose_.position().z();

                double vx = dx / dt;
                double vy = dy / dt;
                double vz = dz / dt;

                tf2::Quaternion current_orientation(
                    pose.orientation().x(),
                    pose.orientation().y(),
                    pose.orientation().z(),
                    pose.orientation().w());
                tf2::Quaternion last_orientation(
                    last_pose_.orientation().x(),
                    last_pose_.orientation().y(),
                    last_pose_.orientation().z(),
                    last_pose_.orientation().w());
                tf2::Quaternion delta_orientation = current_orientation * last_orientation.inverse();
                tf2::Matrix3x3 delta_rotation(delta_orientation);
                double delta_roll, delta_pitch, delta_yaw;
                delta_rotation.getRPY(delta_roll, delta_pitch, delta_yaw);

                double wx = delta_roll / dt;
                double wy = delta_pitch / dt;
                double wz = delta_yaw / dt;

                // Publish odometry
                auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
                odom_msg->header.stamp = current_time;
                odom_msg->header.frame_id = "odom";

                odom_msg->pose.pose.position.x = pose.position().x();
                odom_msg->pose.pose.position.y = pose.position().y();
                odom_msg->pose.pose.position.z = pose.position().z();
                odom_msg->pose.pose.orientation.x = pose.orientation().x();
                odom_msg->pose.pose.orientation.y = pose.orientation().y();
                odom_msg->pose.pose.orientation.z = pose.orientation().z();
                odom_msg->pose.pose.orientation.w = pose.orientation().w();

                // Linear velocity in body frame, using quaterion to rotate the velocity from world frame to body frame 

                tf2::Vector3 global_velocity(vx, vy, vz);
                tf2::Quaternion orientation(
                    pose.orientation().x(),
                    pose.orientation().y(),
                    pose.orientation().z(),
                    pose.orientation().w());

                // Convert quaternion to transform
                tf2::Transform transform;
                transform.setRotation(orientation);

                // Use the inverse rotation to transform velocity to the body frame
                tf2::Vector3 local_velocity = transform.getBasis().transpose() * global_velocity;

                vx = local_velocity.x();
                vy = local_velocity.y();
                vz = local_velocity.z();

                odom_msg->twist.twist.linear.x = vx;
                odom_msg->twist.twist.linear.y = vy;
                odom_msg->twist.twist.linear.z = vz;
                odom_msg->twist.twist.angular.x = wx;
                odom_msg->twist.twist.angular.y = wy;
                odom_msg->twist.twist.angular.z = wz;

                odom_ = *odom_msg;

                odom_pub_->publish(*odom_msg);

                RCLCPP_INFO(this->get_logger(),
                            "Odometry: Pos [%f, %f, %f], Lin Vel [%f, %f, %f], Ang Vel [%f, %f, %f]",
                            pose.position().x(), pose.position().y(), pose.position().z(),
                            vx, vy, vz, wx, wy, wz);

                last_pose_ = pose;
                last_time_ = current_time_secs;
            }
        }
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr scanData;


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
