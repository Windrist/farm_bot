#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

float steering_angle_;
float linear_velocity_x_;
float linear_velocity_y_;
float angular_velocity_z_;
ros::Time last_vel_time_;
float vel_dt_;
float x_pos_;
float y_pos_;
float heading_;

void odomCallback(const nav_msgs::Odometry& odom);
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
void imuCallback(const sensor_msgs::Imu& imu);

ros::Publisher odom_publisher_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh_;
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber encoder_subscriber_ = nh_.subscribe("raw_encoder", 50, &odomCallback);
    ros::Subscriber pose_subscriber_ = nh_.subscribe("hedge_pose", 50, &poseCallback);
    ros::Subscriber imu_subscriber_ = nh_.subscribe("/imu/data", 50, &imuCallback);
    ros::spin();
    return 0;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    x_pos_ = pose.pose.pose.position.x;
    y_pos_ = pose.pose.pose.position.y;
}

void imuCallback(const sensor_msgs::Imu& imu)
{
    heading_ = tf::getYaw(imu.orientation);
}

void odomCallback(const nav_msgs::Odometry& raw_odom)
{
    ros::Time current_time = ros::Time::now();

    linear_velocity_x_ = raw_odom.twist.twist.linear.x;
    linear_velocity_y_ = raw_odom.twist.twist.linear.y;
    angular_velocity_z_ = raw_odom.twist.twist.angular.z;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    // odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_.publish(odom);
}
