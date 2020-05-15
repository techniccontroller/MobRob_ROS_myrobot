#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <myrobot_model/Pose.h>


class OdomSubscribeAndPublish
{
private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber pose_sub;
    tf::TransformBroadcaster odom_broadcaster;
    double x;
    double y;
    double th;
    double vx;
    double vy;
    double vth;

public:
    OdomSubscribeAndPublish()
    {
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        pose_sub = n.subscribe("chatter", 1000, &OdomSubscribeAndPublish::callback, this);
        x = 0.0;
        y = 0.0;
        th = 0.0;
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;
    }


    void callback(const myrobot_model::Pose& pose)
    {
        ros::Time current_time = ros::Time::now();

        x = pose.x;
        y = pose.y;
        th = pose.theta;
        vx = pose.vx;
        vy = pose.vy;
        vth = pose.vth;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
    }

};


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_converter");

    OdomSubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}