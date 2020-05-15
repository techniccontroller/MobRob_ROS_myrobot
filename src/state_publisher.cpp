#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);

  const double degree = M_PI / 180;

  // robot state
  double height = -0.1;
  double hinc = 0.01;
  double angle = 0;

  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  while (ros::ok())
  {
    // update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(7);
    joint_state.position.resize(7);
    joint_state.name[0] = "gripper_stand_to_gripper_joint";
    joint_state.position[0] = height;
    joint_state.name[1] = "right_front_wheel_joint";
    joint_state.position[1] = 0;
    joint_state.name[2] = "left_front_wheel_joint";
    joint_state.position[2] = 0;
    joint_state.name[3] = "right_back_wheel_joint";
    joint_state.position[3] = 0;
    joint_state.name[4] = "left_back_wheel_joint";
    joint_state.position[4] = 0;
    joint_state.name[5] = "gripper_to_right_finger_joint";
    joint_state.position[5] = -0.05;
    joint_state.name[6] = "gripper_to_left_finger_joint";
    joint_state.position[6] = 0.05;

    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = cos(angle) * 2;
    odom_trans.transform.translation.y = sin(angle) * 2;
    odom_trans.transform.translation.z = 0.0475;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle + M_PI / 2);

    // send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    // Create new robot state
    height += hinc;
    if (height > 0 || height < -0.11)
      hinc *= -1;
    angle += degree / 4;

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }

  return 0;
}