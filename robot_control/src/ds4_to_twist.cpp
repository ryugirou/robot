#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#define scale 2

geometry_msgs::Twist cmd_vel;
void joy_callback(const sensor_msgs::Joy& joy_msg){
    cmd_vel.linear.x =scale * -joy_msg.axes[0];
    cmd_vel.linear.y =scale * joy_msg.axes[1];
    cmd_vel.angular.z=scale * joy_msg.buttons[1];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ds4_to_twist");
    ros::NodeHandle n;

    //publish
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    //subscriibe
    ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
