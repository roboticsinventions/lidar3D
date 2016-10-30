#include <ros/ros.h>
#include <ri_platform_msgs/LidarPose.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_scanner_pose");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<ri_platform_msgs::LidarPose>("/lidar3D", 1);


    ros::Rate loop_rate(100);

    int iterator = 0;
    int positive_vel = 1;
    while(ros::ok())
    {
        sov_msgs::LidarPose msg;
        msg.pose = iterator;
        ROS_INFO("pose: %d", iterator);
        pub.publish(msg);

        iterator += positive_vel*10;

        if (iterator >= 900)
            positive_vel = -1;

        if (iterator <= 0)
            positive_vel = 1;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

