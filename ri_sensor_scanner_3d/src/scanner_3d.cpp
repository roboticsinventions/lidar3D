#include <ri_sensor_scanner_3d/scanner_3d.h>
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset
#include <signal.h>

#include <eigen3/Eigen/Eigen>
#include <termios.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;
using namespace ri_sensor::scanner_3d;

Scanner_3d::Scanner_3d() :
    laser_tf("laser"),
    laser_rotation_center_tf("laser_rotation_center"),
    laser_base_tf("laser_base"),
    max_ang_(60),
    step_(14),
    working_mode(LidarWorkingMode::AUTO),
    start_position(0),
    goal_position(0),
    lidar_pose(0),
    in_move(false)
{
    nh_private.getParam("path", path_);
    nh_private.getParam("baudrate", baudrate_);
    nh_private.getParam("max_angle", max_ang_);
    nh_private.getParam("step", step_);
    nh_private.getParam("mode", mode);

    publisher_sonars = nh_private.advertise<std_msgs::Int32MultiArray>("scaner_sonars", 1);

    if (mode == "auto") working_mode = LidarWorkingMode::AUTO;
    else if (mode == "manual") working_mode = LidarWorkingMode::MANUAL;

    ROS_INFO("Lidar 3D is started in %s mode", mode.c_str());

    scanner_ang = ((double)(max_ang_*step_)/step_)*3.14159/180;

    mirror_pose = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));

    initSensor("Scanner 3D", SensorType::SCANNER_3D, getPath(), getBaudrate());

    boost::thread t(&Scanner_3d::recieve_new, this);
    t.join();
}

void
Scanner_3d::calculateScannerAngle(int current_position, ros::Time time)
{
    Eigen::Quaternion<float> q;

    scanner_ang = ((double)(max_ang_*step_-current_position)/step_)*(-1)*3.14159/180;

    q = Eigen::AngleAxis<float>(scanner_ang, Eigen::Vector3f(0, 1, 0));

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    br.sendTransform(tf::StampedTransform(transform, time, laser_base_tf, laser_tf));
}

void Scanner_3d::recieve_new()
{
    static bool first_time = true;

    if(first_time==true)
    {
        if (working_mode == LidarWorkingMode::AUTO)
            pushCommand(LidarCommandType::AON);
        else
            pushCommand(LidarCommandType::MOVE);
        first_time=false;
    }

    signal(SIGINT, Scanner_3d::signal_terminate_handler);
    while(!stop) {
        char c;

        if ( serial->connected() )
        {
            while (serial->read(&c))
            {
                incomming += c;
                if(c == '}')
                {
                    int command_id;
                    sscanf(incomming.c_str(),"{%d}", &command_id);

                    ROS_DEBUG("INCOMING: %s", incomming.c_str());
                    if (command_id==1)
                    {
                        sscanf(incomming.c_str(),"{%d,%d}", &command_id, &lidar_pose);
                        ROS_DEBUG("LIDAR POSE: %d", lidar_pose);
                        calculateScannerAngle(lidar_pose, ros::Time::now());
                    } else if (command_id==2)
                    {
                        int sonar1, sonar2, sonar3;
                        sscanf(incomming.c_str(),"{%d,%d,%d,%d}", &command_id, &sonar1, &sonar2, &sonar3);
                        ROS_DEBUG("SONARY: %d, %d, %d", sonar1, sonar2, sonar3);
                        std_msgs::Int32MultiArray array;
                        array.data.clear();
                        array.data.push_back(sonar1);
                        array.data.push_back(sonar2);
                        array.data.push_back(sonar3);
                        publisher_sonars.publish(array);
                    }
                    incomming.clear();
                }
            }
        } else {
            serial->open_port();
        }
    }
    sleep(3);
    pushCommand(LidarCommandType::AOFF);

}

void Scanner_3d::push()
{
    //pushCommand(LidarCommandType::MOVE);
}

Scanner_3d::~Scanner_3d() {
    pushCommand(LidarCommandType::AOFF);
}

void Scanner_3d::pushCommand(LidarCommandType type)
{
    std::string command = LidarCommandTypeToStrind(type);

    if ( serial->connected() )
    {
        switch (type) {
        case LidarCommandType::AON :
            serial->sendCommand(command);
            break;
        case LidarCommandType::AOFF :
            serial->sendCommand(command);
            break;
        case LidarCommandType::MOVE :
            moveTo(goal_position);
            break;
        default:
            break;
        }
    }

    //ROS_INFO("Sending '%s' command!", command.c_str());

}

void
Scanner_3d::getDynamicParams(::scanner_3d::scanner_3dConfig &config, uint32_t level)
{

//    if (config.mode == true && working_mode == LidarWorkingMode::MANUAL) {
//        working_mode = LidarWorkingMode::AUTO;
//        pushCommand(LidarCommandType::OFF);
//        pushCommand(LidarCommandType::ON);
//    }
//    else if (config.mode == false && working_mode == LidarWorkingMode::AUTO) {
//        working_mode = LidarWorkingMode::MANUAL;
//        pushCommand(LidarCommandType::OFF);
//        //in_move = true;
//        //goal_position = 0;
//    }

//    goal_position = config.goal_position;
//    int current_scaner_angle = (int)scanner_ang*180/M_PI;
//    if (goal_position!=current_scaner_angle && working_mode==LidarWorkingMode::MANUAL && in_move == false) {
//        pushCommand(LidarCommandType::MOVE);
//    }
}

void
Scanner_3d::moveTo(int position) {
    in_move = true;
    position = (max_ang_-position)*step_;
    std::stringstream ss;
    ss << LidarCommandTypeToStrind(LidarCommandType::MOVE) << " " << position;
    std::string str = ss.str();
    ROS_DEBUG("Moving to %d", position);
    serial->sendCommand(str);
}

std::string
Scanner_3d::LidarCommandTypeToStrind(LidarCommandType type)
{
    switch (type)
    {
    case LidarCommandType::AON:             { return "laon";    }
    case LidarCommandType::AOFF:            { return "laoff";    }
    case LidarCommandType::MOVE:            { return "move";    }
    case LidarCommandType::UP:              { return "up";      }
    case LidarCommandType::DOWN:            { return "down";    }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scanner_3d");
    Scanner_3d scanner;
    if (stop) {
        scanner.serial->close();
        exit(0);
    }
    dynamic_reconfigure::Server<scanner_3d::scanner_3dConfig> srv;
    dynamic_reconfigure::Server<scanner_3d::scanner_3dConfig>::CallbackType f;
    f = boost::bind	(&Scanner_3d::getDynamicParams, &scanner,_1, _2);
    srv.setCallback(f);
    ros::Rate r(10);
    ros::spin();
    return 0;
}

