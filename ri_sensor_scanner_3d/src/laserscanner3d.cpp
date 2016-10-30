#include <ri_sensor_scanner_3d/laserscanner3d.h>

LaserScanner3d::LaserScanner3d():
    step_size(0.05),
    min_ang(-37),
    laser_tf("laser"),
    laser_rotation_center_tf("laser_rotation_center"),
    laser_base_tf("laser_base")
{
    node_handle_pub = ros::NodeHandle();
    sub_scanner_pose = node_handle_pub.subscribe("/lidar3D", 1, &LaserScanner3d::scannerAngleCallback, this);
    mirror_pose = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));

    node_handle_priv = ros::NodeHandle("~");
    node_handle_priv.param("step_size", step_size, step_size);
    std::cout << "Step size: " << step_size << std::endl;
}

/**
  * \brief Callback od położenia skanera.
  * Callback poza tym, że odbiera informacje z noda obrotnicy lidaru,
  * zamienia wartość kroku silnika na kąt w radianach i zapisuje go
  * do atrybutu klasy.
  */
void
LaserScanner3d::scannerAngleCallback(const ri_platform_msgs::LidarPose& msg)
{
    //std::cout << "Pose : " << msg.pose << std::endl;
    scanner_ang = (min_ang + step_size*(double)msg.pose)*3.14159/180;
    //std::cout << "Angle : " << scanner_ang << std::endl;
    sendTransformLaserBaseToLaser(scanner_ang, msg.header.stamp);
}

/** \brief Metoda broadcastująca transformację pomiędzy podstawą skanera
  * a punktem jego obrotu na podstawie kąta obrotu skanera wokół swojej
  * osi obrotu.
  * \param [in] angle - wartość kąta na osi obrotu skanera.
  */
void
LaserScanner3d::sendTransformLaserBaseToLaser(double angle, ros::Time time)
{
    Eigen::Quaternion<float> q;

    q = Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0, 1, 0));

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    br.sendTransform(tf::StampedTransform(transform, time, laser_base_tf, laser_rotation_center_tf));
}

/** \brief Metoda broadcastująca transformację pomiędzy punktem obrotu
  * skanera a środkiem zwierciadła.
  */
void
LaserScanner3d::sendTransformLaserToLaserMirror()
{
    br.sendTransform(tf::StampedTransform(mirror_pose, ros::Time::now(), laser_rotation_center_tf, laser_tf));
}
