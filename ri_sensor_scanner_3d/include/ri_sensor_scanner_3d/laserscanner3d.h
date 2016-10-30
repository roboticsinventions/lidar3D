#ifndef LASERSCANNER3D_H
#define LASERSCANNER3D_H

#include <ros/ros.h>
#include <ri_platform_msgs/LidarPose.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

class LaserScanner3d
{
public:

    //! \brief Prywatny uchwyt do węzła ROSa
    ros::NodeHandle node_handle_priv;

    //! \brief Publiczny uchwyt do węzła ROSa
    ros::NodeHandle node_handle_pub;

    //! \brief Subscriber położenia skanera.
    ros::Subscriber sub_scanner_pose;

    //! \brief Konstruktor.
    LaserScanner3d();

    //! \brief Callback od położenia skanera.
    void scannerAngleCallback(const ri_platform_msgs::LidarPose& msg);

    /** \brief Metoda broadcastująca transformację pomiędzy podstawą skanera
      * a punktem jego obrotu na podstawie kąta obrotu skanera wokół swojej
      * osi obrotu.
      */
    void sendTransformLaserBaseToLaser(double angle, ros::Time time);

    /** \brief Metoda broadcastująca transformację pomiędzy punktem obrotu
      * skanera a środkiem zwierciadła.
      */
    void sendTransformLaserToLaserMirror();

private:
    //! \brief Identyfikator układu współrzędnych zwierciadła lasera.
    std::string laser_tf;

    //! \brief Identyfikator układu współrzędnych punktu obrotu lasera.
    std::string laser_rotation_center_tf;

    //! \brief Identyfikator układu współrzędnych podstawy lasera.
    std::string laser_base_tf;

    //! \brief Wartość położenia kątowego skanera w radianach.
    double scanner_ang;

    //! \brief Wartość kątowa w stopniach odpowiadająca jednemu kroku silnika.
    double step_size;

    //! \brief Minimalna wartość kąta odchylenia lidaru
    double min_ang;

    //! \brief Pozycja zwierciadła skanera względem osi obrotu.
    tf::Transform mirror_pose;

    //! \brief Broadcaster transformacji.
    tf::TransformBroadcaster br;
};

#endif // LASERSCANNER3D_H
