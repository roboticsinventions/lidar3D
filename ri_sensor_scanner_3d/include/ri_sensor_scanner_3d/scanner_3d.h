/**
  * @brief
  * @author Michał Kowalski
  * @copyright Robotics Inventions Sp. z o.o.
  */
#ifndef SCANNER_3D_H
#define SCANNER_3D_H

#include <ros/ros.h>
#include <ri_sensor_base/RISensor.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <ri_sensor_scanner_3d/scanner_3dConfig.h>
#include <boost/thread.hpp>
namespace ri_sensor {

//! Obsługa lidara 3D
namespace scanner_3d {

enum class LidarCommandType {
    AON,         ///< Włączenie lidara w trybie auto
    AOFF,        ///< Wyłączenie lidara w trybie auto, zjazd do pozycji 0
    UP,         ///< Przesuń w dół
    DOWN,
    MOVE,
    PWM_CONF,
    HELP
};

enum class LidarWorkingMode {
    MANUAL,
    AUTO
};

int step = 14; // 14 kroków na stopień
static bool stop = false;

/**
 * @brief Klasa obsługująca nowy typ bujadełka do lidara, wraz z samym lidarem
 */
class Scanner_3d : public Sensor
{
public:
    /**
     * @brief Konstruktor klasy Scanner_3d
     */
    Scanner_3d();
    ~Scanner_3d();
    void recieve_new();
    void push();
    void pushCommand(LidarCommandType type);
    void calculateScannerAngle(int current_position, ros::Time time);
    static void mySigIntHandler(int sig);
    /**
     * @brief Odczyt wartośći parametrów dla dynamic reconfigure
     */
    void getDynamicParams(::scanner_3d::scanner_3dConfig &config, uint32_t level);
    void moveTo(int position);
    std::string LidarCommandTypeToStrind(LidarCommandType);
    static void
    signal_terminate_handler(int signum)
    {
        stop = true;
    }

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
    double step_;

    //! \brief Maksymalna wartość kąta odchylenia lidaru
    double max_ang_;

    //! \brief Pozycja zwierciadła skanera względem osi obrotu.
    tf::Transform mirror_pose;

    //! \brief Broadcaster transformacji.
    tf::TransformBroadcaster br;
    ros::Publisher publisher_sonars;
    int lidar_pose;
    std::string mode;
    LidarWorkingMode working_mode;
    int start_position, goal_position;

    bool in_move;
};

}
}
#endif // SCANNER_3D_H
