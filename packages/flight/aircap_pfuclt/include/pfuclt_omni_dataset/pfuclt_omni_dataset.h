// STL libraries
#include <vector>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

// ROS message definitions
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <aircap_pfuclt/BallData.h>

// Eigen libraries
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// Boost libraries
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>

// Auxiliary libraries
#include <pfuclt_omni_dataset/pfuclt_aux.h>
#include <pfuclt_omni_dataset/pfuclt_particles.h>
#include <pfuclt_omni_dataset/pfuclt_publisher.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <uav_msgs/uav_pose.h>

namespace pfuclt_omni_dataset
{

#define STATES_PER_ROBOT 7
#define HEURISTICS_THRESH_DEFAULT                                              \
  {                                                                            \
    2.5, 2.5, 2.5, 2.5, FLT_MAX, FLT_MAX, 3.5, 3.5, FLT_MAX, FLT_MAX           \
  }

// Forward declaration of classes
class Robot;

// useful typedefs
typedef boost::shared_ptr<Robot> Robot_ptr;

/**
 * @brief The RobotFactory class - Creates and keeps information on the
 * robot running the algorithm and its teammates. Is used as a middle-man
 * between all robots
 */
class RobotFactory
{

private:
  ros::NodeHandle& nh_;
  std::vector<Robot_ptr> robots_;

  /**
   * @brief areAllTeammatesActive - uses each robot's public methods to check if
   * they have started yet
   * @return true if every robot is active, false otherwise
   */
  bool areAllRobotsActive();

public:
  boost::shared_ptr<ParticleFilter> pf;

  RobotFactory(ros::NodeHandle& nh);

  /**
   * @brief tryInitializeParticles - checks if every robot is started, and if
   * so, will initiate the particle filter
   */
  void tryInitializeParticles();

};

/**
 * @brief The Robot class - Has the common variables and methods of all robots,
 * and is the base class of any specialized robots who may derive from it.
 * Teammates should be instances of this class
 */
class Robot
{
protected:
  RobotFactory* parent_;
  ParticleFilter* pf_;
  bool started_;
  ros::Time timeStarted_;
  ros::Subscriber cbOdometry, cbTarget, cbMeasurement;
  uint robotNumber_;
  Eigen::Isometry2d initPose_; // x y theta;

  /**
   * @brief startNow - starts the robot
   */
  void startNow();

public:
  /**
   * @brief Robot - constructor, creates a new Robot instance
   * @param nh - reference to the node handler object
   * @param parent - reference to the robot factory creating this object
   * @param initPose - initial (x,y) pose of the robot
   * @param pf - reference to the particle filter to be used for this robot
   * @param robotNumber - the assigned number in the team
   */
  Robot(ros::NodeHandle& nh, RobotFactory* parent,
        ParticleFilter* pf, uint robotNumber);

  /**
   * @brief odometryCallback - event-driven function which should be called when
   * new odometry data is received
   */
  void odometryCallback(const uav_msgs::uav_pose::ConstPtr&);

  /**
   * @brief targetCallBack - event-driven function which should be called when
   * new target prediction is received
   */
  void targetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    /**
   * @brief measurementCallBack - event-driven function which should be called when
   * new target data is received
   */
  void measurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);


  /**
   * @brief hasStarted
   * @return
   */
  bool hasStarted() { return started_; }; 

  uav_msgs::uav_pose lastReceivedOdometry_;
};

// end of namespace pfuclt_omni_dataset
}
