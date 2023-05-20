#include <pfuclt_omni_dataset/pfuclt_aux.h>
#include <pfuclt_omni_dataset/pfuclt_particles.h>
#include <pfuclt_omni_dataset/pfuclt_omni_dataset.h>
#include <tf2/utils.h>

#define ROS_TDIFF(t) (t.toSec() - timeInit.toSec())

namespace pfuclt_omni_dataset
{
int MY_ID;
int MAX_ROBOTS;
int NUM_TARGETS;
int NUM_LANDMARKS;
std::vector<bool> PLAYING_ROBOTS;
float K1, K2; // coefficients for landmark observation covariance
float K3, K4, K5; // coefficients for target observation covariance
float ROB_HT; // Fixed height of the robots above ground in meters
std::vector<double> POS_INIT;

bool USE_CUSTOM_VALUES = false; // If set to true via the parameter server, the custom values will be used
std::vector<double> CUSTOM_PARTICLE_INIT; // Used to set custom values when initiating the particle filter set (will still be a uniform distribution)

bool DEBUG;
bool PUBLISH;

// for ease of access
std::vector<Landmark> landmarks;
ros::Time timeInit;

// Method definitions

RobotFactory::RobotFactory(ros::NodeHandle& nh) : nh_(nh)
{
  ParticleFilter::PFinitData initData(nh, MY_ID, NUM_TARGETS, STATES_PER_ROBOT,
                                      MAX_ROBOTS, PLAYING_ROBOTS);

  if (PUBLISH)
    pf = boost::shared_ptr<PFPublisher>(
        new PFPublisher(initData, PFPublisher::PublishData(ROB_HT)));
  else
    pf = boost::shared_ptr<ParticleFilter>(new ParticleFilter(initData));

  timeInit = ros::Time::now();
  ROS_INFO("Init time set to %f", timeInit.toSec());

  for (int rn = 0; rn < MAX_ROBOTS; rn++)
  {
    robots_.push_back(
      Robot_ptr(new Robot(nh_, this, pf->getPFReference(), rn)));
  }
}

void RobotFactory::tryInitializeParticles()
{
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::tryInitializeParticles: Trying to initialize particles");
  if (!areAllRobotsActive()){
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::tryInitializeParticles: Robots are active");
    return;
  }
  for (std::vector<Robot_ptr>::iterator it = robots_.begin();
       it != robots_.end(); ++it)
  {
    POS_INIT.insert(POS_INIT.end(),(*it)->selfPosInit.begin(),(*it)->selfPosInit.end());
  }
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::tryInitializeParticles: Robots are not active! Initializing now");
  
  pf->init(CUSTOM_PARTICLE_INIT, POS_INIT);
}



bool RobotFactory::areAllRobotsActive()
{
  for (std::vector<Robot_ptr>::iterator it = robots_.begin();
       it != robots_.end(); ++it)
  {
    if (false == (*it)->hasStarted())
      return false;
  }
  return true;
}

void Robot::startNow()
{
  if (selfPosInit.empty()) {
    ROS_WARN("Robot::startNow (Robot %i): selfPosInit is empty, don't start yet",robotNumber_);
    return;
  }
  timeStarted_ = ros::Time::now();
  started_ = true;
  ROS_INFO("OMNI%d has started %.2fs after the initial time", robotNumber_ + 1,
           ROS_TDIFF(timeStarted_));
}

Robot::Robot(ros::NodeHandle& nh, RobotFactory* parent, ParticleFilter* pf,
             uint robotNumber)
    : parent_(parent), pf_(pf), started_(false), robotNumber_(robotNumber)
{
  std::string robotNamespace("/machine_" +
                             boost::lexical_cast<std::string>(robotNumber + 1));

  // Subscribe to topics
  sOdom_ = nh.subscribe<uav_msgs::uav_pose>(
      robotNamespace + "/pf/odometry", 10,
      boost::bind(&Robot::odometryCallback, this, _1));

  sBall_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      robotNamespace + "/pf/orangeBall3DPosition", 10,
      boost::bind(&Robot::targetCallback, this, _1));

  // sLandmark_ = nh.subscribe<read_omni_dataset::LRMLandmarksData>(
  //     robotNamespace + "/landmarkspositions", 10,
  //     boost::bind(&Robot::landmarkDataCallback, this, _1));

  ROS_INFO("Created robot OMNI%d", robotNumber + 1);
}

// void Robot::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
void Robot::odometryCallback(const uav_msgs::uav_pose::ConstPtr& pose)
{
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: Odometry callback");

  if (selfPosInit.empty()) {
    ROS_WARN("Robot::odometryCallback (machine %i): selfPosInit is empty, fill with %f, %f, %f", robotNumber_,pose->position.x, pose->position.y, tf2::getYaw(pose->orientation));
    selfPosInit = {pose->position.x, pose->position.y, tf2::getYaw(pose->orientation)};
  }
  
  if (!started_) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: started_ = false, calling startNow()");
    startNow();
  }
  if (!pf_->isInitialized()) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: isInitialized_ = false, call tryInitializeParticles()");
    parent_->tryInitializeParticles();
  }

  Odometry odomStruct;
  odomStruct.x = pose->position.x;
  odomStruct.y = pose->position.y;
  odomStruct.theta = tf2::getYaw(pose->orientation);

  //  ROS_DEBUG("OMNI%d odometry at time %d = {%f;%f;%f}", robotNumber_ + 1,
  //            odometry->header.stamp.sec, odomStruct.x, odomStruct.y,
  //            odomStruct.theta);

  // Call the particle filter predict step for this robot
  pf_->predict(robotNumber_, odomStruct, pose->header.stamp);
}

// void Robot::targetCallback(const read_omni_dataset::BallData::ConstPtr& target)
void Robot::targetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::targetCallback: target callback");
  if (!started_) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::targetCallback: started_ = false, calling startNow()");
    startNow();
  }
  // If needed, modify here to if(true) to go over the target occlusion from the dataset
  if (true)
  {
    // ROS_DEBUG("OMNI%d ball data at time %d", robotNumber_ + 1,
    //          target->header.stamp.sec);

    TargetObservation obs;

    obs.found = true;
    obs.x = msg->pose.pose.position.x;
    obs.y = msg->pose.pose.position.y;
    obs.z= msg->pose.pose.position.z;
    obs.d = Eigen::Vector2d(obs.x, obs.y).norm();
    obs.r = Eigen::Vector3d(obs.x, obs.y, obs.z).norm();
    obs.phi = atan2(obs.y, obs.x);

    // Auxiliary
    const double cos2p = pow(cos(obs.phi), 2);
    const double sin2p = pow(sin(obs.phi), 2);
    const double d2 = pow(obs.d, 2);
    const double r2 = pow(obs.r, 2);

    // 3D Model
    static const float ballRadius = 0.1;
    static const float ballr2 = pow(ballRadius, 2);
    obs.covDD = K3 * (r2 * sin2p / (2 * ballr2)) +
                K4 * (r2 * sin2p / (2 * (r2 - ballr2))) +
                K3 * K4 * (r2 * cos2p / (4 * ballr2 * (r2 - ballr2)));
    obs.covPP = K5 / (r2 - ballr2 * sin2p);

    // 2D Model
    //    obs.covDD = (double)(1 / target->mismatchFactor) *
    //                (K3 * obs.d + K4 * (obs.d * obs.d));

    //    obs.covPP = K5 * (1 / (obs.d + 1));

    obs.covXX =
        cos2p * obs.covDD + sin2p * (d2 * obs.covPP + obs.covDD * obs.covPP);
    obs.covYY =
        sin2p * obs.covDD + cos2p * (d2 * obs.covPP + obs.covDD * obs.covPP);

    // Save this observation
    pf_->saveTargetObservation(robotNumber_, obs, msg->header.stamp);
  }
  else
  {
    //    ROS_DEBUG("OMNI%d didn't find the ball at time %d", robotNumber_ + 1,
    //              target->header.stamp.sec);

    pf_->saveTargetObservation(robotNumber_, false);
  }

  pf_->saveAllTargetMeasurementsDone(robotNumber_);

  // If this is the "self robot", update the iteration time
  if (MY_ID == (int)robotNumber_ + 1)
    pf_->updateTargetIterationTime(msg->header.stamp);
}



// end of namespace pfuclt_omni_dataset
}

int main(int argc, char* argv[])
{
  using namespace pfuclt_omni_dataset;

  // Parse input parameters
  // TODO Consider using a library for this
  std::cout << "Usage: pfuclt_omni_dataset --debug [true|FALSE] --publish [TRUE|false]" << std::endl;
  if (argc > 2)
  {
      if (!strcmp(argv[2], "true"))
      {
          if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                             ros::console::levels::Debug))
          {
              ros::console::notifyLoggerLevelsChanged();
          }

          DEBUG = true;
      }
      else
          DEBUG = false;
  }
  else
      DEBUG = false;

  if (argc > 4)
  {
      if (!strcmp(argv[4], "true"))
      {
          PUBLISH = true;
      }
      else
          PUBLISH = false;
  }
  else
      PUBLISH = false;

  ROS_INFO_STREAM("DEBUG set to " << std::boolalpha << DEBUG << " and PUBLISH set to " << std::boolalpha << PUBLISH);

  ros::init(argc, argv, "aircap_pfuclt");
  ros::NodeHandle nh("~");

  // read parameters from param server
  readParam<int>(nh, "MAX_ROBOTS", MAX_ROBOTS);
  readParam<float>(nh, "ROB_HT", ROB_HT);
  readParam<int>(nh, "NUM_TARGETS", NUM_TARGETS);


  PLAYING_ROBOTS.resize(MAX_ROBOTS);
  PLAYING_ROBOTS.assign(MAX_ROBOTS,1);
  readParam<bool>(nh, "USE_CUSTOM_VALUES", USE_CUSTOM_VALUES);
  readParam<int>(nh, "MY_ID", MY_ID);



  ROS_INFO("Waiting for /clock");
  ros::Time::waitForValid();
  ROS_INFO("/clock message received");

  pfuclt_omni_dataset::RobotFactory Factory(nh);


  // Factory.initializeFixedLandmarks();

  ros::spin();
  return EXIT_SUCCESS;
}
