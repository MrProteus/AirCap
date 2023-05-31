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
float K1, K2; // coefficients for landmark observation covariance
float K3, K4, K5; // coefficients for target observation covariance
float ROB_HT; // Fixed height of the robots above ground in meters
std::vector<double> POS_INIT;

bool USE_CUSTOM_VALUES = false; // If set to true via the parameter server, the custom values will be used
std::vector<double> CUSTOM_PARTICLE_INIT; // Used to set custom values when initiating the particle filter set (will still be a uniform distribution)

bool DEBUG;
bool PUBLISH;

// for ease of access
ros::Time timeInit;

// Method definitions

RobotFactory::RobotFactory(ros::NodeHandle& nh) : nh_(nh)
{
  ParticleFilter::PFinitData initData(nh, MY_ID, NUM_TARGETS, STATES_PER_ROBOT,
                                      MAX_ROBOTS);

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
    // write initial pose from lastReceivedOdometry to POS_INIT
    const std::vector<double> selfPosInitVector = {(*it)->lastReceivedOdometry_.position.x, (*it)->lastReceivedOdometry_.position.y, (*it)->lastReceivedOdometry_.position.z, (*it)->lastReceivedOdometry_.orientation.x, (*it)->lastReceivedOdometry_.orientation.y, (*it)->lastReceivedOdometry_.orientation.z, (*it)->lastReceivedOdometry_.orientation.w};
    POS_INIT.insert(POS_INIT.end(),selfPosInitVector.begin(),selfPosInitVector.end());
    for (long unsigned int initPos = 0; initPos < selfPosInitVector.size(); initPos++) {
      CUSTOM_PARTICLE_INIT.insert(CUSTOM_PARTICLE_INIT.end(),{selfPosInitVector[initPos] - 0.02, selfPosInitVector[initPos] + 0.02});
    }
  }

  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::tryInitializeParticles: Robots are not active! Initializing now");
  
  // for (auto i: CUSTOM_PARTICLE_INIT){std::cout<<i<<std::endl;}
  
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
  if (lastReceivedOdometry_.header.seq == 0) {
    ROS_DEBUG("Robot::startNow (Robot %i): lastReceivedOdometry is empty, don't start yet",robotNumber_);
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
  cbOdometry = nh.subscribe<uav_msgs::uav_pose>(
      robotNamespace + "/pose", 10,
      boost::bind(&Robot::odometryCallback, this, _1));

  cbTarget = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      robotNamespace + "/target_tracker/pose", 10,
      boost::bind(&Robot::targetCallback, this, _1));

  cbMeasurement = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      robotNamespace + "/object_detections/projected_to_world", 10,
      boost::bind(&Robot::measurementCallback, this, _1));

  // sLandmark_ = nh.subscribe<read_omni_dataset::LRMLandmarksData>(
  //     robotNamespace + "/landmarkspositions", 10,
  //     boost::bind(&Robot::landmarkDataCallback, this, _1));

  ROS_INFO("Created robot OMNI%d", robotNumber + 1);
}

// void Robot::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
void Robot::odometryCallback(const uav_msgs::uav_pose::ConstPtr& pose)
{
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: Odometry callback");

  if (lastReceivedOdometry_.header.seq == 0) {
    ROS_DEBUG("Robot::odometryCallback (machine %i): lastReceivedOdometry is empty, fill with %f, %f, %f, %f, %f, %f, %f", 
        robotNumber_,pose->position.x, pose->position.y, pose->position.z, pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
    lastReceivedOdometry_ = *pose;
  }
  
  if (!started_) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: started_ = false, calling startNow()");
    startNow();
  }
  if (!pf_->isInitialized()) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::odometryCallback: isInitialized_ = false, call tryInitializeParticles()");
    parent_->tryInitializeParticles();
  }


  //  ROS_DEBUG("OMNI%d odometry at time %d = {%f;%f;%f}", robotNumber_ + 1,
  //            odometry->header.stamp.sec, odomStruct.x, odomStruct.y,
  //            odomStruct.theta);

  // Call the particle filter predict step for this robot
  pf_->predict(robotNumber_, pose, lastReceivedOdometry_);
}

void Robot::targetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  pf_->predictTarget(robotNumber_,msg);
}

// void Robot::measurementCallback(const read_omni_dataset::BallData::ConstPtr& target)
void Robot::measurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::measurementCallback: target callback");
  if (!started_) {
    ROS_DEBUG("pfuclt_omni_dataset/RobotFactory::measurementCallback: started_ = false, calling startNow()");
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
    obs.cov = msg->pose.covariance;

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


  pf_->fuseRobots();
  pf_->fuseTarget();
  pf_->resample();
  pf_->estimate();
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
  readParam<int>(nh, "NUM_TARGETS", NUM_TARGETS);


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
