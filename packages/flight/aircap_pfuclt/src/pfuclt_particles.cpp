#include <pfuclt_omni_dataset/pfuclt_particles.h>
#include <boost/foreach.hpp>
#include <angles/angles.h>
#include <pfuclt_omni_dataset/averaging_quaternions.h>
#include <uav_msgs/uav_pose.h>
#include <target_tracker_distributed_kf/DistributedKF3D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#define RECONFIGURE_ALPHAS true

namespace pfuclt_omni_dataset
{

ParticleFilter::ParticleFilter(struct PFinitData& data)
    : dynamicVariables_(data.nh, data.nRobots),
      nh_(data.nh), nParticles_((uint)dynamicVariables_.nParticles), mainRobotID_(data.mainRobotID - 1),
      nTargets_(data.nTargets), nStatesPerRobot_(data.statesPerRobot), nRobots_(data.nRobots),
      nSubParticleSets_(data.nTargets * STATES_PER_TARGET + data.nRobots * data.statesPerRobot + 1),
      particles_(nSubParticleSets_, subparticles_t(nParticles_)),
      weightComponents_(data.nRobots, subparticles_t(nParticles_, 0.0)),
      seed_(time(0)), initialized_(false),
      bufTargetObservations_(data.nRobots),
      durationSum(ros::WallDuration(0)),
      numberIterations(0),
      state_(data.statesPerRobot, data.nRobots),
      iteration_oss(new std::ostringstream("")),
      O_TARGET(data.nRobots * data.statesPerRobot),
      O_WEIGHT(nSubParticleSets_ - 1)
{
  ROS_WARN("nParticles_: %i STATES_PER_TARGET: %i statesPerRobot: %i nSubParticleSets_: %i", nParticles_, STATES_PER_TARGET, data.statesPerRobot, nSubParticleSets_);
  ROS_INFO("Created particle filter with dimensions %d, %d",
           (int)particles_.size(), (int)particles_[0].size());

  // Bind dynamic reconfigure callback
  dynamic_reconfigure::Server<DynamicConfig>::CallbackType
      callback;
  callback = boost::bind(&ParticleFilter::dynamicReconfigureCallback, this, _1);
  dynamicServer_.setCallback(callback);
}

void ParticleFilter::dynamicReconfigureCallback(DynamicConfig& config)
{
  // Skip first callback which is done automatically for some reason
  if (dynamicVariables_.firstCallback)
  {
    dynamicVariables_.firstCallback = false;
    return;
  }

  if (!initialized_)
    return;

  ROS_INFO("Dynamic Reconfigure Callback:\n\tparticles = "
           "%d\n\tresampling_percentage_to_keep = "
           "%f\n\tpredict_model_stddev = "
           "%f\n\tOMNI1_alpha=%s\n\tOMNI3_alpha=%s\n\tOMNI4_alpha=%s\n\tOMNI5_"
           "alpha=%s",
           config.particles,
           config.groups.resampling.percentage_to_keep,
           config.groups.target.predict_model_stddev,
           config.groups.alphas.OMNI1_alpha.c_str(),
           config.groups.alphas.OMNI3_alpha.c_str(),
           config.groups.alphas.OMNI4_alpha.c_str(),
           config.groups.alphas.OMNI5_alpha.c_str());

  // Resize particles and re-initialize the pf if value changed
  if (dynamicVariables_.nParticles != config.particles)
  {
    ROS_INFO("Resizing particles to %d and re-initializing the pf",
             config.particles);

    resize_particles(config.particles);
    nParticles_ = config.particles;
  }

  // Update with desired values
  dynamicVariables_.nParticles = config.particles;
  dynamicVariables_.resamplingPercentageToKeep =
      config.groups.resampling.percentage_to_keep;
  dynamicVariables_.targetRandStddev =
      config.groups.target.predict_model_stddev;

// Alpha values updated only if using the original dataset
#ifdef RECONFIGURE_ALPHAS
  dynamicVariables_.fill_alpha(0, config.groups.alphas.OMNI1_alpha);
  dynamicVariables_.fill_alpha(2, config.groups.alphas.OMNI3_alpha);
  dynamicVariables_.fill_alpha(3, config.groups.alphas.OMNI4_alpha);
  dynamicVariables_.fill_alpha(4, config.groups.alphas.OMNI5_alpha);
#endif
}

void ParticleFilter::spreadTargetParticlesSphere(float particlesRatio,
                                                 pdata_t center[3],
                                                 float radius)
{
  uint particlesToSpread = nParticles_ * particlesRatio;

  boost::random::uniform_real_distribution<> dist(-radius, radius);

  for (uint p = 0; p < particlesToSpread; ++p)
  {
    for (uint s = 0; s < STATES_PER_TARGET; ++s)
      particles_[O_TARGET + s][p] = center[s] + dist(seed_);
  }
}

void ParticleFilter::predictTarget(const uint robotNumber, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  *iteration_oss << "predictTarget() -> ";

  using namespace boost::random;

  int robot_offset = robotNumber * nStatesPerRobot_;

  for (uint p = 0; p < nParticles_; p++)
  {
    if (PREDICT_TARGET) {
      bool isSelf = (robotNumber == mainRobotID_);

      geometry_msgs::PoseWithCovarianceStamped stateFromParticle; 
      stateFromParticle.pose.pose.position.x = particles_[robot_offset + O_TX][p];
      stateFromParticle.pose.pose.position.y = particles_[robot_offset + O_TY][p];
      stateFromParticle.pose.pose.position.z = particles_[robot_offset + O_TZ][p];

      target_tracker_distributed_kf::CacheElement previousState((int) nStatesPerRobot_,stateFromParticle, isSelf, robotNumber);
      target_tracker_distributed_kf::CacheElement prediction(pose->header, nStatesPerRobot_, isSelf, robotNumber);

      // predict with function from Kalman Filter
      if (!predictKF(previousState, prediction))
        return;
      
      // insert predicted state into particle
      particles_[robot_offset + O_TX][p] = prediction.state(0);
      particles_[robot_offset + O_TY][p] = prediction.state(1);
      particles_[robot_offset + O_TZ][p] = prediction.state(2);
    }
    else {
      particles_[robot_offset + O_TX][p] = pose->pose.pose.position.x;
      particles_[robot_offset + O_TY][p] = pose->pose.pose.position.y;
      particles_[robot_offset + O_TZ][p] = pose->pose.pose.position.z;
    }
  }
}

void ParticleFilter::fuseRobots()
{
  *iteration_oss << "fuseRobots() -> ";

  // Save the latest observation time to be used when publishing
  savedLatestObservationTime_ = latestObservationTime_;

  // Keeps track of number of landmarks seen for each robot
  std::vector<uint> landmarksSeen(nRobots_, 0);

  // Will track the probability propagation based on the landmark observations
  // for each robot
  std::vector<subparticles_t> probabilities(nRobots_,
                                            subparticles_t(nParticles_, 1.0));

  // For every robot
  for (uint r = 0; r < nRobots_; ++r)
  {

    // Index offset for this robot in the particles vector
    uint o_robot = r * nStatesPerRobot_;

    // // For every landmark
    // for (uint l = 0; l < nLandmarks_; ++l)
    // {
    //   // If landmark not seen, skip
    //   if (false == bufLandmarkObservations_[r][l].found)
    //     continue;
    //   else
    //     ++(landmarksSeen[r]);

    //   // Reference to the observation for easier access
    //   LandmarkObservation& m = bufLandmarkObservations_[r][l];

    //   // Observation in robot frame
    //   Eigen::Matrix<pdata_t, 2, 1> Zrobot(m.x, m.y);

    //   // Landmark in global frame
    //   Eigen::Matrix<pdata_t, 2, 1> LMglobal(landmarksMap_[l].x,
    //                                         landmarksMap_[l].y);

#pragma omp parallel for
      for (uint p = 0; p < nParticles_; ++p)
      {

        // // Robot pose <=> frame
        // Eigen::Rotation2D<pdata_t> Rrobot(-particles_[o_robot + O_THETA][p]);
        // Eigen::Matrix<pdata_t, 2, 1> Srobot(particles_[o_robot + O_X][p],
        //                                     particles_[o_robot + O_Y][p]);

        // // Landmark to robot frame
        // Eigen::Matrix<pdata_t, 2, 1> LMrobot = Rrobot * (LMglobal - Srobot);

        // // Error in observation
        // Eigen::Matrix<pdata_t, 2, 1> Zerr = LMrobot - Zrobot;

        // // The values of interest to the particle weights
        // // Note: using Eigen wasn't of particular interest here since it does
        // // not allow for transposing a non-dynamic matrix
        // float expArg = -0.5 * (Zerr(O_X) * Zerr(O_X) / m.covXX +
        //                        Zerr(O_Y) * Zerr(O_Y) / m.covYY);
        float expArg = 0;
        float detValue = 1.0; // pow((2 * M_PI * m.covXX * m.covYY), -0.5);

        /*
        ROS_DEBUG_COND(
            p == 0,
            "OMNI%d's particle 0 is at {%f;%f;%f}, sees landmark %d with "
            "certainty %f%%, and error {%f;%f}",
            r + 1, particles_[o_robot + O_X][p], particles_[o_robot + O_Y][p],
            particles_[o_robot + O_THETA][p], l, 100 * (detValue * exp(expArg)),
            Zerr(0), Zerr(1));
        */

        // Update weight component for this robot and particular particle
        probabilities[r][p] *= detValue * exp(expArg);
      }
    // }
  }

  // Reset weights, later will be multiplied by weightComponents of each robot
  resetWeights(1.0);

  // Duplicate particles
  particles_t dupParticles(particles_);

  for (uint r = 0; r < nRobots_; ++r)
  {
    // Check that at least one landmark was seen, if not send warning
    // If seen use probabilities vector, if not keep using the previous
    // weightComponents for this robot
    // if (0 == landmarksSeen[r])
    // {
    //   ROS_WARN("In this iteration, OMNI%d didn't see any landmarks", r + 1);

    //   // weightComponent stays from previous iteration
    // }

    // else
    // {
      weightComponents_[r] = probabilities[r];
    // }

    // Index offset for this robot in the particles vector
    uint o_robot = r * nStatesPerRobot_;

    // Create a vector of indexes according to a descending order of the weights
    // components of robot r
    std::vector<uint> sorted = order_index<pdata_t>(weightComponents_[r], DESC);

    // For every particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Re-order the particle subsets of this robot
      uint sort_index = sorted[p];

      // Copy this sub-particle set from dupParticles' sort_index particle
      copyParticle(particles_, dupParticles, p, sort_index, o_robot,
                   o_robot + nStatesPerRobot_ - 1);

      // Update the particle weight (will get multiplied nRobots times and get a
      // lower value)
      particles_[O_WEIGHT][p] *= weightComponents_[r][sort_index];
    }
  }
}

void ParticleFilter::fuseTarget()
{
  *iteration_oss << "fuseTarget() -> ";

  // If ball not seen by any robot, just skip all of this
  bool ballSeen = false;
  for (std::vector<TargetObservation>::iterator it =
           bufTargetObservations_.begin();
       it != bufTargetObservations_.end(); ++it)
  {
    if (it->found)
    {
      ballSeen = true;
      break;
    }
  }

  // Update ball state
  state_.target.seen = ballSeen;

  // exit if ball not seen by any robot
  if (!ballSeen)
  {
    *iteration_oss << "Ball not seen ->";
    return;
  }
  // If program is here, at least one robot saw the ball

  // Instance variables to be worked in the loops
  pdata_t maxTargetSubParticleWeight, totalWeight;
  uint m, p, mStar, r, o_robot;
  float expArg, detValue, Z[3], Zcap[3], Z_Zcap[3];
  TargetObservation* obs;

  // For every particle m in the particle set [1:M]
  for (m = 0; m < nParticles_; ++m)
  {
    // Keep track of the maximum contributed weight and that particle's index
    maxTargetSubParticleWeight = -1.0f;
    mStar = m;

// Find the particle m* in the set [m:M] for which the weight contribution
// by the target subparticle to the full weight is maximum
#pragma omp parallel for private(p, r, o_robot, obs, expArg, detValue, Z,      \
                                 Zcap, Z_Zcap)
    for (p = m; p < nParticles_; ++p)
    {
      // Vector with probabilities for each robot, starting at 0.0 in case the
      // robot hasn't seen the ball
      std::vector<pdata_t> probabilities(nRobots_, 0.0);

      // Observations of the target by all robots
      for (r = 0; r < nRobots_; ++r)
      {
        if (false == bufTargetObservations_[r].found)
          continue;

        // Usefull variables
        obs = &bufTargetObservations_[r];
        o_robot = r * nStatesPerRobot_;

        // Observation model
        Z[0] = obs->x;
        Z[1] = obs->y;
        Z[2] = obs->z;
        Zcap[0] = particles_[O_TARGET + O_TX][p];
        Zcap[1] = particles_[O_TARGET + O_TY][p];
        Zcap[2] = particles_[O_TARGET + O_TZ][p];
        Z_Zcap[0] = Z[0] - Zcap[0];
        Z_Zcap[1] = Z[1] - Zcap[1];
        Z_Zcap[2] = Z[2] - Zcap[2];

        expArg = -0.5 * (Z_Zcap[0] * Z_Zcap[0] / obs->cov[0 * 6 + 0] +
                         Z_Zcap[1] * Z_Zcap[1] / obs->cov[1 * 6 + 1] +
                         Z_Zcap[2] * Z_Zcap[2] / obs->cov[2 * 6 + 2]);
        detValue =
            1.0; // pow((2 * M_PI * obs->covXX * obs->covYY * 10.0), -0.5);

        // Probability value for this robot and this particle
        probabilities[r] = detValue * exp(expArg);
      }

      // Total weight contributed by this particle
      totalWeight =
          std::accumulate(probabilities.begin(), probabilities.end(), 0.0);

      // If the weight is the maximum as of now, update the maximum and set
      // particle p as mStar
      if (totalWeight > maxTargetSubParticleWeight)
      {
// Swap particle m with m* so that the most relevant (in terms of
// weight)
// target subparticle is at the lowest indexes
#pragma omp critical
        {
          if (totalWeight > maxTargetSubParticleWeight)
          {
            maxTargetSubParticleWeight = totalWeight;
            mStar = p;
          }
        }
      }
    }

    // Particle m* has been found, let's swap the subparticles
    for (uint i = 0; i < STATES_PER_TARGET; ++i)
      std::swap(particles_[O_TARGET + i][m], particles_[O_TARGET + i][mStar]);

    // Update the weight of this particle
    particles_[O_WEIGHT][m] *= maxTargetSubParticleWeight;

    // The target subparticles are now reordered according to their weight
    // contribution

    // printWeights("After fuseTarget(): ");
  }
}

void ParticleFilter::modifiedMultinomialResampler(uint startAt)
{
  // Implementing a very basic resampler... a particle gets selected
  // proportional to its weight and startAt% of the top particles are kept

  particles_t duplicate(particles_);

  std::vector<pdata_t> cumulativeWeights(nParticles_);
  cumulativeWeights[0] = duplicate[O_WEIGHT][0];

  for (uint par = 1; par < nParticles_; par++)
  {
    cumulativeWeights[par] =
        cumulativeWeights[par - 1] + duplicate[O_WEIGHT][par];
  }

  int startParticle = nParticles_ * startAt;

  // Robot particle resampling starts only at startParticle
  for (uint par = startParticle; par < nParticles_; par++)
  {
    boost::random::uniform_real_distribution<> dist(0, 1);
    double randNo = dist(seed_);

    int m = 0;
    while (randNo > cumulativeWeights[m])
      m++;

    copyParticle(particles_, duplicate, par, m, 0, O_TARGET - 1);
  }

  // Target resampling is done for all particles
  for (uint par = 0; par < nParticles_; par++)
  {
    boost::random::uniform_real_distribution<> dist(0, 1);
    double randNo = dist(seed_);

    int m = 0;
    while (randNo > cumulativeWeights[m])
      m++;

    copyParticle(particles_, duplicate, par, m, O_TARGET,
                 nSubParticleSets_ - 1);
  }

  // ROS_DEBUG("End of modifiedMultinomialResampler()");
}

void ParticleFilter::resample()
{
  *iteration_oss << "resample() -> ";

  for (uint r = 0; r < nRobots_; ++r)
  {

    uint o_robot = r * nStatesPerRobot_;

    pdata_t stdX = calc_stdDev<pdata_t>(particles_[o_robot + O_X]);
    pdata_t stdY = calc_stdDev<pdata_t>(particles_[o_robot + O_Y]);
    pdata_t stdZ = calc_stdDev<pdata_t>(particles_[o_robot + O_Z]);
    pdata_t stdQ1 = calc_stdDev<pdata_t>(particles_[o_robot + O_Q1]);
    pdata_t stdQ2 = calc_stdDev<pdata_t>(particles_[o_robot + O_Q2]);
    pdata_t stdQ3 = calc_stdDev<pdata_t>(particles_[o_robot + O_Q3]);
    pdata_t stdQ4 = calc_stdDev<pdata_t>(particles_[o_robot + O_Q4]);

    state_.robots[r].conf = 1 / (stdX + stdY + stdZ + stdQ1 + stdQ2 + stdQ3 + stdQ4);

    // ROS_DEBUG("OMNI%d stdX = %f, stdY = %f, stdTheta = %f", r + 1, stdX,
    // stdY,
    //          stdTheta);
  }

  // Calc. sum of weights
  double weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  // ROS_DEBUG("WeightSum before resampling = %f", weightSum);

  // printWeights("before resampling: ");

  if (weightSum < MIN_WEIGHTSUM)
  {
    ROS_WARN("Zero weightsum - returning without resampling");

    // Print iteration and state information
    *iteration_oss << "FAIL! -> ";

    converged_ = false;
    resetWeights(1.0 / nParticles_);
    return;
  }

  converged_ = true;

  // All resamplers use normalized weights
  for (uint p = 0; p < nParticles_; ++p)
    particles_[O_WEIGHT][p] = (pdata_t)(particles_[O_WEIGHT][p] / weightSum);

  modifiedMultinomialResampler(dynamicVariables_.resamplingPercentageToKeep /
                               100.0);

  // printWeights("after resampling: ");
}

void ParticleFilter::estimate()
{
  *iteration_oss << "estimate() -> ";

  pdata_t weightSum = std::accumulate(particles_[O_WEIGHT].begin(),
                                      particles_[O_WEIGHT].end(), 0.0);

  // ROS_DEBUG("WeightSum when estimating = %f", weightSum);

  subparticles_t normalizedWeights(particles_[O_WEIGHT]);

  // Normalize the weights
  for (uint p = 0; p < nParticles_; ++p)
    normalizedWeights[p] = normalizedWeights[p] / weightSum;

  if (weightSum < MIN_WEIGHTSUM)
  {
    *iteration_oss << "DONE without estimating!";

    // Increase standard deviation for target prediction
    if (dynamicVariables_.targetRandStddev != TARGET_RAND_STDDEV_LOST)
    {
      dynamicVariables_.oldTargetRandSTddev =
          dynamicVariables_.targetRandStddev;
      dynamicVariables_.targetRandStddev = TARGET_RAND_STDDEV_LOST;
    }

    // Don't estimate
    return;
  }

  // Return (if necessary) to old target prediction model stddev
  if(dynamicVariables_.targetRandStddev != dynamicVariables_.oldTargetRandSTddev)
    dynamicVariables_.targetRandStddev = dynamicVariables_.oldTargetRandSTddev;

  // For each robot
  for (uint r = 0; r < nRobots_; ++r)
  {

   uint o_robot = r * nStatesPerRobot_;

    // A vector of weighted means that will be calculated in the next loop
    std::vector<double> weightedMeans(3, 0.0);

    // For theta we will obtain the mean of circular quantities, by converting
    // to cartesian coordinates, placing each angle in the unit circle,
    // averaging these points and finally converting again to polar
    double weightedMeanThetaCartesian[2] = { 0, 0 };

    std::vector<tf::Quaternion> allOrientations;
    std::vector<double> allWeights;

    // ..and each particle
    for (uint p = 0; p < nParticles_; ++p)
    {
      // Accumulate the state proportionally to the particle's normalized weight
      weightedMeans[O_X] += particles_[o_robot + O_X][p] * normalizedWeights[p];
      weightedMeans[O_Y] += particles_[o_robot + O_Y][p] * normalizedWeights[p];
      weightedMeans[O_Z] += particles_[o_robot + O_Z][p] * normalizedWeights[p];

      tf::Quaternion thisOrientation(
          particles_[o_robot + O_Q1][p],
          particles_[o_robot + O_Q2][p],
          particles_[o_robot + O_Q3][p],
          particles_[o_robot + O_Q4][p]);

      allOrientations.push_back(thisOrientation);
      allWeights.push_back(normalizedWeights[p]);

    }

    tf::Quaternion weightedMeanOrientation = getAverageQuaternion(allOrientations, allWeights);

    // Save in the robot state
    // Can't use easy copy since one is using double precision
    state_.robots[r].pose[O_X] = weightedMeans[O_X];
    state_.robots[r].pose[O_Y] = weightedMeans[O_Y];
    state_.robots[r].pose[O_Z] = weightedMeans[O_Z];
    state_.robots[r].pose[O_Q1] = weightedMeanOrientation[0];
    state_.robots[r].pose[O_Q2] = weightedMeanOrientation[1];
    state_.robots[r].pose[O_Q3] = weightedMeanOrientation[2];
    state_.robots[r].pose[O_Q4] = weightedMeanOrientation[3];
  }

  // Target weighted means
  std::vector<double> targetWeightedMeans(STATES_PER_TARGET, 0.0);

  // For each particle
  for (uint p = 0; p < nParticles_; ++p)
  {
    for (uint t = 0; t < STATES_PER_TARGET; ++t)
    {
      targetWeightedMeans[t] +=
          particles_[O_TARGET + t][p] * normalizedWeights[p];
    }
  }

  // Update position
  // Can't use easy copy since one is using double precision
  state_.target.pos[O_TX] = targetWeightedMeans[O_TX];
  state_.target.pos[O_TY] = targetWeightedMeans[O_TY];
  state_.target.pos[O_TZ] = targetWeightedMeans[O_TZ];

  *iteration_oss << "DONE!";
}

void ParticleFilter::printWeights(std::string pre)
{
  std::ostringstream debug;
  debug << "Weights " << pre;
  for (uint i = 0; i < nParticles_; ++i)
    debug << particles_[O_WEIGHT][i] << " ";

  ROS_DEBUG("%s", debug.str().c_str());
}

// TODO set different values for position and orientation, targets, etc
// Simple version, use default values - randomize all values between [-10,10]
void ParticleFilter::init()
{
  if (initialized_)
    return;

  int lvalue = -10;
  int rvalue = 10;

  std::vector<double> defRand((nSubParticleSets_ - 1) * 2);

  for (size_t i = 0; i < defRand.size(); i += 2)
  {
    defRand[i] = lvalue;
    defRand[i + 1] = rvalue;
  }

  std::vector<double> defPos((nRobots_ * 2), 0.0);

  // Call the custom function with these values
  init(defRand, defPos);
}

// Overloaded function when using custom values
void ParticleFilter::init(const std::vector<double>& customRandInit,
                          const std::vector<double>& customPosInit)
{
  if (initialized_)
    return;

  // Set flag
  initialized_ = true;

  // bool flag_theta_given = (customPosInit.size() == nRobots_ * 7 &&
  //                          customRandInit.size() == nSubParticleSets_ * 7);
  // size_t numVars =
  //     flag_theta_given ? customRandInit.size() / 7 : customRandInit.size() / 2;

  size_t numVars = customRandInit.size() / 2;

  ROS_INFO("Initializing particle filter");

  // For all subparticle sets except the particle weights
  for (size_t i = 0; i < numVars; ++i)
  {
    ROS_DEBUG("Values for distribution: %.4f %.4f", customRandInit[2 * i],
              customRandInit[2 * i + 1]);

    boost::random::uniform_real_distribution<> dist(customRandInit[2 * i],
                                                    customRandInit[2 * i + 1]);

    // Sample a value from the uniform distribution for each particle
    for (uint p = 0; p < nParticles_; ++p)
      particles_[i][p] = (pdata_t)dist(seed_);
  }

  // Particle weights init with same weight (1/nParticles)
  resetWeights(1.0 / nParticles_);

  // Initialize pose with initial belief
  for (uint r = 0; r < nRobots_; ++r)
  {
    pdata_t tmp[] = { (pdata_t)customPosInit[7 * r + O_X], (pdata_t)customPosInit[7 * r + O_Y],
                      (pdata_t)customPosInit[7 * r + O_Z] };
    state_.robots[r].pose = std::vector<pdata_t>(tmp, tmp + nStatesPerRobot_);
    // if (flag_theta_given)
      state_.robots[r].pose.push_back(customPosInit[7 * r + O_Q1]);
      state_.robots[r].pose.push_back(customPosInit[7 * r + O_Q2]);
      state_.robots[r].pose.push_back(customPosInit[7 * r + O_Q3]);
      state_.robots[r].pose.push_back(customPosInit[7 * r + O_Q4]);
  }

  // State should have the initial belief

  ROS_INFO("Particle filter initialized");
}

void ParticleFilter::predict(const uint robotNumber, const uav_msgs::uav_poseConstPtr &pose,
                             const uav_msgs::uav_pose lastReceivedOdometry)
{
  if (!initialized_)
    return;

  *iteration_oss << "predict(machine_" << robotNumber + 1 << ") -> ";

  // If this is the main robot, update the odometry time
  if (mainRobotID_ == robotNumber)
  {
    odometryTime_.updateTime(ros::WallTime::now());
    iterationEvalTime_ = ros::WallTime::now();
  }
  using namespace boost::random;

  // Variables concerning this robot specifically
  int robot_offset = robotNumber * nStatesPerRobot_;

  if (ESTIMATE_ROBOT_MOVEMENT) {
    // get difference between this and previous odometry message 
    // to apply to robot state in particles
    geometry_msgs::Pose deltaOdometry;
    deltaOdometry.position.x = pose->position.x - lastReceivedOdometry.position.x;
    deltaOdometry.position.y = pose->position.y - lastReceivedOdometry.position.y;
    deltaOdometry.position.z = pose->position.z - lastReceivedOdometry.position.z;
    // newOrientation = delta * oldOrientation
    // delta = newOrientation * oldOrientation^-1
    tf2::Quaternion oldOrientation(lastReceivedOdometry.orientation.x,lastReceivedOdometry.orientation.y,lastReceivedOdometry.orientation.z,(-1)*lastReceivedOdometry.orientation.w);
    tf2::Quaternion newOrientation(pose->orientation.x,pose->orientation.y,pose->orientation.z,pose->orientation.w);
    newOrientation *= oldOrientation;
    newOrientation.normalize();
    deltaOdometry.orientation.x = newOrientation.getX();
    deltaOdometry.orientation.y = newOrientation.getY();
    deltaOdometry.orientation.z = newOrientation.getZ();
    deltaOdometry.orientation.w = newOrientation.getW();

    for (uint i = 0; i < nParticles_; i++) {
      particles_[robot_offset + O_X][i] += deltaOdometry.position.x;
      particles_[robot_offset + O_Y][i] += deltaOdometry.position.y;
      particles_[robot_offset + O_Z][i] += deltaOdometry.position.z;
      tf2::Quaternion particleOrientation(particles_[robot_offset + O_Q1][i],particles_[robot_offset + O_Q2][i],particles_[robot_offset + O_Q3][i],particles_[robot_offset + O_Q4][i]);
      newOrientation *= particleOrientation;
      newOrientation.normalize();
      particles_[robot_offset + O_Q1][i] = newOrientation.getX();
      particles_[robot_offset + O_Q2][i] = newOrientation.getY();
      particles_[robot_offset + O_Q3][i] = newOrientation.getZ();
      particles_[robot_offset + O_Q4][i] = newOrientation.getW();
    }
  }

  else {
    for (uint i = 0; i < nParticles_; i++) {
      particles_[robot_offset + O_X][i] = pose->position.x;
      particles_[robot_offset + O_Y][i] = pose->position.y;
      particles_[robot_offset + O_Z][i] = pose->position.z;
      particles_[robot_offset + O_Q1][i] = pose->orientation.x;
      particles_[robot_offset + O_Q2][i] = pose->orientation.y;
      particles_[robot_offset + O_Q3][i] = pose->orientation.z;
      particles_[robot_offset + O_Q4][i] = pose->orientation.w;
    }
  }

    // Randomize a bit for this robot since it does not see landmarks and target
    // isn't seen
    boost::random::uniform_real_distribution<> randPar(-0.05, 0.05);

    for (uint p = 0; p < nParticles_; ++p)
    {
      for (uint s = 0; s < nStatesPerRobot_; ++s)
        particles_[robot_offset + s][p] += randPar(seed_);
    }

  // If this is the main robot, perform one PF-UCLT iteration
  if (mainRobotID_ == robotNumber)
  {
    ROS_DEBUG("ParticleFilter::Predict: Full iteration started");
    // Lock mutex
    boost::mutex::scoped_lock(mutex_);

    ROS_DEBUG("Full iteration complete");

    ROS_INFO("(WALL TIME) Odometry analyzed with = %fms",
             1e3 * odometryTime_.diff);

    deltaIteration_ = ros::WallTime::now() - iterationEvalTime_;
    if (deltaIteration_ > maxDeltaIteration_)
      maxDeltaIteration_ = deltaIteration_;

    durationSum += deltaIteration_;
    numberIterations++;

    ROS_INFO_STREAM("(WALL TIME) Iteration time: "
                    << 1e-6 * deltaIteration_.toNSec() << "ms ::: Worst case: "
                    << 1e-6 * maxDeltaIteration_.toNSec() << "ms ::: Average: "
                    << 1e-6 * (durationSum.toNSec() / numberIterations) << "ms");

    // ROS_DEBUG("Iteration: %s", iteration_oss->str().c_str());
    // Clear ostringstream
    iteration_oss->str("");
    iteration_oss->clear();

    // Start next iteration
    nextIteration();
  }
}


void ParticleFilter::saveAllTargetMeasurementsDone(const uint robotNumber)
{
  *iteration_oss << "allTargets(OMNI" << robotNumber + 1 << ") -> ";
}

ParticleFilter::dynamicVariables_s::dynamicVariables_s(ros::NodeHandle& nh,
                                                       const uint nRobots)
    : firstCallback(true), alpha(nRobots, std::vector<float>(NUM_ALPHAS))
{
  // Get node parameters, assume they exist
  readParam<double>(nh, "percentage_to_keep", resamplingPercentageToKeep);

  readParam<int>(nh, "particles", nParticles);

  readParam<double>(nh, "predict_model_stddev", targetRandStddev);
  oldTargetRandSTddev = targetRandStddev;

  // Get alpha values for some robots (hard-coded for our 4 robots..)
  for (uint r = 0; r < nRobots; ++r)
  {
    std::string paramName =
        "OMNI" + boost::lexical_cast<std::string>(r + 1) + "_alpha";

    std::string str;
    if (readParam<std::string>(nh, paramName, str))
      fill_alpha(r, str); // value was provided
    else
      fill_alpha(r, "0.015,0.1,0.5,0.001"); // default
  }
}

void ParticleFilter::dynamicVariables_s::fill_alpha(const uint robot,
                                                    const std::string& str)
{
  // Tokenize the string of comma-separated values
  std::istringstream iss(str);
  std::string token;
  uint tokenCount = 0;
  while (std::getline(iss, token, ','))
  {
    if (tokenCount >= NUM_ALPHAS)
      break;

    std::istringstream tokss(token);
    float val;
    tokss >> val;

    if (val < 0)
    {
      ROS_WARN("Invalid alpha value %f", val);
      continue;
    }

    alpha[robot][tokenCount] = val;
    ++tokenCount;
  }

  ROS_WARN_COND(tokenCount != NUM_ALPHAS,
                "Number of alpha values provided is not the required number %d",
                NUM_ALPHAS);
}


bool ParticleFilter::predictKF(const target_tracker_distributed_kf::CacheElement &in, target_tracker_distributed_kf::CacheElement &out) {

        // Easy access
        const Eigen::VectorXd &ins = in.state;
        Eigen::VectorXd &outs = out.state;

        // Time passed from one to next
        if (!out.stamp.isValid() || !in.stamp.isValid()) {
            ROS_WARN("One of the stamps is invalid, returning false from predict() without doing anything else");
            return false;
        }

        const double deltaT = out.stamp.toSec() - in.stamp.toSec();

        const static double velocityDecayTime = 3.0;

        const static double velocityDecayTo = 0.1;
        const double velocityDecayAlpha = pow(velocityDecayTo, 1.0 / velocityDecayTime);
        const double velocityDecayFactor = pow(velocityDecayAlpha, deltaT);
        const double velocityIntegralFactor = (velocityDecayFactor - 1) / log(velocityDecayAlpha);

        // Decreasing velocity model
        outs(0) = ins(0) + ins(3) * velocityIntegralFactor;
        outs(1) = ins(1) + ins(4) * velocityIntegralFactor;
        outs(2) = ins(2) + ins(5) * velocityIntegralFactor;

        outs(3) = ins(3) * velocityDecayFactor;
        outs(4) = ins(4) * velocityDecayFactor;
        outs(5) = ins(5) * velocityDecayFactor;

        // const static double offsetDecayTo = 0.1;
        // const double offsetDecayAlpha = pow(offsetDecayTo, 1.0 / offsetDecayTime);
        // const double offsetDecayFactor = pow(offsetDecayAlpha, deltaT);
        // //const double offsetIntegralFactor = (offsetDecayFactor - 1)/log(offsetDecayAlpha);

        // outs(6) = (ins(6) - posGlobalOffsetBiasX) * offsetDecayFactor;
        // outs(7) = (ins(7) - posGlobalOffsetBiasY) * offsetDecayFactor;
        // outs(8) = (ins(8) - posGlobalOffsetBiasZ) * offsetDecayFactor;

        // Construct jacobian G based on deltaT
        Eigen::MatrixXd G((int) nStatesPerRobot_, (int) nStatesPerRobot_);
        // offset assumed independent from target detection
        G << 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        double  noisePosXVar{0.01},
        noiseVelXVar{0.01},
        noiseOffXVar{0.01},
        noisePosYVar{0.01},
        noiseVelYVar{0.01},
        noiseOffYVar{0.01},
        noisePosZVar{0.01},
        noiseVelZVar{0.01},
        noiseOffZVar{0.01};
        
        Eigen::MatrixXd R((int) nStatesPerRobot_, (int) nStatesPerRobot_);
        R << noisePosXVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, noisePosYVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, noisePosZVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, noiseVelXVar, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, noiseVelYVar, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, noiseVelZVar, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffXVar, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffYVar, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffZVar;
        // Update covariance from one to next
        out.cov = Eigen::MatrixXd((G * in.cov * G.transpose()) + (deltaT / 1.0) * R);

        return true;
    }
// end of namespace pfuclt_omni_dataset
}
