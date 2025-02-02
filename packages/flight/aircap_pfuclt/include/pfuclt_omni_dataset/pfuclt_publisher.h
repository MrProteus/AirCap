#ifndef PFUCLT_PUBLISHER_H
#define PFUCLT_PUBLISHER_H

#include <pfuclt_omni_dataset/pfuclt_particles.h>
#include <aircap_pfuclt/RobotState.h>
#include <aircap_pfuclt/LRMGTData.h>
#include <aircap_pfuclt/Estimate.h>

#include <vector>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace pfuclt_omni_dataset {

/**
 * @brief The PFPublisher class - implements publishing for the ParticleFilter
 * class using ROS
 */
class PFPublisher : public ParticleFilter {
public:

    /**
     * @brief resize_particles - change to a different number of particles and
     * resize the publishing message
     * @param n - the desired number of particles
     */
    void resize_particles(const uint n) {
      // Call base class method
      ParticleFilter::resize_particles(n);

      ROS_INFO("Resizing particle message");

      // Resize particle message
      msg_particles_.particles.resize(n);
      for (uint p = 0; p < n; ++p) {
        msg_particles_.particles[p].particle.resize(nSubParticleSets_);
      }
    }

private:
    ros::Subscriber GT_sub_;
    ros::Publisher estimatePublisher_, particlePublisher_,
            targetEstimatePublisher_, targetGTPublisher_, targetParticlePublisher_;
    std::vector<ros::Publisher> particleStdPublishers_;
    std::vector<ros::Publisher> robotGTPublishers_;
    std::vector<ros::Publisher> robotEstimatePublishers_;
    ros::Publisher targetObservationsPublisher_;

    read_omni_dataset::LRMGTData msg_GT_;
    pfuclt_omni_dataset::particles msg_particles_;
    read_omni_dataset::Estimate msg_estimate_;

    std::vector<tf2_ros::TransformBroadcaster> robotBroadcasters;

    void publishParticles();

    void publishRobotStates();

    void publishTargetState();

    void publishEstimate();

    void publishGTData();

    void publishTargetObservations();

    void getGTTarget();

    void robotGTCallback(geometry_msgs::Pose msg);

    void targetGTCallback(geometry_msgs::PoseWithCovarianceStamped msg);

    void robotKFCallback(uav_msgs::uav_pose msg);

    void targetKFCallback(geometry_msgs::PoseWithCovarianceStamped msg);

    void robotPFCallback(geometry_msgs::PoseStamped msg);

    void targetPFCallback(geometry_msgs::PointStamped msg);

    geometry_msgs::PoseWithCovarianceStamped gtTarget_, pfDelta_, kfDelta_;
    double pfNorm_, kfNorm_, pfCovNorm_, kfCovNorm_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> gtRobots_, pfRobotDeltas_, kfRobotDeltas_;
    std::vector<double> pfRobotNorms_, kfRobotNorms_, pfRobotAngles_, kfRobotAngles_;
    ros::Publisher pubGtTarget_, pubKfNorm_, pubPfNorm_, pubKfCov_, pubPfCov_;
    ros::Subscriber robotGTSub_, targetGTSub_, robotKFSub_, targetKFSub_, robotPFSub_, targetPFSub_;
    ros::ServiceClient gtService_;
    gazebo_msgs::GetModelState srv_;

public:
    /**
     * @brief PFPublisher - constructor
     * @param data - a structure with the necessary initializing data for the
     * ParticleFilter class
     * @param publishData - a structure with some more data for this class
     */
    PFPublisher(struct PFinitData &data);

    /**
     * @brief getPFReference - retrieve a reference to the base class's members
     * @remark C++ surely is awesome
     * @return returns a reference to the base ParticleFilter for this object
     */
    ParticleFilter *getPFReference() { return (ParticleFilter *) this; }

    /**
     * @brief gtDataCallback - callback of ground truth data
     */
    void gtDataCallback(const read_omni_dataset::LRMGTData::ConstPtr &);

    /**
     * @brief nextIteration - extends the base class method to add the ROS
     * publishing
     */
    void nextIteration();
};
}

#endif //PFUCLT_PUBLISHER_H