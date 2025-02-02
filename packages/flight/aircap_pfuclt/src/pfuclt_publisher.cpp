#include <aircap_pfuclt/read_omni_dataset.h> // defines version of messages
#include <pfuclt_omni_dataset/pfuclt_publisher.h>

namespace pfuclt_omni_dataset {

PFPublisher::PFPublisher(struct ParticleFilter::PFinitData &data)
        : ParticleFilter(data), 
          particleStdPublishers_(data.nRobots),
          robotGTPublishers_(data.nRobots), robotEstimatePublishers_(data.nRobots),
          robotBroadcasters(data.nRobots){
    // Prepare particle message
    resize_particles(nParticles_);

    // Subscribe to GT data
    GT_sub_ = nh_.subscribe<read_omni_dataset::LRMGTData>(
            "/gtData_4robotExp", 10,
            boost::bind(&PFPublisher::gtDataCallback, this, _1));

    std::string robotNamespace("/machine_" +
                        boost::lexical_cast<std::string>(data.mainRobotID));

        // Other publishers
        estimatePublisher_ =
            nh_.advertise<read_omni_dataset::Estimate>(robotNamespace + "/pf/pfuclt_estimate", 10);
        particlePublisher_ =
            nh_.advertise<pfuclt_omni_dataset::particles>(robotNamespace + "/pf/pfuclt_particles", 10);

        // Rviz visualization publishers
        // Target
        targetEstimatePublisher_ =
            nh_.advertise<geometry_msgs::PointStamped>(robotNamespace + "/pf/target/estimatedPose", 10);
        targetGTPublisher_ =
            nh_.advertise<geometry_msgs::PointStamped>(robotNamespace + "/pf/target/gtPose", 10);
        targetParticlePublisher_ =
            nh_.advertise<sensor_msgs::PointCloud>(robotNamespace + "/pf/target/particles", 10);

        // target observations publisher
        targetObservationsPublisher_ =
            nh_.advertise<visualization_msgs::Marker>(robotNamespace + "/pf/targetObservations", 10);

    // Robots
    for (uint r = 0; r < nRobots_; ++r) {
        std::string robotName("/machine_" + boost::lexical_cast<std::string>(r + 1));

        // particle publisher
        particleStdPublishers_[r] = nh_.advertise<geometry_msgs::PoseArray>(
                robotNamespace + "/pf/" + robotName + "/particles", 1000);

        // estimated state
        robotEstimatePublishers_[r] = nh_.advertise<geometry_msgs::PoseStamped>(
                robotNamespace + "/pf/" + robotName + "/estimatedPose", 1000);

        // build estimate msg
        msg_estimate_.robotEstimates.push_back(geometry_msgs::Pose());
        msg_estimate_.targetVisibility.push_back(false);

// ground truth publisher, in the simulation package we have PoseStamped
#ifndef USE_NEWER_READ_OMNI_PACKAGE
        robotGTPublishers_[r] = nh_.advertise<geometry_msgs::PointStamped>(
                robotNamespace + "/pf/" + robotName + "/gtPose", 1000);
#else
        robotGTPublishers_[r] = nh_.advertise<geometry_msgs::PoseStamped>(
                robotNamespace + "/pf/" + robotName + "/gtPose", 1000);
#endif  
    }

        // pub/sub for evaluation
        robotGTSub_ = nh_.subscribe<geometry_msgs::Pose>("/firefly_" + boost::lexical_cast<std::string>(data.mainRobotID) + "/ground_truth/pose",10,&PFPublisher::robotGTCallback, this);
        targetGTSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/target/pose",10, &PFPublisher::targetGTCallback,this);
        robotKFSub_ = nh_.subscribe<uav_msgs::uav_pose>(robotNamespace + "/pose", 10, &PFPublisher::robotKFCallback, this);
        targetKFSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotNamespace + "/target_tracker/pose", 10, &PFPublisher::targetKFCallback, this);
        robotPFSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(robotNamespace + "/pf/machine_" + boost::lexical_cast<std::string>(data.mainRobotID) + "/pose", 10, &PFPublisher::robotPFCallback, this);
        targetPFSub_ = nh_.subscribe<geometry_msgs::PointStamped>(robotNamespace + "/pf/target/estimatedPose", 10, &PFPublisher::targetPFCallback, this);

        pubGtTarget_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(robotNamespace + "/eval/gt/target", 10);
        pubKfNorm_ = nh_.advertise<std_msgs::Float64>(robotNamespace + "/eval/kf/norm",10);
        pubPfNorm_ = nh_.advertise<std_msgs::Float64>(robotNamespace + "/eval/pf/norm",10);
        pubKfCov_ = nh_.advertise<std_msgs::Float64>(robotNamespace + "eval/kf/cov",10);
        pubPfCov_ = nh_.advertise<std_msgs::Float64>(robotNamespace + "eval/pf/cov",10);

    gtService_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    srv_.request.model_name = "actor";

    ROS_INFO("It's a publishing particle filter!");
}


// void PFPublisher::getGTTarget() {
//     if (gtService_.call(srv_)) {
//         gazebo_msgs::GetModelState::Response modelState = srv_.response;
//         geometry_msgs::PoseWithCovarianceStamped gtTargetMsg;
//         gtTargetMsg.header.stamp = ros::Time::now();
//         gtTargetMsg.pose.pose.position.x = modelState.pose.position.y;
//         gtTargetMsg.pose.pose.position.y = modelState.pose.position.x;
//         gtTargetMsg.pose.pose.position.z = (-1) * modelState.pose.position.z;
//         gtTarget_ = gtTargetMsg;
//         pubGtTarget_.publish(gtTargetMsg);
//     }
// }

void PFPublisher::robotGTCallback(geometry_msgs::Pose msg){
    // gtRobots_[data.mainRobotID -1] = msg;
}

void PFPublisher::targetGTCallback(geometry_msgs::PoseWithCovarianceStamped msg) {
    gtTarget_ = msg;
}

void PFPublisher::robotKFCallback(uav_msgs::uav_pose msg) {

}

void PFPublisher::targetKFCallback(geometry_msgs::PoseWithCovarianceStamped msg) {
    // getGTTarget();
    kfDelta_.header = msg.header;
    kfDelta_.pose.pose.position.x = msg.pose.pose.position.x - gtTarget_.pose.pose.position.x;
    kfDelta_.pose.pose.position.y = msg.pose.pose.position.y - gtTarget_.pose.pose.position.y;
    kfDelta_.pose.pose.position.z = msg.pose.pose.position.z - gtTarget_.pose.pose.position.z;
    // kfDelta_.pose.pose.orientation.x = msg.pose.pose.orientation.x - gtTarget_.pose.pose.orientation.x;
    // kfDelta_.pose.pose.orientation.y = msg.pose.pose.orientation.y - gtTarget_.pose.pose.orientation.y;
    // kfDelta_.pose.pose.orientation.z = msg.pose.pose.orientation.z - gtTarget_.pose.pose.orientation.z;
    // kfDelta_.pose.pose.orientation.w = msg.pose.pose.orientation.w - gtTarget_.pose.pose.orientation.w;
    
    kfNorm_ = sqrt(pow(kfDelta_.pose.pose.position.x,2) + pow(kfDelta_.pose.pose.position.y,2) + pow(kfDelta_.pose.pose.position.z,2));
    std_msgs::Float64 msgNorm;
    msgNorm.data = kfNorm_;
    pubKfNorm_.publish(msgNorm);

    double cov = msg.pose.covariance[0 * 6 + 0] + msg.pose.covariance[1 * 6 + 1] + msg.pose.covariance[2 * 6 + 2];
    msgNorm.data = cov;
    pubKfCov_.publish(msgNorm);
}

void PFPublisher::robotPFCallback(geometry_msgs::PoseStamped msg) {

}

void PFPublisher::targetPFCallback(geometry_msgs::PointStamped msg) {
    // getGTTarget();
    pfDelta_.header = msg.header;
    pfDelta_.pose.pose.position.x = msg.point.x - gtTarget_.pose.pose.position.x;
    pfDelta_.pose.pose.position.y = msg.point.y - gtTarget_.pose.pose.position.y;
    pfDelta_.pose.pose.position.z = msg.point.z - gtTarget_.pose.pose.position.z;
    // pfDelta_.pose.pose.orientation.x = msg.pose.pose.orientation.x - gtTarget_.pose.pose.orientation.x;
    // pfDelta_.pose.pose.orientation.y = msg.pose.pose.orientation.y - gtTarget_.pose.pose.orientation.y;
    // pfDelta_.pose.pose.orientation.z = msg.pose.pose.orientation.z - gtTarget_.pose.pose.orientation.z;
    // pfDelta_.pose.pose.orientation.w = msg.pose.pose.orientation.w - gtTarget_.pose.pose.orientation.w;

    pfNorm_ = sqrt(pow(pfDelta_.pose.pose.position.x,2) + pow(pfDelta_.pose.pose.position.y,2) + pow(pfDelta_.pose.pose.position.z,2));
    std_msgs::Float64 msgNorm;
    msgNorm.data = pfNorm_;
    pubPfNorm_.publish(msgNorm);

    // double cov = msg.pose.covariance[0 * 6 + 0] + msg.pose.covariance[1 * 6 + 1] + msg.pose.covariance[2 * 6 + 2];
    // msgNorm.data = cov;
    // pubPfCov_.publish(msgNorm);
}


void PFPublisher::publishParticles() {
    // The eval package would rather have the particles in the format
    // particle->subparticle instead, so we have to inverse it
    for (uint p = 0; p < nParticles_; ++p) {
        for (uint s = 0; s < nSubParticleSets_; ++s) {
            msg_particles_.particles[p].particle[s] = particles_[s][p];
        }
    }

    // Send it!
    particlePublisher_.publish(msg_particles_);

    // Also send as a series of PoseArray messages for each robot
    for (uint r = 0; r < nRobots_; ++r) {

        uint o_robot = r * nStatesPerRobot_;
        geometry_msgs::PoseArray msgStd_particles;
        msgStd_particles.header.stamp = savedLatestObservationTime_;
        msgStd_particles.header.frame_id = "world";

        for (uint p = 0; p < nParticles_; ++p) {
            tf2::Quaternion tf2q(particles_[o_robot + O_Q1][p],particles_[o_robot + O_Q2][p],particles_[o_robot + O_Q3][p],
                                 particles_[o_robot + O_Q4][p]);
            tf2::Transform tf2t(tf2q, tf2::Vector3(particles_[o_robot + O_X][p],
                                                   particles_[o_robot + O_Y][p],
                                                   particles_[o_robot + O_Z][p]));

            geometry_msgs::Pose pose;
            tf2::toMsg(tf2t, pose);
            msgStd_particles.poses.insert(msgStd_particles.poses.begin(), pose);
        }

        particleStdPublishers_[r].publish(msgStd_particles);
    }

    // Send target particles as a pointcloud
    sensor_msgs::PointCloud target_particles;
    target_particles.header.stamp = ros::Time::now();
    target_particles.header.frame_id = "world";

    for (uint p = 0; p < nParticles_; ++p) {
        geometry_msgs::Point32 point;
        point.x = particles_[O_TARGET + O_TX][p];
        point.y = particles_[O_TARGET + O_TY][p];
        point.z = particles_[O_TARGET + O_TZ][p];

        target_particles.points.insert(target_particles.points.begin(), point);
    }
    targetParticlePublisher_.publish(target_particles);
}

void PFPublisher::publishRobotStates() {
    // This is pretty much copy and paste
    for (uint r = 0; r < nRobots_; ++r) {

        std::ostringstream robotName;
        robotName << "machine_" << r + 1;

        msg_estimate_.header.stamp = savedLatestObservationTime_;

        ParticleFilter::State::robotState_s &pfState = state_.robots[r];
        geometry_msgs::Pose &rosState = msg_estimate_.robotEstimates[r];

        // Create from Euler angles
        tf2::Quaternion tf2q(pfState.pose[O_Q1],pfState.pose[O_Q2],pfState.pose[O_Q3], pfState.pose[O_Q4]);
        tf2::Transform tf2t(tf2q, tf2::Vector3(pfState.pose[O_X], pfState.pose[O_Y],
                                               pfState.pose[O_Z]));

        // Transform to our message type
        tf2::toMsg(tf2t, rosState);

        // TF2 broadcast
        geometry_msgs::TransformStamped estTransf;
        estTransf.header.stamp = savedLatestObservationTime_;
        estTransf.header.frame_id = "world";
        estTransf.child_frame_id = robotName.str() + "est";
        estTransf.transform = tf2::toMsg(tf2t);
        robotBroadcasters[r].sendTransform(estTransf);

        // Publish as a standard pose msg using the previous TF
        geometry_msgs::PoseStamped estPose;
        estPose.header.stamp = estTransf.header.stamp;
        estPose.header.frame_id = estTransf.child_frame_id;
        // Pose is everything at 0 as it's the same as the TF

        robotEstimatePublishers_[r].publish(estPose);
    }
}

void PFPublisher::publishTargetState() {
    msg_estimate_.targetEstimate.header.frame_id = "world";

    // Our custom message type
    msg_estimate_.targetEstimate.x = state_.target.pos[O_TX];
    msg_estimate_.targetEstimate.y = state_.target.pos[O_TY];
    msg_estimate_.targetEstimate.z = state_.target.pos[O_TZ];
    msg_estimate_.targetEstimate.found = state_.target.seen;

    for (uint r = 0; r < nRobots_; ++r) {
        msg_estimate_.targetVisibility[r] = bufTargetObservations_[r].found;
    }

    // Publish as a standard pose msg using the previous TF
    geometry_msgs::PointStamped estPoint;
    estPoint.header.stamp = ros::Time::now();
    estPoint.header.frame_id = "world";
    estPoint.point.x = state_.target.pos[O_TX];
    estPoint.point.y = state_.target.pos[O_TY];
    estPoint.point.z = state_.target.pos[O_TZ];

    targetEstimatePublisher_.publish(estPoint);
}

void PFPublisher::publishEstimate() {
    // msg_estimate_ has been built in other methods (publishRobotStates and
    // publishTargetState)
    msg_estimate_.computationTime = deltaIteration_.toNSec() * 1e-9;
    msg_estimate_.converged = converged_;

    estimatePublisher_.publish(msg_estimate_);
}

void PFPublisher::publishTargetObservations() {
    static std::vector<bool> previouslyPublished(nRobots_, false);

    for (uint r = 0; r < nRobots_; ++r) {
        // Publish as rviz standard visualization types (an arrow)
        visualization_msgs::Marker marker;

        // Robot and observation
        std::ostringstream robotName;
        robotName << "machine_" << r + 1;

        TargetObservation &obs = bufTargetObservations_[r];

        // If not found, let's just publish that one time
        if (obs.found == false) {
            if (previouslyPublished[r])
                continue;
            else
                previouslyPublished[r] = true;
        } else
            previouslyPublished[r] = false;

        marker.header.frame_id = robotName.str() + "est";
        marker.header.stamp = savedLatestObservationTime_;

        // Setting the same namespace and id will overwrite the previous marker
        marker.ns = robotName.str() + "_target_observations";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Arrow draw
        geometry_msgs::Point tail;
        tail.x = state_.robots[r].pose[O_X];
        tail.y = state_.robots[r].pose[O_Y];
        tail.z = state_.robots[r].pose[O_Z];
        marker.points.push_back(tail);
        // Point at index 1 - head - is the target pose in the local frame
        geometry_msgs::Point head;
        head.x = obs.x;
        head.y = obs.y;
        head.z = obs.z;
        marker.points.push_back(head);

        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0.05;

        // Colour
        marker.color.a = 1;
        if (obs.found)
            marker.color.r = marker.color.g = marker.color.b = 0.6;
        else {
            marker.color.r = 1.0;
            marker.color.g = marker.color.b = 0.0;
        }

        // Other
        marker.lifetime = ros::Duration(2);

        // Send it
        targetObservationsPublisher_.publish(marker);
    }
}

void PFPublisher::publishGTData() {
    geometry_msgs::PointStamped gtPoint;
    gtPoint.header.stamp = savedLatestObservationTime_;
    gtPoint.header.frame_id = "world";

#ifdef USE_NEWER_READ_OMNI_PACKAGE
    geometry_msgs::PoseStamped gtPose;
    gtPose.header.stamp = savedLatestObservationTime_;
    gtPose.header.frame_id = "world";

    for (uint r = 0; r < nRobots_; ++r) {
        gtPose.pose = msg_GT_.poseOMNI[r].pose;
        robotGTPublishers_[r].publish(gtPose);
    }

#else
    if (true == robotsUsed_[0])
    {
        gtPoint.point = msg_GT_.poseOMNI1.pose.position;
        robotGTPublishers_[0].publish(gtPoint);
    }

    if (true == robotsUsed_[2])
    {
        gtPoint.point = msg_GT_.poseOMNI3.pose.position;
        robotGTPublishers_[2].publish(gtPoint);
    }

    if (true == robotsUsed_[3])
    {
        gtPoint.point = msg_GT_.poseOMNI4.pose.position;
        robotGTPublishers_[3].publish(gtPoint);
    }

    if (true == robotsUsed_[4])
    {
        gtPoint.point = msg_GT_.poseOMNI5.pose.position;
        robotGTPublishers_[4].publish(gtPoint);
    }
#endif

    // Publish for the target as well
    if (msg_GT_.orangeBall3DGTposition.found) {
        gtPoint.point.x = msg_GT_.orangeBall3DGTposition.x;
        gtPoint.point.y = msg_GT_.orangeBall3DGTposition.y;
        gtPoint.point.z = msg_GT_.orangeBall3DGTposition.z;
        targetGTPublisher_.publish(gtPoint);
    }
}

void PFPublisher::gtDataCallback(
        const read_omni_dataset::LRMGTData::ConstPtr &gtMsgReceived) {
    msg_GT_ = *gtMsgReceived;
}

void PFPublisher::nextIteration() {
    // Call the base class method
    ParticleFilter::nextIteration();

    // Publish the particles first
    publishParticles();

    // Publish robot states
    publishRobotStates();

    // Publish target state
    publishTargetState();

    // Publish estimate
    publishEstimate();

    // Publish robot-to-target lines
    publishTargetObservations();

// Publish GT data if we have received any callback
#ifdef USE_NEWER_READ_OMNI_PACKAGE
    if (!msg_GT_.poseOMNI.empty())
        publishGTData();
#else
    publishGTData();
#endif
};
}