#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gttargetpub");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState srv;
    
    // Set the model name for which you want to get the state
    srv.request.model_name = "actor";

    // publish pose to topic
    ros::Publisher pubGtTarget = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/target/pose", 10);
        
    ros::Rate loop_rate(50);

    while(ros::ok()) {
        // Call the service and check if it was successful
        if (client.call(srv))
        {
            // Access the model state using srv.response
            gazebo_msgs::GetModelState::Response modelState = srv.response;

            // Access the desired information from the model state
            // For example, to get the position of the model:
            geometry_msgs::PoseWithCovarianceStamped gtTargetMsg;
            gtTargetMsg.header.stamp = ros::Time::now();
            gtTargetMsg.pose.pose.position.x = modelState.pose.position.y;
            gtTargetMsg.pose.pose.position.y = modelState.pose.position.x;
            gtTargetMsg.pose.pose.position.z = (-1) * modelState.pose.position.z;

            pubGtTarget.publish(gtTargetMsg);

        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
            // return 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}