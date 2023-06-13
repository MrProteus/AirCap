#!/bin/bash

datafolder="/home/friedrich/otherdisk/"
scriptfolder="/home/friedrich/catkin_ws/src/AirCap/scripts/simulation/"
# Check if folders exist and create them if not
for ((i = 1; i <= 5; i++)); do
    folder_name=$datafolder
    folder_name+="robots$i"
    if [ ! -d "$folder_name" ]; then
        mkdir "$folder_name"
    fi
done

# topics="/target/pose"
# # Go into each directory and run the commands
# for ((i = 1; i <= 5; i++)); do
#     folder_name="robots$i"
#     cd "$folder_name"
#     topics+=" /machine_$i/pose /machine_$i/target_tracker/pose /machine_$i/object_detections/projected_to_world /firefly_$i/ground_truth/pose"
#     for ((j = 1; j <= 5; j++)); do
#         ~/catkin_ws/src/AirCap/scripts/simulation/setup_mavocap_gazebo.sh $i
#         sleep 20
#         rosbag record -o attempt$j $topics __name:=this_recording &
#         sleep 60
#         rosnode kill /this_recording
#         ~/catkin_ws/src/AirCap/scripts/simulation/cleanup.sh
#         sleep 10
#     done
#     cd ..
# done

# number of attempts
for ((j = 1; j <= 5; j++)); do
    topics="/clock /target/pose"
    # number of robots
    for ((i = 1; i <= 5; i++)); do
        cd $scriptfolder
        ./setup_mavocap_gazebo.sh $i
        sleep 20
        folder_name=$datafolder
        folder_name+="robots$i"
        cd "$folder_name"
        topics+=" /machine_$i/pose /machine_$i/target_tracker/pose /machine_$i/object_detections/projected_to_world /firefly_$i/ground_truth/pose"
        
        rosbag record -o attempt$j $topics __name:=this_recording &
        sleep 300
        rosnode kill /this_recording
        cd $scriptfolder
        ./cleanup.sh
        sleep 10
    done
done