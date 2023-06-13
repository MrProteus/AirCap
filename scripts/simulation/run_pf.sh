#!/bin/bash

datafolder="/home/friedrich/otherdisk/"
outputfolder="/home/friedrich/otherdisk/pf/"
scriptfolder="/home/friedrich/catkin_ws/src/AirCap/scripts/simulation/"
particlecount=(10 25 50 100 250 500 1000 2500)
# particlecount=(10)
# Check if folders exist and create them if not
for ((i = 1; i <= 5; i++)); do
    folder_name=$outputfolder
    folder_name+="robots$i"
    if [ ! -d "$folder_name" ]; then
        mkdir "$folder_name"
    fi
    for k in "${particlecount[@]}"; do
        folder_name2=$folder_name
        folder_name2+="/particles$k"
        if [ ! -d "$folder_name2" ]; then
            mkdir "$folder_name2"
        fi
    done
done

# number of attempts
for ((j = 1; j <= 5; j++)); do
    # number of robots
    for ((i = 1; i <= 5; i++)); do
        # number of particles
        for k in "${particlecount[@]}"; do
            roslaunch aircap_pfuclt aircap_pfuclt.launch numRobots:=$i &
            sleep 3

            outputfoldername=$outputfolder
            outputfoldername+="robots$i/particles$k"
            cd $outputfoldername
            rosbag record -a -O attempt$j __name:=that_recording &

            sourcefoldername=$datafolder
            sourcefoldername+="robots$i"
            cd $sourcefoldername
            databag=$(find attempt$j*.bag | head -n 1)
            echo $databag
            rosbag play $databag
            
            echo "finished playing"
            rosnode kill /that_recording
            cd $scriptfolder
            ./cleanup.sh
        done
    done
done