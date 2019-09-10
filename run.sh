#!/bin/bash

out_addr="$1"
rm -rf build/*
rm -rf Thirdparty/DBoW2/build/*
rm -rf Thirdparty/g2o/build/*
rm -rf Examples/ROS/ORB_SLAM2/build/*
./build.sh
./build_ros.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Examples/ROS
min_matches='' #10 15 20 50 100' 
for min_match in $min_matches;
do
	rm -rf ~/.ros/log/*
	rm -rf final_map.map
	var=$(printf 'log_%d.txt' "$min_match")
        var_map=$(printf '%s/map_%d.map' "$out_addr" "$min_match")
        var_traj=$(printf '%s/traj_%d.txt' "$out_addr" "$min_match")

	rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/cooke.yaml $min_match > $var&
	sleep 20
	PID=$!
	./run_data.sh
	kill -s SIGINT $PID	
    	sleep 20
	kill -9 $PID
    	mv $var $out_addr
    	mv final_map.map $var_map
    	mv KeyFrameTrajectory.txt $var_traj
done
