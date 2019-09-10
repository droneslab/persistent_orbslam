#!/bin/bash


#cd /home/droneslab/DATASET/cooke
#rosbag play session.orig.bag
#cd /media/droneslab/Data/Jarvis
#rosbag play -u 280 session_0.orig.bag
#rosbag play -s 280 -u 20 -r 0.2 session_0.orig.bag
#rosbag play -s 300 -u 230 session_0.orig.bag
#rosbag play -s 530 -u 20 -r 0.2 session_0.orig.bag
#rosbag play -s 550 session_0.orig.bag
#rosbag play -u 135 session_1.orig.bag
#rosbag play -s 135 -u 10 -r 0.2 session_1.orig.bag
#rosbag play -s 145 -u 145 session_1.orig.bag
#rosbag play -s 290 -u 20 -r 0.1 session_1.orig.bag
#rosbag play -s 310 -u 70 session_1.orig.bag
#rosbag play -s 380 -u 10 -r 0.05 session_1.orig.bag
#rosbag play -s 390 session_1.orig.bag
#rosbag play -u 75 session_2.orig.bag
#rosbag play -s 75 -u 15 -r 0.1 session_2.orig.bag
#rosbag play -s 90 -u 390 session_2.orig.bag
#rosbag play -s 480 -u 15 -r 0.2 session_2.orig.bag
#rosbag play -s 495 session_2.orig.bag
#rosbag play -u 230 session_3.orig.bag
#rosbag play -s 230 -u 15 -r 0.2 session_3.orig.bag
#rosbag play -s 245 session_3.orig.bag
#cd "/media/droneslab/Data/Bell"
#for i in {0..51}
#do
#	var=$(printf 'session_%d.orig.bag' "$i")
#	rosbag play -r 0.8 $var
#done
cd /home/zakieh/PhD/data/semantic_mapping/comparison/kidnap_360
rosbag play rgbd_dataset_freiburg2_360_kidnap.bag
cd /home/zakieh/PhD/orbslam_wifi
