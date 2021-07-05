#!/bin/bash

./2D_feature_tracking SHITOMASI BRISK
./2D_feature_tracking SHITOMASI BRIEF
./2D_feature_tracking SHITOMASI ORB
./2D_feature_tracking SHITOMASI FREAK
./2D_feature_tracking SHITOMASI AKAZE
./2D_feature_tracking SHITOMASI SIFT

./2D_feature_tracking HARRIS BRISK
./2D_feature_tracking HARRIS BRIEF
./2D_feature_tracking HARRIS ORB
./2D_feature_tracking HARRIS FREAK
./2D_feature_tracking HARRIS AKAZE
./2D_feature_tracking HARRIS SIFT

./2D_feature_tracking FAST BRISK
./2D_feature_tracking FAST BRIEF
./2D_feature_tracking FAST ORB
./2D_feature_tracking FAST FREAK
./2D_feature_tracking FAST AKAZE
./2D_feature_tracking FAST SIFT

./2D_feature_tracking BRISK BRISK
./2D_feature_tracking BRISK BRIEF
./2D_feature_tracking BRISK ORB
./2D_feature_tracking BRISK FREAK
./2D_feature_tracking BRISK AKAZE
./2D_feature_tracking BRISK SIFT

./2D_feature_tracking ORB BRISK 
./2D_feature_tracking ORB BRIEF
./2D_feature_tracking ORB ORB
./2D_feature_tracking ORB FREAK
./2D_feature_tracking ORB AKAZE
./2D_feature_tracking ORB SIFT

./2D_feature_tracking AKAZE BRISK
./2D_feature_tracking AKAZE BRIEF
./2D_feature_tracking AKAZE ORB
./2D_feature_tracking AKAZE FREAK
./2D_feature_tracking AKAZE AKAZE
./2D_feature_tracking AKAZE SIFT

./2D_feature_tracking SIFT BRISK
./2D_feature_tracking SIFT BRIEF
./2D_feature_tracking SIFT ORB
./2D_feature_tracking SIFT FREAK
./2D_feature_tracking SIFT AKAZE
./2D_feature_tracking SIFT SIFT
