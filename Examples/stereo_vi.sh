#!/bin/bash
pathDatasetEuroc='/home/lz/hutpond/dataset/orbslam' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Stereo Examples
echo "Launching V202 with Stereo sensor"
./Stereo/stereo_euroc ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuroc"/V202 ./Stereo/EuRoC_TimeStamps/V202.txt dataset-V202_stereo
