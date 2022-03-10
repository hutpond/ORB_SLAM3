#!/bin/bash
pathDatasetEuroc='/home/lz/hutpond/dataset/orbslam' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Stereo-Inertial Examples
echo "Launching MH02 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_euroc ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH02 ./Stereo-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_stereoi
