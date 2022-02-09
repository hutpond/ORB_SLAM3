#!/bin/bash
pathDatasetEuroc='/home/lz/hutpond/dataset/orbslam' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Stereo Examples
echo "Launching MH01 with Stereo sensor"
./Stereo/stereo_euroc_test ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
