/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{  
    cv::VideoCapture capture(2);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ROBvoc.txt: 特征空间的划分和Bag-of-Words Vector的计算，
    // 采用Bag-of-Words的方式将图像中的多个局部描述符表示成一个定长的全局特征，
    // 而Vocabulary Tree是局部描述符量化和索引的一种高效数据结构。
    // Vocabulary Tree实际上就是通过学习(Learning)的方法实现对原始特征空间的一种划分而已。
    // EuRoCTest.yaml：单目相机内参
    ORB_SLAM3::System SLAM("../Vocabulary/ORBvoc.txt",
      "./Monocular/EuRoCTest.yaml", ORB_SLAM3::System::MONOCULAR, true);

    int index = -1;
    int interval = 3;
    while (true)
    {
      // Read image from file
      cv::Mat im;
      capture >> im;
      ++ index;
      if ((index % interval) != 0) {
        continue;
      }
      im = cv::Mat(im, cv::Rect(640, 0, 640, 480));
      if(im.empty())
      {
          cerr << endl << "Failed to read video " << endl;
          return 1;
      }

      auto microseconds = std::chrono::duration_cast<std::chrono::nanoseconds> 
        (std::chrono::high_resolution_clock::now().time_since_epoch()).count(); 
      double ttrack = microseconds / 1.0e9;
      // std::cout << ttrack << endl;

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im,ttrack);

#ifdef REGISTER_TIMES
      double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
      SLAM.InsertTrackTime(t_track);
#endif

      // SLAM.ChangeDataset();
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}
