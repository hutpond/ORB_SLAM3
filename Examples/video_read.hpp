#ifndef VIDEO_READ_H_
#define VIDEO_READ_H_

#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <opencv2/opencv.hpp>

class VideoRead
{
public:
  VideoRead() {

  }
  ~VideoRead() {
    stop();
  }

  void start(int index) {
    capture_ = cv::VideoCapture(index);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    auto thread = new std::thread(&VideoRead::read_frame, this);
    read_thread_.reset(thread);
  }
  void start(const std::string &source, int interval = 5) {
    capture_ = cv::VideoCapture(source);
    camera_falg_ = false;
    frame_interval_ = interval;
  }
  void stop() {
    if (camera_falg_) {
      read_flag_ = false;
      read_thread_->join();
    }
    capture_.release();
  }
  cv::Mat get_frame() {
      cv::Mat frame;
      if (camera_falg_) {
        std::lock_guard<std::mutex> gaurd(mutex_);
        frame = frame_.clone();
      }
      else {
        for (int i = 0; i < frame_interval_; ++i) {
          capture_ >> frame;
        }
      }
      return frame;
  }

protected:
  void read_frame() {
    read_flag_ = true;
    while (read_flag_ && capture_.isOpened()) {
      {
        std::lock_guard<std::mutex> gaurd(mutex_);
        capture_ >> frame_;
      }
      if (frame_.empty()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    read_flag_ = false;
  }

private:
  std::shared_ptr<std::thread> read_thread_;
  std::mutex mutex_;
  std::atomic_bool read_flag_;
  cv::VideoCapture capture_;
  cv::Mat frame_;

  bool camera_falg_{true};
  int frame_interval_;
};

#endif  // VIDEO_READ_H_
