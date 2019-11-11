// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_SAMPLES_PC_VIEWER_H_  // NOLINT
#define MYNTEYE_SAMPLES_PC_VIEWER_H_
#pragma once

#define WITH_OPENCV2

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "mynteyed/types.h"
#include "KeyFrame.h"
#include "System.h"
#include <pcl/common/transforms.h>

class PCViewer {
 public:
  using point_t = pcl::PointXYZRGBA;
  using pointcloud_t = pcl::PointCloud<point_t>;

  PCViewer(const MYNTEYE_NAMESPACE::CameraIntrinsics& cam_in, float cam_factor);
  ~PCViewer();

  bool Update(const cv::Mat &rgb, const cv::Mat& depth);
  bool UpdateDirectly(const cv::Mat &rgb, const cv::Mat& depth);
  bool UpdateDirectly2(const cv::Mat &rgb, const cv::Mat& depth, cv::Mat &Tcw);
  void generatePointCloud(ORB_SLAM2::KeyFrame* kf, cv::Mat& color, cv::Mat& depth, pcl::PointCloud<point_t>::Ptr cloud);
  bool UpdateDirectly21(const cv::Mat &rgb, const cv::Mat& depth, ORB_SLAM2::KeyFrame* pLastKeyFrame);
  bool UpdateDirectly22(const cv::Mat &rgb, const cv::Mat& depth, std::vector<float> vTcw);
  bool UpdateDirectly3();

  bool WasVisual() const;
  bool WasStopped() const;

 private:
  void Update(pointcloud_t::ConstPtr cloud);

  void ConvertToPointCloud(const cv::Mat &rgb, const cv::Mat& depth,
      pcl::PointCloud<point_t>::Ptr cloud);

  void Start();
  void Stop();

  void Run();

  void KeyboardCallback(const pcl::visualization::KeyboardEvent& event);

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

  MYNTEYE_NAMESPACE::CameraIntrinsics cam_in_;
  float cam_factor_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool generating_;

  std::thread thread_;
  bool running_;

  cv::Mat rgb_;
  cv::Mat depth_;

  pointcloud_t::ConstPtr cloud_;
  pointcloud_t::Ptr m_cloud;

  std::string save_dir_;

private:
  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  vector<ORB_SLAM2::KeyFrame*>       m_keyframes;
  vector<cv::Mat>         m_colorImgs;
  vector<cv::Mat>         m_depthImgs;
  mutex                   m_keyframeMutex;
  uint16_t                m_lastKeyframeSize =0;
};

#endif  // MYNTEYE_SAMPLES_PC_VIEWER_H_ NOLINT
