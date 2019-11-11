#include <iostream>
#define WITH_OPENCV2
#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/visualization/cloud_viewer.h>
#pragma GCC diagnostic pop

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"
#include "util/pc_viewer.h"
#include "pangolin/pangolin.h"
#include "Converter.h"
#include "KeyFrame.h"

#define CAMERA_FACTOR 1000.0

MYNTEYE_USE_NAMESPACE
using namespace std;
#define COMPILEDWITHC11

int MYNT_RGBD() {
    cout << "Hello World!" << endl;

  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  std::string argv[] = {"","/home/wood/tool/ORB_SLAM2/Vocabulary/ORBvoc.txt",
//                        "/home/wood/Documents/CPP/test_ORB_MYNT/MYNT_RGBD.yaml",
//                        "/home/wood/Documents/CPP/test_ORB_MYNT/MYNT_RGBD_20191023.yaml",
                        "/home/wood/Documents/CPP/test_ORB_MYNT/MYNT_RGBD_20191031.yaml",
                        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk",
                        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk/associations.txt"};
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

  OpenParams params(dev_info.index);
  {
    // Framerate: 30(default), [0,60], [30](STREAM_2560x720)
    params.framerate = 30;

    // Device mode, default DEVICE_ALL
    //   DEVICE_COLOR: IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH x
    //   DEVICE_DEPTH: IMAGE_LEFT_COLOR x IMAGE_RIGHT_COLOR x IMAGE_DEPTH ✓
    //   DEVICE_ALL:   IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH ✓
    // Note: ✓: available, x: unavailable, ?: depends on #stream_mode
    // params.dev_mode = DeviceMode::DEVICE_ALL;

    // Color mode: raw(default), rectified
     params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
     params.depth_mode = DepthMode::DEPTH_RAW;

    // Stream mode: left color only
//     params.stream_mode = StreamMode::STREAM_640x480;  // vga
    // params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    params.stream_mode = StreamMode::STREAM_2560x720;  // hd

    // Auto-exposure: true(default), false
    // params.state_ae = false;

    // Auto-white balance: true(default), false
    // params.state_awb = false;

    // IR Depth Only: true, false(default)
    // Note: IR Depth Only mode support frame rate between 15fps and 30fps.
    //     When dev_mode != DeviceMode::DEVICE_ALL,
    //       IR Depth Only mode not be supported.
    //     When stream_mode == StreamMode::STREAM_2560x720,
    //       frame rate only be 15fps in this mode.
    //     When frame rate less than 15fps or greater than 30fps,
    //       IR Depth Only mode will be not available.
     params.ir_depth_only = true;

    // Infrared intensity: 0(default), [0,10]
    params.ir_intensity = 7;

    // Colour depth image, default 5000. [0, 16384]
    params.colour_depth_value = 5000;
  }

  // Enable what process logics
  // cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

  // Enable image infos
  cam.EnableImageInfo(true);

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  bool is_left_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
  bool is_right_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
  bool is_depth_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH);

//  if (is_left_ok) cv::namedWindow("left color");
//  if (is_right_ok) cv::namedWindow("right color");
//  if (is_depth_ok) cv::namedWindow("depth");

  CVPainter painter;
  util::Counter counter;
  auto stream_intrinsics = cam.GetStreamIntrinsics(params.stream_mode);
  PCViewer viewer(stream_intrinsics.left, CAMERA_FACTOR);
  for (int i=0;i<25*500;i++) {
    cam.WaitForStream();
    counter.Update();

    cv::Mat left;
    if (is_left_ok) {
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img) {
        left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(left, CVPainter::TOP_LEFT);
        painter.DrawStreamData(left, left_color, CVPainter::TOP_RIGHT);
        painter.DrawInformation(left, util::to_string(counter.fps()),
            CVPainter::BOTTOM_RIGHT);
//        cv::imshow("left color", left);
      }
    }

    cv::Mat right;
    if (is_right_ok) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(right, CVPainter::TOP_LEFT);
        painter.DrawStreamData(right, right_color, CVPainter::TOP_RIGHT);
//        cv::imshow("right color", right);
      }
    }

    if (is_depth_ok) {
      auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
      if (image_depth.img) {
        cv::Mat depth;
//        if (params.depth_mode == DepthMode::DEPTH_COLORFUL) {
//          depth = image_depth.img->To(ImageFormat::DEPTH_BGR)->ToMat();
//        } else {
//          depth = image_depth.img->ToMat();
//        }
//        painter.DrawSize(depth, CVPainter::TOP_LEFT);
//        painter.DrawStreamData(depth, image_depth, CVPainter::TOP_RIGHT);
//        cv::imshow("depth", depth);

        if (image_depth.img && depth.empty()) {
          depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
        }

        if(!left.empty() && !depth.empty())
        {
//            cv::waitKey(1);
            struct timeval tv;
            gettimeofday(&tv,NULL);
//            printf("second:%ld\n",tv.tv_sec);  //秒
//            printf("millisecond:%ld\n",tv.tv_sec*1000 + tv.tv_usec/1000);  //毫秒
            double timestamp = ((tv.tv_sec*1000 + tv.tv_usec/1000)%1000000)/1000.0;

            bool bKeyFrame = false;
            vector<float> vTcw;
//            ORB_SLAM2::KeyFrame* pLastKeyFrame=NULL;
            cv::Mat Tcw = SLAM.TrackRGBD(left,depth,timestamp, &bKeyFrame, &vTcw);
//            cv::Mat Tcw = SLAM.TrackRGBD(left,depth,timestamp);

//            if(bKeyFrame)
//            {
//                cv::imwrite(to_string(timestamp)+"_grb.BMP",left);
//                vector<int> compression_params;
//                compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//                compression_params.push_back(0);
//                cv::imwrite(to_string(timestamp)+"_deep.png",depth, compression_params);
//            }
//            else {
//                bool tellme = true;
//            }

//            if(bKeyFrame)
//                viewer.UpdateDirectly22(left, depth, vTcw);
//            else
//                viewer.UpdateDirectly3();

        }
      }
    }

//    char key = static_cast<char>(cv::waitKey(1));
//    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
//      break;
//    }
//    if(key == 's')
//    {
//        cv::imwrite("/home/wood/Documents/CPP/test_ORB_MYNT/build-alpha-Desktop-Release/qipang.jpg",left);
//    }
  }

  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}


int MYNT_MONO() {
    cout << "Hello World!" << endl;

  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  std::string argv[] = {"","/home/wood/tool/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                        "/home/wood/Documents/CPP/test_ORB_MYNT/MYNT_MONO.yaml",
                        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk",
                        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk/associations.txt"};
  ORB_SLAM2::System SLAM(argv[1].c_str(),argv[2].c_str(),ORB_SLAM2::System::MONOCULAR,true);

  OpenParams params(dev_info.index);
  {
    // Framerate: 30(default), [0,60], [30](STREAM_2560x720)
    params.framerate = 15;

    // Device mode, default DEVICE_ALL
    //   DEVICE_COLOR: IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH x
    //   DEVICE_DEPTH: IMAGE_LEFT_COLOR x IMAGE_RIGHT_COLOR x IMAGE_DEPTH ✓
    //   DEVICE_ALL:   IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH ✓
    // Note: ✓: available, x: unavailable, ?: depends on #stream_mode
    // params.dev_mode = DeviceMode::DEVICE_ALL;

    // Color mode: raw(default), rectified
    // params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // params.depth_mode = DepthMode::DEPTH_GRAY;

    // Stream mode: left color only
//     params.stream_mode = StreamMode::STREAM_640x480;  // vga
    // params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    params.stream_mode = StreamMode::STREAM_2560x720;  // hd

    // Auto-exposure: true(default), false
    // params.state_ae = false;

    // Auto-white balance: true(default), false
    // params.state_awb = false;

    // IR Depth Only: true, false(default)
    // Note: IR Depth Only mode support frame rate between 15fps and 30fps.
    //     When dev_mode != DeviceMode::DEVICE_ALL,
    //       IR Depth Only mode not be supported.
    //     When stream_mode == StreamMode::STREAM_2560x720,
    //       frame rate only be 15fps in this mode.
    //     When frame rate less than 15fps or greater than 30fps,
    //       IR Depth Only mode will be not available.
    // params.ir_depth_only = true;

    // Infrared intensity: 0(default), [0,10]
    params.ir_intensity = 10;

    // Colour depth image, default 5000. [0, 16384]
    params.colour_depth_value = 5000;
  }

  // Enable what process logics
  // cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

  // Enable image infos
  cam.EnableImageInfo(true);

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  bool is_left_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
  bool is_right_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
  bool is_depth_ok = cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH);

//  if (is_left_ok) cv::namedWindow("left color");
//  if (is_right_ok) cv::namedWindow("right color");
//  if (is_depth_ok) cv::namedWindow("depth");

  CVPainter painter;
  util::Counter counter;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    if (is_left_ok) {
      auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
      if (left_color.img) {
        cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(left, CVPainter::TOP_LEFT);
        painter.DrawStreamData(left, left_color, CVPainter::TOP_RIGHT);
        painter.DrawInformation(left, util::to_string(counter.fps()),
            CVPainter::BOTTOM_RIGHT);
//        cv::imshow("left color", left);
      }
    }

    cv::Mat right;
    if (is_right_ok) {
      auto right_color = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
      if (right_color.img) {
        right = right_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
        painter.DrawSize(right, CVPainter::TOP_LEFT);
        painter.DrawStreamData(right, right_color, CVPainter::TOP_RIGHT);
//        cv::imshow("right color", right);
      }
    }

    if (is_depth_ok) {
      auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
      if (image_depth.img) {
        cv::Mat depth;
        if (params.depth_mode == DepthMode::DEPTH_COLORFUL) {
          depth = image_depth.img->To(ImageFormat::DEPTH_BGR)->ToMat();
        } else {
          depth = image_depth.img->ToMat();
        }
        painter.DrawSize(depth, CVPainter::TOP_LEFT);
        painter.DrawStreamData(depth, image_depth, CVPainter::TOP_RIGHT);
//        cv::imshow("depth", depth);

        if(!right.empty() && !depth.empty())
        {
//            cv::waitKey(1);
            struct timeval tv;
            gettimeofday(&tv,NULL);
//            printf("second:%ld\n",tv.tv_sec);  //秒
//            printf("millisecond:%ld\n",tv.tv_sec*1000 + tv.tv_usec/1000);  //毫秒

            SLAM.TrackMonocular(right,((tv.tv_sec*1000 + tv.tv_usec/1000)%1000000)/1000.0);
        }
      }
    }

//    char key = static_cast<char>(cv::waitKey(1));
//    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
//      break;
//    }
//    if(key == 's')
//    {
//        cv::imwrite("qipang.jpg",right);
//    }
  }

  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

int rbgd_main(int argc, std::string *argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = argv[4];
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of imaduration_castges and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

//        // Pass the image to the SLAM system
//        SLAM.TrackRGBD(imRGB,imD,tframe);

        bool bKeyFrame = false;
        vector<float> vTcw;
//            ORB_SLAM2::KeyFrame* pLastKeyFrame=NULL;
        cv::Mat Tcw = SLAM.TrackRGBD(imRGB,imD,tframe, &bKeyFrame, &vTcw);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

int main_Point() {
  // About warning in vtkOutputWindow with prebuilt version PCL on Windows.
  // Please see: Ugrade vtk api to 8.1 for 1.9,
  //   https://github.com/PointCloudLibrary/pcl/issues/2619
  // vtkObject::GlobalWarningDisplayOff();

  Camera cam;
  DeviceInfo dev_info;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  util::print_stream_infos(cam, dev_info.index);

  std::cout << "Open device: " << dev_info.index << ", "
      << dev_info.name << std::endl << std::endl;

  OpenParams params(dev_info.index);
  params.color_mode = ColorMode::COLOR_RECTIFIED;
  // Note: must set DEPTH_RAW to get raw depth values for points
  params.depth_mode = DepthMode::DEPTH_RAW;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 4;

  StreamMode stream_mode = params.stream_mode;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Open device success" << std::endl << std::endl;

  std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

  cv::namedWindow("color");

  auto stream_intrinsics = cam.GetStreamIntrinsics(stream_mode);

  CVPainter painter;
  PCViewer viewer(stream_intrinsics.left, CAMERA_FACTOR);
  util::Counter counter;
  cv::Mat color;
  cv::Mat depth;
  for (;;) {
    cam.WaitForStream();
    counter.Update();

    auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (image_color.img && color.empty()) {
      color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
    }
    if (image_depth.img && depth.empty()) {
      depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
    }

    if (color.empty() || depth.empty()) { continue; }

//    cout<<depth<<endl;
//    cv::imshow("depth", depth);
    viewer.UpdateDirectly(color, depth);

    painter.DrawSize(color, CVPainter::TOP_LEFT);
    painter.DrawStreamData(color, image_color, CVPainter::TOP_RIGHT);
    painter.DrawInformation(color, util::to_string(counter.fps()),
        CVPainter::BOTTOM_RIGHT);

    cv::imshow("color", color);

    color.release();
    depth.release();

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (viewer.WasStopped()) {
      break;
    }
  }

  cam.Close();
  cv::destroyAllWindows();
  return 0;
}


int Test_Pangolin()
{
    pangolin::CreateWindowAndBind("Plane_Slam: Map Viewer",1024,768);

    // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
    glEnable(GL_DEPTH_TEST);
    // 在OpenGL中使用颜色混合
    glEnable(GL_BLEND);
    // 选择混合选项
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //定义按钮面板
    //新建按钮和选择框
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,0.2);
    //第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::Var<bool> menu("menu.test",true,true);

    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    // 定义地图面板
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.2, 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //定义图片面板
    pangolin::View& rgb_image = pangolin::Display("rgb")
      .SetBounds(0,0.2,0.2,0.4,1024.0f/768.0f)
      .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::View& depth_image = pangolin::Display("depth")
      .SetBounds(0,0.2,0.4,0.6,1024.0f/768.0f)
      .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    //初始化
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);


//     cv::namedWindow("Plane_Slam: rgb");
//  cv::namedWindow("Plane_Slam: depth");

    bool Follow = true;
//     bool LocalizationMode = false;

    while(1)
    {
        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //画出点云地图
        //_MapDrawer->DrawPointCloud();


        //cv::Mat 格式读取并显示RGB图片
        cv::Mat rgb = cv::imread("/home/wood/Documents/CPP/test_ORB_MYNT/sample/231.307000_grb.BMP");
        //_FrameDrawer->DrawFrame(rgb);
        imageTexture.Upload(rgb.data,GL_BGR,GL_UNSIGNED_BYTE);

        //display the image
        rgb_image.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();
        //注意，这里由于Upload函数无法将cv::Mat格式的图片数据作为输入，因此使用opencv的data函数将Mat格式的数据变为uchar格式，但是opencv中Mat存储图片是自下而上的，单纯的渲染所渲染出来的图片是倒置的，因此需使用RenderToViewportFlipY（）函数进行渲染，将原本上下倒置的图片进行自下而上渲染，使显示的图片是正的。

        pangolin::FinishFrame();

    }

}

int main()
{
    cout << "Hello World!" << endl;

    string agrv[]={"",
        "/home/wood/tool/ORB_SLAM2/Vocabulary/ORBvoc.txt",
        "/home/wood/tool/IndoorMapping/test/TUM1.yaml",
        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk",
        "/home/wood/share/database/TUM/rgbd_dataset_freiburg1_desk/associations.txt"
    };

    MYNT_RGBD();
//    MYNT_MONO();
//    main_Point();
//    Test_Pangolin();
//    rbgd_main(5,agrv);
    return 0;
}
