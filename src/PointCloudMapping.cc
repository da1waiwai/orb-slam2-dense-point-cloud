//
// Created by ichigoi7e on 17/07/2018.
//

#include "Converter.h"
#include "PointCloudMapping.h"

#include <KeyFrame.h>
#include <boost/make_shared.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    m_bkeyFrameUpdated = false;
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
//    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
//    for ( int m=0; m<depth.rows; m+=3 )
    for ( int m=0; m<depth.rows; m+=2 )//hzg
    {
//        for ( int n=0; n<depth.cols; n+=3 )
        for ( int n=0; n<depth.cols; n+=2 )    //hzg
        {
            float d = depth.ptr<float>(m)[n];
//            if (d < 0.01 || d>0.1)
//                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

//    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

void PointCloudMapping::viewer()
{
    cout<<"PointCloudMapping::viewer() 01"<<endl;
//    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.setCameraPosition(0, 0, -0.1, 0, -1, 0);
//    viewer.addCoordinateSystem(0.1);
    cout<<"PointCloudMapping::viewer() 02"<<endl;

    bool bFirst = true;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(1);

        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }

        size_t badKeyFrameN=0;
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(lastKeyframeSize != N)
        {
            cout<<"PointCloudMapping::viewer() N="<<N<<endl;//接下来这段代码不稳定，小概率会引起程序中断，参考下面同名注释的函数可能可以解决
            globalMap = boost::make_shared< PointCloud >( );
            for ( size_t i=0; i<N; i++ )
            {
                if(keyframes[i]->isBad())
                {
                    badKeyFrameN++;
                    continue;
                }
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
            }
            if(bFirst)
            {
                bFirst = false;
                viewer.addPointCloud(globalMap);
            }
            else
            {
                viewer.updatePointCloud(globalMap);
            }
            cout<<",badN="<<badKeyFrameN;
            cout<<",updatePointCloud Point size="<<globalMap->size()<<endl;
            lastKeyframeSize = N;
        }
    }
    pcl::io::savePCDFileASCII("map.pcd", *globalMap);
}

//void PointCloudMapping::viewer()
//{
//    cout<<"PointCloudMapping::viewer() 01"<<endl;
////    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
//    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
//    viewer.setCameraPosition(0, 0, -0.1, 0, -1, 0);
////    viewer.addCoordinateSystem(0.1);
//    cout<<"PointCloudMapping::viewer() 02"<<endl;

//    bool bFirst = true;
////    while(1)
//    while (!viewer.wasStopped())
//    {
//        viewer.spinOnce(1);

//        {
//            unique_lock<mutex> lck_shutdown( shutDownMutex );
//            if (shutDownFlag)
//            {
//                break;
//            }
//        }
////        {
////            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
////            keyFrameUpdated.wait( lck_keyframeUpdated );
////        }

//        // keyframe is updated
//        size_t N=0;
//        {
//            unique_lock<mutex> lck( keyframeMutex );
//            N = keyframes.size();
//        }

////        cout<<"PointCloudMapping::viewer() keyframes.size(): N="<<N<<endl;
//        for ( size_t i=lastKeyframeSize; i<N ; i++ )
//        {
//            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );

//            *globalMap += *p;

//            if(bFirst)
//            {
//                bFirst = false;
//                viewer.addPointCloud(globalMap);
//            }
//            else
//            {
//                viewer.updatePointCloud(globalMap);
////                Eigen::Affine3f aaa = viewer.getViewerPose();
////                cout<<"getViewerPose:"<<aaa.matrix()<<endl;
////                cv::Mat R = keyframes[i]->GetPose().rowRange(0,3).colRange(0,3).clone();
////                vector<float> q = Converter::toQuaternion(R);
////                cv::Mat t = keyframes[i]->GetCameraCenter();
////                cout << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
////                  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
////                vTcw->push_back(t.at<float>(0));
////                vTcw->push_back(t.at<float>(1));
////                vTcw->push_back(t.at<float>(2));
////                vTcw->push_back(q[0]);
////                vTcw->push_back(q[1]);
////                vTcw->push_back(q[2]);
////                vTcw->push_back(q[3]);
////                viewer.setCameraPosition(0, 0, -0.1, 0, -1, -1);
////                viewer.addCoordinateSystem(0.1);
//            }
//        }
////        PointCloud::Ptr tmp(new PointCloud());
////        voxel.setInputCloud( globalMap );
////        voxel.filter( *tmp );//hzg::will error
////        globalMap->swap( *tmp );
////        viewer.showCloud( globalMap );


////        cout << "show global map, size=" << globalMap->points.size() << endl;
//        lastKeyframeSize = N;

//    }
//    pcl::io::savePCDFileASCII("map.pcd", *globalMap);
//}
