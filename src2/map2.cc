#include <iostream>
#include <fstream>

using namespace std;

#include <boost/timer/timer.hpp>

// for sophus
#include "sophus/se3.h"

using Sophus::SE3;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
#include <vector>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace cv;
// const double fx = 481.2f;
// const double fy = -480.0f;
// const double cx = 319.5f;
// const double cy = 239.5f;
// void find_feature_matches(const Mat &img_1, const Mat &img_2,
//                           std::vector<KeyPoint> &keypoints_1,
//                           std::vector<KeyPoint> &keypoints_2,
//                           std::vector<DMatch> &matches)
// {
//     //-- 初始化
//     Mat descriptors_1, descriptors_2;
//     // used in OpenCV3
//     Ptr<FeatureDetector> detector = ORB::create();
//     Ptr<DescriptorExtractor> descriptor = ORB::create();
//     // use this if you are in OpenCV2
//     // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
//     // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//     //-- 第一步:检测 Oriented FAST 角点位置
//     detector->detect(img_1, keypoints_1);
//     detector->detect(img_2, keypoints_2);

//     //-- 第二步:根据角点位置计算 BRIEF 描述子
//     descriptor->compute(img_1, keypoints_1, descriptors_1);
//     descriptor->compute(img_2, keypoints_2, descriptors_2);

//     //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
//     vector<DMatch> match;
//     // BFMatcher matcher ( NORM_HAMMING );
//     matcher->match(descriptors_1, descriptors_2, match);

//     //-- 第四步:匹配点对筛选
//     double min_dist = 10000, max_dist = 0;

//     //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
//     for (int i = 0; i < descriptors_1.rows; i++)
//     {
//         double dist = match[i].distance;
//         if (dist < min_dist)
//             min_dist = dist;
//         if (dist > max_dist)
//             max_dist = dist;
//     }

//     // printf ( "-- Max dist : %f \n", max_dist );

//     // printf ( "-- Min dist : %f \n", min_dist );

//     //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
//     for (int i = 0; i < descriptors_1.rows; i++)
//     {
//         if (match[i].distance <= max(2 * min_dist, 30.0))
//         {
//             matches.push_back(match[i]);
//         }
//     }
// }
// void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
//                           std::vector<KeyPoint> keypoints_2,
//                           const std::vector<DMatch> &matches,
//                           Mat &R, Mat &t)
// {
//     // 相机内参,TUM Freiburg2
//     Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

//     //-- 把匹配点转换为vector<Point2f>的形式
//     vector<Point2f> points1;
//     vector<Point2f> points2;

//     for (auto &matche : matches)
//     {
//         points1.push_back(keypoints_1[matche.queryIdx].pt);
//         points2.push_back(keypoints_2[matche.trainIdx].pt);
//     }

//     //-- 计算基础矩阵
//     Mat fundamental_matrix;
//     fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
//     // cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

//     //-- 计算本质矩阵
//     // Point2d principal_point ( cx, cy );	//相机光心, TUM dataset标定值

//     // double focal_length = fx;			//相机焦距, TUM dataset标定值
//     Mat essential_matrix;
//     essential_matrix = findEssentialMat(points1, points2, K);

//     // cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

//     //-- 计算单应矩阵
//     // Mat homography_matrix;
//     // homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
//     // cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

//     //-- 从本质矩阵中恢复旋转和平移信息.
//     recoverPose(essential_matrix, points1, points2, K, R, t);
//     // cout<<"R is "<<endl<<R<<endl;
//     // cout<<"t is "<<endl<<t<<endl;
// }


// void startCV3(char *filePath1, char *filePath2, vector<double> points)
// {
//     Mat img1 = imread(filePath1);
//     Mat img2 = imread(filePath2);
//     vector<Eigen::Isometry3d> pose1;
//     vector<KeyPoint> keypoints_1, keypoints_2;
//     vector<DMatch> matches;
//     Mat R, t;
//     Isometry3d T2;


  
//         // find_feature_matches(before,img1,keypoints_1,keypoints_2,matches);
//             pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
//             Matrix3d R_d;
//             cv2eigen(R,R_d);
//             Quaterniond q(R_d);
//             Eigen::Isometry3d T1(q);
//             // T1.pretranslate(Eigen::Vector3d(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0)));
            

    
// }