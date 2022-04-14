#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp> // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
using namespace cv;
using namespace Eigen;
// 计算点云并拼接
// 相机内参
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;
double depthScale = 1000.0;
void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);
void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t);

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void bundleAdjustment(const vector<Point3f> &pts1,
                      const vector<Point3f> &pts2,
                      Mat &R, Mat &t);
//   Point2d pixel2cam(const Point2d &p, const Mat &K)
// {
//     return Point2d(
//         (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
//         (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
// }

int main(int argc, char **argv)
{
    Mat ref;
    vector<cv::Mat> colorImgs, depthImgs; // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;      // 相机位姿
    vector<Eigen::Isometry3d> pose1;
    // ifstream fin("/home/guojiawei/cv/cv/after/data/pose.txt");
    // if (!fin)
    // {
    //     cerr << "cannot find pose file" << endl;
    //     return 1;
    // }

    for (int i = 0; i < 5; i++)
    {
        boost::format fmt("/home/guojiawei//cv/cv/after/data/%s/%d.%s"); //图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像

        // double data[7] = {0};
        // for (int i = 0; i < 7; i++)
        // {
        //     fin >> data[i];
        // }
        // Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        // Eigen::Isometry3d T(q);
        // T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        // poses.push_back( T );

        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches;
        Mat R, t;

        if (i >= 1)
        {
            // ushort ds=depthImgs[i].ptr(20)[20];
            // cout<<ds<<endl;
            // if(i==1){
            //     Mat test=depthImgs[i];
            // // cout<<test<<endl;
            // // imwrite()
            // }
            // imshow("test",test);
            find_feature_matches(colorImgs[i - 1], colorImgs[i], keypoints_1, keypoints_2, matches);
            cout << "一共找到了" << matches.size() << "组匹配点" << endl;

            // 建立3D点
            // Mat depth1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED); // 深度图为16位无符号数，单通道图像
            // Mat depth2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED); // 深度图为16位无符号数，单通道图像
            Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

            vector<Point3f> pts1, pts2;


            for (DMatch m : matches)
            {
                ushort d1 = depthImgs[i - 1].ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
                ushort d2 = depthImgs[i].ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
                // cout<<d2<<endl;
                // cout<<int(keypoints_2[m.trainIdx].pt.y)<<endl;
                // cout<<int(keypoints_2[m.trainIdx].pt.x)<<endl;
                if (d1 == 0 || d2 == 0) // bad depth
                    continue;
                Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
                Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
                float dd1 = float(d1) / depthScale;
                float dd2 = float(d2) / depthScale;
                pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
                pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
            }
            // cout<<pts2.size()<<endl;

            // cout << "3d-3d pairs: " << pts1.size() << endl;
            pose_estimation_3d3d(pts1, pts2, R, t);
            // cout << "ICP via SVD results: " << endl;
            // cout << "R = " << R << endl;
            // cout << "t = " << t << endl;
            // cout << "R_inv = " << R.t() << endl;
            // cout << "t_inv = " << -R.t() * t << endl;

            // cout << "calling bundle adjustment" << endl;

            bundleAdjustment(pts1, pts2, R, t);

            Matrix3d R_d;
            cv2eigen(R, R_d);
            Quaterniond q(R_d);
            Eigen::Isometry3d T1(q);
            T1.pretranslate(Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
            if (pose1.size() != 0)
            {
                Eigen::Isometry3d temp = pose1[pose1.size() - 1];
                pose1.push_back(temp * T1);
                //  Eigen::Isometry3d T3=poses[i-1].inverse()*poses[i];
                //  cout<<T1.matrix()<<endl;
                // // cout<<R<<endl;
                // // cout<<t<<endl;
                //  cout<<"------"<<endl;
                //  cout<<T3.matrix()<<endl;

                
                // int te = pose1.size();
                // cout << "???" << endl;
            }
            else
            {
                pose1.push_back(T1);
                // cout<<T1.matrix()<<endl;
                // cout<<"???"<<endl;
            }
        }

        // Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        // Eigen::Isometry3d T(q);
        // T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        // poses.push_back(T);
    }

    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < 5; i++)
    {
        PointCloud::Ptr current(new PointCloud);
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T;
        // Eigen::Isometry3d T2;
        if (i > 0)
        {
            T = pose1[i-1];
            // std::cout<<Eigen::Isometry3d(T).matrix()<<endl;
            // cout<<"------"<<endl;
            // T2=poses[0].inverse()  *poses[i];
        // cout<<(poses[0].inverse()  *poses[i]).matrix()<<endl;
        
        }
        
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0)
                    continue; // 为0表示没有测量到
                if (d >= 7000)
                    continue; // 深度太大时不稳定，去掉

                // std::cout<<color.rows<<endl;   
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld;
                if (i > 0)
                {
                    pointWorld = T * point;

                }
                else
                {
                    pointWorld = point;
                }

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistical removal
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*pointCloud) += *tmp;
    }

    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;

    // voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(0.01, 0.01, 0.01); // resolution
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);

    cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    return 0;
}
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map(_point);
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    bool read(istream &in)
    {
        return true;
    }
    bool write(ostream &out) const
    {
        return true;
    }

protected:
    Eigen::Vector3d _point;
};
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

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    // printf("-- Max dist : %f \n", max_dist);
    // printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 70.0))


        {
            matches.push_back(match[i]);
        }
    }
}

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    // cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
    }

    // cout << "U=" << U << endl;
    // cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
    cout<< R_ * Eigen::Vector3d(p2.x, p2.y, p2.z)<<endl;
    // convert to cv::Mat
    R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
         R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2));
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
    cout<<t<<endl;
    // cout<<"???"<<endl;
}

void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    // 初始化g2o
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // pose维度为 6, landmark 维度为 3
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>());
    // Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly *> edges;
    for (size_t i = 0; i < pts1.size(); i++)
    {
        EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *>(pose));
        edge->setMeasurement(Eigen::Vector3d(
            pts1[i].x, pts1[i].y, pts1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    //     cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    //     cout << endl
    //          << "after optimization:" << endl;
    //     cout << "T=" << endl
    //          << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}
