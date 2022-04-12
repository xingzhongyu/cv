#include "map.h"

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

bool init = false;
const int boarder = 20;
const int width = 640;
const int height = 480;
const double fx = 481.2f;
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 3;
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
const double min_cov = 0.1;
const double max_cov = 10;

void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    const std::vector<DMatch> &matches,
    Mat &R, Mat &t);

bool update(
    const Mat &ref,
    const Mat &curr,
    const SE3 &T_C_R,
    Mat &depth,
    Mat &depth_cov2);
bool epipolarSearch(
    const Mat &ref,
    const Mat &curr,
    const SE3 &T_C_R,
    const Vector2d &pt_ref,
    const double &depth_mu,
    const double &depth_cov,
    Vector2d &pt_curr,
    Vector2d &epipolar_direction);
bool updateDepthFilter(
    const Vector2d &pt_ref,
    const Vector2d &pt_curr,
    const SE3 &T_C_R,
    const Vector2d &epipolar_direction,
    Mat &depth,
    Mat &depth_cov2);
std::string getDepthData(
    Mat &depth

);

double NCC(const Mat &ref, const Mat &curr, const Vector2d &pt_ref, const Vector2d &pt_curr);
Point2d pixel2cam(const Point2d &p, const Mat &K);

inline double getBilinearInterpolatedValue(const Mat &img, const Vector2d &pt)
{
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - floor(pt(0, 0));
    double yy = pt(1, 0) - floor(pt(1, 0));
    return ((1 - xx) * (1 - yy) * double(d[0]) +
            xx * (1 - yy) * double(d[1]) +
            (1 - xx) * yy * double(d[img.step]) +
            xx * yy * double(d[img.step + 1])) /
           255.0;
}
inline Vector3d px2cam(const Vector2d px)
{
    return Vector3d(
        (px(0, 0) - cx) / fx,
        (px(1, 0) - cy) / fy,
        1);
}
Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return {
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)};
}
inline Vector2d cam2px(const Vector3d p_cam)
{
    return Vector2d(
        p_cam(0, 0) * fx / p_cam(2, 0) + cx,
        p_cam(1, 0) * fy / p_cam(2, 0) + cy);
}
inline bool inside(const Vector2d &pt)
{
    return pt(0, 0) >= boarder && pt(1, 0) >= boarder && pt(0, 0) + boarder < width && pt(1, 0) + boarder <= height;
}
vector<SE3> poses_T_W_C;
SE3 pose_ref_T_W_C;
Mat ref;
int i = 0;
double init_depth = 3.0; // 深度初始值
double init_cov2 = 3.0;
Mat depth(height, width, CV_64F, init_depth); // 深度图
Mat depth_cov2(height, width, CV_64F, init_cov2);
// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 新建一个点云
PointCloud::Ptr pointCloud(new PointCloud);

std::string startCV(char *filePath, double *data)
{
    string str;
    SE3 pose_T_W_C = SE3(Quaterniond(data[6], data[3], data[4], data[5]),
                         Vector3d(data[0], data[1], data[2]));
    poses_T_W_C.push_back(pose_T_W_C);
    if (!init)
    {
        // printf("%d",11);

        ::ref = imread(filePath, 0);

        pose_ref_T_W_C = pose_T_W_C;
        // imwrite("/home/xzy/object_node2/test/depth0.png", depth);
        init = true;
        printf("%d\n", i);
        str = getDepthData(depth);
        i++;
    }
    else
    {
        Mat curr = imread(filePath, 0);
        if (curr.data == nullptr)
        {
            return str;
        }
        SE3 pose_curr_T_W_C = pose_T_W_C;
        SE3 pose_T_C_R = pose_curr_T_W_C.inverse() * pose_ref_T_W_C;
        update(::ref, curr, pose_T_C_R, depth, depth_cov2);
        // string filename = "/home/xzy/object_node2/test/depth" + to_string(i) + ".png";
        // imwrite(filename, depth);
        str = getDepthData(depth);
        // imshow("depth", depth * 0.4);
        // waitKey(1);
        printf("%d\n", i);
        // cout<<filename<<endl;
        i++;
    }
    return str;
}
vector<Mat> imgs;
vector<Isometry3d> Ts;

void changeInit()
{
    init = false;
    depth = Mat(height, width, CV_64F, init_depth);
    depth_cov2 = Mat(height, width, CV_64F, init_cov2);
    pointCloud = ((new PointCloud)->makeShared());
    
   
}
vector<KeyPoint> keypoints_1, keypoints_2;
vector<DMatch> matches;
Mat R, t;
Matrix3d R_d;

std::string startCV2(char *filePath)
{
    string str;
    if (!init)
    {

        ::ref = imread(filePath, 0);

        init = true;
        str = getDepthData(depth);

        printf("%d\n", i);
        i++;
    }
    else
    {
        Mat curr = imread(filePath, 0);
        if (curr.data == nullptr)
        {
            return str;
        }
        find_feature_matches(::ref, curr, keypoints_1, keypoints_2, matches);
        pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
        cv::cv2eigen(R, R_d);
        SE3 pose_T_C_R1 = SE3(R_d, Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
        update(::ref, curr, pose_T_C_R1, depth, depth_cov2);
        str = getDepthData(depth);

        // imshow("depth",depth*0.4);
        // waitKey(1);
        printf("%d\n", i);
        i++;
    }
    return str;
}


// 对整个深度图进行更新
bool update(const Mat &ref, const Mat &curr, const SE3 &T_C_R, Mat &depth, Mat &depth_cov2)
{
    for (int x = boarder; x < width - boarder; x++)
    {
        for (int y = boarder; y < height - boarder; y++)
        {
            // 遍历每个像素
            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov) // 深度已收敛或发散
                continue;
            // 在极线上搜索 (x,y) 的匹配
            Vector2d pt_curr; // pt_curr当前点
            Vector2d epipolar_direction;
            bool ret = epipolarSearch(
                ref,
                curr,
                T_C_R,
                Vector2d(x, y),
                depth.ptr<double>(y)[x],
                sqrt(depth_cov2.ptr<double>(y)[x]),
                pt_curr,
                epipolar_direction);

            if (ret == false) // 匹配失败
                continue;

            // 取消该注释以显示匹配
            // showEpipolarMatch(ref, curr, Vector2d(x, y), pt_curr);

            // 匹配成功，更新深度图
            updateDepthFilter(Vector2d(x, y), pt_curr, T_C_R, epipolar_direction, depth, depth_cov2);
        }
    }

    cout << "深度图已经更新" << endl;

    return true;
}
static std::string base64Encode(const unsigned char *Data, int DataByte)
{
    //编码表
    const char EncodeTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    //返回值
    std::string strEncode;
    unsigned char Tmp[4] = {0};
    int LineLength = 0;
    for (int i = 0; i < (int)(DataByte / 3); i++)
    {
        Tmp[1] = *Data++;
        Tmp[2] = *Data++;
        Tmp[3] = *Data++;
        strEncode += EncodeTable[Tmp[1] >> 2];
        strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
        strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
        strEncode += EncodeTable[Tmp[3] & 0x3F];
        if (LineLength += 4, LineLength == 76)
        {
            strEncode += "\r\n";
            LineLength = 0;
        }
    }
    //对剩余数据进行编码
    int Mod = DataByte % 3;
    if (Mod == 1)
    {
        Tmp[1] = *Data++;
        strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
        strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
        strEncode += "==";
    }
    else if (Mod == 2)
    {
        Tmp[1] = *Data++;
        Tmp[2] = *Data++;
        strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
        strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
        strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
        strEncode += "=";
    }

    return strEncode;
}
string getDepthData(Mat &depth)
{
    Mat Gray = Mat(depth.rows, depth.cols, CV_8UC1);
    vector<uchar> buf;
    int i = 0;
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            double d = depth.ptr<double>(m)[n];
            d *= 0.4;

            // color.push_back(d*255.0f);
            Gray.ptr(m)[n] = d * 255.0f;
        }
    }
    imencode(".png", Gray, buf);
    unsigned char *enc_msg = reinterpret_cast<unsigned char *>(buf.data());
    std::string encoded = base64Encode(enc_msg, buf.size());
    return encoded;
}
/*********************************************主要算法**************************************************/
// 极线搜索
// 方法见书 12.2 12.3 两节
bool epipolarSearch(
    const Mat &ref, const Mat &curr,
    const SE3 &T_C_R, const Vector2d &pt_ref,
    const double &depth_mu, const double &depth_cov, // depth_mu:深度均值depth_cov深度方差
    Vector2d &pt_curr, Vector2d &epipolar_direction)
{
    Vector3d f_ref = px2cam(pt_ref); //像素坐标系转换成相机坐标系
    // cout<<"f_ref:"<<f_ref<<endl;
    f_ref.normalize(); //归一化
    // cout<<"f_ref.normalize()"<<f_ref<<endl;
    Vector3d P_ref = f_ref * depth_mu; // 参考帧的 P 向量

    Vector2d px_mean_curr = cam2px(T_C_R * P_ref);                             // 按深度均值投影的像素
    double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov; //(u-3q,u+3q)
    if (d_min < 0.1)
        d_min = 0.1;
    Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min)); // 按最小深度投影的像素
    Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max)); // 按最大深度投影的像素

    Vector2d epipolar_line = px_max_curr - px_min_curr; // 极线（线段形式）
    epipolar_direction = epipolar_line;                 // 极线方向
    epipolar_direction.normalize();                     //对所有数据进行归一化，相当于等比例缩小
    double half_length = 0.5 * epipolar_line.norm();    // 极线线段的半长度
    if (half_length > 100)
        half_length = 100; // 我们不希望搜索太多东西

    // 取消此句注释以显示极线（线段）
    // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );

    // 在极线上搜索，以深度均值点为中心，左右各取半长度
    double best_ncc = -1.0;
    Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7)
    {                                                             // l+=sqrt(2)
        Vector2d px_curr = px_mean_curr + l * epipolar_direction; // 待匹配点
        if (!inside(px_curr))
            continue;
        // 计算待匹配点与参考帧的 NCC
        double ncc = NCC(ref, curr, pt_ref, px_curr); // ref:参考图像curr:当前图像pt_ref:参考点px_curr:待匹配点
        if (ncc > best_ncc)
        {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }
    if (best_ncc < 0.85f) // 只相信 NCC 很高的匹配
        return false;
    pt_curr = best_px_curr;
    return true;
}
/*****************************计算两张图像像素块的相关性***********************************************/
double NCC(
    const Mat &ref, const Mat &curr,
    const Vector2d &pt_ref, const Vector2d &pt_curr)
{ // ref:参考图像curr:当前图像pt_ref:参考点px_curr:待匹配点
    // 零均值-归一化互相关
    // 先算均值
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
    for (int x = -ncc_window_size; x <= ncc_window_size; x++)
        for (int y = -ncc_window_size; y <= ncc_window_size; y++)
        {
            double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
            mean_ref += value_ref;
            //依次累加窗口内的值，得到参考帧中极线上某个点所在块内的灰度值之和
            double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
            mean_curr += value_curr;
            //依次累加窗口内的值，得到当前帧中极线上的对应点所在块内的灰度值之和，注意这里用的是双线性插值法
            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }
    //除以面积，得到均值
    mean_ref /= ncc_area;
    mean_curr /= ncc_area;

    // 计算 Zero mean NCC(公式13.12)
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++)
    {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10); // 防止分母出现零
}
//更新深度滤波器
bool updateDepthFilter(
    const Vector2d &pt_ref,
    const Vector2d &pt_curr,
    const SE3 &T_C_R,
    const Vector2d &epipolar_direction,
    Mat &depth,      //深度均值
    Mat &depth_cov2) //深度方差
{

    // 用三角化计算深度
    SE3 T_R_C = T_C_R.inverse();
    Vector3d f_ref = px2cam(pt_ref);   //参考点像素转换为相机坐标系的坐标
    f_ref.normalize();                 //归一化处理
    Vector3d f_curr = px2cam(pt_curr); //当前点像素转换为相机坐标系的坐标
    f_curr.normalize();                //归一化处理

    // 方程
    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC（视觉SLAM十四讲P156,公式7.24）
    // f2 = R_RC * f_cur
    // 转化成下面这个矩阵方程组
    // => [ f_ref^T f_ref, -f_ref^T f2 ] [d_ref]   [f_ref^T t]
    //    [ f_cur^T f_ref, -f2^T f2    ] [d_cur] = [f2^T t   ]

    //重点：
    // s1*x1=s2*R*x2+t,移项得到s1*x1-s2*R*x2=t   (1)
    //(1)两边同乘x1的转置x1T，得s1*x1T*x1-s2*x1T*R*x2=x1T*t   (2) 注意"T"指的是转置的意思
    //因此接下来的A[]就是存放未知数为s1和s2的线性方程组的系数矩阵，b存放方程组右端的两个常数
    // cout<<"fref"<<f_ref<<endl;
    Vector3d t = T_R_C.translation();
    // cout<<"t"<<t<<endl;
    // cout<<"t.dot(f_ref)"<<t.dot(f_ref)<<endl;
    Vector3d f2 = T_R_C.so3() * f_curr;
    // AX=B;
    Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
    Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    Vector2d ans = A.inverse() * b;
    Vector3d xm = ans[0] * f_ref;            // ref 侧的结果
    Vector3d xn = t + ans[1] * f2;           // cur 结果
    Vector3d p_esti = (xm + xn) / 2.0;       // P的位置，取两者的平均
    double depth_estimation = p_esti.norm(); // 深度值

    // 计算不确定性（以一个像素为误差）
    Vector3d p = f_ref * depth_estimation;
    Vector3d a = p - t; // 327公式13.7
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos(f_ref.dot(t) / t_norm);                   // a=arccos<p,t>
    double beta = acos(-a.dot(t) / (a_norm * t_norm));            // beta=arccos<a,-t>
    Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction); //
    f_curr_prime.normalize();
    double beta_prime = acos(f_curr_prime.dot(-t) / t_norm); //计算beta'
    double gamma = M_PI - alpha - beta_prime;                //计算y'
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);  //计算p‘的大小||p'||
    double d_cov = p_prime - depth_estimation;               //计算sigma/obs
    double d_cov2 = d_cov * d_cov;                           //计算方差

    // 高斯融合
    //新的数据到来的深度估计均值以及方差
    double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    //具体高斯融合
    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2); //公式13.6
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);
    //融合后的高斯分布
    depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    // cout<<mu_fuse<<endl;
    depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

    return true;
}

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

    // printf ( "-- Max dist : %f \n", max_dist );

    // printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}
void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          const std::vector<DMatch> &matches,
                          Mat &R, Mat &t)
{
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (auto &matche : matches)
    {
        points1.push_back(keypoints_1[matche.queryIdx].pt);
        points2.push_back(keypoints_2[matche.trainIdx].pt);
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
    // cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    // Point2d principal_point ( cx, cy );	//相机光心, TUM dataset标定值

    // double focal_length = fx;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, K);

    // cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    // Mat homography_matrix;
    // homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    // cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, K, R, t);
    // cout<<"R is "<<endl<<R<<endl;
    // cout<<"t is "<<endl<<t<<endl;
}
