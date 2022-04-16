#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <stdlib.h>
using namespace std; //命名空间
int main(int argc,char** argv)
//运行时把命令行窗口传给主程序
{
    //创建一个名为cloud的指针，储存XYZ类型的点云数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/guojiawei/cv/cv2/cv/after/myslam/bin/testmap.pcd", *cloud) == -1)
    //打开一个文件，如果有错，返回值-1，表示函数失败
    {
        PCL_ERROR("Couldn't read file 111.pcd\n");
        return (-1);
    }
    std::cout << "Loaded:"
              // 符号“->”表示提取cloud中的width
              << cloud->width * cloud->height
              << "从pcd文件中读取点云数据:"
              << std::endl;
    std::fstream fs;
    
    //读写文件
    fs.open("./11.txt", std::fstream::out);
    //打开文件路径，没有则创建文件
    for (size_t i = 0; i < cloud->points.size(); ++i) // points.size表示点云数据的大小
    {
        // std::cout << " " << int(cloud->points[i].r)
        //           << " " << int(cloud->points[i].g)
        //           << " " << int(cloud->points[i].b) << std::endl;
        fs << cloud->points[i].x << "\t"
           //写入数据
           << cloud->points[i].y << "\t" << cloud->points[i].z <<"\t"<< int(cloud->points[i].b) << "\t" << int(cloud->points[i].g) << "\t" << int(cloud->points[i].r) << "\t"
           << "\n";
    }
    fs.close();      //关闭
    system("pause"); //程序暂停
    return 0;
}
