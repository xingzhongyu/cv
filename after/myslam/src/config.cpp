//
// Created by xzy on 3/6/22.
//
#include "myslam/config.h"
#include <fstream>
namespace myslam
{

    void Config::setParameterFile( const std::string& filename )
    {
        if ( config_ == nullptr )
            config_ = shared_ptr<Config>(new Config);

//        ifstream f(filename.c_str());
//        cout<<f.good()<<endl;
        // cout<<filename.c_str()<<endl;
        static cv::FileStorage file1( filename.c_str(), cv::FileStorage::READ );
        config_->file_=file1;
    //    cout<<filename.c_str()<<endl;                   
        if ( config_->file_.isOpened() == false )
        {

            std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
            config_->file_.release();
            return;
        }
    }

    Config::~Config()
    {
        if ( file_.isOpened() )
            file_.release();
    }

    shared_ptr<Config> Config::config_ = nullptr;

}
