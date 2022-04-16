add_library(myslam SHARED
        camera.cpp ../include/myslam/frame.h frame.cpp ../include/myslam/mapponit.h ../include/myslam/map.h 
        ../include/myslam/testPoint.h testPoint.cpp 
        
        

        ../include/myslam/config.h config.cpp  ../include/myslam/visual_odometry.h visual_odometry.cpp map.cpp mappoint.cpp ../test/run_vo.cpp ../include/myslam/g2o_types.h g2o_types.cpp)
target_link_libraries( myslam

        ${THIRD_PARTY_LIBS}

        )
        