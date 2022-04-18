add_library(myslam SHARED
<<<<<<< HEAD
        camera.cpp ../include/myslam/frame.h frame.cpp ../include/myslam/mapponit.h ../include/myslam/map.h ../include/myslam/testPoint.h testPoint.cpp ../include/myslam/config.h config.cpp  ../include/myslam/visual_odometry.h visual_odometry.cpp map.cpp mappoint.cpp ../test/run_vo.cpp ../include/myslam/g2o_types.h g2o_types.cpp)
=======
        camera.cpp ../include/myslam/frame.h frame.cpp ../include/myslam/mapponit.h ../include/myslam/map.h 
        ../include/myslam/testPoint.h testPoint.cpp 
        
        

        ../include/myslam/config.h config.cpp  ../include/myslam/visual_odometry.h visual_odometry.cpp map.cpp mappoint.cpp ../test/run_vo.cpp ../include/myslam/g2o_types.h g2o_types.cpp)
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
target_link_libraries( myslam

        ${THIRD_PARTY_LIBS}

<<<<<<< HEAD
        )
=======
        )
        
>>>>>>> fbc9515c775cd61c2cb5a913f937852e2c828cb9
