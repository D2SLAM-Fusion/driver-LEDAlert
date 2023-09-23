#ifdef ROS_VERSION_ROS1
    #include "ros/service_ros1.h"
#else
    #include "ros/service_ros2.h"
#endif

int main(int argc, char** argv) {
#ifdef ROS_VERSION_ROS1
    ros::init(argc, argv, "healthManager");
#endif
    rosService* rosServiceHandler = new rosService;

    while(true) {
        sleep(1);
    }

    return 0;
}