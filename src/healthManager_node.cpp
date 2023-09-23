#include "services/services.h"

#ifdef DEBUG
    #define BACKWARD_HAS_BFD 1
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    backward::SignalHandling sh;
#endif

int main(int argc, char** argv) {
#ifdef ROS_VERSION_ROS1
    ros::init(argc, argv, "healthManager");
#endif
    rosService* rosServiceHandler = new rosService;
    vinsChecker* vinsCheckerHandler = new vinsChecker(rosServiceHandler);

    while(true) {
        sleep(1);
    }

    return 0;
}