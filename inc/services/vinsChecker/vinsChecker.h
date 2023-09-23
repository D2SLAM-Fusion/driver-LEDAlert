#ifndef __VINS_CHECKER_H__
#define __VINS_CHECKER_H__
#include <string>
#include <vector>
#include <queue>
#include <thread>

class vinsCheckerBase
{
    private:
        bool vinsCheckerThreadRunning;
        std::thread* vinsCheckerThread;
        void vinsCheckerThreadFunc();

    public:
        vinsCheckerBase();
        ~vinsCheckerBase();
        std::string serviceName = "vinsChecker";
        std::vector<std::string> topicNameList = {"vins_estimator/imu_propagate", "vins_estimator/odometry"};
        int imuMsgQueueSize = 50;
        int odomMsgQueueSize = 50;

        bool healthStatus;
        virtual bool getOneMsg() = 0;
        virtual bool prepareData() = 0;
        virtual bool dataReady() = 0;
        virtual bool checkHealth() = 0;
};

#ifdef ROS_VERSION_ROS1
//ROS1
#include "ros/service_ros1.h"

class vinsChecker: public vinsCheckerBase
{
    private:
        std::queue<sensor_msgs::Imu*> imuMsgQueue;
        std::queue<nav_msgs::Odometry*> odomMsgQueue;
        rosService* rosServiceHandler;

    public:
        vinsChecker(rosService* rosServiceHandler);
        ~vinsChecker();

        bool getOneMsg() override;
        bool prepareData() override;
        bool dataReady() override;
        bool checkHealth() override;
};

#else
//ROS2
#include "ros/service_ros2.h"

//TODO: ROS2

#endif

#endif