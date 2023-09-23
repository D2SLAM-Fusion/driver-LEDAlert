#include "services/vinsChecker/vinsChecker.h"

vinsCheckerBase::vinsCheckerBase() {
    this->healthStatus = true;
    this->vinsCheckerThreadRunning = true;
    this->vinsCheckerThread = new std::thread(&vinsCheckerBase::vinsCheckerThreadFunc, this);
}

vinsCheckerBase::~vinsCheckerBase() {
    this->vinsCheckerThreadRunning = false;
    if(this->vinsCheckerThread->joinable()){
        this->vinsCheckerThread->join();
    }
}

void vinsCheckerBase::vinsCheckerThreadFunc() {
    while(this->vinsCheckerThreadRunning){
        if(!prepareData()){
            ROS_ERROR("Failed to prepare data");
        }
        if(!checkHealth()){
            ROS_ERROR("Failed to check health");
        }
        sleep(1);
    }
}

vinsChecker::vinsChecker(rosService* rosServiceHandler): vinsCheckerBase() {
    this->rosServiceHandler = rosServiceHandler;
    this->healthStatus = true;
}

vinsChecker::~vinsChecker() { }

bool vinsChecker::getOneMsg() {
    sensor_msgs::Imu* imuMsg = nullptr;
    nav_msgs::Odometry* odomMsg = nullptr;

    for(auto& topicName : topicNameList){
        rosTopicBase* topicObj = rosServiceHandler->getTopicObj(serviceName, topicName);
        if (topicObj != NULL) {
            topicObj->popMsg();
            if(topicObj->dataAvailable){
                if(topicName == "vins_estimator/imu_propagate"){
                    imuMsg = (sensor_msgs::Imu*)(topicObj->dataOutPtr);
                    imuMsgQueue.push(imuMsg);
                }
                else if(topicName == "vins_estimator/odometry"){
                    odomMsg = (nav_msgs::Odometry*)(topicObj->dataOutPtr);
                    odomMsgQueue.push(odomMsg);
                }
                else {
                    // ROS_ERROR("Unknown topic name: %s", topicName.c_str());
                    return false;
                }
            }
            else {
                // ROS_ERROR("No data available for topic: %s", topicName.c_str());
                return false;
            }
        }
        else {
            // ROS_ERROR("Unknown topic name: %s", topicName.c_str());
            return false;
        }
    }

    return true;
}

bool vinsChecker::prepareData() {
    while (!imuMsgQueue.empty()) {
        imuMsgQueue.pop();
    }
    while (!odomMsgQueue.empty()) {
        odomMsgQueue.pop();
    }

    while(!dataReady()){
        if(!getOneMsg()){
            // ROS_ERROR("Failed to get one message");
        }
    }

    return true;
}

bool vinsChecker::dataReady() {
    if(imuMsgQueue.size() >= imuMsgQueueSize && odomMsgQueue.size() >= odomMsgQueueSize)
        return true;
    else
        return false;
}

bool vinsChecker::checkHealth() {
    // Calculate the queue data variance
    double imuAccXVariance = 0;
    double imuAccYVariance = 0;
    double imuAccZVariance = 0;
    double odomPosXVariance = 0;
    double odomPosYVariance = 0;
    double odomPosZVariance = 0;

    double imuAccXMean = 0;
    double imuAccYMean = 0;
    double imuAccZMean = 0;
    double odomPosXMean = 0;
    double odomPosYMean = 0;
    double odomPosZMean = 0;

    // Calculate IMU linear acceleration mean
    for (int i = 0; i < imuMsgQueueSize; i++) {
        double imuAccX = imuMsgQueue.front()->linear_acceleration.x;
        double imuAccY = imuMsgQueue.front()->linear_acceleration.y;
        double imuAccZ = imuMsgQueue.front()->linear_acceleration.z;

        imuAccXMean += imuAccX;
        imuAccYMean += imuAccY;
        imuAccZMean += imuAccZ;

        imuAccXVariance += pow(imuAccX, 2);
        imuAccYVariance += pow(imuAccY, 2);
        imuAccZVariance += pow(imuAccZ, 2);

        imuMsgQueue.pop();
    }

    imuAccXMean /= imuMsgQueueSize;
    imuAccYMean /= imuMsgQueueSize;
    imuAccZMean /= imuMsgQueueSize;

    imuAccXVariance = imuAccXVariance / imuMsgQueueSize - pow(imuAccXMean, 2);
    imuAccYVariance = imuAccYVariance / imuMsgQueueSize - pow(imuAccYMean, 2);
    imuAccZVariance = imuAccZVariance / imuMsgQueueSize - pow(imuAccZMean, 2);

    // Calculate odometry position mean
    for (int i = 0; i < odomMsgQueueSize; i++) {
        double odomPosX = odomMsgQueue.front()->pose.pose.position.x;
        double odomPosY = odomMsgQueue.front()->pose.pose.position.y;
        double odomPosZ = odomMsgQueue.front()->pose.pose.position.z;

        odomPosXMean += odomPosX;
        odomPosYMean += odomPosY;
        odomPosZMean += odomPosZ;

        odomPosXVariance += pow(odomPosX, 2);
        odomPosYVariance += pow(odomPosY, 2);
        odomPosZVariance += pow(odomPosZ, 2);

        odomMsgQueue.pop();
    }

    odomPosXMean /= odomMsgQueueSize;
    odomPosYMean /= odomMsgQueueSize;
    odomPosZMean /= odomMsgQueueSize;

    odomPosXVariance = odomPosXVariance / odomMsgQueueSize - pow(odomPosXMean, 2);
    odomPosYVariance = odomPosYVariance / odomMsgQueueSize - pow(odomPosYMean, 2);
    odomPosZVariance = odomPosZVariance / odomMsgQueueSize - pow(odomPosZMean, 2);

    ROS_INFO("imuAccXVariance: %f", imuAccXVariance);
    ROS_INFO("imuAccYVariance: %f", imuAccYVariance);
    ROS_INFO("imuAccZVariance: %f", imuAccZVariance);
    ROS_INFO("odomPosXVariance: %f", odomPosXVariance);
    ROS_INFO("odomPosYVariance: %f", odomPosYVariance);
    ROS_INFO("odomPosZVariance: %f", odomPosZVariance);

    return true;
}