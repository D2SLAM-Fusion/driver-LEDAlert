#include "ros/service_ros1.h"

rosTopicBase::rosTopicBase(ros::NodeHandle& nh, std::string topicName, int msgQueueSize): nh(nh), topicName(topicName), msgQueueSize(msgQueueSize)
{

}

rosTopicBase::~rosTopicBase()
{

}

rosService::rosService()
{
    this->serviceSpinFlag = true;
    startRosServiceThread();
}

rosService::~rosService()
{
    this->serviceSpinFlag = false;
    this->rosServiceThread.join();
}

bool rosService::rosServiceThreadFunc(){
    std::string configFile;
    if (nh.getParam("/healthManager_node/config_file", configFile)) {
        ROS_INFO("Retrieved configFile parameter: %s", configFile.c_str());
        if (parseServiceConfig(configFile)) {
            ROS_INFO("Successfully read config file");
        } else {
            ROS_ERROR("Failed to read config file");
            return false;
        }
    } else {
        ROS_ERROR("Failed to retrieve configFile parameter");
        return false;
    }

    if (initServiceList()) {
        ROS_INFO("Successfully initialized service list");
    } else {
        ROS_ERROR("Failed to initialize service list");
        return false;
    }

    ros::Rate loop_rate(1);
    while (this->serviceSpinFlag)
    {
        sensor_msgs::Imu* imuMsg;
        nav_msgs::Odometry* odomMsg;

        std::vector<std::string> serviceNameList;
        getServiceNameList(serviceNameList);
        for (auto& serviceName : serviceNameList) {
            ROS_INFO("Service name: %s", serviceName.c_str());
            std::vector<std::string> topicNameList;
            getTopicNameList(serviceName, topicNameList);
            for (auto& topicName : topicNameList) {
                ROS_INFO("Topic name: %s", topicName.c_str());
                rosTopicBase* topicObj = getTopicObj(serviceName, topicName);
                if (topicObj != NULL) {
                    if(topicName == "vins_estimator/imu_propagate"){
                        imuMsg = (sensor_msgs::Imu*)(topicObj->dataOutPtr);
                        ROS_INFO("imuMsg: %f", imuMsg->linear_acceleration.x);
                    }
                    else if(topicName == "vins_estimator/odometry"){
                        odomMsg = (nav_msgs::Odometry*)(topicObj->dataOutPtr);
                        ROS_INFO("odomMsg: %f", odomMsg->pose.pose.position.x);
                    }
                    else
                        ROS_ERROR("Unknown topic name: %s", topicName.c_str());
                }
            }
        }
        loop_rate.sleep();
    }

    return true;
}

bool rosService::parseServiceConfig(std::string configFilePath) {
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        ROS_ERROR("Failed to open config file");
        return false;
    }

    try {
        configFile >> this->configJson;
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse config file: %s", e.what());
        configFile.close();
        return false;
    }

    configFile.close();

    return true;
}

bool rosService::initServiceList(){
    if (this->configJson.empty()) {
        ROS_ERROR("Config file is empty");
        return false;
    }

    for (auto& service : this->configJson["serviceList"]) {
        struct serviceInfo ServiceInfo;
        ServiceInfo.serviceName = service["name"];
        for (auto& topic : service["topicList"]) {
            std::string topicName = topic["name"];
            std::string topicType = topic["dataType"];
            int msgQueueSize = topic["queueSize"];
            if (topicType == "sensor_msgs::Imu") {
                ServiceInfo.topicList.push_back(new rosTopic<sensor_msgs::Imu>(this->nh, topicName, msgQueueSize));
            } else if (topicType == "nav_msgs::Odometry") {
                ServiceInfo.topicList.push_back(new rosTopic<nav_msgs::Odometry>(this->nh, topicName, msgQueueSize));
            } else {
                ROS_ERROR("Unknown topic type: %s", topicType.c_str());
                return false;
            }
        }
        this->serviceList.push_back(ServiceInfo);
    }
    return true;
}


bool rosService::startRosServiceThread(){
    this->rosServiceThread = std::thread(&rosService::rosServiceThreadFunc, this);
    return true;
}

bool rosService::getServiceNameList(std::vector<std::string>& serviceNameList){
    for (auto& service : this->serviceList) {
        serviceNameList.push_back(service.serviceName);
    }
    return true;
}

bool rosService::getTopicNameList(std::string serviceName, std::vector<std::string>& topicNameList){
    for (auto& service : this->serviceList) {
        if (service.serviceName == serviceName) {
            for (auto& topic : service.topicList) {
                topicNameList.push_back(topic->topicName);
            }
            return true;
        }
    }
    return false;
}

rosTopicBase* rosService::getTopicObj(std::string serviceName, std::string topicName){
    for (auto& service : this->serviceList) {
        if (service.serviceName == serviceName) {
            for (auto& topic : service.topicList) {
                if (topic->topicName == topicName) {
                    return topic;
                }
            }
        }
    }
    return NULL;
}