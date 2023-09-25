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

    ros::Rate loop_rate(100);
    while (this->serviceSpinFlag)
    {
        #ifdef DEBUG

        #endif

        ros::spinOnce();
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