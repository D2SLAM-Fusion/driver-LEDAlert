#ifndef __SERVICE_ROS1_H__
#define __SERVICE_ROS1_H__

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <map>
#include <nlohmann/json.hpp>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class rosTopicBase
{
    private:

    public:
        rosTopicBase(ros::NodeHandle& nh, std::string topicName, int msgQueueSize);
        ~rosTopicBase();

        ros::NodeHandle& nh;
        ros::Subscriber sub;
        std::string topicName;
        int msgQueueSize;
        std::mutex msgQueueMutex;
        void* dataOutPtr;
        bool dataAvailable;
        virtual bool popMsg(){ return false;};
};

template <typename T>
class rosTopic: public rosTopicBase
{
    private:
        std::queue<T> msgQueue;

    public:
        rosTopic(ros::NodeHandle& nh, std::string topicName, int msgQueueSize): rosTopicBase(nh, topicName, msgQueueSize)
        {
            ROS_INFO("Subscribe to topic: %s", topicName.c_str());
            this->sub = this->nh.subscribe(topicName, msgQueueSize, &rosTopic::callback, this);
            this->msgQueue = std::queue<T>();
        }
        ~rosTopic();

        void callback(const typename T::ConstPtr& msg)
        {
            if (this->msgQueue.size() >= this->msgQueueSize)
            {
                this->msgQueue.pop();
            }
            this->pushMsg(msg);
        }

        bool popMsg() override
        {
            std::lock_guard<std::mutex> lock(this->msgQueueMutex);
            if (!this->msgQueue.empty())
            {
                dataOutPtr = (void*)(&this->msgQueue.front());
                this->msgQueue.pop();
                dataAvailable = true;
                return true;
            }
            dataAvailable = false;
            return false;
        }

        bool pushMsg(const typename T::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> lock(this->msgQueueMutex);
            if (this->msgQueue.size() >= this->msgQueueSize)
            {
                return false;
            }
            this->msgQueue.push(*msg);
            return true;
        }
};

struct serviceInfo
{
    std::string serviceName;
    std::vector<rosTopicBase*> topicList;
};

class rosService
{
    private:
        ros::NodeHandle nh;
        std::vector<struct serviceInfo> serviceList;

        std::thread rosServiceThread;
        bool serviceSpinFlag;
        bool rosServiceThreadFunc();
        bool startRosServiceThread();

        nlohmann::json configJson;
        bool parseServiceConfig(std::string configFilePath);
        bool initServiceList();

    public:
        rosService();
        ~rosService();

        bool getServiceNameList(std::vector<std::string>& serviceNameList);
        bool getTopicNameList(std::string serviceName, std::vector<std::string>& topicNameList);
        rosTopicBase* getTopicObj(std::string serviceName, std::string topicName);
};

#endif