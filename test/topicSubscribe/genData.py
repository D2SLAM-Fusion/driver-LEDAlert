#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import random


def generate_imu_data():
    imu_msg = Imu()
    imu_msg.header = Header()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.linear_acceleration.x = random.uniform(-100, 100)
    imu_msg.linear_acceleration.y = random.uniform(-100, 100)
    imu_msg.linear_acceleration.z = random.uniform(-100, 100)
    imu_msg.angular_velocity.x = random.uniform(-100, 100)
    imu_msg.angular_velocity.y = random.uniform(-100, 100)
    imu_msg.angular_velocity.z = random.uniform(-100, 100)
    return imu_msg


def generate_odometry_data():
    odometry_msg = Odometry()
    odometry_msg.header = Header()
    odometry_msg.header.stamp = rospy.Time.now()
    odometry_msg.pose.pose.position.x = random.uniform(-100, 100)
    odometry_msg.pose.pose.position.y = random.uniform(-100, 100)
    odometry_msg.pose.pose.position.z = random.uniform(-100, 100)
    odometry_msg.twist.twist.linear.x = random.uniform(-100, 100)
    odometry_msg.twist.twist.linear.y = random.uniform(-100, 100)
    odometry_msg.twist.twist.linear.z = random.uniform(-100, 100)
    return odometry_msg


def main():
    rospy.init_node('data_generator_node', anonymous=True)
    imu_pub = rospy.Publisher('vins_estimator/imu_propagate', Imu, queue_size=10)
    odometry_pub = rospy.Publisher('vins_estimator/odometry', Odometry, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        imu_data = generate_imu_data()
        odometry_data = generate_odometry_data()

        imu_pub.publish(imu_data)
        odometry_pub.publish(odometry_data)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass