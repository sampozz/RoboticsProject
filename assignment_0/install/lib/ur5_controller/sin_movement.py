#!/usr/bin/env python3
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def talker():
    # Create a ros node 
    ros.init_node('mkthis_fcking_rbtmv', anonymous=True)
    # This is the initial position that you find in params.py
    q_des_q0 = np.array([-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558])
    # Specify the topic where to publish data
    pub_reduced_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
    time = 0

    while not ros.is_shutdown():

        # Generate sin wave
        q_des = q_des_q0  + 0.1 * np.sin(2 * np.pi * 0.5 * time)
        print(q_des)

        # Send message
        msg = Float64MultiArray()
        msg.data = q_des
        pub_reduced_des_jstate.publish(msg)

        time = np.round(time + np.array([0.001]),  3)


if __name__ == '__main__':
    try:
        talker()
    except ros.ROSInterruptException:
        pass