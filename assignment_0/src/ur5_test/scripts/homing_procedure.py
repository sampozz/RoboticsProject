#!/usr/bin/env python
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

q_des = [0, 0, 0, 0, 0, 0]  # Current joints values
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # from params.py


def receive_jstates(msg):
    global q_des
    # Get joints values from topic
    for msg_idx in range(len(msg.name)):
        for joint_idx in range(len(joint_names)):
            if joint_names[joint_idx] == msg.name[msg_idx]:
                q_des[joint_idx] = msg.position[msg_idx]


def homing_procedure():
    print('HOMING PROCEDURE STARTED')
    
    # Create a ros node 
    ros.init_node('homing_procedure', anonymous=True)

    # Specify the topic where to publish data
    pub_reduced_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)

    # Final home joints positions (value that you find in params.py)
    q_des_q1 = np.array([-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558])

    # Wait to obtain current joints values from subscriber
    ros.sleep(2.)

    # Initial joints values
    q_des_q0 = np.copy(q_des)

    # Other params
    dt = 0.001  # from params.py
    rate = ros.Rate(1 / dt)
    v_ref = 0.0
    v_des = 0.6

    while True:
        e = q_des_q1 - q_des_q0
        e_norm = np.linalg.norm(e)

        if (e_norm != 0.0):
            v_ref += 0.005 * (v_des - v_ref)
            q_des_q0 += dt * v_ref * e / e_norm
            
            # Send message to topic
            msg = Float64MultiArray()
            msg.data = q_des_q0
            pub_reduced_des_jstate.publish(msg)

        rate.sleep()

        if (e_norm < 0.001):
            print('HOMING PROCEDURE ACCOMPLISHED')
            break


if __name__ == '__main__':
    try:
        # Get current joints values from topic 
        ros.Subscriber("/ur5/joint_states", JointState, callback=receive_jstates, queue_size=1)

        homing_procedure()

    except ros.ROSInterruptException:
        pass