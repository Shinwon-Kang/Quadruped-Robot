import rospy
from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import LowCmd

import random
import time

def action_publish():
    servo_pub = []
    legs_list = ['FL', 'FR', 'RR', 'RL']
    joints_list = ['hip', 'calf', 'thigh']
    servo_pub = [rospy.Publisher('/a1_gazebo/' + i + '_' + j + '_controller/command', MotorCmd, queue_size=1) for i in legs_list for j in joints_list]

    rospy.init_node('random_action', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    lowCmd = LowCmd()
    for i in range(len(legs_list) * len(joints_list)):
        lowCmd.motorCmd[i].mode = 0x0A
        lowCmd.motorCmd[i].Kp = 100
        lowCmd.motorCmd[i].dq = 0
        lowCmd.motorCmd[i].Kd = 4   
        lowCmd.motorCmd[i].tau = 0

    # lowCmd.motorCmd[0].q = -1.3
    last_pos = [0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                0.0, 0.67, -1.3, -0.0, 0.67, -1.3]
    update_pos = [0. for i in range(12)]
    print(update_pos)
    duration = 10000.
    while not rospy.is_shutdown():
        # TODO: Random Choice action (move only one foot position)
        # leg = random.choice(legs_list) # claf -> -2.2

        target_pos = [0.0, 0.67, -1.5, -0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3]
        
        for i in range(int(duration)):
            percent = i / duration
            for j in range(len(legs_list) * len(joints_list)):
                update_pos[j] = last_pos[j] * (1 - percent) + target_pos[j] * percent
                # Pose Optimizer


            # Publish
            for j in range(len(legs_list) * len(joints_list)):
                lowCmd.motorCmd[j].q = update_pos[j]
                servo_pub[j].publish(lowCmd.motorCmd[j])
            
            print('1', (len(legs_list) * len(joints_list)), ':', update_pos)


        target_pos = [0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3]

        for i in range(int(duration)):
            percent = i / duration
            for j in range(len(legs_list) * len(joints_list)):
                update_pos[j] = last_pos[j] * (1 - percent) + target_pos[j] * percent
                # Pose Optimizer

            # Publish
            for j in range(len(legs_list) * len(joints_list)):
                lowCmd.motorCmd[j].q = update_pos[j]
                servo_pub[j].publish(lowCmd.motorCmd[j])

            print('2', (len(legs_list) * len(joints_list)), ':', update_pos)


if __name__ == '__main__':
    try:
        action_publish()
    except rospy.ROSInterruptException:
        pass