import rospy
import time
import numpy as np
from geometry_msgs.msg import Pose
from ur_custom_motion.bot_move_group import BotMoveGroup

class SinePublisher(BotMoveGroup):
    def __init__(self):
        super().__init__()
        rospy.init_node('sine_publisher')
        self.interface_primary_movegroup()
        self.counter = 0
        self.publish_joints()
    
    def publish_pose(self):
        target_pose = Pose()
        target_pose.position.x = -0.065
        target_pose.position.y = 0.191
        target_pose.position.z = 0.745
        target_pose.orientation.x = -0.496
        target_pose.orientation.y = 0.504
        target_pose.orientation.z = 0.504
        target_pose.orientation.w = 0.496
        self.execute_pose_trajectory(target_pose)
        time.sleep(5)
        self.end_generation()
    

    def publish_joints(self):
        self.start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.counter == 10:
                self.end_generation()
                return
            current_time = rospy.Time.now()
            joint_values = [
                np.sin(0.2 * (current_time - self.start_time).to_sec()),
                np.sin(0.2 * (current_time - self.start_time).to_sec() + np.pi/2),
                np.sin(0.2 * (current_time - self.start_time).to_sec() + np.pi),
                np.sin(0.2 * (current_time - self.start_time).to_sec() + 3*np.pi/2),
                np.sin(0.2 * (current_time - self.start_time).to_sec() + 2*np.pi),
                np.sin(0.2 * (current_time - self.start_time).to_sec() + 5*np.pi/2)
            ]
            rospy.loginfo(joint_values)
            self.execute_joint_trajectory(joint_values)
            time.sleep(5)
            self.counter += 1
    
    def end_generation(self):
        rospy.loginfo("End of sine generation")
        joint_values = [0, -1.57, 0, -1.57, 0, 0]
        self.execute_joint_trajectory(joint_values)
        rospy.signal_shutdown("End of sine generation")
    
    
def main():
    SinePublisher()
    rospy.spin()    



if __name__ == '__main__':
    main()
    
