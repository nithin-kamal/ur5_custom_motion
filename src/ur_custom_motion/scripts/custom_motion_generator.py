import rospy
import time
from ur_custom_motion.srv import TargetGoal, TargetGoalResponse
from ur_custom_motion.bot_move_group import BotMoveGroup

class MotionGenerator(BotMoveGroup):
    def __init__(self):
        super().__init__()
        rospy.init_node('motion_generator')
        print("Motion Generator Node Initialized")
        self.interface_primary_movegroup()
        self.joint_service = rospy.Service('joint_motion', TargetGoal, self.handle_joint_motion)
        self.pose_service= rospy.Service('pose_motion', TargetGoal, self.handle_pose_motion)
        
        
    def handle_joint_motion(self, req):
        joint1 = req.goal1
        joint2 = req.goal2        
        output1 = self.execute_joint_trajectory(joint1, req.velocity, req.acceleration)
        time.sleep(5)
        output2 = self.execute_joint_trajectory(joint2, req.velocity, req.acceleration)
        return TargetGoalResponse(str(output1 and output2))
    
    def handle_pose_motion(self, req):
        pose1 = req.goal1
        pose2 = req.goal2
        output1 = self.execute_pose_trajectory(pose1, req.velocity, req.acceleration)
        time.sleep(5)
        output2 = self.execute_pose_trajectory(pose2, req.velocity, req.acceleration)
        return TargetGoalResponse(str(output1 and output2))
    
    
def main():
    MotionGenerator()
    rospy.spin()    



if __name__ == '__main__':
    main()
    
