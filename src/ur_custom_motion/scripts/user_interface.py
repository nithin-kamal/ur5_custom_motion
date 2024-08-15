import rospy
from sensor_msgs.msg import JointState
from ur_custom_motion.srv import TargetGoal, TargetGoalResponse
from ur_custom_motion.copilot import CoPilot

class UserInterface():
    def __init__(self):
        rospy.init_node('user_interface')
        rospy.wait_for_service('/joint_motion')
        rospy.wait_for_service('/pose_motion')
        self.joint_service = rospy.ServiceProxy('joint_motion', TargetGoal)
        self.pose_service = rospy.ServiceProxy('pose_motion', TargetGoal)
        self.current_states = JointState()
        rospy.Subscriber("ur5/joint_states", JointState, self.set_states)
        self.copilot = CoPilot()
        self.engage_user_interface()
        
        
        
    def set_states(self, data):
        self.current_states = data
        
    def engage_user_interface(self):
        while not rospy.is_shutdown():
            print("Choose a motion type: \n1. Joint Motion\n2. Pose Motion\n3. Get Robot State \n4. Get Code Suggestion \n5. Exit")
            choice = int(input("Enter your choice: "))
            if choice == 1:
                self.handle_joint_motion()
            elif choice == 2:
                self.handle_pose_motion()
            elif choice == 3:
                self.get_robot_state()
            elif choice == 4:
                print(self.copilot.suggest_code())                
            elif choice == 5:
                rospy.signal_shutdown("User requested shutdown")
            else:
                print("Invalid choice. Please try again.")
        
    def handle_joint_motion(self, joint1 = None, joint2 = None, velocity = None, acceleration = None):
        if joint1 is None or joint2 is None :
            print("Enter 6 float values separated by space for the each goal: ")
        if joint1 is None:
            print("Enter the joint values for the first goal: ")
            joint1 = [float(x) for x in input().split()]
        if joint2 is None:
            print("Enter the joint values for the second goal: ")
            joint2 = [float(x) for x in input().split()]
        if velocity is None:
            print('Enter the joint velocity as a float value: ')
            velocity = float(input())
        if acceleration is None:
            print('Enter the joint acceleration as a float value: ')
            acceleration = float(input())
        try:
            output = self.joint_service(joint1, joint2, velocity, acceleration)
            output = True if output.status == 'True' else False
        except:
            output = False
        if output:
            print("Joint motion executed successfully.")
        else:
            print("Joint motion failed.")
    
    def handle_pose_motion(self, pose1 = None, pose2 = None, velocity = None, acceleration = None):
        if pose1 is None or pose2 is None:
            print("Enter 4 float values separated by space for the each goal: ")
            print("The values should be in the order position.x, position.y, position.z, orientation.w")
        if pose1 is None:
            print("Enter the pose values for the first goal: ")
            pose1 = [float(x) for x in input().split()]
        if pose2 is None:
            print("Enter the pose values for the second goal: ")
            pose2 = [float(x) for x in input().split()]
        if velocity is None:
            print('Enter the linear velocity as a float value: ')
            velocity = float(input())
        if acceleration is None:
            print('Enter the linear acceleration as a float value: ')
            acceleration = float(input())
        try:
            output = self.pose_service(pose1, pose2, velocity, acceleration)
            output = True if output.status == 'True' else False
        except:
            output = False
        print(output)
        if output:
            print("Pose motion executed successfully.")
        else:
            print("Pose motion failed.")
            
    
    
    def get_robot_state(self):
        print("Current robot state: ", self.current_states)
        
    
def main():
    UserInterface()
    rospy.spin()    



if __name__ == '__main__':
    main()
    
