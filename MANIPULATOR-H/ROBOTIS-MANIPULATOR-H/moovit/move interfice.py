import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#from ROS_RUN import ROS_master as ros


## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipolator_h_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 4
        joint_goal[2] = 0
        joint_goal[3] = -pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,pos,ori):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()


        pose_goal.orientation.x = ori[0]
        pose_goal.orientation.y = ori[1]
        pose_goal.orientation.z = ori[2]
        pose_goal.orientation.w =  (1-( ori[0]**2+ori[1]**2+ori[2]**2))**0.5

        pose_goal.position.x =  pos[0]
        pose_goal.position.y =  pos[1]
        pose_goal.position.z = pos[2]


        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, move, scale=1):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.orientation.w = move[3] * scale
        wpose.position.x = move[0] * scale
        wpose.position.y = move[1] * scale
        wpose.position.z = move[2] * scale
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):

        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self,box_name, timeout=4):

        #box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "end_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        # box_pose.pose.position.y = 1  # slightly above the end effector
        box_pose.pose.position.z = 0  # slightly above the end effector
        # slightly above the end effector
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self,box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self,box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


if __name__ == '__main__':
    try:
        movit = MoveGroupPythonInteface()
        # "============ movement using a joint state goal ============"
        #movit.go_to_joint_state()
        # "============ movement using a pose goal ============"
       # movit.remove_box()
        #movit.add_box()
        movit.go_to_pose_goal([0.2,0,0.5],[0,0,0])
        movit.add_box("box")
        #movit.attach_box("box")

        #movit.go_to_pose_goal()
        #movit.detach_box("box")

        #
       # input("sddddddd")

        # # "============ plan and display a Cartesian path ============"
        # cartesian_plan, fraction = movit.plan_cartesian_path((0, 0, 0.4, 0.0))
        # # "============ display a saved trajectory ============"
        # movit.display_trajectory(cartesian_plan)
        # b = []
        # for i in cartesian_plan.joint_trajectory.points:
        #     b.append(i.positions)
        # c = ros(b, 50)
        # c.send_to_arm()
        # # "============ execute a saved path ============"
        # movit.execute_plan(cartesian_plan)
        # "============ add a box to the planning scene ============"
        # movit.add_box()
        # "============ attach a Box to the  robot ============"
        # movit.attach_box()
        # "============ plan and execute a path with an attached collision object ============"
        # cartesian_plan, fraction = movit.plan_cartesian_path(scale=-1)
        # movit.execute_plan(cartesian_plan)
        # print "============ detach the box from the robot ============"
        # movit.detach_box()

        # print "============remove the box from the planning scene ============"
        # movit.remove_box()

    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
    except KeyboardInterrupt:
        print(KeyboardInterrupt)
