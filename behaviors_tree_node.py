#!/usr/bin/env python3

import rospy
import py_trees
import tf
import math
from std_msgs.msg import Float64MultiArray,Bool
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool
from nav_msgs.msg import Path, Odometry
from vacuum_control import vacuum_control 


vacuum_controller = None  # Global instance
import py_trees.display



# Object and drop positions
OBJECT_X = 2.0
OBJECT_Y = 0.0
DROP_X = 2.5
DROP_Y = 0.2

# Utilities
def stop_robot(pub):
    rospy.loginfo("[RECOVERY] Stopping robot...")
    pub.publish(Twist())
    rospy.sleep(1.0)

def make_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

# Planner
class Planner:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_orientation = None
        self.odom_sub = rospy.Subscriber("/turtlebot/kobuki/odom_ground_truth", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_orientation = msg.pose.pose.orientation

    def plan_to(self, goal):
        path = Path()
        path.header.frame_id = "map"
        waypoints = [(goal.pose.position.x, goal.pose.position.y)]
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

# Behavior Nodes
class PlanPath(py_trees.behaviour.Behaviour):
    def __init__(self, name, goal):
        super(PlanPath, self).__init__(name)
        self.goal = goal
        self.planner = Planner()
        self.path = None

    def update(self):
        pub = rospy.Publisher("/turtlebot/kobuki/commands/velocity", Twist, queue_size=1)
        stop_robot(pub)
        self.path = self.planner.plan_to(goal=self.goal)
        return py_trees.common.Status.SUCCESS if self.path else py_trees.common.Status.FAILURE

class FollowPath(py_trees.behaviour.Behaviour):
    def __init__(self, name, plan_path_node):
        super(FollowPath, self).__init__(name)
        self.plan_path_node = plan_path_node
        self.reached = False

    def initialise(self):
        self.reached = False
        self.pub = rospy.Publisher("/turtlebot/kobuki/commands/velocity", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.planner = self.plan_path_node.planner
        self.current_index = 0

    def update(self):
        path = self.plan_path_node.path
        if not path or self.reached:
            return py_trees.common.Status.SUCCESS if self.reached else py_trees.common.Status.FAILURE

        if self.current_index >= len(path.poses):
            self.pub.publish(Twist())
            self.reached = True
            return py_trees.common.Status.SUCCESS

        pose = path.poses[self.current_index].pose
        x, y, yaw = self.get_robot_pose()
        dx = pose.position.x - x
        dy = pose.position.y - y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(angle - yaw), math.cos(angle - yaw))

        #rospy.loginfo(f"finish go near")

        cmd = Twist()
   

        if distance > 0.1:
            cmd.linear.x = 0.2
            self.pub.publish(cmd)
        else:
            cmd.linear.x = 0
            self.pub.publish(cmd)
            return py_trees.common.Status.SUCCESS

        
        return py_trees.common.Status.RUNNING

    def get_robot_pose(self):
        x = self.planner.robot_x
        y = self.planner.robot_y
        q = self.planner.robot_orientation
        if q is None:
            return x, y, 0.0
        quat = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quat)
        return x, y, yaw


class StopRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Stop Robot"):
        super(StopRobot, self).__init__(name)
        self.pub = rospy.Publisher("/turtlebot/kobuki/commands/velocity", Twist, queue_size=1)
        self.done = False

    def update(self):
        if not self.done:
            stop_robot(self.pub)
            self.done = True
        return py_trees.common.Status.SUCCESS

class TriggerJointRotation(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(TriggerJointRotation, self).__init__(name)
        self.pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.triggered = False
        self.start_time = None

    def update(self):
        if self.triggered:
            if rospy.Time.now().to_sec() - self.start_time > 2.0:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING

        dq = Float64MultiArray()
        dq.data = [1.57, 0.0, 0.0, 0.0]
        self.pub.publish(dq)
        rospy.loginfo("[Joint Rotate] Triggered joint rotation to ~90 degrees.")
        self.triggered = True
        self.start_time = rospy.Time.now().to_sec()
        return py_trees.common.Status.RUNNING

class SendGoalToManipulator(py_trees.behaviour.Behaviour):
    def __init__(self, name, x, y):
        super(SendGoalToManipulator, self).__init__(name)
        self.pub = rospy.Publisher("/swift_pro/goal_position", PoseStamped, queue_size=1)
        self.sent = False
        self.x = x
        self.pub = rospy.Publisher("/swift_pro/goal_position", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("/swift_pro/goal_reach", Bool, self.get_goal_state)

        self.x = x
        self.y = y
        self.sent = False
        self.goal_reach = False
        
    def get_goal_state(self,msg:Bool):
        self.goal_reach = msg.data


    def update(self):
        if self.goal_reach:
            return py_trees.common.Status.SUCCESS
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.position.z = -0.15
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)
        rospy.loginfo(f"[Manipulator] Goal sent to ({self.x:.2f}, {self.y:.2f})")
        self.sent = True
        return py_trees.common.Status.RUNNING


class AdjustManipulatorZ(py_trees.behaviour.Behaviour):
    def __init__(self, name, z, x, y):
        super(AdjustManipulatorZ, self).__init__(name)
        self.z = z
        self.x = x
        self.y = y
        self.sent = False
        self.goal_reach = False

        self.vacuum_ready = False  
        rospy.Subscriber("/vacuum_ready", Bool, self.ready_callback)
        self.pub = rospy.Publisher("/swift_pro/goal_position", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("/swift_pro/goal_reach", Bool, self.get_goal_state)

    def initialise(self):
        self.sent = False
        self.goal_reach = False

    def get_goal_state(self, msg: Bool):
        self.goal_reach = msg.data

    def ready_callback(self, msg: Bool):
        self.vacuum_ready = msg.data  # Set flag when vacuum is ready

    def update(self):
        #rospy.loginfo(f"[Manipulator] {self.name}: Z = {self.z:.2f}, sent={self.sent}, goal_reach={self.goal_reach}, vacuum_ready={self.vacuum_ready}")
        rospy.loginfo(f"[Manipulator] {self.name}: update called, sent={self.sent}, goal_reach={self.goal_reach}, vacuum_ready={self.vacuum_ready}")

        # Only block upward motion until vacuum is ON
        if self.name.lower().__contains__("up") or self.z > -0.15:
            if not self.vacuum_ready:
                rospy.loginfo(f"[Manipulator] {self.name}: Waiting for vacuum to be ON before moving up...")
                return py_trees.common.Status.RUNNING

        if self.goal_reach:
            rospy.loginfo(f"[Manipulator] {self.name}: Z movement goal reached ")
            return py_trees.common.Status.SUCCESS

        # move up
        if not self.sent:
            rospy.loginfo(f"[Manipulator] {self.name}: Adjusting Z to {self.z:.2f}")
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.x
            goal.pose.position.y = self.y
            goal.pose.position.z = self.z
            goal.pose.orientation.w = 1.0
            self.pub.publish(goal)
            self.sent = True

        return py_trees.common.Status.RUNNING


class VacuumControl(py_trees.behaviour.Behaviour):
    def __init__(self, name, turn_on=True):
        super(VacuumControl, self).__init__(name)
        self.turn_on = turn_on
        self.done = False
        self.confirmed = False

        self.state_sub = rospy.Subscriber("/vacuum_state", Bool, self.state_callback)
        self.ready_pub = rospy.Publisher("/vacuum_ready", Bool, queue_size=1)

    def state_callback(self, msg: Bool):
        if self.turn_on:
            self.confirmed = msg.data
        else:
            self.confirmed = not msg.data

    def initialise(self):
        self.done = False
        self.confirmed = False

    def update(self):
        global vacuum_controller
        if not self.done:
            rospy.loginfo(f"[VacuumControl] Requesting vacuum {'ON' if self.turn_on else 'OFF'}")
            if self.turn_on:
                vacuum_controller.vacuum_on()
            else:
                vacuum_controller.vacuum_off()
            self.done = True
            return py_trees.common.Status.RUNNING

        if self.confirmed:
            rospy.loginfo(f"[VacuumControl] Vacuum state confirmed: {'ON' if self.turn_on else 'OFF'} ")
            self.ready_pub.publish(Bool(data=self.turn_on))
            return py_trees.common.Status.SUCCESS  

        return py_trees.common.Status.RUNNING
    
class GoUpZAfterVacuum(py_trees.behaviour.Behaviour):
    def __init__(self, name, z, x, y):
        super(GoUpZAfterVacuum, self).__init__(name)
        self.z = z
        self.x = x
        self.y = y
        self.sent = False
        self.goal_reach = False
        self.vacuum_ready = False

        self.pub = rospy.Publisher("/swift_pro/goal_position", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("/swift_pro/goal_reach", Bool, self.goal_callback)
        self.ready_sub = rospy.Subscriber("/vacuum_ready", Bool, self.vacuum_callback)

    def goal_callback(self, msg: Bool):
        self.goal_reach = msg.data

    def vacuum_callback(self, msg: Bool):
        self.vacuum_ready = msg.data

    def initialise(self):
        self.sent = False
        self.goal_reach = False

    def update(self):
        rospy.loginfo(f"[GoUpZ] Z = {self.z:.2f}, sent={self.sent}, goal_reach={self.goal_reach}, vacuum_ready={self.vacuum_ready}")

        if not self.vacuum_ready:
            rospy.loginfo("[GoUpZ] Waiting for vacuum to be ON...")
            return py_trees.common.Status.RUNNING

        if self.goal_reach:
            rospy.loginfo("[GoUpZ] Goal reached ✅")
            return py_trees.common.Status.SUCCESS

        if not self.sent:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.x
            goal.pose.position.y = self.y
            goal.pose.position.z = self.z
            goal.pose.orientation.w = 1.0
            self.pub.publish(goal)
            self.sent = True
            rospy.loginfo(f"[GoUpZ] Sent Z goal: {self.z:.2f}")

        return py_trees.common.Status.RUNNING





# Build Behavior Tree
def create_behavior_tree():
    """go_near = PlanPath("Plan to Object", make_pose(1.65, OBJECT_Y))
    follow1 = FollowPath("Go Near", go_near)
    rotate_joint = TriggerJointRotation("Rotate Joint")
    send_goal = SendGoalToManipulator("Send Goal", OBJECT_X, OBJECT_Y)
    move_down = AdjustManipulatorZ("Move Down", -0.15, OBJECT_X, OBJECT_Y)
    vacuum_on = VacuumControl("Vacuum ON", True)
    move_up = AdjustManipulatorZ("Move Up", 0.10, OBJECT_X, OBJECT_Y)
    drop_plan = PlanPath("Plan Drop", make_pose(DROP_X, DROP_Y))
    drop_move = FollowPath("Move to Drop", drop_plan)
    vacuum_off = VacuumControl("Vacuum OFF", False)"""



    go_near = PlanPath("Plan to Object", make_pose(1.60, OBJECT_Y))
    follow1 = FollowPath("Go Near", go_near)
    rotate_joint = TriggerJointRotation("Rotate Joint")
    send_goal = SendGoalToManipulator("Send Goal", OBJECT_X, OBJECT_Y)
    move_down = AdjustManipulatorZ("Move Down", -0.15, OBJECT_X, OBJECT_Y)
    vacuum_on = VacuumControl("Vacuum ON", True)
    move_up = GoUpZAfterVacuum("Move Up", -0.05, OBJECT_X, OBJECT_Y)
    rospy.loginfo(f"[TREE] Vacuum node initialized with confirmed = {vacuum_on.confirmed}")


    root = py_trees.composites.Sequence("Pick and Place", memory=True)
    root.add_children([
        go_near, follow1,
        StopRobot("Stop at Object"),
        rotate_joint,
        send_goal,
        move_down,
        StopRobot("Hold still"),
        vacuum_on,
        move_up,                     
        StopRobot("Hold again"),
    ])



    """root = py_trees.composites.Sequence("Pick and Place", memory=True)
    root.add_children([
        go_near, follow1, StopRobot("Stop at Object"),
        rotate_joint,
        send_goal,
        move_down, StopRobot("Hold still"),
        vacuum_on,
        move_up, StopRobot("Hold again"),
        drop_plan, drop_move, StopRobot("Stop at Drop"),
        vacuum_off
    ])"""
    return root



# Main
if __name__ == '__main__':
    rospy.init_node("pickup_behavior_node")
    rospy.loginfo("[BT] Starting behavior tree...")
    vacuum_controller = vacuum_control()
    tree = create_behavior_tree()
    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        behaviour_tree.tick()
        rate.sleep()
