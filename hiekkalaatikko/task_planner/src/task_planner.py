#!/usr/bin/env python
import roslib
roslib.load_manifest('task_planner')
import rospy
import numpy as np
import math
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Vector3, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid,Path
from std_msgs.msg import String
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
import sys
#sys.path.insert(0, '../../explorer/src/')
#from explorer import Explorer

goal_states={0:'PENDING',1:'ACTIVE',2:'PREEMPTED',3:'SUCCEEDED',4:'ABORTED',5:'REJECTED',6:'PREEMPTING',7:'RECALLING',8:'RECALLED',9:'LOST'}
robot_states={-1:'Exploring',0:'Seeking for pirates',1:'Grabbing figure',2:'Taking injured pirate home',3:'Dropping injured pirate home'}

class TaskPlanner():


    def __init__(self):
        self.state = 0
        self.move_goal = None
        #self.explorer = Explorer()
        self.manipulator_action = rospy.Publisher('/manip_servo_angles', Vector3, latch=False)
        self.driver = rospy.Publisher('RosAria/cmd_vel', Twist, latch=False)

        self.laser = rospy.Subscriber('/map',OccupancyGrid,self.map_callback)

        self.state_pub = rospy.Publisher('/task_planner/state', String, latch=False)
        self.textbox_pub = rospy.Publisher('/task_planner/textbox', String)

        self.actionclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.actionclient.wait_for_server()
        self.pose_sub = rospy.Subscriber('RosAria/pose', Odometry, self.pose_callback)

        self.pirate_detector = rospy.Subscriber('/Pirates', Path, self.pirate_callback)
        self.origin = None
        self.pose = None
        self.pirates = []
        self.pirates_called = False
        self.waiting = False
        rospy.sleep(1.0)
        self.home = [0.0, 0.0]
        self.openManipulator() # To ensure it's all the way opened

    def map_callback(self, msg):
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def update_textbox(self, header, txt):
        msg = String()
        msg.data=str(header)+'|HEADERMSGSPLITTER|'+str(txt)
        self.textbox_pub.publish(msg)

    def pirate_callback(self, data):
        for z in data.poses:
            self.pirates.append(z)
        if self.state == 0:
            self.pirates_called = True

    def next_state(self,state = None):
        if state is None:
            self.state += 1
        else:
            self.state = state
        if robot_states.get(self.state) is None:
            self.state = 0
        elif robot.state == 0:
            self.pirates_called = False
        print 'Task planner state changed to: '+str(self.state)+' - '+robot_states.get(self.state)
        statemsg = String()
        statemsg.data = robot_states.get(self.state)
        self.state_pub.publish(statemsg)

    def explore(self):
        self.next_state(-1)
        self.state_pub.publish(robot_states.get(self.state))

    def execute(self):
        if not self.pirates:
            print 'NO PIRATES ASSHOLE!'
            if tmp:
                print 'yay'
            self.explorer_pub.publish('Gimme sum coordinates, mate')
            self.update_textbox('Explorer', 'Asking next coordinates')
            # Trying with just one movement, reassigned when action movement succeeded (done_callback or feedback)
            self.explorer_pub.unregister()
            self.exit = False
        else:
            print 'executing task'
            while True:
                if not self.waiting:
                    self.update_textbox('Current state',robot_states.get(self.state)+' ('+str(self.state)+')')
                    if self.state == 0: 
                        print 'Moving to pirate for first time'
                        self.move_to_pirate()
                        self.waiting = True
                        
                    elif self.state == 1:
                        self.actionclient.cancel_all_goals()
                        print 'closing'
                        self.waiting = True
                        self.grab_figure()
                        
                    elif self.state == 2:
                        self.goHomeBase()
                        self.waiting = True
                        
                    elif self.state == 3:
                        self.actionclient.cancel_all_goals()
                        print 'dropping'
                        self.waiting = True
                        self.drop_figure()
                        rospy.sleep(2.0)
                        if not self.pirates:
                            print 'No more pirates lol'
                            self.exit = True
                    elif self.state == -1:
                        print 'Should get next point from explorer here'
                    else:
                        self.next_state()

    def feedback(self, feedback):
        print 'in the feedback lol'
        pose_stamp = feedback.base_position
        self.timer += 1
        #if self.pirate_detector.move_goal:
            #self.update_textbox('Current distance from goal', str(self.distance(self.pirate_detector.move_goal, pose_stamp)))
        if self.taskplanner.move_goal and self.distance(self.taskplanner.move_goal, pose_stamp) < 0.2:
            self.timer = 0
            #print str(self.distance(self.pirate_detector.move_goal, pose_stamp))
            # this check cancels the goal if the robot is "close enough"
            # move_base will endlessly spin sometimes without this code
            self.actionclient.cancel_goal()
            rospy.sleep(1.0)
            self.goal = None
            #notification = RideNotification()
            #notification.level = RideNotification.INFO
            #notification.msg = "Movement Complete!"
            #self.notificationPub.publish(notification)
            #self.update_textbox('Moving to grabbing', 'YAY!')
            print 'Moving to next state from ' + str(self.taskplanner.state)
            self.next_state()
            self.waiting = False
        if self.timer > 500:
            self.actionclient.cancel_goal()
            self.goal = None
            self.next_state()
            self.waiting = False
            self.timer = 0

    def move_to_pirate(self):
        self.actionclient.cancel_all_goals()
        self.move_goal = MoveBaseGoal(target_pose=self.pirates.pop())
        print 'Pirate at: ' + str(self.move_goal)
        self.update_textbox('Moving towards pirate:', str(self.move_goal))
        self.actionclient.send_goal(self.move_goal, feedback_cb=self.feedback)

    def manipulatorCb(self, msg):
        self.update_textbox('Manipulator subscription',msg)
        
    def goToLocation(self,x,y):
        location = PoseStamped()
        quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.origin[1], x-self.origin[0]))
        location.header.frame_id = 'map'
        location.header.stamp = rospy.Time.now()
        location.pose.position.x = x
        location.pose.position.y = y
        location.pose.orientation.w = quaternion[3]
        location.pose.orientation.z = quaternion[2]
        location = MoveBaseGoal(target_pose=location)
        self.move_goal = location
        self.actionclient.send_goal(location, feedback_cb=self.feedback)
        
    def grab_figure(self):
        self.closeManipulator()
        # Closing
        rospy.sleep(1.0)
        self.next_state()
        self.waiting = False

    def drop_figure(self):
        self.openManipulator()
        # Opening
        rospy.sleep(1.0)
        reverse_pose = self.pose
        reverse_pose.pose.pose.position.x -= 0.2
        self.reverse(0.3)
        timer = 0
        while self.waiting:
            print self.reverse_feedback(self.pose, reverse_pose)
            if self.reverse_feedback(self.pose, reverse_pose) < 0.4:
                self.update_textbox('Reversing done','Yay!')
                #Go back to first state
                self.next_state()
                print 'setting the state back to 0'
                self.waiting = False
            timer+=1
            if timer > 1000:
                self.next_state()
                print 'timer limit reached'
                self.waiting = False
                break
        
    def reverse_feedback(self, t1, t2):
        """
        Given two PoseStamped's, determine the distance
        """
        out = 0
        for dimension in ('x', 'y'):
            out += math.pow(getattr(t1.pose.pose.position, dimension) - getattr(t2.pose.pose.position, dimension), 2)
        return math.sqrt(out)

    def reverse(self,distance=0.2):
        r = rospy.Rate(1.0) # 1 Hz
        movement = Twist()
        movement.linear.x = -0.15 # speed
        time = (-1.0*distance/movement.linear.x)
        for i in range(int(time)): # ~ distance
            self.driver.publish(movement)
            r.sleep()
        movement = Twist()
        movement.angular.z = 1.57/2 # ~45 deg/s
        for i in range(4):
            self.driver.publish(movement)
            r.sleep()
        self.driver.publish(Twist())

    def goHomeBase(self):
        self.goToLocation(self.home[0],self.home[1])
        print 'Getting this pirate home'

    def closeManipulator(self):
        self.manipulator_action.publish(Vector3(x=0.0))
        self.update_textbox('Manipulator action','closing')
        #time.sleep(10) # just for testing before manipulator state publisher

    def openManipulator(self):
        self.manipulator_action.publish(Vector3(x=180.0))
        self.update_textbox('Manipulator action','opening')
        #time.sleep(10) # just for testing before manipulator state publisher

def main(args):
    rospy.init_node('task_planner')
    rospy.sleep(1.0)
    taskplanner = TaskPlanner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down task_planner node.'

if __name__ == "__main__":
    main(sys.argv)


