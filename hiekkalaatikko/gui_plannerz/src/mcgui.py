#!/usr/bin/env python
import roslib
roslib.load_manifest('gui_plannerz')
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import math
import tf
from tf.transformations import quaternion_from_euler
import python_qt_binding
from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt, QTimer
from python_qt_binding.QtGui import QWidget, QMessageBox, QTextEdit, QLabel, QPixmap, QBrush, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QPolygonF, QVBoxLayout, QHBoxLayout, QColor, qRgb, QPushButton, QRadioButton
from geometry_msgs.msg import PoseStamped, Vector3, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
import os
#import cv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
import sys
#sys.path.insert(0, '../../pirate_detector/src/')
#from pirate_detector import pirate_detector

goal_states={0:'PENDING',1:'ACTIVE',2:'PREEMPTED',3:'SUCCEEDED',4:'ABORTED',5:'REJECTED',6:'PREEMPTING',7:'RECALLING',8:'RECALLED',9:'LOST'}

class Widgetti(QWidget):

    def __init__(self):
        super(Widgetti, self).__init__()
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()
        self.map_layout = QHBoxLayout()
        self.tf = tf.TransformListener()

        self.pirates = []
        self.dead_pirates = []
        self.dead_pirate_objects = []
        self.pirate_update = True
        self.dead_pirate_update = False
        self.pose = None
        self.waiting = False
        #self.pirate_detector = pirate_detector() #Initializing pirate detector

        self.setWindowTitle('Gui plannerz lol')

        self.gui_publisher = rospy.Publisher('gui_plan', Path)
        self.actionclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        #self.notificationPub = rospy.Publisher('notification', RideNotification)
        self.actionclient.wait_for_server()
        self.debug_stream = QTextEdit(self)
        
        self.robomap = RoboMap(tf = self.tf, parent=self)
        rospy.sleep(1.0)
        self.taskplanner = TaskPlanner(parent = self)

        # task planner stuff
        self.open_manip = QPushButton('Open manipulator')
        self.open_manip.clicked.connect(self.taskplanner.openManipulator)
        self.close_manip = QPushButton('Close manipulator')
        self.close_manip.clicked.connect(self.taskplanner.closeManipulator)

        self.taskplanning = QPushButton('Collect figures')
        self.taskplanning.clicked.connect(self.taskplanner.execute)
        self.button_layout.addWidget(self.open_manip)
        self.button_layout.addWidget(self.close_manip)
        self.button_layout.addWidget(self.taskplanning)

        self.button_layout.addWidget(self.open_manip)
        self.button_layout.addWidget(self.close_manip)

        self.map_layout.addWidget(self.robomap)
        self.map_layout.addWidget(self.debug_stream)
        self.layout.addLayout(self.map_layout)
        self.layout.addLayout(self.button_layout)
        self.layout.addWidget(QLabel('Graphical interface to visualize stuff'))
        self.setLayout(self.layout)
        self.timer = 0
        self.pose_sub = rospy.Subscriber('RosAria/pose', Odometry, self.pose_callback)

        # Pirate detector subscribers
        self.pirates_sub = rospy.Subscriber('/Pirates', Path, self.pirate_callback)
        self.dead_pirates_sub = rospy.Subscriber('/Dead', Path, self.dead_pirate_callback)
        
        self.pirate_sub = rospy.Subscriber('Pirates', Path, self.pirate_callback)
        self.dead_pirate_sub = rospy.Subscriber('Dead', Path, self.dead_pirate_callback)
        self.pirate_update = True
        self.dead_pirate_update = True
        
    def pirate_callback(self, data):
        if self.pirate_update:
            for z in data.poses:
                self.pirates.append(z)
            self.pirate_update = False
    
    def dead_pirate_callback(self, data):
        if self.dead_pirate_update:
            for z in data.poses:
                self.dead_pirates.append(z)
            self.dead_pirate_update = False
        
    def get_data_from_camera(self):
        for z in self.dead_pirate_objects:
            self.robomap.scene.removeItem(z)
        #self.dead_pirate_objects = None
        #self.pirate_detector.activate_node()
        #while not self.pirate_detector.pirate_coordinates:
        #    rospy.sleep(0.5)
        #for z in self.pirate_detector.pirate_coordinates.poses:
        #    self.pirates.append(z)
        #for z in self.pirate_detector.dead_pirate_coordinates.poses:
        #    self.dead_pirates.append(z)
        #self.robomap.update_map(self.dead_pirates)
        return True

    def update_textbox(self, header, txt):
        self.debug_stream.insertPlainText(header + '\n')
        self.debug_stream.insertPlainText(txt+'\n')
        
    def pose_callback(self, data):
        if self.robomap.point:
            self.robomap.scene.removeItem(self.robomap.point)
            self.robomap.point = None
        self.pose = data
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        #transform pose coordinates to map coordinates
        map_y = (y - self.robomap.origin[1])/self.robomap.resolution
        map_x = -((x - self.robomap.origin[0])/self.robomap.resolution) + self.robomap.w
        self.robomap.point = self.robomap.draw_point(map_x, map_y, color=Qt.blue, rad=3.0)
        
    def done_callback(self, status, result):
        if status is GoalStatus.RECALLED:
            print 'recalled'
        elif status is GoalStatus.SUCCEEDED:
            print 'success'
        elif status is GoalStatus.REJECTED:
            print 'rejected'
        elif status is GoalStatus.ABORTED:
            print 'aborted'
        else:
            print 'sumthing else'
            print goal_states.get(status)

    def pirate_callback(self, data):
        if self.pirate_update:
            for z in data.poses:
                self.pirates.append(z)
            self.update_textbox('Number of pirates: ', str(len(self.pirates)))
            self.pirate_update = False

    def dead_pirate_callback(self, data):
        if self.dead_pirate_update:
            for z in data.poses:
                self.dead_pirates.append(z)
            self.dead_pirate_update = False
            self.robomap.insert_to_map(self.dead_pirates)

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
            self.taskplanner.state = self.taskplanner.state + 1
            self.waiting = False
            self.taskplanner.explorer_pub = rospy.Publisher('explore_next_point', String, latch=False)
        if self.timer > 500:
            self.actionclient.cancel_goal()
            self.goal = None
            self.taskplanner.state = self.taskplanner.state + 1
            self.waiting = False
            self.timer = 0
            
    def distance(self, t1, t2):
        """
        Given two PoseStamped's, determine the distance
        """
        out = 0
        for dimension in ('x', 'y'):
            out += math.pow(getattr(t1.target_pose.pose.position, dimension) - getattr(t2.pose.position, dimension), 2)
        return math.sqrt(out)

    def Engage(self):
        plan = self.robomap.get_plan()
        if not plan:
            QMessageBox.critical(self, "No plan or map :(", "OK")
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for z in plan:
            stamp = PoseStamped()
            pos = z[0]
            x = pos[0]
            y = pos[1]
            quaternion = z[1]
            stamp.header.stamp = rospy.Time.now()
            stamp.header.frame_id = "map"
            stamp.pose.position.x = x
            stamp.pose.position.y = y
            stamp.pose.orientation.w = quaternion[3]
            stamp.pose.orientation.z = quaternion[2]
            path.poses.append(stamp)
            self.update_textbox('path coordinate_point:', (str(x) + ' ' + str(y)))
        self.gui_publisher.publish(path)

        
class RoboMap(QGraphicsView):

    map_change = Signal()

    def __init__(self, topic='/map', tf=None, parent=None):

        super(RoboMap, self).__init__()
        self.parent = parent
        self.tf = tf
        self.topic = topic
        self.map_change.connect(self.update)
        self.destroyed.connect(self.close)
        self.map = None
        self.mapitem = None
        self.w = 0
        self.h = 0
        self.resolution = None
        self.origin = None
        self.polygon = None
        self.point = None
        self.points = None
        self.scene = QGraphicsScene()
        self.subscriber = rospy.Subscriber(topic, OccupancyGrid, self.callback)
        self.setScene(self.scene)
        self.timer = None
    
    def update(self):
        if self.mapitem:
            self.scene.removeItem(self.mapitem)
        qpix = QPixmap.fromImage(self.map)
        self.mapitem = self.scene.addPixmap(qpix)
        self.mirror(self.mapitem)
        self.show()
    
    def mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)
    
    def callback(self, msg):
        self.w = msg.info.width
        self.h = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        arr = np.array(msg.data, dtype=np.uint8, copy=False, order='C')
        arr = arr.reshape((self.h, self.w))
        img = QImage(arr.reshape((arr.shape[0] * arr.shape[1])), self.w, self.h, QImage.Format_Indexed8)
        #need to invert some colors :)
        for z in reversed(range(101)):
            img.setColor(100 - z, qRgb(z*2.55, z*2.55, z*2.55))
        img.setColor(101, qRgb(255, 0, 0))
        img.setColor(255, qRgb(100, 100, 100))
        self.map = img
        self.setSceneRect(0, 0, self.w, self.h)
        self.map_change.emit()

    def get_plan(self):
        if self.polygon and self.resolution:
            porygon = self.polygon.polygon()
            point_list = []
            for z in range(porygon.size()):
                x = (((self.w/2) - porygon[z].x()) + (self.w/2)) * self.resolution + self.origin[0]
                if z < porygon.size()-1:
                    nxt = porygon[z+1]
                else: #meaning last point
                    nxt = porygon[0]
                nxt_x = (((self.w/2) - nxt.x()) + (self.w/2)) * self.resolution + self.origin[0]
                y = porygon[z].y() * self.resolution + self.origin[1]
                nxt_y = nxt.y() * self.resolution + self.origin[1]
                angle = math.atan2(nxt_y - y, nxt_x - x)
                quaternion = quaternion_from_euler(0, 0, angle)
                point_list.append(((x, y), quaternion))
            return point_list
        else:
            return None
            
    def update_map(self, dead_pirates):
        tmp = []
        for z in dead_pirates:
            x = z.pose.position.x
            y = z.pose.position.y
            #transform pose coordinates to map coordinates
            map_y = (y - self.origin[1])/self.resolution
            map_x = -((x - self.origin[0])/self.resolution) + self.w
            tmp.append(self.draw_point(map_x, map_y, color=Qt.red))
        self.parent.dead_pirate_objects = tmp

    def draw_point(self, x, y, color=Qt.magenta, rad=1.0, add_point=False, message=None):
        ell = self.scene.addEllipse(x-rad, y-rad, rad*2.0, rad*2.0, color, QBrush(Qt.SolidPattern))
        if message:
            self.parent.update_textbox(message, (str(x) + ' ' + str(y)))
        ell.setZValue(2000.0)
        if add_point:
            if self.points:
                self.points.append(ell)
            else:
                self.points = [ell]
        return ell
        
    def wheelEvent(self, e):
        e.ignore()
        if e.delta() > 0:
            self.scale(1.30, 1.30)
        else:
            self.scale(0.7, 0.7)

        
class TaskPlanner():


    def __init__(self, manip_topic = '/manip_servo_angles', driver_topic = 'RosAria/cmd_vel', parent = None):
        self.state = 0
        self.move_goal = None
        self.parent = parent
        self.manipulator = manip_topic
        self.manipulator_action = rospy.Publisher(self.manipulator, Vector3, latch=False)
        self.driver = rospy.Publisher(driver_topic, Twist, latch=False)
        self.explorer_sub = rospy.Subscriber('/explore_point', PoseStamped, self.explorer_callback)
        self.explorer_pub = rospy.Publisher('explore_next_point', String, latch=False)
        rospy.sleep(1.0)
        self.home = [0.0, 0.0]
        self.exit = False
        
        self.openManipulator() # To ensure it's all the way opened

    def execute(self):
        if not self.parent.pirates:
            print 'NO PIRATES ASSHOLE!'
            tmp = self.parent.get_data_from_camera()
            if tmp:
                print 'yay'
            self.explorer_pub.publish('Gimme sum coordinates, mate')
            self.parent.update_textbox('Explorer', 'Asking next coordinates')
            # Trying with just one movement, reassigned when action movement succeeded (done_callback or feedback)
            self.explorer_pub.unregister()
            self.exit = False
        else:
            print 'executing task'
            while True:
                if self.exit:
                    break
                if not self.parent.waiting:
                    self.parent.update_textbox('Current state',str(self.state))
                    if self.state == 0:
                        #self.parent.update_textbox('Moving to pirate','Trolloloo')
                        print 'Moving to pirate for first time'
                        self.move_to_pirate()
                        self.parent.waiting = True
                        
                    elif self.state == 1:
                        self.parent.actionclient.cancel_all_goals()
                        #self.parent.update_textbox('Closing and going home','Trolloloo')
                        print 'closing'
                        self.parent.waiting = True
                        self.grab_figure()
                        
                    elif self.state == 2:
                        self.goHomeBase()
                        self.parent.waiting = True
                        #rospy.sleep(2.0)
                        #self.state = 2
                        
                    elif self.state == 3:
                        self.parent.actionclient.cancel_all_goals()
                        print 'dropping'
                        self.parent.waiting = True
                        self.drop_figure()
                        rospy.sleep(2.0)
                        if not self.parent.pirates:
                            print 'No more pirates lol'
                            self.exit = True
                    
                    elif self.state == 4:
                        print 'State 4 what the nigger?!'
                        self.state = 0
                else:
                    pass

    def explorer_callback(self,data):
        self.parent.update_textbox('Next coordinates from explorer', '('+str(data.pose.position.x)+','+str(data.pose.position.y)+')')
        print data
        self.goToLocation(data.pose.position.x,data.pose.position.y)

    def move_to_pirate(self):
	self.parent.actionclient.cancel_all_goals()
        self.move_goal = MoveBaseGoal(target_pose=self.parent.pirates.pop())
        print 'Pirate at: ' + str(self.move_goal)
        self.parent.update_textbox('Moving towards pirate:', str(self.move_goal))
        self.parent.actionclient.send_goal(self.move_goal, feedback_cb=self.parent.feedback)

    def manipulatorCb(self, msg):
        self.parent.update_textbox('Manipulator subscription',msg)
        
    def goToLocation(self,x,y):
        location = PoseStamped()
        quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.parent.robomap.origin[1], x-self.parent.robomap.origin[0]))
        location.header.frame_id = 'map'
        location.header.stamp = rospy.Time.now()
        location.pose.position.x = x
        location.pose.position.y = y
        location.pose.orientation.w = quaternion[3]
        location.pose.orientation.z = quaternion[2]
        location = MoveBaseGoal(target_pose=location)
        self.move_goal = location
        self.parent.actionclient.send_goal(location, feedback_cb=self.parent.feedback)
        
    def grab_figure(self):
        self.closeManipulator()
        # Closing
        rospy.sleep(1.0)
        self.state += 1
        self.parent.waiting = False

    def drop_figure(self):
        self.openManipulator()
        # Opening
        rospy.sleep(1.0)
        reverse_pose = self.parent.pose
        reverse_pose.pose.pose.position.x -= 0.2
        self.reverse(0.3)
        timer = 0
        while self.parent.waiting:
            print self.reverse_feedback(self.parent.pose, reverse_pose)
            if self.reverse_feedback(self.parent.pose, reverse_pose) < 0.4:
                self.parent.update_textbox('Reversing done','Yay!')
                #Go back to first state
                self.state = 0
                print 'setting the state back to 0'
                self.parent.waiting = False
            timer+=1
            if timer > 1000:
                self.state = 0
                print 'timer limit reached'
                self.parent.waiting = False
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
        self.parent.update_textbox('Manipulator action','closing')
        #time.sleep(10) # just for testing before manipulator state publisher

    def openManipulator(self):
        self.manipulator_action.publish(Vector3(x=180.0))
        self.parent.update_textbox('Manipulator action','opening')
        #time.sleep(10) # just for testing before manipulator state publisher
        

if __name__ == "__main__":
    from python_qt_binding.QtGui import QApplication
    import sys
    print 'lol'
    rospy.init_node("gui_plannerz")
    app = QApplication(sys.argv)
    q = Widgetti()
    q.setMinimumSize(600, 700)
    q.show()
    app.exec_()
