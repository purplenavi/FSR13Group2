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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
import os
import sys
import cv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from ride_msgs.msg import RideNotification, RidePose
import actionlib
from actionlib_msgs.msg import GoalStatus

goal_states={'0':'PENDING','1':'ACTIVE','2':'PREEMPTED','3':'SUCCEEDED','4':'ABORTED','5':'REJECTED','6':'PREEMPTING','7':'RECALLING','8':'RECALLED','9':'LOST'}

class Widgetti(QWidget):

    def __init__(self):
        super(Widgetti, self).__init__()
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()
        self.map_layout = QHBoxLayout()
        self.tf = tf.TransformListener()

        self.pirates = []
        self.dead_pirates = []
        self.pirate_update = True
        self.dead_pirate_update = False
        self.pose = None
        self.waiting = False

        self.setWindowTitle('Gui plannerz lol')
        self.drive = QPushButton('Removed driving lol!')
        #Removing driving, might mess up the task planner?
        #self.drive.clicked.connect(self.Engage)

        self.gui_publisher = rospy.Publisher('gui_plan', Path)
        self.actionclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pose_sub = rospy.Subscriber('RosAria/pose', Odometry, self.pose_callback)
        self.pirate_sub = rospy.Subscriber('Pirates', Path, self.pirate_callback)
        self.dead_pirate_sub = rospy.Subscriber('Dead', Path, self.dead_pirate_callback)
        #self.notificationPub = rospy.Publisher('notification', RideNotification)
        self.actionclient.wait_for_server()
        self.debug_stream = QTextEdit(self)
        
        self.robomap = RoboMap(tf = self.tf, parent=self)
        rospy.sleep(1.0)
        self.taskplanner = TaskPlanner(parent = self)
        self.delete_plan = QPushButton('Delete planz')
        self.delete_plan.clicked.connect(self.robomap.deletePlan)
        self.button_layout.addWidget(self.drive)
        self.button_layout.addWidget(self.delete_plan)

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

        self.stop_robot = QPushButton('STOP!')
        self.stop_robot.clicked.connect(self.robomap.stop_robot)
        self.button_layout.addWidget(self.open_manip)
        self.button_layout.addWidget(self.close_manip)
        self.button_layout.addWidget(self.stop_robot)

        self.map_layout.addWidget(self.robomap)
        self.map_layout.addWidget(self.debug_stream)
        self.layout.addLayout(self.map_layout)
        self.layout.addLayout(self.button_layout)
        self.layout.addWidget(QLabel('Click path points with mouze. Last point with rclickz'))
        self.setLayout(self.layout)
        
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

    def update_textbox(self, header, txt):
        self.debug_stream.insertPlainText(header + '\n')
        self.debug_stream.insertPlainText(txt+'\n')
        
    def pose_callback(self, data):
        self.pose = data
        
    def done_callback(self, status, result):
        #if self.pirate_detector.move_goal is None:
        #    return
        #elif status == self.actionclient.get_state():
        #    self.pirate_detector.move_goal = None
        # Send notification to UI
        #notification = RideNotification()
        if status is GoalStatus.RECALLED:
            print 'recalled'
            #self.update_textbox('Goal Status', 'RECALLED (movement cancelled)')
            #notification.level = RideNotification.DEBUG
            #notification.msg = "Movement Cancelled"
        elif status is GoalStatus.SUCCEEDED:
            print 'success'
            self.taskplanner.state += 1
            #self.taskplanner.explorer_pub = rospy.Publisher('explore_next_point', String, latch=False)
            #notification.level = RideNotification.INFO
            #notification.msg = "Movement Complete!"
            #self.update_textbox('Goal Status' ,'SUCCEEDED')
            #self.taskplanner.state = self.taskplanner.state + 1
        elif status is GoalStatus.REJECTED:
            print 'rejected'
            #self.update_textbox('Goal Status', 'REJECTED')
            #notification.level = RideNotification.WARN
            #notification.msg = "Destination Rejected!"
        elif status is GoalStatus.ABORTED:
            print 'aborted'
            #self.update_textbox('Goal Status', 'ABORTED')
            #notification.level = RideNotification.ERROR
            #notification.msg = "Movement Aborted!"
        else:
            print 'sumthing else'
            print goal_states.get(str(status))
            #self.update_textbox('Goal Status', 'WTF? something else')
            #notification.level = RideNotification.DEBUG
            #notification.msg = "[GoTo] Odd Completion"
        #self.notificationPub.publish(notification)
    
    def feedback(self, feedback):
        pose_stamp = feedback.base_position
        #if self.pirate_detector.move_goal:
            #self.update_textbox('Current distance from goal', str(self.distance(self.pirate_detector.move_goal, pose_stamp)))
        if self.taskplanner.move_goal and self.distance(self.taskplanner.move_goal, pose_stamp) < 0.2:
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

    def stop_robot(self):
        plan = self.get_plan()
        print 'Got plan'
        if plan:
            print 'Plan exists'
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
                break
            self.parent.gui_publisher.publish(path)
            self.deletePlan()
    
    def mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)
    
    def callback(self, msg):
        self.w = msg.info.width
        self.h = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        #print 'Origin at:' + str(msg.info.origin.position.x) + ' ' + str(msg.info.origin.position.y)
        #self.parent.update_textbox('Origin at:', (str(msg.info.origin.position.x) + ' ' + str(msg.info.origin.position.y)))
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

    def deletePlan(self):
        if self.polygon:
            self.scene.removeItem(self.polygon)
            self.polygon = None
        if self.point:
            self.scene.removeItem(self.point)
            self.point = None
        if self.points:
            for z in self.points:
                self.scene.removeItem(z)
            self.points = None

    def draw_point(self, x, y, color=Qt.magenta, rad=1.0, add_point=False):
        ell = self.scene.addEllipse(x-rad, y-rad, rad*2.0, rad*2.0, color, QBrush(Qt.SolidPattern))
        self.parent.update_textbox('Point added:', (str(x) + ' ' + str(y)))
        ell.setZValue(2000.0)
        if add_point:
            if self.points:
                self.points.append(ell)
            else:
                self.points = [ell]
        return ell
    
    def mousePressEvent(self, e):
        if e.button() == Qt.RightButton:
            return
        self.setMouseTracking(True)
        
    def mouseMoveEvent(self, e):
        point = self.mapToScene(e.x(), e.y())
        if self.point:
            self.scene.removeItem(self.point)
        self.point = self.draw_point(point.x(), point.y(), Qt.yellow, 1.0)
        if self.polygon:
            porygon = self.polygon.polygon()
            if porygon.size() > 1:
                porygon.replace(porygon.size()-1, QPointF(point.x(), point.y()))
            else:
                porygon.append(QPointF(point.x(), point.y()))
            self.polygon.setPolygon(porygon)
        else:
            return

    def wheelEvent(self, e):
        e.ignore()
        if e.delta() > 0:
            self.scale(1.30, 1.30)
        else:
            self.scale(0.7, 0.7)
    
    def mouseReleaseEvent(self, e):
        point = self.mapToScene(e.x(), e.y())
        if e.button() == Qt.RightButton:
            self.setMouseTracking(False)
            self.draw_point(point.x(), point.y(), add_point=True)
            return
        if self.point:
            self.scene.removeItem(self.point)
            self.point = None
        if self.polygon:
            porygon = self.polygon.polygon()
            porygon.append(QPointF(point.x(), point.y()))
            self.polygon.setPolygon(porygon)
            self.draw_point(point.x(), point.y(), add_point=True)
        else:
            self.polygon = self.scene.addPolygon(QPolygonF([QPointF(point.x(), point.y())]), Qt.red)
            self.polygon.setZValue(1000.0)
            self.draw_point(point.x(), point.y(), Qt.yellow, add_point=True)

class TaskPlanner():


    def __init__(self, manip_topic = '/manip_servo_angles', driver_topic = 'RosAria/cmd_vel', parent = None):
        self.state = 0
        self.move_goal = None
        self.parent = parent
        self.manipulator = manip_topic
        #self.subscriber = rospy.Subscriber(self.manipulator,upancyGrid, self.manipulatorCb)
        self.manipulator_action = rospy.Publisher(self.manipulator, Vector3, latch=False)
        self.driver = rospy.Publisher(driver_topic, Twist, latch=False)
        self.explorer_sub = rospy.Subscriber('/explore_point', PoseStamped, self.explorer_callback)
        self.explorer_pub = rospy.Publisher('explore_next_point', String, latch=False)
        #self.manipulator_state = rospy.Subscriber(self.manipulator, String)
        #self.parent.update_textbox('Task Planner', 'Task planner initialized')
        #print 'Task planner initialized'
        rospy.sleep(1.0)
        self.home = [0, 0]
        self.exit = False
        
        self.openManipulator() # To ensure it's all the way opened

    def execute(self):
        if not self.parent.pirates:
            print 'NO PIRATES ASSHOLE!'
            self.parent.pirate_update = True
            self.explorer_pub.publish('Gimme sum coordinates, mate')
            # Trying with just one movement, reassigned when action movement succeeded (done_callback or feedback)
            explorer_pub = None
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
                        #self.parent.update_textbox('Closing and going home','Trolloloo')
                        print 'closing'
                        self.parent.waiting = True
                        self.grab_figure()
                        
                    elif self.state == 2:
                        self.goHomeBase()
                        #rospy.sleep(2.0)
                        #self.state = 2
                        
                    elif self.state == 3:
                        print 'dropping'
                        self.parent.waiting = True
                        self.drop_figure()
                        rospy.sleep(2.0)
                        if not self.parent.pirates:
                            self.exit = True
                    
                    elif self.state == 4:
                        print 'State 4 what the nigger?!'
                else:
                    pass

    def explorer_callback(self,data):
        self.goToLocation(data.pose.position.x,data.pose.position.y)

    def move_to_pirate(self):
        self.move_goal = MoveBaseGoal(target_pose=self.parent.pirates.pop())
        print 'Pirate at: ' + str(self.move_goal)
        self.parent.actionclient.send_goal(self.move_goal, done_cb = self.parent.done_callback, feedback_cb=self.parent.feedback)

    def manipulatorCb(self, msg):
        self.parent.update_textbox('Manipulator subscription',msg)
        
    def goToLocation(self,x,y):
        location = PoseStamped()
        quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.parent.robomap.origin[1], x-self.parent.robomap.origin[0]))
        location.header.frame_id = 'map'
        location.header.stamp = rospy.Time.now()
        location.pose.position.x = x
        location.pose.position.y = y
        location.pose.orientation.w = self.parent.pose.pose.pose.orientation.w
        location.pose.orientation.z = self.parent.pose.pose.pose.orientation.z
        location = MoveBaseGoal(target_pose=location)
        print 'Sending home goal'
        self.parent.actionclient.send_goal(location, done_cb = self.parent.done_callback, feedback_cb=self.parent.feedback)
        
    def grab_figure(self):
        self.closeManipulator()
        # Closing
        rospy.sleep(0.5)
        self.state += 1
        self.parent.waiting = False

    def drop_figure(self):
        self.openManipulator()
        # Opening
        rospy.sleep(0.5)
        reverse_pose = self.parent.pose
        reverse_pose.pose.pose.position.x -= 0.2
        self.reverse()
        while self.parent.waiting:
            if self.reverse_feedback(self.parent.pose, reverse_pose) < 0.2:
                self.parent.update_textbox('Reversing done','Yay!')
                #Go back to first state
                self.state = 0
                print 'setting the state back to 0'
                self.parent.waiting = False
        
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
        self.parent.waiting = True
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
    rospy.init_node("gui_plannerz")
    app = QApplication(sys.argv)
    q = Widgetti()
    q.setMinimumSize(600, 700)
    q.show()
    app.exec_()
