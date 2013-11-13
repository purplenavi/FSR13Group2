#!/usr/bin/env python
import roslib
roslib.load_manifest('gui_plannerz')
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy
from math import atan2
import tf
from tf.transformations import quaternion_from_euler
import python_qt_binding
from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt, QTimer
from python_qt_binding.QtGui import QWidget, QMessageBox, QTextEdit, QLabel, QPixmap, QBrush, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QPolygonF, QVBoxLayout, QHBoxLayout, QColor, qRgb, QPushButton, QRadioButton
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import String
import pcl

class Widgetti(QWidget):

    def __init__(self):
        super(Widgetti, self).__init__()
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()
        self.map_layout = QHBoxLayout()
        self.tf = tf.TransformListener()

        

        self.setWindowTitle('Gui plannerz lol')
        self.drive = QPushButton('DRIVE!')
        self.drive.clicked.connect(self.Engage)

        self.gui_publisher = rospy.Publisher('gui_plan', Path)
     

        self.debug_stream = QTextEdit(self)
        
        self.robomap = RoboMap(tf = self.tf, parent=self)
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
        #self.taskplanning.clicked.connect(self.taskplanner.execute)
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

    def update_textbox(self, header, txt):
        self.debug_stream.insertPlainText(header + '\n')
        self.debug_stream.insertPlainText(txt+'\n')

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
        if not self.timer:
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_position_current)
            self.timer.start(1000)
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
        print 'Origin at:' + str(msg.info.origin.position.x) + ' ' + str(msg.info.origin.position.y)
        self.parent.update_textbox('Origin at:', (str(msg.info.origin.position.x) + ' ' + str(msg.info.origin.position.y)))
        arr = numpy.array(msg.data, dtype=numpy.uint8, copy=False, order='C')
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
                angle = atan2(nxt_y - y, nxt_x - x)
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

    #STATES = Enum('RANDOM','REACHING','PICKING','RETURNING','RELEASING')

    def __init__(self, manip_topic = '/manip_servo_angles', driver_topic = 'RosAria/cmd_vel', detector_topic = '/pirate_detector', parent = None):
        self.state = None
        self.parent = parent
        self.manipulator = manip_topic
        #self.subscriber = rospy.Subscriber(self.manipulator,upancyGrid, self.manipulatorCb)
        self.manipulator_action = rospy.Publisher(self.manipulator, Vector3, latch=False)
        self.pirate_detector = rospy.Subscriber(detector_topic, String, self.detectorCb)
        self.driver = rospy.Publisher(driver_topic, Twist, latch=False)
        #self.manipulator_state = rospy.Subscriber(self.manipulator, String)
        #self.parent.update_textbox('Task Planner', 'Task planner initialized')
        #print 'Task planner initialized'
        rospy.sleep(1.0)
        self.openManipulator() # To ensure it's all the way opened


    def manipulatorCb(self, msg):
        self.parent.update_textbox('Manipulator subscription',msg)

    def goHomeBase(self):
        # How to really do this? Should we use ActionLib?
        plan = self.parent.robomap.get_plan()
        # Maybe not like this
        self.parent.robomap.deletePlan()
        self.parent.robomap.draw_point(0,0,add_point=True)
    
    def closeManipulator(self):
        self.manipulator_action.publish(Vector3(x=0.0))
        self.parent.update_textbox('Manipulator action','closing')
        #time.sleep(10) # just for testing before manipulator state publisher

    def openManipulator(self):
        self.manipulator_action.publish(Vector3(x=180.0))
        self.parent.update_textbox('Manipulator action','opening')
        #time.sleep(10) # just for testing before manipulator state publisher
        
    def taskplanning(self):
        # Starting from beginning
        self.parent.robomap.deletePlan()
        self.state = STATES.RANDOM

    # Some possible messages, should be other than string?
    def detectorCb(self, data):
        if data == 'init':
            self.parent.update_textbox('Pirate detector', 'initialized')
        elif data == 'found':
            self.parent.update_textbox('Pirate detector', 'going for figure')
        elif data == 'inplace':
            self.parent.update_textbox('Pirate detector', 'got to figure')
            self.closeManipulator()
            self.goHomeBase()    

    def dropFigure(self):
        self.openManipulator()
        # Opening
        rospy.sleep(0.5) 
        movement = Twist()
        movement.linear.x = -0.15
        self.driver.publish(movement)
        rospy.sleep(0.15)
        self.driver.publish(Twist())

if __name__ == "__main__":
    from python_qt_binding.QtGui import QApplication
    import sys
    rospy.init_node("gui_plannerz")
    app = QApplication(sys.argv)
    q = Widgetti()
    q.setMinimumSize(600, 700)
    q.show()
    app.exec_()
