#!/usr/bin/env python
import roslib
roslib.load_manifest('gui_plannerz')
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from math import atan2
import tf
from tf.transformations import quaternion_from_euler
import python_qt_binding
from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt, QTimer
from python_qt_binding.QtGui import QWidget, QMessageBox, QTextEdit, QLabel, QPixmap, QBrush, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QPolygonF, QVBoxLayout, QHBoxLayout, QColor, qRgb, QPushButton, QRadioButton
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
import os
import sys
import cv

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
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped)
        self.pirate_detector = pirate_detector(parent=self)
     

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


    def __init__(self, manip_topic = '/manip_servo_angles', driver_topic = 'RosAria/cmd_vel', detector_topic = '/pirate_detector', parent = None):
        self.state = 'detect_pirate'
        self.parent = parent
        self.manipulator = manip_topic
        #self.subscriber = rospy.Subscriber(self.manipulator,upancyGrid, self.manipulatorCb)
        self.manipulator_action = rospy.Publisher(self.manipulator, Vector3, latch=False)
        self.driver = rospy.Publisher(driver_topic, Twist, latch=False)
        #self.manipulator_state = rospy.Subscriber(self.manipulator, String)
        #self.parent.update_textbox('Task Planner', 'Task planner initialized')
        #print 'Task planner initialized'
        rospy.sleep(1.0)
        self.home = self.parent.robomap.origin
        
        self.openManipulator() # To ensure it's all the way opened

    def execute(self):
        print 'executing task'
        while True:
            if self.state == 'detect_pirate':
                self.parent.pirate_detector.move_to_pirate()
                print 'moving'
                goal = None
                while not goal:
                    goal = self.parent.pirate_detector.get_move_goal()
                    print goal
                    print self.parent.robomap.origin
                if abs(goal.pose.position.x - self.parent.robomap.origin[0]) < 0.05 and abs(goal.pose.position.y - self.parent.robomap.origin[1]) < 0.05:
                    self.state = 'grab_pirate_go_home'
            
            elif self.state == 'grab_pirate_go_home':
                print 'closing'
                self.grab_figure()
                rospy.sleep(2.0)
                self.state = 'drop_pirate'
                
            elif self.state == 'drop_pirate':
                print 'dropping'
                self.drop_figure()
                rospy.sleep(2.0)
                self.state = 'detect_pirate'

    def manipulatorCb(self, msg):
        self.parent.update_textbox('Manipulator subscription',msg)
        
    def goToLocation(self,x,y):
        location = PoseStamped()
        quaternion = quaternion_from_euler(0, 0, atan2(y-self.parent.robomap.origin[1], x-self.parent.robomap.origin[0]))
        location.header.frame_id = 'map'
        location.header.stamp = rospy.Time.now()
        location.pose.position.x = x
        location.pose.position.y = y
        location.pose.orientation.w = quaternion[3]
        location.pose.orientation.z = quaternion[2]
        self.parent.goal_publisher.publish(location)
        
    def grab_figure(self):
        self.closeManipulator()
        # Closing
        rospy.sleep(0.5)
        self.reverse()
        # Go home
        self.goHomeBase()

    def drop_figure(self):
        self.openManipulator()
        # Opening
        rospy.sleep(0.5)
        self.reverse()

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
    
    def closeManipulator(self):
        self.manipulator_action.publish(Vector3(x=0.0))
        self.parent.update_textbox('Manipulator action','closing')
        #time.sleep(10) # just for testing before manipulator state publisher

    def openManipulator(self):
        self.manipulator_action.publish(Vector3(x=180.0))
        self.parent.update_textbox('Manipulator action','opening')
        #time.sleep(10) # just for testing before manipulator state publisher
        
        
class pirate_detector:
    def __init__(self, parent=None):
        self.pirates = []
        self.parent = parent
        #Give the opencv display window a name
        self.cv_window_name = 'OpenCVImage'
        self.rgb_window_name = 'RGBimage'
        self.depth_window = 'Depth'
        self.UPDATE_PIRATE_DATA = True
        #create the window and make it re-sizeable (second parameter 0)
        cv.NamedWindow(self.cv_window_name)
        #cv.NamedWindow(self.rgb_window_name)
        #create the cv_bridge object
        self.bridge = CvBridge()
        #subscribe to image data
        self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.image_callback)
        #self.rgb_subscribe = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgb_callback)
        self.point_cloud2_subscribe = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl2_callback)
        self.camera_publisher = rospy.Publisher('/ptu_servo_angles', Vector3, latch=False)
        rospy.sleep(1.0)
        self.camera_publisher.publish(Vector3(x=0.0, y=110.0, z=90.0))
        self.img = None
        self.move = False
        self.move_goal = None
        self.pcl_data = None
        
    def move_to_pirate(self):
        self.UPDATE_PIRATE_DATA = True
        self.move = True
        #self.pcl2_callback(self.pcl_data)
        
    def get_move_goal(self):
        if self.move_goal:
            print 'returning move_goal ' + str(self.move_goal) + '!'
            self.parent.goal_publisher.publish(self.move_goal)
        else:
            print 'No move goal, retrying...'
            rospy.sleep(2.0)
        return self.move_goal
        
    def pcl2_callback(self, data):
        print 'PCL2 Callback'
        self.pcl_data = data
        print 'We has piratez'
        xyzArr = self.pointcloud2_to_xyz_array(data, False)
        pirate = self.pirates.pop()
        self.pirates = []
        #Create a P2P message to reach the figure
        camerapoint = PoseStamped()
        x = xyzArr[pirate[1]][pirate[0]][0]+0.135 + self.parent.robomap.origin[0]
        y = xyzArr[pirate[1]][pirate[0]][1] + self.parent.robomap.origin[1]
        z = xyzArr[pirate[1]][pirate[0]][2]-0.31 #Hihavakioita, koska fuck yeah :P
        quaternion = quaternion_from_euler(0, 0, atan2(y-self.parent.robomap.origin[1], x-self.parent.robomap.origin[0]))
        camerapoint.header.frame_id = 'map'
        camerapoint.header.stamp = rospy.Time.now()
        camerapoint.pose.position.x = x
        camerapoint.pose.position.y = y
        camerapoint.pose.orientation.w = quaternion[3]
        camerapoint.pose.orientation.z = quaternion[2]
        self.parent.update_textbox('camera: ',str(camerapoint))
        self.move_goal = camerapoint
        cv.WaitKey(3000)
        
    def image_callback(self, data):
        print 'image callback'
        if self.UPDATE_PIRATE_DATA:
            self.move_goal = None
            try:
                #convert image to opencv format
                cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
            except CvBridgeError, e:
                print e
            #Canny detecting
            cv.EqualizeHist(cv_image, cv_image)
            cv.Smooth(cv_image, cv_image, cv.CV_GAUSSIAN, 11, 11)
            yuv = cv.CreateImage(cv.GetSize(cv_image), 8, 3)
            gray = cv.CreateImage(cv.GetSize(cv_image), 8, 1)
            canny = cv.CreateImage(cv.GetSize(cv_image),8, 1)
            cv.Canny(cv_image,canny,50,250)
            bwimage = cv.CreateImage(cv.GetSize(canny), 8, 1)
            cv.Threshold(canny, bwimage, 128, 255, cv.CV_THRESH_BINARY)
        
            a = np.asarray(canny[:,:])
            cv.FloodFill(canny, (0, 0), 0)
            storage = cv.CreateMemStorage()
            contour_pointer = cv.FindContours(canny, storage, method=cv.CV_CHAIN_APPROX_TC89_KCOS,mode=cv.CV_RETR_EXTERNAL)
            contour_areas = []
        
            while contour_pointer is not None:
                contour = contour_pointer[ : ]
                centre_of_mass = self.find_centre_of_mass(contour)
                rotated_contour = self.rotate_contour(contour, centre_of_mass, 0)
                lst = []
        
                for i in rotated_contour:
                    lst.append(list(i))
                rotated_contour = []
        
                for i in lst:
                    i[0] = int(i[0])
                    i[1] = int(i[1])
                    rotated_contour.append(tuple(i))
                self.draw_contour(cv_image, rotated_contour, cv.CV_RGB( 255 , 255 , 255 ), 5)
                x, y, w, h = cv.BoundingRect(contour)
                if h > w and h > 20 and h < 170:
                    self.pirates.append((self.get_center_coordinates(x, y, h, w)))
                    cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), cv.CV_FILLED)
                contour_pointer = contour_pointer.h_next()
            print self.pirates
            self.img = cv_image
            cv.ShowImage(self.cv_window_name, cv_image)
            cv.WaitKey(3000)

    def rgb_callback(self, data):
        print "RGB Callback"
        try:
            rgb_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print "RGB callback failure" + e
        self.rgb_image = rgb_image

        cv.ShowImage(self.rgb_window_name, rgb_image)
        cv.WaitKey(3000)
        
    #Code samples fetched from http://www.pirobot.org/blog/0016/
    # Draw contour from list of tuples.
    def draw_contour( self, im , contour , color , thickness = 1 , linetype = 8 ,
                      shift = 0 ) :
        if thickness == -1 :
            cv.FillPoly( im , [contour] , color , linetype , shift )
        else :
            cv.PolyLine( im , [contour] , True , color , thickness , linetype , shift )

    # Rotate contour around centre point using numpy.
    def rotate_contour( self, contour , centre_point , theta ) :
        rotation = np.array( [ [ np.cos( theta ) , -np.sin( theta ) ] , [ np.sin( theta ) ,  np.cos( theta ) ] ] )
        centre = np.vstack( [ centre_point ] * len( contour ) )
        contour = np.vstack( contour ) - centre
        contour = np.dot( contour , rotation ) + centre
        return [ tuple ( each_row ) for each_row in contour ]

    # Find centre of mass by drawing contour in closed form and using moments.
    def find_centre_of_mass( self, contour ) :
        bottom_right = np.max( contour , axis = 0 )
        blank = cv.CreateImage( tuple ( bottom_right ) , 8 , 1 )
        cv.Set( blank , 0 )
        self.draw_contour( blank , contour , 1, -1 )
        matBlank = cv.GetMat(blank)
        moments = cv.Moments( matBlank , 1 )  
        sM00 = float ( cv.GetSpatialMoment( moments , 0 , 0 ) )
        sM01 = float ( cv.GetSpatialMoment( moments , 0 , 1 ) )
        sM10 = float ( cv.GetSpatialMoment( moments , 1 , 0 ) )
        if sM00 == 0:
            sM00 = 1
        return ( sM10 / sM00 , sM01 / sM00 )
      
    def get_center_coordinates(self, x, y, w, h):
        center_x = float(x + w/2.0)
        center_y = float(y + h/2.0)
        return (center_x, center_y)

    def pointcloud2_to_array(self, cloud_msg):
        dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        return np.reshape(cloud_arr, (cloud_msg.width, cloud_msg.width)) 
     
    def get_xyz_points(self, cloud_array, remove_nans=True):
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        return points

    def pointcloud2_to_xyz_array(self, cloud_msg, remove_nans=True):
        return self.get_xyz_points(self.pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)


if __name__ == "__main__":
    from python_qt_binding.QtGui import QApplication
    import sys
    rospy.init_node("gui_plannerz")
    app = QApplication(sys.argv)
    q = Widgetti()
    q.setMinimumSize(600, 700)
    q.show()
    app.exec_()
