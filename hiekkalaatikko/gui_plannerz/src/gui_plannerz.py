#!/usr/bin/env python
import roslib
roslib.load_manifest('gui_plannerz')
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import python_qt_binding
from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt, QTimer
from python_qt_binding.QtGui import QWidget, QMessageBox, QDoubleSpinBox, QLabel, QPixmap, QBrush, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QPolygonF, QVBoxLayout, QHBoxLayout, QColor, qRgb, QPushButton, QRadioButton
import numpy
from math import sqrt, atan2,atan, pi, degrees
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped



class Widgetti(QWidget):

    def __init__(self):
        super(Widgetti, self).__init__()
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()
        self.tf = tf.TransformListener()

        self.robomap = RoboMap(tf = self.tf, parent=self)

        self.setWindowTitle('Gui plannerz lol')
        self.drive = QPushButton('DRIVE!')
        self.drive.clicked.connect(self.Engage)

        self.gui_publisher = rospy.Publisher('gui_plan', Path)
     
        self.delete_plan = QPushButton('Delete planz')
        self.delete_plan.clicked.connect(self.robomap.deletePlan)
     
        self.gmode = QRadioButton('GMode')
        self.gmode.toggled.connect(self.setMode)
        self.gmode.setChecked(True)
        self.pmode = QRadioButton('Pmode')
        self.mode_layout = QHBoxLayout()
        self.mode_layout.addWidget(self.gmode)
        self.mode_layout.addWidget(self.pmode)
        self.gpublisher = rospy.Publisher('move_base_simple/goal', PoseStamped)
     
        self.button_layout.addWidget(self.drive)
        self.button_layout.addWidget(self.delete_plan)
        self.layout.addWidget(QLabel('Use mouse to draw path. Rclick adds last point'))
        self.layout.addLayout(self.button_layout)
        self.layout.addLayout(self.mode_layout)
        self.layout.addWidget(self.robomap)
        self.setLayout(self.layout)

    def setMode(self, gmode_toggle):
        self.drive.setDisabled(gmode_toggle)
        self.delete_plan.setDisabled(gmode_toggle)
        self.robomap.gmode=gmode_toggle
        self.robomap.deletePlan()
        return

    def Engage(self):
        plan = self.robomap.get_plan()
        if not plan:
            QMessageBox.critical(self, "No plan or map :(", "OK")
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for z in plan:
            stampedPose = PoseStamped()
            pos = z[0]
            x = pos[0]
            y = pos[1]
            quat = z[1]
            stampedPose.header.stamp = rospy.Time.now()
            stampedPose.header.frame_id = "map"
            stampedPose.pose.position.x = x
            stampedPose.pose.position.y = y
            stampedPose.pose.orientation.w = quat[3]
            stampedPose.pose.orientation.z = quat[2]
            path.poses.append(stampedPose)
        self.gui_publisher.publish(path)

class RoboMap(QGraphicsView):

    map_change = Signal()
    position_change = Signal(float, float)
   
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
        self.gmode = None
        self.gpoint = None
        self.gline = None
        self.timer = None
        self.robot_point = None
        self.position_change.connect(self.update_position)
       
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
       
       
    def mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)
   
    def update_position(self, x, y):
        point = QPoint(x, y)
        if self.robot_point:
            self.scene.removeItem(self.robot_point)
        self.robot_point = self.draw_point(p.x(), p.y(), color=Qt.green, rad=self.transform().m11()*2.0)
        self.mirror(self.robot_point)
        self.scene.removeItem(self.robot_point)
        self.scene.addItem(self.robot_point)
       
    def update_position_current(self):
        (t, r) = self.tf.lookupTransform('/map','base_link', rospy.Time(0))
        self.position_changed.emit(t[0], t[1])
   
    def callback(self, msg):
        self.w = msg.info.width
        self.h = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        arr = numpy.array(msg.data, dtype=numpy.uint8, copy=False, order='C')
        arr = arr.reshape((self.h, self.w))
        if self.w % 4:
            tmp = numpy.empty((self.h, 4 - self.w % 4), dtype=arr.dtype, order='C')
            arr = numpy.append(arr, tmp, axis=1)
        img = QImage(arr.reshape((arr.shape[0] * arr.shape[1])), self.w, self.h, QImage.Format_Indexed8)
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
        if self.gpoint:
            self.scene.removeItem(self.gpoint)
            self.gpoint = None
        if self.gline:
            self.scene.removeItem(self.gline)
            self.gline = None
   
    def draw_point(self, x, y, color=Qt.magenta, rad=1.0, add_point=False):
        ell = self.scene.addEllipse(x-rad, y-rad, rad*2.0, rad*2.0, color, QBrush(Qt.SolidPattern))
        ell.setZValue(2000.0)
        if add_point:
            if self.points:
                self.points.append(ell)
            else:
                self.points = [ell]
        return ell
       
    def mousePressEvent(self, e):
        if self.gmode:
            if self.gpoint:
                self.scene.removeItem(self.gpoint)
                self.gpoint = None
            if self.gline:
                self.scene.removeItem(self.gline)
                self.gline = None
            point = self.mapToScene(e.x(), e.y())
            self.gpoint = self.draw_point(point.x(), point.y(), Qt.red, 2.0)
            self.gline = self.scene.addLine(point.x(), point.y(), point.x(), point.y(), Qt.red)
            self.gline.setZValue(1000.0)
        else:
            if e.button() == Qt.RightButton:
                return
            self.setMouseTracking(True)
           
    def mouseMoveEvent(self, e):
        point = self.mapToScene(e.x(), e.y())
        if self.gmode and self.gline:
            tmp = self.gline.line()
            tmp.setP2(QPointF(point.x(), point.y()))
            self.gline.setLine(tmp)
        else:
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
       
    def mouseReleaseEvent(self, e):
        point = self.mapToScene(e.x(), e.y())
        if self.gmode:
            if self.resolution:
                rect = self.gpoint.rect().center()
                x1 = (((self.w/2) - rect.x()) + (self.w/2)) * self.resolution + self.origin[0]
                x2 = (((self.w/2) - point.x()) + (self.w/2)) * self.resolution + self.origin[0]
                y1 = rect.y() * self.resolution + self.origin[1]
                y2 = point.y() * self.resolution + self.origin[1]
                quaternion = quaternion_from_euler(0, 0, atan2(y2-y1, x2-x1))
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"
                goal.pose.position.x = x1
                goal.pose.position.y = y1
                goal.pose.orientation.w = quaternion[3]
                goal.pose.orientation.z = quaternion[2]
                self.parent.gpublisher.publish(goal)
            else:
                QMessageBox.critical(self, "No map, no goal man!", "Y Dig?")
        else:
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
                self.draw_point(point.x(), point.y(), Qt.pink, add_point=True)

if __name__ == "__main__":
    from python_qt_binding.QtGui import QApplication
    import sys
    rospy.init_node("gui_plannerz")
    app = QApplication(sys.argv)
    q = Widgetti()
    q.setMinimumSize(400, 500)
    q.show()
    app.exec_()
