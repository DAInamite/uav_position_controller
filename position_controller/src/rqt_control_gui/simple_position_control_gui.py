## rqt widgetfor simple copter control
#Created on 12.11.2015
#@author: stephan

import os
import tf
import math
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from position_controller.srv import SetPos, SetAltitude
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion

# Convert from given degree of yaw rotation to geometry_msgs.msg.Quaternion
def quaternion_from_yaw_degree(yaw_degree):
    q = quaternion_from_euler(0, 0, math.radians(yaw_degree))
    return  Quaternion(*q)


class Overview(Plugin):
    def __init__(self, context):
        super(Overview, self).__init__(context)
        self.__tfListener = tf.TransformListener()

        self.__copterFrame = "base_link"
        self.__worldFrame = "world"

        # Give QObjects reasonable names
        self.setObjectName('simple_position_control_gui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-c", "--copterFrame", dest="copterFrame", help="Specify the copter frame")
        parser.add_argument("-w", "--worldFrame", dest="worldFrame", help="Specify the world frame")

        args, unknowns = parser.parse_known_args(context.argv())
        if args.copterFrame:
            self.__copterFrame = args.copterFrame
            rospy.loginfo("using %s as copter frame", self.__copterFrame)
            
        if args.worldFrame:
            self.__worldFrame = args.worldFrame
            rospy.loginfo("using %s as world frame", self.__worldFrame)
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('position_controller'), 'src', 'rqt_control_gui', 'resource', 'overview.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('OverviewUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.forwardButton.clicked.connect(self.goForward)
        self._widget.backwardButton.clicked.connect(self.goBackward)
        self._widget.leftButton.clicked.connect(self.goLeft)
        self._widget.rightButton.clicked.connect(self.goRight)
        self._widget.rotateButton.clicked.connect(self.rotate)
        self._widget.landButton.clicked.connect(self.land)
        self._widget.startButton.clicked.connect(self.start)
        self._widget.absoluteControlButton.clicked.connect(self.absoluteControl)
        self._widget.tfButton.clicked.connect(self.goToTF)
        self.__statusLabel = self._widget.statusLabel
        
        
    def goForward(self):
        try:
            distance = float(self._widget.distanceEdit.text())
            (position, quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            x = math.cos(yaw) * distance
            y = math.sin(yaw) * distance
            self.__statusLabel.setText("going to x: {0} y: {1} yaw: {2}".format(position[0] + x, position[1] + y, yaw))
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(position[0] + x, position[1] + y, yaw)
        except Exception as e:
            self.__statusLabel.setText(str(e))
    
    def goBackward(self):
        try:
            distance = float(self._widget.distanceEdit.text())
            (position, quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            x = math.cos(yaw) * -distance
            y = math.sin(yaw) * -distance
            self.__statusLabel.setText("going to x: {0} y: {1} yaw: {2}".format(position[0] + x, position[1] + y, yaw))
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(position[0] + x, position[1] + y, yaw)
        except Exception as e:
            self.__statusLabel.setText(str(e))
        
    def goLeft(self):
        try:
            distance = float(self._widget.distanceEdit.text())
            (position, quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            x = math.sin(yaw) * distance
            y = math.cos(yaw) * distance
            self.__statusLabel.setText("going to x: {0} y: {1} yaw: {2}".format(position[0] + x, position[1] + y, yaw))
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(position[0] + x, position[1] + y, yaw)
        except Exception as e:
            self.__statusLabel.setText(str(e))
    
    def goRight(self):
        try:
            distance = float(self._widget.distanceEdit.text())
            (position, quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            x = math.sin(yaw) * -distance
            y = math.cos(yaw) * -distance
            self.__statusLabel.setText("going to x: {0} y: {1} yaw: {2}".format(position[0] + x, position[1] + y, yaw))
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(position[0] + x, position[1] + y, yaw)
        except Exception as e:
            self.__statusLabel.setText(str(e))
    
    def rotate(self):
        try:
            angle = math.pi * float(self._widget.rotationEdit.text()) / 180.0
            (position, quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            self.__statusLabel.setText("new yaw: {0}".format(yaw + angle))
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(position[0], position[1], yaw + angle)
        except Exception as e:
            self.__statusLabel.setText(str(e))
            
    def land(self):
        try:
            self.setAltitude(0.0)
        except ValueError:
            pass
        
    def start(self):
        try:
            altitude = float(self._widget.zEdit.text())
            self.setAltitude(altitude)
            self.setAltitude(altitude)
            statusMessage = "starting to altitude {0}".format(altitude)
            self.__statusLabel.setText(statusMessage)
        except ValueError:
            pass
            
    def setAltitude(self, altitude):
        rospy.wait_for_service('/position_controller/set_altitude', 1)
        setAltitudeService = rospy.ServiceProxy('/position_controller/set_altitude', SetAltitude)
        setAltitudeService(altitude)
    
    def absoluteControl(self):
        statusMessage = ""
        try:
            ((x, y, _z), quaternion) = self.__tfListener.lookupTransform(self.__worldFrame, self.__copterFrame, rospy.Time(0))
            (_roll, _pitch, yaw) = euler_from_quaternion(quaternion)
            try:
                yaw = math.pi * float(self._widget.angleEdit.text()) / 180.0
            except ValueError:
                pass                
            try:
                x = float(self._widget.xEdit.text())
            except ValueError:
                pass
            try:
                y = float(self._widget.yEdit.text())
            except ValueError:
                pass
            statusMessage += "going to x: {0} y: {1} yaw: {2} ".format(x, y, yaw)
            rospy.wait_for_service('/position_controller/set_target_pos', 1)
            setPose = rospy.ServiceProxy('/position_controller/set_target_pos', SetPos)
            setPose(x, y, yaw)
            self.__statusLabel.setText(statusMessage)
            try:
                altitude = float(self._widget.zEdit.text())
                statusMessage += "setting altitude to {0}".format(altitude)
                self.setAltitude(altitude)
                self.__statusLabel.setText(statusMessage)
            except ValueError:
                pass
        except Exception as e:
            self.__statusLabel.setText(str(e))
    
    def goToTF(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        rospy.loginfo("saving simple position controller gui setting")
        instance_settings.set_value("worldFrame", self.__worldFrame)
        instance_settings.set_value("copterFrame", self.__copterFrame)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        rospy.loginfo("restoring simple position controller gui setting")
        storedWorldFrame = instance_settings.value("worldFrame")
        if type(storedWorldFrame) == unicode:
            storedWorldFrame = storedWorldFrame.encode('ascii', 'ignore')
        if storedWorldFrame:
            self.__worldFrame = storedWorldFrame
        storedCopterFrame = instance_settings.value("copterFrame")
        if type(storedCopterFrame) == unicode:
            storedCopterFrame = storedCopterFrame.encode('ascii', 'ignore')
        if storedCopterFrame:
            self.__copterFrame = storedCopterFrame

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
