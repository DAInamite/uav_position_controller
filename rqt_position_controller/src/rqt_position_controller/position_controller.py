#
# Author: Christopher-Eyk Hrabia
# christopher-eyk.hrabia@dai-labor.de
#

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QVBoxLayout

from pid_controller import PIDConfiguration

from collections import deque

# RQT plugin for position_controller configuration
class Configuration(Plugin):
    
    isAutoSend = False

    def __init__(self, context):
        super(Configuration, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PositionControllerConfiguration')
        
        self.__context = context

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_position_controller'), 'resource', 'configuration.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ControllerUi')
        
        self.__pids = deque()
        
        self.__pids.append(PIDConfiguration('Altitude Pos','altitude_pos'))
        self.__pids.append(PIDConfiguration('Altitude','altitude'))
        self.__pids.append(PIDConfiguration('Pitch Pos','pitch_pos'))
        self.__pids.append(PIDConfiguration('Pitch','pitch'))
        self.__pids.append(PIDConfiguration('Roll Pos','roll_pos'))
        self.__pids.append(PIDConfiguration('Roll','roll'))        
        self.__pids.append(PIDConfiguration('Yaw Angle','yaw_angle'))
        self.__pids.append(PIDConfiguration('Yaw','yaw'))
           
        self.__pidLayout = QVBoxLayout()
           
        for pid in self.__pids:
            self.__pidLayout.addWidget(pid)
            pid.updated.connect(self.onPidUpdate)
            
        self._scrollWidget = QWidget()
        self._scrollWidget.setLayout(self.__pidLayout)        
        self._widget.pidScrollArea.setWidget(self._scrollWidget)
                    
        self._widget.pushButtonRefresh.clicked.connect(self.refresh)
        
        self._widget.pushButtonSend.clicked.connect(self.send)
        
        self._widget.checkBoxAutoSend.stateChanged.connect(self.changedAutoSend)
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self.refresh()
        
    # update all pids
    def refresh(self):
        for pid in self.__pids:
            pid.refresh()
        
    # send all pid configs to remote
    def send(self):
        for pid in self.__pids:
            pid.send_current()
            
    def onPidUpdate(self):
        if self.isAutoSend:
            self.sender().send_current()
            
    def changedAutoSend(self, state):
        self.isAutoSend =  state > 0

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
