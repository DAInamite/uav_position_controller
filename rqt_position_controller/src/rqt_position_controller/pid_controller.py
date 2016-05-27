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

from PyQt4.QtCore import pyqtSignal

from control_toolbox.srv import *
from control_toolbox.srv._SetPidGains import SetPidGainsRequest

# Custum Widget for PID controller configuration of control_toolbox pid services
class PIDConfiguration(QWidget):
    
    updated = pyqtSignal()

    # visual_name shown name of the pid in the ui
    # service_postfix contro_toolbox pid name aka postfix of the resulting service
    def __init__(self, visual_name, service_postfix):
        super(PIDConfiguration, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('PIDControllerConfiguration')
        
        self._service_prefix = service_postfix
        
        self._service_get_prefix = '/position_controller/position_controller_node/get_gains_'
        self._service_set_prefix = '/position_controller/position_controller_node/set_gains_'
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_position_controller'), 'resource', 'pid.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
         
        self.pidNameLabel.setText(visual_name)
        
        #register update handling
        self.iMaxValueSpinBox.valueChanged.connect(self.onUpdate)
        self.iMinValueSpinBox.valueChanged.connect(self.onUpdate)
        self.pValueSpinBox.valueChanged.connect(self.onUpdate)
        self.iValueSpinBox.valueChanged.connect(self.onUpdate)
        self.dValueSpinBox.valueChanged.connect(self.onUpdate)
        self.checkBoxAntiWindUp.stateChanged.connect(self.onUpdateCheckbox)

    def __del__(self):
        self.__deleted = True
        
    # update current settings from service
    def refresh(self):
        """
        Updates the widget.
        """
        pid_setting = self.__get_current_pid()
        
    def onUpdateCheckbox(self, state):
        self.updated.emit()
        
    def onUpdate(self):
        self.updated.emit()
       
    # send current configuration to the remote pid 
    def send_current(self):
        self.__set_current_pid()
      
    # get pid config via ros service call  
    def __get_current_pid(self):
        service_name = self._service_get_prefix + self._service_prefix
        try:
            rospy.wait_for_service(service_name,timeout=0.1)
        except rospy.ROSException, e:
            self.setEnabled(False)
            print "Service not available: %s" %service_name
            #TODO error MESSAGE
            return
        try:
            get_pids = rospy.ServiceProxy(service_name, GetPidGains)
            pid_setting = get_pids()
            self.setEnabled(True)
            self.iMaxValueSpinBox.setValue(pid_setting.i_clamp_max)
            self.iMinValueSpinBox.setValue(pid_setting.i_clamp_min)
            self.pValueSpinBox.setValue(pid_setting.p)
            self.iValueSpinBox.setValue(pid_setting.i)
            self.dValueSpinBox.setValue(pid_setting.d)
            self.checkBoxAntiWindUp.setChecked(pid_setting.antiwindup == 1)
            
            return pid_setting
        except rospy.ServiceException, e:
            self.setEnabled(False)
            print "Service call failed: %s"%e
            
    # set pid config via ros service call
    def __set_current_pid(self):
        service_name = self._service_set_prefix + self._service_prefix
        try:
            rospy.wait_for_service(service_name,timeout=5.0) # 5 seconds
        except rospy.ROSException, e:
            print "Service not available: %s" %service_name
            #TODO error MESSAGE
            return    
        try:
            set_pids = rospy.ServiceProxy(service_name, SetPidGains)
            
            req = SetPidGainsRequest()
            req.p = self.pValueSpinBox.value()
            req.i = self.iValueSpinBox.value()
            req.d = self.dValueSpinBox.value()
            req.i_clamp_max = self.iMaxValueSpinBox.value()
            req.i_clamp_min = self.iMinValueSpinBox.value()
            req.antiwindup = self.checkBoxAntiWindUp.isChecked()
            
            resp1 = set_pids(req)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e