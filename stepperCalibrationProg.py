import os
import sys
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5 import uic
import RPi.GPIO as GPIO
import time


class stepperCalibration(QtWidgets.QMainWindow):
    def __init__(self, whoami = 'stepperCalibration', version = 'v1.0', switch0_GPIO = 17, switch1_GPIO = 18):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = uic.loadUi('steppercalibration.ui',self) 
        self.resize(1700, 1000)
        self.me = whoami

        self.settings = QtCore.QSettings(self.me, version)
        
        self.Widgets = {}
        self.Widgets = {'state0':self.LE_switch0state, 'state1':self.LE_switch1state}

        GPIO.setmode(GPIO.BCM)
        self.switch0pin = switch0_GPIO
        self.switch1pin = switch1_GPIO
        GPIO.setup(self.switch0pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.switch1pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


        self._sample_interval = 100
        self._timer = QtCore.QTimer()
        self._timer.setInterval(self._sample_interval) #msec
        self._timer.timeout.connect(self.runtime_function)
        self._timer.start()


        quit = QtWidgets.QAction("Quit", self)
        quit.triggered.connect(self.close)


    # ---- MAIN RUNTIME ----
    def runtime_function(self):
        self.check_switches()


    def check_switches(self):
        switch0_state = GPIO.input(self.switch0pin)
        switch1_state = GPIO.input(self.switch1pin)

        if switch0_state == GPIO.HIGH:
            self.Widgets['state0'].setText("1")
        else:
            self.Widgets['state0'].setText("0")

        if switch1_state == GPIO.HIGH:
            self.Widgets['state1'].setText("1")
        else:
            self.Widgets['state1'].setText("0")
  
    def closeEvent(self, event):
        self._stop_stage_continuous()
        self._storeSettings()
        reply = QtWidgets.QMessageBox.question(self, 'Quit?',
                                     'Are you sure you want to quit?',
                                     QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)

        if reply == QtWidgets.QMessageBox.Yes:
            if not type(event) == bool:
                event.accept()
            else:
                sys.exit()
        else:
            if not type(event) == bool:
                event.ignore()
