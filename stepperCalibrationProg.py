import os
import sys
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5 import uic
import RPi.GPIO as GPIO
import time
from RpiMotorLib import RpiMotorLib


class stepperCalibration(QtWidgets.QMainWindow):
    def __init__(self, whoami = 'stepperCalibration', version = 'v1.0', switch0_GPIO = 17, switch1_GPIO = 18, stepDir_GPIO = 22, stepSize_GPIO = 23, stepEnable_GPIO = 24):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = uic.loadUi('steppercalibration.ui',self) 
        self.resize(800, 600)
        self.me = whoami

        self.settings = QtCore.QSettings(self.me, version)
        
        self.Widgets = {}
        self.Widgets = {'state0':self.LE_switch0state, 'state1':self.LE_switch1state, 'movesteppos':self.PB_movepos, 'movestepneg':self.PB_moveneg, 'stepsize':self.SB_stepsize, 'position':self.LE_position, 'calibrate':self.PB_calibrate}

        self.Widgets['movesteppos'].clicked.connect(lambda : self._manualMoveStepper(True))
        self.Widgets['movestepneg'].clicked.connect(lambda : self._manualMoveStepper(False))
        self.Widgets['calibrate'].clicked.connect(lambda : self._calibrateStepper())

        GPIO.setmode(GPIO.BCM)
        self.switch0pin = switch0_GPIO
        self.switch1pin = switch1_GPIO
        GPIO.setup(self.switch0pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.switch1pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.stepDirpin = stepDir_GPIO
        self.stepSizepin = stepSize_GPIO
        self.stepEnablepin = stepEnable_GPIO
        self.mymotor = RpiMotorLib.A4988Nema(self.stepDirpin, self.stepSizepin, (21,21,21), "DRV8825")
        GPIO.setup(self.stepEnablepin, GPIO.OUT) # set enable pin as output
        GPIO.output(self.stepEnablepin, GPIO.LOW)
        self.steps_in_rotation = 200 # steps per revolution


        self.stepperPosition = 0.0
        self.steptime = 0.005 # seconds per step
        self.calibrated = False

        self._sample_interval = 100
        self._timer = QtCore.QTimer()
        self._timer.setInterval(self._sample_interval) #msec
        self._timer.timeout.connect(self.runtime_function)
        self._timer.start()


        quit = QtWidgets.QAction("Quit", self)
        quit.triggered.connect(self.close)


    # ---- MAIN RUNTIME ----
    def runtime_function(self):
        self._check_switches()
        self._update_stepper_position()

    def _check_switches(self):
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

    def _update_stepper_position(self):
        if self.calibrated:
            self.Widgets['position'].setText("{:.1f}°".format(self.stepperPosition))
        else:
            self.Widgets['position'].setText("uncalibrated")

    def _update_stepper_position(self):
        if self.calibrated:
            self.Widgets['movepos'].setText("Move to: {:.2f}°".format(self.stepperPosition))
        else:
            self.Widgets['movepos'].setText("Move to: N/A")

    def _manualMoveStepper(self, direction):
        stepSize_deg = self.widgets['stepsize'].getValue()
        numSteps = (stepSize_deg / 360) * self.steps_in_rotation
        self._moveStepper(numSteps, direction)

    def _calibrateStepper(self):
        self.calibrated = False
        # Move to the first switch and wait for it to be pressed
        numSteps = self.steps_in_rotation
        while GPIO.input(self.switch0pin) == GPIO.LOW and GPIO.input(self.switch1pin) == GPIO.LOW:
            self._moveStepper(self, numSteps, False)
            time.sleep(0.05 + self.steptime*numSteps)
            self._check_switches()
        if GPIO.input(self.switch0pin) == GPIO.HIGH:
            self._binarySwitchSearch(self.switch0pin, False)
        else:
            self._binarySwitchSearch(self.switch1pin, False)

    def _moveStepper(self, numSteps, direction):
        self.mymotor.motor_go(direction, "Full", numSteps, self.steptime, False, 0.05)
        time.sleep(0.05 + self.steptime*numSteps)
        if self.calibrated:
            if direction:
                self.stepperPosition += numSteps * (360 / self.steps_in_rotation)
            else:
                self.stepperPosition -= numSteps * (360 / self.steps_in_rotation)

    def _binarySwitchSearch(self, switchPin, direction):
        rotation = 1 / 2  # Start with half a rotation
        while rotation >= (1 / 8):  # Continue until step size is 1/8 of a rotation
            numSteps = (rotation * self.steps_in_rotation)
            if GPIO.input(switchPin) == GPIO.HIGH:
                # Switch is still pressed, move back
                self.mymotor.motor_go(not direction, "Full", numSteps, self.steptime, False, 0.05)
                time.sleep(0.05 + self.steptime*numSteps)
            else:
                # Switch is not pressed, move forward
                self.mymotor.motor_go(direction, "Full", numSteps, self.steptime, False, 0.05)
                time.sleep(0.05 + self.steptime*numSteps)
            self._check_switches()
            # Reduce step size for finer adjustment
            rotation /= 2
        # Mark the current position as zero
        self.stepperPosition = 0.0
        self.calibrated = True
        self.mymotor.motor_go(not direction, "Full", self.steps_in_rotation, self.steptime, False, 0.05)
        time.sleep(0.05 + self.steptime*numSteps)
        self._check_switches()

    def closeEvent(self, event):
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

def runApp():
    app = QtWidgets.QApplication(sys.argv)

    #lakeshoreControl = lsc.lakeshoreWindow()
    steppercalib = stepperCalibration(version = 'v1.2')
    steppercalib.show()
    # add other applications here, then drop into the next class as an arg

    #manWindow = managerWindow(pulseSearch)
    #manWindow.show()

    sys.exit(app.exec_())

runApp()