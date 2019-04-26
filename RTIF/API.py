# -*- coding: utf-8 -*-

import math
import numpy as np
import os.path
from RTIF.LowLevel import rtif


class API(object):
    """
    Low level API interface, only make factory API call in UR5 controller remotely
    """
    def __init__(self, IP_ADDRESS):
        self.IP_ADDRESS = IP_ADDRESS
        # in direct control mode, each now command will terminate the previous command
        # in buffered control mode, the command will keep in command buffer until calling run_buffer
        # each time switching between these two mode will clear the command buffer
        # all 'get' command will be ignored when in buffered mode, and sleep can only be used in buffered mode
        self.__direct_control_mode = True
        self.__command_buffer = []
        self.rtif = rtif.RTIF(IP_ADDRESS)
        self.connect()
        if not self.rtif.is_connected():
            print ("Failed When Setting Up Connection With %s" % IP_ADDRESS)
            exit(1)
        self.disconnect()

    def __del__(self):
        self.disconnect()

    def connect(self):
        """
        connect to robot arm
        :return: None
        """
        if not self.rtif.is_connected():
            self.rtif.connect()

    def disconnect(self):
        """
        disconnect from robot arm
        :return: None
        """
        self.rtif.disconnect()

    @staticmethod
    def from_q_to_rad_axis(q):
        """
        :param q: quaternion in w-i-j-k format
        :return: rad in format x-y-z
        """
        norm_axis = math.sqrt(q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        rad = 2.0 * math.atan2(norm_axis, q[0])
        if norm_axis < 1e-5:
            return np.zeros((3,), dtype=np.float32)
        else:
            return rad * np.asarray(q[1:], dtype=np.float32) / norm_axis

    @staticmethod
    def from_rad_axis_to_q(r):
        """
        :param q: rad in format (x-y-z) * r
        :return: quaternion in w-i-j-k format
        """
        norm_axis = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
        if norm_axis < 1e-5:
            return np.asarray((1.0, 0., 0., 0.), dtype=np.float32)
        else:
            return np.asarray((math.cos(0.5 * norm_axis),
                               math.sin(0.5 * norm_axis) * r[0] / norm_axis,
                               math.sin(0.5 * norm_axis) * r[1] / norm_axis,
                               math.sin(0.5 * norm_axis) * r[2] / norm_axis), dtype=np.float32)

    def MoveEndPointToPosition(self, pos=None, rotation=None, a=1.2, v=0.25, t=None):
        """
        :param pos: position of the end point in x-y-z tuple, if None, use current position
        :param rotation: orientation of the end point in w-i-j-k quaternion tuple, if None, use current orientation
                         or if x-y-z * r is also accepted
        :param t: action time before achieve final point, if not None, override a and v value
        :return: None
        """
        if rotation is None or pos is None:
            current = self.rtif.receive()["Actual Tool Coordinates"]
            if rotation is None:
                rotation = current[3:]
            if pos is None:
                pos = current[:3]

        if len(rotation) == 4:
            rotation = self.from_q_to_rad_axis(rotation)

        cmd = ""
        if t is None:
            cmd = "movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f)" %\
                  (pos[0], pos[1], pos[2], rotation[0], rotation[1], rotation[2], a, v)
        else:
            cmd = "movel(p[%f, %f, %f, %f, %f, %f], t=%f)" %\
                  (pos[0], pos[1], pos[2], rotation[0], rotation[1], rotation[2], t)

        if self.__direct_control_mode:
            self.rtif.call_function(cmd)
        else:
            self.__command_buffer.append(cmd)


    def MoveJointToRad(self, q, a=1.4, v=1.05, t=None):
        """
        :param q: Joint rad value of 6-double tuple
        :param t: action time before achieve final point, if not None, override a and v value
        :return: None
        """
        cmd = ""
        if t is None:
            cmd = "movej([%f, %f, %f, %f, %f, %f], a=%f, v=%f)" %\
                  (q[0], q[1], q[2], q[3], q[4], q[5], a, v)
        else:
            cmd = "movej([%f, %f, %f, %f, %f, %f], t=%f)" %\
                  (q[0], q[1], q[2], q[3], q[4], q[5], t)

        if self.__direct_control_mode:
            self.rtif.call_function(cmd)
        else:
            self.__command_buffer.append(cmd)

    def SpeedJointRad(self, q, a=1.4, t=None):
        """
        :param q: Joint rad velocity of 6-double tuple
        :param t: action time before achieve final point, if not None, override a and v value
        :return: None
        """
        cmd = ""
        if t is None:
            cmd = "speedj([%f, %f, %f, %f, %f, %f], a=%f)" %\
                  (q[0], q[1], q[2], q[3], q[4], q[5], a)
        else:
            cmd = "speedj([%f, %f, %f, %f, %f, %f], a=%f, t=%f)" %\
                  (q[0], q[1], q[2], q[3], q[4], q[5], a, t)

        if self.__direct_control_mode:
            self.rtif.call_function(cmd)
        else:
            self.__command_buffer.append(cmd)

    def GetCurrentJointRad(self):
        """
        :return: q in 6-double tuple
        """
        if not self.__direct_control_mode:
            print ("Warning: in buffered mode, Get operation will be ignored!")
        return np.asarray(self.rtif.receive()["Actual Joint Positions"], dtype=np.float32)

    def GetTargetJointRad(self):
        """
        :return: q in 6-double tuple
        """
        if not self.__direct_control_mode:
            print ("Warning: in buffered mode, Get operation will be ignored!")
        return np.asarray(self.rtif.receive()["Target Joint Positions"], dtype=np.float32)

    def GetCurrentEndPos(self):
        """
        :return: (x,y,z), (w,i,j,k)
        """
        if not self.__direct_control_mode:
            print ("Warning: in buffered mode, Get operation will be ignored!")
        ret = self.rtif.receive()["Actual Tool Coordinates"]
        return np.asarray(ret[:3], dtype=np.float32), self.from_rad_axis_to_q(ret[3:])

    def GetTargetEndPos(self, RAW=False):
        """
        :return: (x,y,z), (w,i,j,k)
        """
        if not self.__direct_control_mode:
            print ("Warning: in buffered mode, Get operation will be ignored!")
        ret = self.rtif.receive()["Target Tool Coordinates"]
        if RAW:
            return ret
        return np.asarray(ret[:3], dtype=np.float32), self.from_rad_axis_to_q(ret[3:])

    def GetCurrentEndForce(self):
        """
        :return: (x, y, z, rx, ry, rz)
        """
        if not self.__direct_control_mode:
            print ("Warning: in buffered mode, Get operation will be ignored!")
        return np.asarray(self.rtif.receive()["Generalized Tool Force"], dtype=np.float32)

    def Sleep(self, sec=1.0):
        """
        let robot sleep for some time (second). can only be used in buffered command mode
        :return: None
        """
        if self.__direct_control_mode:
            print ("Warning: you are not in buffered mode, command sleep ignored")
        else:
            self.__command_buffer.append("sleep(%f)" % sec)

    def Stop(self, immediate=True):
        """
        let robot stop now!
        :immediate: if True, will stop robot in buffered mode immediately, or will push in buffer
        :return: None
        """
        if immediate or self.__direct_control_mode:
            self.rtif.call_function("stopj(5.0)")
        else:
            self.__command_buffer.append("stopj(5.0)")

    def switch_mode(self, direct_mode):
        """
        switch between direct mode and buffered command mode. will clear buffer even if not switch
        :param direct_mode: True for direct mode and False for buffered mode
        :return: None
        """
        self.__command_buffer = []
        if direct_mode:
            self.__direct_control_mode = True
        else:
            self.__direct_control_mode = False

    def is_direct_mode(self):
        """
        :return: boolean
        """
        return self.__direct_control_mode

    def clear_command_buffer(self):
        """
        clear command buffer
        :return:
        """
        if self.__direct_control_mode:
            print ("Warning: you are not in buffered mode")
        self.__command_buffer = []

    def TeachMode(self, duration=180):
        """
        Enter Teaching Mode.
        :duration: duration time in second
        :return: None
        """
        if self.__direct_control_mode:
            self.rtif.call_function("def enter_teach_mode():\n"
                                    "    teach_mode()\n"
                                    "    sleep(%f)\n"
                                    "end\n"
                                    "enter_teach_mode()" % duration)
        else:
            if duration > 0:
                self.__command_buffer.append("teach_mode()\n sleep(%f)" % duration)
            else:
                self.__command_buffer.append("teach_mode()")

    def EndTeachMode(self):
        """
        Exit Teaching Mode
        :return: None
        """
        if self.__direct_control_mode:
            self.rtif.call_function("stopj(5.0)")
            self.rtif.call_function("end_teach_mode()")
        else:
            self.__command_buffer.append("end_teach_mode()")

    def ForceMode(self, taskframe=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                  selection=(0, 0, 0, 0, 0, 0),
                  wrench=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
                  limits=(0.2, 0.2, 0.2, 0.2, 0.2, 0.2), duration=180):
        """
        Force control
        :param taskframe: transformation of original tool coordinate
        :param selection: only 0, 1. 0 means correlated axis is fixed by position, 1 means by force
        :param wrench: for selected axis, means target force, else means additional force
        :param limits: for selected axis, means maxim speed else means maxim position deviation
        :duration: duration time in second
        :return: None
        """
        if self.__direct_control_mode:
            self.rtif.call_function(
                "def enter_force_mode():\n"
                "    force_mode(p[%f, %f, %f, %f, %f, %f], [%d, %d, %d, %d, %d, %d],"
                " [%f, %f, %f, %f, %f, %f], 2, [%f, %f, %f, %f, %f, %f])\n"
                "    sleep(%f)\n"
                "end\n"
                "enter_force_mode()" %
                (taskframe + selection + wrench + limits + (duration,)))
        else:
            if duration > 0:
                self.__command_buffer.append("force_mode(p[%f, %f, %f, %f, %f, %f], [%d, %d, %d, %d, %d, %d],"
                                             " [%f, %f, %f, %f, %f, %f], 2, [%f, %f, %f, %f, %f, %f])\n sleep(%f)" %
                                             (taskframe + selection + wrench + limits + (duration,)))
            else:
                self.__command_buffer.append("force_mode(p[%f, %f, %f, %f, %f, %f], [%d, %d, %d, %d, %d, %d],"
                                             " [%f, %f, %f, %f, %f, %f], 2, [%f, %f, %f, %f, %f, %f])" %
                                             (taskframe + selection + wrench + limits))

    def EndForceMode(self):
        """
        Exit force control mode
        :return: None
        """
        if self.__direct_control_mode:
            self.rtif.call_function("stopj(5.0)")
            self.rtif.call_function("end_force_mode()")
        else:
            self.__command_buffer.append("end_force_mode()")

    def run_buffer(self):
        if self.__direct_control_mode:
            print ("Warning: you are not in buffered command mode!")
        cmd = "def myfunc():\n"
        for c in self.__command_buffer:
            cmd += c + '\n'
        cmd += "end\n"
        cmd += "myfunc()"
        self.rtif.call_function(cmd)
        print ("Total %d commands sent" % len(self.__command_buffer))
        self.__command_buffer = []

    def run_script(self, script_path):
        """
        Read and run a UR script from .script file
        :param script_path: path to .script file
        :return: None
        """
        if script_path == '' or os.path.isfile(script_path):
            print ("Error: path not correct! path=%s" % script_path)
            return
        func_name = os.path.basename(script_path).split('.')[0]
        with open(script_path, 'r') as f:
            func_script = f.read()
            self.rtif.call_function(func_script + ("%s()" % func_name))
