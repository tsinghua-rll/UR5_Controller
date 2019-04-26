import RTIF.LowLevel.quaternion as quat
from RTIF.API import API
import numpy as np
import time


class HAPI(API):
    """
    Higher level API interface, including some feedback, robot state checking and so on
    """
    def __init__(self, IP_ADDRESS):
        super(HAPI, self).__init__(IP_ADDRESS)
        self.__base_t = np.asarray((0, 0, 0), dtype=np.float32)
        self.__base_q = np.asarray((1, 0, 0, 0), dtype=np.float32)

    def isLastMovementEnd(self, select=(1, 1, 1, 1, 1, 1)):
        """
        :param select: 1 if dimension is selected
        :return: boolean
        """
        select = np.asarray(select, dtype=np.float32)
        data = self.rtif.receive()
        tar_rad = np.asarray(data["Target Joint Positions"], dtype=np.float32)
        cur_rad = np.asarray(data["Actual Joint Positions"], dtype=np.float32)
        speed = np.asarray(data["Actual Joint Velocities"])
        return np.max(np.abs((tar_rad - cur_rad) * select)) < 1e-4 and np.max(np.abs(speed * select)) < 1e-1

    def RecordingActions(self):
        """
        start recording mode, input ' ' to capture and 'e' to exit
        :return: list of control points
        """
        def __cap_one_point():
            data = self.rtif.receive()
            cap = {"Tool Position": data["Actual Tool Coordinates"][:],
                   "Tool Force": data["Generalized Tool Force"][:],
                   "Joint Position": data["Actual Joint Positions"][:],
                   "Time Step": data["Time Step"]
                   }
            return cap
        caps = []

        self.TeachMode(18000)
        while True:
            i = raw_input("Input 'Enter' to capture, 'e+Enter' to exit, m+Enter to enter edit mode >>")
            if i == 'e':
                break
            elif i == '':
                caps.append(__cap_one_point())
                print ("captured %d points" % len(caps))
            elif i == 'm':
                current_id = 1
                while True:
                    for ind, c in enumerate(caps):
                        if ind == current_id:
                            print ("->%02d : End point (%04f, %04f, %04f, %04f, %04f, %04f)" % ((ind,) + c["Joint Position"]))
                        else:
                            print ("  %02d : End point (%04f, %04f, %04f, %04f, %04f, %04f)" % ((ind,) + c["Joint Position"]))
                    print ("Edit mode: 'e+Enter' to exit, 'u+Enter' to move cursor up, 'd+Enter' down,")
                    print (" 'm+Enter' to modify current, 't+Enter to delete, 'i' to insert before current, 'r+Enter to replay current'")
                    j = raw_input()
                    if j == 'e':
                        break
                    if j == 'u':
                        if current_id == len(caps)-1:
                            current_id = 0
                        else:
                            current_id += 1
                    elif j == 'd':
                        if current_id == 0:
                            current_id = len(caps)-1
                        else:
                            current_id -= 1
                    elif j == 'r':
                        self.MoveJointToRad(caps[current_id]["Joint Position"])
                        while self.is_direct_mode() and not self.isLastMovementEnd():
                            time.sleep(0.5)
                        self.TeachMode(18000)
                    elif j == 't':
                        caps.pop(current_id)
                        if current_id >= len(caps):
                            caps -= 1
                        if caps < 0:
                            break
                    elif j == 'm':
                        cap = __cap_one_point()
                        cap["Time Step"] = caps[current_id]["Time Step"]
                        caps[current_id] = cap
                    elif j == 'i':
                        cap = __cap_one_point()
                        if current_id == 0:
                            cap["Time Step"] = caps[current_id]["Time Step"] - 3.0
                        else:
                            cap["Time Step"] = (caps[current_id]["Time Step"] + caps[current_id - 1]["Time Step"]) / 2.0
                        caps.insert(current_id, cap)
                    else:
                        print ("Unknown input command %s", j)
                        break

            else:
                print ("Unknown input command %s", i)
        self.EndTeachMode()
        return caps

    def ReplayCapturedData(self, caps, with_time=False):
        """
        will replay captured data in fixed speed
        :param caps: list returned by 'RecordingActions'
        :return: None
        """
        if len(caps) <= 0:
            return
        if with_time:
            self.MoveJointToRad(caps[0]["Joint Position"], a=1.2, v=0.25)
            while self.is_direct_mode() and not self.isLastMovementEnd():
                time.sleep(0.5)
            for i in range(1, len(caps)):
                self.MoveJointToRad(caps[i]["Joint Position"], t=(caps[i]["Time Step"] - caps[i-1]["Time Step"]))
                while self.is_direct_mode() and not self.isLastMovementEnd():
                    time.sleep(0.5)
        else:
            for cap in caps:
                self.MoveJointToRad(cap["Joint Position"], a=1.2, v=0.25)
                while self.is_direct_mode() and not self.isLastMovementEnd():
                    time.sleep(0.5)

    def FineTuningPosition(self, interactive=False, add_value=((0.01, 0.01, 0.01), (1.0, 0.0, 0.0, 0.0)), factor=10.0):
        """
        :param interactive: if True, call curses to capture keyboard input, else add add_value to current position
        :param add_value: step size, for interactive mode, (dx, dy, dz, drx, dry, drz), else ((dx, dy, dz), (qw, qi, qj, qk)
        :param factor: if in interactive mode, means key_hold boosting factor, the max speed will achieve after holding
                        a key for some time
        :return: current position after fine tuning in format (x,y,z),(w,i,j,k)
        """
        if interactive:
            if len(add_value) == 6:
                import curses
                stdscr = curses.initscr()
                curses.noecho()
                curses.cbreak()
                stdscr.keypad(1)
                stdscr.nodelay(1)
                stdscr.addstr("Start Interactive Fine Tune Mode\n\r")
                stdscr.addstr("Press 'a','d' for moving along x axis, 'A', 'D' for rotate\n\r")
                stdscr.addstr("Press 's','w' for moving along y axis, 'S', 'W' for rotate\n\r")
                stdscr.addstr("Press 'q','e' for moving along z axis, 'Q', 'E' for rotate\n\r")
                stdscr.addstr("Press 'esc' to exit >>>\n\r")
                step = list(add_value)
                while 1:
                    char = stdscr.getch()
                    curses.flushinp()
                    p, q = self.GetCurrentEndPos()
                    stdscr.clrtoeol()
                    rot = self.from_q_to_rad_axis(q)
                    stdscr.addstr("\r(x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f)" % (p[0], p[1], p[2], rot[0], rot[1], rot[2]))
                    if char == curses.KEY_LEFT or char == ord('a'):     # -X
                        p[0] -= step[0]
                        if step[0] < add_value[0] * factor:
                            step[0] += add_value[0] * factor / 10.0
                    elif char == curses.KEY_RIGHT or char == ord('d'):  # +X
                        p[0] += step[0]
                        if step[0] < add_value[0] * factor:
                            step[0] += add_value[0] * factor / 10.0
                    elif char == curses.KEY_UP or char == ord('w'):     # +Y
                        p[1] += step[1]
                        if step[1] < add_value[1] * factor:
                            step[1] += add_value[1] * factor / 10.0
                    elif char == curses.KEY_DOWN or char == ord('s'):   # -Y
                        p[1] -= step[1]
                        if step[1] < add_value[1] * factor:
                            step[1] += add_value[1] * factor / 10.0
                    elif char == curses.KEY_PPAGE or char == ord('e'):  # +Z
                        p[2] += step[2]
                        if step[2] < add_value[2] * factor:
                            step[2] += add_value[2] * factor / 10.0
                    elif char == curses.KEY_NPAGE or char == ord('q'):  # -Z
                        p[2] -= step[2]
                        if step[2] < add_value[2] * factor:
                            step[2] += add_value[2] * factor / 10.0
                    elif char == ord('A'):                              # -rX
                        q = quat.qmul(q, self.from_rad_axis_to_q((-step[3], 0, 0)))
                        if step[3] < add_value[3] * factor:
                            step[3] += add_value[3] * factor / 10.0
                    elif char == ord('D'):                              # +rX
                        q = quat.qmul(q, self.from_rad_axis_to_q((step[3], 0, 0)))
                        if step[3] < add_value[3] * factor:
                            step[3] += add_value[3] * factor / 10.0
                    elif char == ord('W'):                              # +rY
                        q = quat.qmul(q, self.from_rad_axis_to_q((0, step[4], 0)))
                        if step[4] < add_value[4] * factor:
                            step[4] += add_value[4] * factor / 10.0
                    elif char == ord('S'):                              # -rY
                        q = quat.qmul(q, self.from_rad_axis_to_q((0, -step[4], 0)))
                        if step[4] < add_value[4] * factor:
                            step[4] += add_value[4] * factor / 10.0
                    elif char == ord('E'):                              # +rZ
                        q = quat.qmul(q, self.from_rad_axis_to_q((0, 0, step[5])))
                        if step[5] < add_value[5] * factor:
                            step[5] += add_value[5] * factor / 10.0
                    elif char == ord('Q'):                              # -rZ
                        q = quat.qmul(q, self.from_rad_axis_to_q((0, 0, -step[5])))
                        if step[5] < add_value[5] * factor:
                            step[5] += add_value[5] * factor / 10.0
                    elif char == 27:
                        break
                    else:
                        step = list(add_value)
                        continue
                    self.MoveEndPointToPosition(pos=p, rotation=q, v=0.1, a=0.25)
                    time.sleep(0.1)
                curses.endwin()
                print ("Exit Interactive Fine Tune Mode")
            else:
                print ("Error add_value format, need (dx, dy, dz, drx, dry, drz) but get", add_value)
        else:
            if len(add_value) == 2 and len(add_value[0]) == 3 and len(add_value[1]) == 4:
                p, q = self.GetCurrentEndPos()
                p_next = p + np.asarray(add_value[0], dtype=np.float32)
                q_next = quat.qmul(add_value[1], q)
                self.MoveEndPointToPosition(pos=p_next, rotation=q_next)
                while not self.isLastMovementEnd():
                    time.sleep(0.5)
            else:
                print ("Error add_value format, need ((dx, dy, dz), (qw, qi, qj, qk)) but get", add_value)
        return self.GetCurrentEndPos()

    def set_coordinate_origin(self, ori=None):
        """
        Setting the Coordinate origin point. If ori is None, will automatically use teach mode.
        operating coordinate is: +X from robot center to end tool, +Z toward sky.
        tool coordinate is: the connector of tool towards +X and tool face -Z
        :param ori: 3d tuple (x, y, z) or ((x, y, z), (w ,i, j, k)), or None for teach mode
                    conversion will be automatically done
        :return: basic transform (x,y,z), (w,i,j,k)
        """
        if ori is None:
            self.switch_mode(True)
            self.TeachMode()
            print ("Please move robot arm to origin point")
            print ("Notice: the external sensor connector points to +X, and tool towards -Z")
            print ("And press Enter key >>>")
            if raw_input() == '':
                ori = self.GetCurrentEndPos()
                print ("New origin point is (%f, %f, %f)" % ori)
            else:
                print ("Cancel without changing coordinate")
                return

        if len(ori) != 2:
            if len(ori) != 3:
                print ("Error value! ori should be (x, y, z)")
                return
            # convert coordinate
            unix = ori / np.linalg.norm(ori[:2])
            unix[2] = 0
            uniz = np.asarray((0, 0, 1), dtype=np.float32)
            unio = ori

            q, t = quat.from_vector_to_q((1, 0, 0), unio + unix, (0, 0, 1), unio + uniz, (0, 0, 0), unio)
            ori = (t, q)

        elif len(ori[0]) != 3 or len(ori[1]) != 4:
            print ("Error value! ori should be (x, y, z), (w, i, j, k)")
            return

        self.__base_t = np.asarray(ori[0], dtype=np.float32)
        self.__base_q = np.asarray(ori[1], dtype=np.float32)

    def get_coordinate_origin(self):
        return self.__base_t, self.__base_q

    def GetCurrentEndForce(self):
        f = super(HAPI, self).GetCurrentEndForce()
        f, t = f[:3], f[3:]
        f = quat.qrotote_v(quat.qconj(self.__base_q), f)
        t = quat.qrotote_v(quat.qconj(self.__base_q), t)
        return np.asarray((f[0], f[1], f[2], t[0], t[1], t[2]), dtype=np.float32)

    def GetCurrentEndPos(self):
        p, q = super(HAPI, self).GetCurrentEndPos()
        p = quat.qrotote_v(quat.qconj(self.__base_q), p - self.__base_t)
        q = quat.qmul(quat.qconj(self.__base_q), q)
        return p, q

    def GetTargetEndPos(self, RAW=False):
        p, q = super(HAPI, self).GetTargetEndPos()
        p = quat.qrotote_v(quat.qconj(self.__base_q), p - self.__base_t)
        q = quat.qmul(quat.qconj(self.__base_q), q)
        if RAW:
            q = self.from_q_to_rad_axis(q)
            return np.asarray((p[0], p[1], p[2], q[0], q[1], q[2]), dtype=np.float32)
        else:
            return p, q

    def MoveEndPointToPosition(self, pos=None, rotation=None, a=1.2, v=0.25, t=None):
        if pos is not None:
            pos = quat.qrotote_v(self.__base_q, pos, self.__base_t)
        if rotation is not None:
            if len(rotation) == 4:
                rotation = quat.qmul(self.__base_q, rotation)
            else:
                rotation = quat.qmul(self.__base_q, self.from_rad_axis_to_q(rotation))
        super(HAPI, self).MoveEndPointToPosition(pos=pos, rotation=rotation, a=a, v=v, t=t)

    def ForceMode(self, taskframe=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                  selection=(0, 0, 0, 0, 0, 0),
                  wrench=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
                  limits=(0.2, 0.2, 0.2, 0.2, 0.2, 0.2), duration=180):
        p, q = taskframe[:3], self.from_rad_axis_to_q(taskframe[3:])
        p = quat.qrotote_v(self.__base_q, p)
        q = self.from_q_to_rad_axis(quat.qmul(self.__base_q, q))
        super(HAPI, self).ForceMode(taskframe=(p[0], p[1], p[2], q[0], q[1], q[2]),
                                    selection=selection, wrench=wrench, limits=limits, duration=duration)
