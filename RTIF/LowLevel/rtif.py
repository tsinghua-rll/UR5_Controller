import struct
from collections import namedtuple
import socket

DEFAULT_TIMEOUT = 1.0

SW_PROTOCOL_VERSION = 3.1


class ConnectionState:
    DISCONNECTED = 0
    CONNECTED = 1
    STARTED = 2
    PAUSED = 3


class StateMessageReceiver(object):
    @staticmethod
    def unpack(byte_stream):
        joint_data = namedtuple("vector6d", ("p0", "p1", "p2", "p3", "p4", "p5"))
        coordinate_data = namedtuple("coordinate6d", ("x", "y", "z", "rx", "ry", "rz"))
        message = {"Time Step": 0.0,
                   "Target Joint Positions": joint_data(0., 0., 0., 0., 0., 0.),
                   "Target Joint Velocities": joint_data(0., 0., 0., 0., 0., 0.),
                   "Target Joint Accelerations": joint_data(0., 0., 0., 0., 0., 0.),
                   "Target Joint Currents": joint_data(0., 0., 0., 0., 0., 0.),
                   "Target Joint Torques": joint_data(0., 0., 0., 0., 0., 0.),
                   "Actual Joint Positions": joint_data(0., 0., 0., 0., 0., 0.),
                   "Actual Joint Velocities": joint_data(0., 0., 0., 0., 0., 0.),
                   "Actual Joint Currents": joint_data(0., 0., 0., 0., 0., 0.),
                   "Joint Control Currents": joint_data(0., 0., 0., 0., 0., 0.),
                   "Actual Tool Coordinates": coordinate_data(0., 0., 0., 0., 0., 0.),
                   "Actual Tool Speed": coordinate_data(0., 0., 0., 0., 0., 0.),
                   "Generalized Tool Force": coordinate_data(0., 0., 0., 0., 0., 0.),
                   "Target Tool Coordinates": coordinate_data(0., 0., 0., 0., 0., 0.),
                   "Target Tool Speed": coordinate_data(0., 0., 0., 0., 0., 0.),
                   "Digit Input": 0.0,
                   "Temperature": joint_data(0., 0., 0., 0., 0., 0.),
                   "Execute Time": 0.0,
                   "Robot Mode": 0.0,
                   "Joint Mode": joint_data(0., 0., 0., 0., 0., 0.),
                   "Safety Mode": 0.0,
                   }
        cnt, message["Time Step"] = struct.unpack_from('!Id', byte_stream)
        bias = 12                       # byte count + time step = 12 byte
        while 0 < bias < cnt:
            if bias == 12:
                message["Target Joint Positions"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 0 * 48))
                message["Target Joint Velocities"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 1 * 48))
                message["Target Joint Accelerations"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 2 * 48))
                message["Target Joint Currents"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 3 * 48))
                message["Target Joint Torques"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 4 * 48))
                message["Actual Joint Positions"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 5 * 48))
                message["Actual Joint Velocities"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 6 * 48))
                message["Actual Joint Currents"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 7 * 48))
                message["Joint Control Currents"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 7 * 48))
                bias += 9 * 48
            elif bias == 444:
                message["Actual Tool Coordinates"] = coordinate_data(*struct.unpack_from('!6d', byte_stream, bias + 0 * 48))
                message["Actual Tool Speed"] = coordinate_data(*struct.unpack_from('!6d', byte_stream, bias + 1 * 48))
                message["Generalized Tool Force"] = coordinate_data(*struct.unpack_from('!6d', byte_stream, bias + 2 * 48))
                message["Target Tool Coordinates"] = coordinate_data(*struct.unpack_from('!6d', byte_stream, bias + 3 * 48))
                message["Target Tool Speed"] = coordinate_data(*struct.unpack_from('!6d', byte_stream, bias + 4 * 48))
                bias += 5 * 48
            elif bias == 684:
                message["Digit Input"] = struct.unpack_from('!d', byte_stream, bias + 0)
                message["Temperature"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 8))
                message["Execute Time"] = struct.unpack_from('!d', byte_stream, bias + 56)
                message["Robot Mode"] = struct.unpack_from('!d', byte_stream, bias + 72)
                message["Joint Mode"] = joint_data(*struct.unpack_from('!6d', byte_stream, bias + 80))
                message["Safety Mode"] = struct.unpack_from('!d', byte_stream, bias + 128)
                bias = -1
        return message, (bias < 0)


class RTIF(object):
    def __init__(self, hostname):
        self.hostname = hostname
        self.port = 30003
        self.__conn_state = ConnectionState.DISCONNECTED
        self.__sock = None
        self.__buf = ''
        
    def connect(self):
        if self.__sock:
            return

        self.__buf = ''

        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.setblocking(DEFAULT_TIMEOUT)
            self.__sock.connect((self.hostname, self.port))
            self.__conn_state = ConnectionState.CONNECTED
        except (socket.timeout, socket.error):
            self.__sock = None
            raise

    def disconnect(self):
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.__conn_state = ConnectionState.DISCONNECTED
        
    def is_connected(self):
        return self.__conn_state is not ConnectionState.DISCONNECTED

    def receive(self):
        if not self.is_connected():
            self.connect()
        data = self.__recv()
        self.disconnect()
        return data

    def call_function(self, function):
        self.connect()
        self.__sock.send(function + '\n')
        self.disconnect()

    def __recv(self):
        self.__buf = ''

        while 1:
            self.__buf += self.__sock.recv(4096)
            # unpack_from requires a buffer of at least 3 bytes
            if len(self.__buf) >= 880:
                # Attempts to extract a packet
                data, _ = StateMessageReceiver.unpack(self.__buf)
                return data
