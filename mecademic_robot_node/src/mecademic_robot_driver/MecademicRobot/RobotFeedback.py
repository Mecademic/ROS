import socket


class RobotFeedback:
    """Class for the Mecademic Robot allowing for live positional 
    feedback of the Mecademic Robot 

    Attributes:
        Address: IP Address
        socket: socket connecting to physical Mecademic Robot
        joints: tuple of the joint angles in degrees
        cartesian: tuple of the cartesian values in mm and degrees
    """

    def __init__(self, address):
        """Constructor for an instance of the Class Mecademic Robot 

        :param address: The IP address associated to the Mecademic Robot
        """
        self.address = address
        self.socket = None
        self.joints = ()  # Joint Angles, angles in degrees | [theta_1, theta_2, ... theta_n]
        self.cartesian = ()  # Cartesian coordinates, distances in mm, angles in degrees | [x,y,z,alpha,beta,gamma]

    def Connect(self):
        """Connects Mecademic Robot object communication to the physical Mecademic Robot
        Returns the status of the connection, true for success, false for failure

        :return status: Return whether the connection is established
        """
        try:
            self.socket = socket.socket()  # Get a socket
            self.socket.settimeout(0.1)  # set the timeout to 100ms
            try:
                self.socket.connect((self.address, 10001))  # connect to the robot's address
            except socket.timeout:  # catch if the robot is not connected to in time
                raise TimeoutError
                # Receive confirmation of connection
            if self.socket is None:  # check that socket is not connected to nothing
                raise RuntimeError
            self.socket.settimeout(1)  # set timeout to 1 seconds
            try:
                self.getData()
                return True
            except socket.timeout:
                raise RuntimeError
        except TimeoutError:
            return False
        # OTHER USER !!!
        except RuntimeError:
            return False

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot
        """
        if (self.socket is not None):
            self.socket.close()
            self.socket = None

    def getData(self, delay=0.1):
        """Receives message from the Mecademic Robot and 
        saves the values in appropriate variables

        :param delay: time to set for timeout of the socket (int)
        """
        if self.socket is None:  # check that the connection is established
            return  # if no connection, nothing to receive
        self.socket.settimeout(delay)  # set read timeout to desired delay
        try:
            response = self.socket.recv(256).decode("ascii")  # read message from robot
            print(response)
            self._getJoints(response)
            self._getCartesian(response)
        except TimeoutError:
            pass

    def _getJoints(self, response):
        """Gets the joint values of the variables from the message sent by the Robot

        :param response: message received from the Robot
        """
        init_resp = response
        start = response.find("[2026]")  # isolate data from message format
        response = response[start:]
        end = response.find("\x00")
        response = response[:end]
        response = response.replace("[2026][", "").replace("]", "")
        joints_str = response.split(',')
        # print(joints_str)
        try:
            self.joints = tuple((float(x) for x in joints_str))  # convert position data to floats
        except ValueError as err:
            raise ValueError(str(err) + init_resp)

    def _getCartesian(self, response):
        """Gets the cartesian values of the variables from the message sent by the Robot

        :param response: message received from the Robot
        """
        init_resp = response
        start = response.find("[2027]")  # isolate data from message format
        response = response[start:]
        end = response.find("\x00")
        response = response[:end]
        response = response.replace("[2027][", "").replace("]", "")
        cartesian_str = response.split(',')
        try:
            self.cartesian = tuple((float(x) for x in cartesian_str))  # convert position data to floats
        except ValueError as err:
            raise ValueError(str(err) + init_resp)
