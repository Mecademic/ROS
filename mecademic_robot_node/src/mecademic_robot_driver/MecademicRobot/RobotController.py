import socket
import time


class RobotController:
    """Class for the Mecademic Robot allowing for communication and control of the
    Mecademic Robot with all of its features available

    Attributes:
        Address: IP Address
        socket: socket connecting to physical Mecademic Robot
        End of Block: Setting for EOB reply
        End of Movement: Setting for EOM reply
        Error: Error Status of the Mecademic Robot
    """

    def __init__(self, address):
        """Constructor for an instance of the Class Mecademic Robot

        :param address: The IP address associated to the Mecademic Robot
        """
        self.address = address
        self.socket = None
        self.EOB = 1
        self.EOM = 1
        self.error = False

    def isInError(self):
        """Status method that checks whether the Mecademic Robot is in error mode.
        Returns 1 for error and 0 otherwise.

        :return error: Returns the error flag
        """
        return self.error  # return the global variable error, which is updated by the other methods

    def ResetError(self):
        """Resets the error in the Mecademic Robot
        """
        self.error = False
        cmd = "ResetError"
        response = self.exchangeMsg(cmd)
        reset_success = self._response_contains(response, ["The error was reset", "There was no error to reset"])
        if reset_success:
            self.error = False
        else:
            self.error = True
        return response

    def Connect(self):
        """Connects Mecademic Robot object communication to the physical Mecademic Robot
        Returns the status of the connection, true for success, false for failure

        :return status: Return whether the connection is established
        """
        try:
            self.socket = socket.socket()  # Get a socket
            self.socket.settimeout(0.1)  # set the timeout to 100ms
            try:
                self.socket.connect((self.address, 10000))  # connect to the robot's address
            except socket.timeout:  # catch if the robot is not connected to in time
                raise TimeoutError
                # Receive confirmation of connection
            if self.socket is None:  # check that socket is not connected to nothing
                raise RuntimeError
            self.socket.settimeout(10)  # set timeout to 10 seconds
            try:
                response = self.socket.recv(1024).decode("ascii")  # receive message from robot
            except socket.timeout:
                raise RuntimeError
            response_found = self._response_contains(response,
                                                     ["[3000]"])  # search for key [3000] in the received packet
            if not response_found:
                raise RuntimeError
            else:
                return response_found  # return if key was found in packet
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

    @staticmethod
    def _response_contains(response, code_list):
        """Scans received response for code IDs

        :param response: Message to scan for codes
        :param code_list: List of codes to look for in the response
        :return response_found: Returns whether the response contains a code ID of interest
        """
        response_found = False
        for code in code_list:
            if response.find(code) != -1:
                response_found = True
                break
        return response_found

    def _send(self, cmd):
        """Sends a command to the physical Mecademic Robot

        :param cmd: Command to be sent  (string)
        :return status: Returns whether the message is sent (boolean)
        """
        if self.socket is None or self.error:  # check that the connection is established or the robot is in error
            return False  # if issues detected, no point in trying to send a cmd that won't reach the robot
        cmd = cmd + '\0'
        status = 0
        while status == 0:
            try:  # while the message hasn't been sent
                status = self.socket.send(cmd.encode("ascii"))  # send command in ascii encoding
            except:
                break
            if status != 0:
                return True  # return true when the message has been sent
        # Message failed to be sent, return false
        return False

    def _receive(self, answer_list, delay):
        """Receives message from the Mecademic Robot and
        looks for expected answer in the reply

        :param answer_list: codes to look for in the response (list)
        :param delay: time to set for timeout of the socket (int)
        :return response: Response received from Mecademic Robot (string)
        """
        if self.socket is None:  # check that the connection is established
            return  # if no connection, nothing to receive
        response_list = []
        response_found = False
        for x in answer_list:  # convert codes to search for in answer into comparable format
            response_list.append("[" + str(x) + "]")
        error_found = False
        # Make error codes in a comparable format
        error_list = ["[" + str(i) + "]" for i in range(1000, 1039)] + ["[" + str(i) + "]" for i in
                                                                        [3001, 3003, 3005, 3009, 3014,
                                                                         3026]]
        self.socket.settimeout(delay)  # set read timeout to desired delay
        while not response_found and not error_found:  # while no answers have been received, keep looking
            try:
                response = self.socket.recv(1024).decode("ascii")  # read message from robot
            except socket.timeout:
                # if timeout reached, either connection lost or nothing was sent from robot (damn disabled EOB and EOM)
                return
            if len(response_list) != 0:  # if the message has a code to look for, find them
                response_found = self._response_contains(response, response_list)
            error_found = self._response_contains(response, error_list)  # search message for errors
        if error_found:  # if errors have been found, flag the script
            self.error = True
        return response  # return the retrieved message

    def exchangeMsg(self, cmd, delay=20, decode=False):
        """Sends and receives with the Mecademic Robot

        :param cmd: Command to send to the Mecademic Robot  (string)
        :param delay: timeout to set for the socket (int)
        :return response: Response with desired code ID (string)
        """
        response_list = self._getAnswerList(cmd)
        if self.error:  # if there is error, no need to send the message
            return
        status = self._send(cmd)  # send the command to the robot
        if status is not True:
            # if message didn't send correctly, reboot communication
            self.Disconnect()
            time.sleep(1)
            self.Connect()
            return
        answer = self._receive(response_list, delay)  # get response from robot
        if len(response_list) == 0:  # if we aren't expecting anything, don't bother looking
            return answer.replace("\x00", "")
        elif answer is None:
            return
        else:  # if message was retrieved
            answer = answer.replace("\x00", "")
            for response in response_list:  # search for response codes
                if self._response_contains(answer, [str(response)]):
                    if decode:
                        # decrypt response based on right response code
                        return self._decodeMsg(answer, response)
                    else:
                        return answer
            # Make error codes in a comparable format
            error_list = [str(i) for i in range(1000, 1039)] + [str(i) for i in [3001, 3003, 3005, 3009, 3014, 3026]]
            for response in error_list:
                if self._response_contains(answer, [str(response)]):
                    if decode:
                        # decrypt response based on right response code
                        return self._decodeMsg(answer, response)
                    else:
                        return answer
            return answer

    def _buildCommand(self, cmd, arg_list=[]):
        """Builds the command string to send to the Mecademic Robot
        from the function name and arguments the command needs

        :param cmd: command name to send to the Mecademic Robot
        :param arg_list: list of arguments the command requires
        :return command: final command for the Mecademic Robot
        """
        command = cmd
        if len(arg_list) != 0:
            command = command + '('
            for index in range(0, (len(arg_list) - 1)):
                command = command + str(arg_list[index]) + ','
            command = command + str(arg_list[-1]) + ')'
        return command

    def _decodeMsg(self, response, response_key):
        """Decrypt information from the Mecademic Robot response to useful information
        that can be manipulated

        :param response: Response from the Mecademic Robot
        :param response_key: Code ID of response to decrypt
        :return code: Decrypted information
        """
        # remove delimiters and \x00 bytes
        code = response.replace("[" + str(response_key) + "][", "").replace("]", "").replace("\x00", "")
        code_list = code.split(",")  # split packets into their individual selves
        if response_key == 2026 or response_key == 2027:
            # if expected packet is from GetJoints (2026) or GetPose (2027), rest of packet is position data
            code_list_float = tuple((float(x) for x in code_list))  # convert position data to floats
            return code_list_float
        elif response_key == 2029 or response_key == 2007 or response_key == 2079:
            # if expected packet is from GetConf (2029), GetStatusRobot (2007) or GetStatusGripper (2079), rest of packet is data
            code_list_int = tuple((int(x) for x in code_list))  # convert status data into integers
            return code_list_int
        else:
            return code  # nothing to decrypt or decryption not specified

    def _getAnswerList(self, command):
        """Retrieve the expected answer codes that the Mecademic Robot should send as feedback after
        a command.

        :param command: command that is to be sent to the Mecademic Robot (string)
        :return answer_list: list of answer codes to search for in response (list)
        """
        if (command.find('ActivateRobot') != -1):
            return [2000, 2001]
        elif (command.find('ActivateSim') != -1):
            return [2045]
        elif (command.find('ClearMotion') != -1):
            return [2044]
        elif (command.find("DeactivateRobot") != -1):
            return [2004]
        elif (command.find("BrakesOn") != -1):
            return [2010]
        elif (command.find("BrakesOff") != -1):
            return [2008]
        elif (command.find("GetConf") != -1):
            return [2029]
        elif (command.find('GetJoints') != -1):
            return [2026]
        elif (command.find('GetStatusRobot') != -1):
            return [2007]
        elif (command.find('GetStatusGripper') != -1):
            return [2079]
        elif (command.find('GetPose') != -1):
            return [2027]
        elif (command.find('Home') != -1):
            return [2002, 2003]
        elif (command.find('PauseMotion') != -1):
            answer_list = [2042]
            if (self.EOM == 1):
                answer_list.append(3004)
            return answer_list
        elif (command.find('ResetError') != -1):
            return [2005, 2006]
        elif (command.find('ResumeMotion') != -1):
            return [2043]
        elif (command.find('SetEOB') != -1):
            return [2054, 2055]
        elif (command.find('SetEOM') != -1):
            return [2052, 2053]
        else:
            answer_list = []
            if (self.EOB == 1):
                answer_list.append(3012)
            if (self.EOM == 1):
                for name in ['MoveJoints', 'MoveLin', 'MoveLinRelTRF', 'MoveLinRelWRF', 'MovePose', 'SetCartAcc',
                             'SetJointAcc', 'SetTRF', 'SetWRF']:
                    if (command.find(name) != -1):
                        answer_list.append(3004)
                        break
            return answer_list

    def Activate(self):
        """Activates the Mecademic Robot

        :return response: Returns receive decrypted response
        """
        cmd = "ActivateRobot"
        return self.exchangeMsg(cmd)

    def Deactivate(self):
        """Deactivates the Mecademic Robot

        :return response: Returns receive decrypted response
        """
        cmd = "DeactivateRobot"
        return self.exchangeMsg(cmd)

    def ActivateSim(self):
        """Activates the Mecademic Robot simulation mode

        :return response: Returns receive decrypted response
        """
        cmd = "ActivateSim"
        return self.exchangeMsg(cmd)

    def DeactivateSim(self):
        """Deactivate the Mecademic Robot simulation mode

        :return response: Returns receive decrypted response
        """
        cmd = "DeactivateSim"
        return self.exchangeMsg(cmd)

    def SwitchToEtherCAT(self):
        """Places the Mecademic Robot in EtherCat mode
        """
        cmd = "SwitchToEtherCAT"
        return self.exchangeMsg(cmd)

    def SetEOB(self, e):
        """Sets End of Block answer active or inactive in the Mecademic Robot

        :param e: Enables (1) EOB or Disables (0) EOB
        :return response: Returns receive decrypted response
        """
        if (e == 1):
            self.EOB = 1
        else:
            self.EOB = 0
        raw_cmd = "SetEOB"
        cmd = self._buildCommand(raw_cmd, [e])
        return self.exchangeMsg(cmd)

    def SetEOM(self, e):
        """Sets End of Movement answer active or inactive in the Mecademic Robot

        :param e: Enables (1) EOM or Disables (0) EOM
        :return response: Returns receive decrypted response
        """
        if (e == 1):
            self.EOM = 1
        else:
            self.EOM = 0
        raw_cmd = "SetEOM"
        cmd = self._buildCommand(raw_cmd, [e])
        return self.exchangeMsg(cmd)

    def Home(self):
        """Homes the Mecademic Robot

        :return response: Returns receive decrypted response
        """
        cmd = "Home"
        return self.exchangeMsg(cmd)

    def Delay(self, t):
        """Gives the Mecademic Robot a wait time before performing another action

        :return response: Returns receive decrypted response
        """
        if (not isinstance(t, float)):
            t = float(t)
        raw_cmd = "delay"
        cmd = self._buildCommand(raw_cmd, [t])
        return self.exchangeMsg(cmd, t * 2)

    def GripperOpen(self):
        """Opens the gripper of the end-effector

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = 'GripperOpen'
        return self.exchangeMsg(cmd)

    def GripperClose(self):
        """Closes the gripper of the end-effector

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = 'GripperClose'
        return self.exchangeMsg(cmd)

    def MoveJoints(self, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6):
        """Moves the joints of the Mecademic Robot to the desired angles

        :param theta_1: Angle of joint 1
        :param theta_2: Angle of joint 2
        :param theta_3: Angle of joint 3
        :param theta_4: Angle of joint 4
        :param theta_5: Angle of joint 5
        :param theta_6: Angle of joint 6
        :return response: Returns receive decrypted response
        """
        raw_cmd = "MoveJoints"
        cmd = self._buildCommand(raw_cmd, [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
        return self.exchangeMsg(cmd)

    def MoveLin(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot tool reference in a straight line to final
        point with specified direction

        :param x: Final x coordinate
        :param y: Final y coordinate
        :param z: Final z coordinate
        :param alpha: Final Alpha angle
        :param beta: Final Beta angle
        :param gamma: Final Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "MoveLin"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def MoveLinRelTRF(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot tool reference frame to specified coordinates and heading

        :param x: New Reference x coordinate
        :param y: New Reference y coordinate
        :param z: New Reference z coordinate
        :param alpha: New Reference Alpha angle
        :param beta: New Reference Beta angle
        :param gamma: New Reference Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "MoveLinRelTRF"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def MoveLinRelWRF(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot world reference frame to specified coordinates and heading

        :param x: New Reference x coordinate
        :param y: New Reference y coordinate
        :param z: New Reference z coordinate
        :param alpha: New Reference Alpha angle
        :param beta: New Reference Beta angle
        :param gamma: New Reference Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "MoveLinRelWRF"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def MovePose(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot joints to have the TRF at (x,y,z)
        with heading (alpha, beta, gamma)

        :param x: Final x coordinate
        :param y: Final y coordinate
        :param z: Final z coordinate
        :param alpha: Final Alpha angle
        :param beta: Final Beta angle
        :param gamma: Final Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "MovePose"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def SetBlending(self, p):
        """Sets the blending of the Mecademic Robot

        :param p: Enable(1-100)/Disable(0) Mecademic Robot's blending
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetBlending"
        cmd = self._buildCommand(raw_cmd, [p])
        return self.exchangeMsg(cmd)

    def SetAutoConf(self, e):
        """Enables or Disables the automatic robot configuration
        selection and has effect only on the MovePose command

        :param e: Enable(1)/Disable(0) Mecademic Robot's automatic configuration selection
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetAutoConf"
        cmd = self._buildCommand(raw_cmd, [e])
        return self.exchangeMsg(cmd)

    def SetCartAcc(self, p):
        """Sets the cartesian accelerations of the linear and angular movements of the
        Mecademic Robot end effector

        :param p: value between 1 and 100
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetCartAcc"
        cmd = self._buildCommand(raw_cmd, [p])
        return self.exchangeMsg(cmd)

    def SetCartAngVel(self, w):
        """Sets the cartesian angular velocity of the Mecademic Robot TRF with respect to its WRF

        :param w: value between 0.001 and 180
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetCartAngVel"
        cmd = self._buildCommand(raw_cmd, [w])
        return self.exchangeMsg(cmd)

    def SetCartLinVel(self, v):
        """Sets the cartesian linear velocity of the Mecademic Robot's TRF relative to its WRF

        :param v: between 0.001 and 500
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetCartLinVel"
        cmd = self._buildCommand(raw_cmd, [v])
        return self.exchangeMsg(cmd)

    def SetConf(self, c1, c3, c5):
        """Sets the desired Mecademic Robot inverse kinematic configuration to be observed in the
        MovePose command

        :param c1: -1 or 1
        :param c3: -1 or 1
        :param c5: -1 or 1
        :return response: Returns received decrypted response
        """
        raw_cmd = "SetConf"
        cmd = self._buildCommand(raw_cmd, [c1, c3, c5])
        return self.exchangeMsg(cmd)

    def SetGripperForce(self, p):
        """Sets the Gripper's grip force

        :param p: value between 1 to 100
        :return response: Returns the decrypted response from the Mecademic Robot
        """
        raw_cmd = "SetGripperForce"
        cmd = self._buildCommand(raw_cmd, [p])
        return self.exchangeMsg(cmd)

    def SetGripperVel(self, p):
        """Sets the Gripper fingers' velocity with respect to the gripper

        :param p: value between 1 to 100
        :return response: Returns the decrypted response from the Mecademic Robot
        """
        raw_cmd = "SetGripperVel"
        cmd = self._buildCommand(raw_cmd, [p])
        return self.exchangeMsg(cmd)

    def SetJointAcc(self, p):
        """Sets the acceleration of the joints

        :param p: value between 1 to 100
        :return response: Returns the decrypted response from the Mecademic Robot
        """
        raw_cmd = "SetJointAcc"
        cmd = self._buildCommand(raw_cmd, [p])
        return self.exchangeMsg(cmd)

    def SetJointVel(self, velocity):
        """Sets the angular velocities of the Mecademic Robot's joints

        :param velocity: value between 1 to 100
        :return response: Returns the decrypted response from the Mecademic Robot
        """
        raw_cmd = "SetJointVel"
        cmd = self._buildCommand(raw_cmd, [velocity])
        return self.exchangeMsg(cmd)

    def SetTRF(self, x, y, z, alpha, beta, gamma):
        """Sets the Mecademic Robot TRF at (x,y,z) and heading (alpha, beta, gamma)
        with respect to the FRF

        :param x: Final x coordinate
        :param y: Final y coordinate
        :param z: Final z coordinate
        :param alpha: Final Alpha angle
        :param beta: Final Beta angle
        :param gamma: Final Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetTRF"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def SetWRF(self, x, y, z, alpha, beta, gamma):
        """Sets the Mecademic Robot WRF at (x,y,z) and heading (alpha, beta, gamma)
        with respect to the BRF

        :param x: Final x coordinate
        :param y: Final y coordinate
        :param z: Final z coordinate
        :param alpha: Final Alpha angle
        :param beta: Final Beta angle
        :param gamma: Final Gamma angle
        :return response: Returns receive decrypted response
        """
        raw_cmd = "SetWRF"
        cmd = self._buildCommand(raw_cmd, [x, y, z, alpha, beta, gamma])
        return self.exchangeMsg(cmd)

    def GetStatusRobot(self):
        """Retrieves the robot status of the Mecademic Robot

        :return status: Returns tuple with status of Activation, Homing, Simulation
        Error, Paused, EOB and EOM
        """
        received = None
        while received is None:
            cmd = "GetStatusRobot"
            received = self.exchangeMsg(cmd, decode=True)
        code_list_int = received
        return {"Activated": code_list_int[0],
                "Homing": code_list_int[1],
                "Simulation": code_list_int[2],
                "Error": code_list_int[3],
                "Paused": code_list_int[4],
                "EOB": code_list_int[5],
                "EOM": code_list_int[6]}

    def GetStatusGripper(self):
        """Retrieves the gripper status of the Mecademic Robot

        :return status: Returns tuple with status of Gripper enabled, Homing state, Holding part
        Limit reached, Error state and force overload
        """
        received = None
        while received is None:
            cmd = "GetStatusGripper"
            received = self.exchangeMsg(cmd)
        code_list_int = received
        return {"Gripper enabled": code_list_int[0],
                "Homing state": code_list_int[1],
                "Holding part": code_list_int[2],
                "Limit reached": code_list_int[3],
                "Error state": code_list_int[4],
                "force overload": code_list_int[5]}

    def GetConf(self):
        """Retrieves the current inverse kinematic configuration

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "GetConf"
        return self.exchangeMsg(cmd)

    def GetJoints(self):
        """Retrieves the Mecademic Robot joint angles in degrees

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "GetJoints"
        return self.exchangeMsg(cmd)

    def GetPose(self):
        """Retrieves the current pose of the Mecademic Robot TRF with
        respect to the WRF

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "GetPose"
        return self.exchangeMsg(cmd)

    def PauseMotion(self):
        """Stops the robot movement and holds until ResumeMotion

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "PauseMotion"
        return self.exchangeMsg(cmd)

    def ResumeMotion(self):
        """Resumes the robot movement after being Paused from PauseMotion
        or ClearMotion

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "ResumeMotion"
        return self.exchangeMsg(cmd)

    def ClearMotion(self):
        """Stops the robot movement and deletes the rest of the robot's
        trajectory. Holds until a ResumeMotion

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "ClearMotion"
        return self.exchangeMsg(cmd)

    def BrakesOn(self):
        """These commands enables the brakes of joints 1, 2 and 3,
        if and only if the robotis powered but deactivated.

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "BrakesOn"
        return self.exchangeMsg(cmd)

    def BrakesOff(self):
        """These commands disables the brakes of joints 1, 2 and 3,
        if and only if the robotis powered but deactivated.

        :return response: Returns the decrypted response from the Mecademic Robot
        """
        cmd = "BrakesOff"
        return self.exchangeMsg(cmd)
