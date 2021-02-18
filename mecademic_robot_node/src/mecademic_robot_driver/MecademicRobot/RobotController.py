#!/usr/bin/env python3
import socket
import time


class RobotController:
    """Class for the Mecademic Robot allowing for communication and control of the
    Mecademic Robot with all of its features available.

    Attributes
    ----------
    address : string
        The IP address associated to the Mecademic Robot
    socket : socket object
        Socket connecting to physical Mecademic Robot.
    EOB : int
        Setting for EOB (End of Block) reply.
    EOM : int
        Setting for EOM (End of Movement) reply.
    error : boolean
        Error Status of the Mecademic Robot.
    queue : boolean
        Queuing option flag.

    """

    def __init__(self, address):
        """Constructor for an instance of the Class Mecademic Robot.

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic Robot.

        """
        self.address = address
        self.socket = None
        self.EOB = 1
        self.EOM = 1
        self.error = False
        self.queue = False

    def is_in_error(self):
        """Status method that checks whether the Mecademic Robot is in error mode.

        Return the global variable error, which is updated by the other methods.

        Returns
        -------
        error : boolean
            Returns the error flag, True for error and False otherwise.

        """
        return self.error

    def ResetError(self):
        """Resets the error in the Mecademic Robot.

        Returns
        -------
        response : string
            Message from the robot.

        """
        self.error = False
        cmd = 'ResetError'
        response = self.exchange_msg(cmd)
        reset_success = self._response_contains(response, ['The error was reset', 'There was no error to reset'])
        if reset_success:
            self.error = False
        else:
            self.error = True
        return response

    def connect(self):
        """Connects Mecademic Robot object communication to the physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure

        """
        try:
            self.socket = socket.socket()
            self.socket.settimeout(0.1)  # 100ms
            try:
                self.socket.connect((self.address, 10000))
            except socket.timeout:
                raise TimeoutError

            # Receive confirmation of connection
            if self.socket is None:
                raise RuntimeError

            self.socket.settimeout(10)  # 10 seconds
            try:
                response = self.socket.recv(1024).decode('ascii')
            except socket.timeout:
                raise RuntimeError

            if self._response_contains(response, ['[3001]']):
                print(f'Another user is already connected, closing connection.')
            elif self._response_contains(response, ['[3000]']):     # search for key [3000] in the received packet
                return True
            else:
                print(f'Unexpected code returned.')
                print(f'response: {response}')
                raise RuntimeError

        except TimeoutError:
            return False
        except RuntimeError:
            return False

    def disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        if(self.socket is not None):
            self.socket.close()
            self.socket = None

    @staticmethod
    def _response_contains(response, code_list):
        """Scans received response for code IDs.

        Parameters
        ----------
        response :
            Message to scan for codes.
        code_list :
            List of codes to look for in the response.

        Returns
        -------
        response_found :
            Returns whether the response contains a code ID of interest.

        """
        response_found = False
        for code in code_list:
            if response.find(code) != -1:
                response_found = True
                break
        return response_found

    def _send(self, cmd):
        """Sends a command to the physical Mecademic Robot.

        Parameters
        ----------
        cmd : string
            Command to be sent.

        Returns
        -------
        status : boolean
            Returns whether the message is sent.

        """
        if self.socket is None or self.error:               #check that the connection is established or the robot is in error
            return False                                    #if issues detected, no point in trying to send a cmd that won't reach the robot
        cmd = cmd + '\0'
        status = 0
        while status == 0:
            try:                                            #while the message hasn't been sent
                status = self.socket.send(cmd.encode('ascii'))  #send command in ascii encoding
            except:
                break
            if status != 0:
                return True                                 #return true when the message has been sent
                                                            #Message failed to be sent, return false
        return False

    def _receive(self, answer_list, delay=20):
        """Receives message from the Mecademic Robot and looks for
        expected answer in the reply.

        Parameters
        ----------
        answer_list : list
            Codes to look for in the response.
        delay : int
            Time to set for timeout of the socket.

        Returns
        -------
        response : string
            Response received from Mecademic Robot.

        """
        if self.socket is None:                         #check that the connection is established
            return                                      #if no connection, nothing to receive
        response_list = []
        response_found = False
        for x in answer_list:                           #convert codes to search for in answer into comparable format
            response_list.append(f'[{str(x)}]')
        error_found = False
        error_list = [f'[{str(i)}]' for i in range(1000, 1039)]+[f'[{str(i)}]' for i in [3001,3003,3005,3009,3014,3026]]  #Make error codes in a comparable format
        self.socket.settimeout(delay)                   #set read timeout to desired delay
        while not response_found and not error_found:   #while no answers have been received, keep looking
            try:
                response = self.socket.recv(1024).decode('ascii')   #read message from robot
            except socket.timeout:
                return                                  #if timeout reached, either connection lost or nothing was sent from robot (damn disabled EOB and EOM)
            if(len(response_list)!=0):                  #if the message has a code to look for, find them
                response_found = self._response_contains(response, response_list)
            error_found = self._response_contains(response, error_list) #search message for errors
        if error_found:                                 #if errors have been found, flag the script
            self.error = True
        return response                                 #return the retrieved message

    def exchange_msg(self, cmd, delay=20, decode=True):
        """Sends and receives with the Mecademic Robot.

        Parameters
        ----------
        cmd : string
            Command to send to the Mecademic Robot.
        delay : int
            Timeout to set for the socket.
        decode : string
            decrypt response based on right response code

        Returns
        -------
        response : string
            Response with desired code ID.

        """
        response_list = self._get_answer_list(cmd)
        if(not self.error):                                 #if there is no error
            status = self._send(cmd)                        #send the command to the robot
            if status is True:                              #if the command was sent
                if self.queue:                              #if Queueing enabled skip receving responses
                    return
                else:
                    answer = self._receive(response_list, delay)#get response from robot
                    if answer is not None:                      #if message was retrieved
                        for response in response_list:          #search for response codes
                            if self._response_contains(answer, [str(response)]):
                                if(decode):
                                    return self._decode_msg(answer, response)   #decrypt response based on right response code
                                else:
                                    return answer
                        error_list = [str(i) for i in range(1000, 1039)]+[str(i) for i in [3001,3003,3005,3009,3014,3026]]  #Make error codes in a comparable format
                        for response in error_list:
                            if self._response_contains(answer, [str(response)]):
                                if(decode):
                                    return self._decode_msg(answer, response)   #decrypt response based on right response code
                                else:
                                    return answer
                    else:
                        if(len(response_list) == 0):            #if we aren't expecting anything, don't bother looking
                            return
                        else:
                            return
            #if message didn't send correctly, reboot communication
            self.disconnect()
            time.sleep(1)
            self.connect()
            return

    def _build_command(self, cmd, arg_list=[]):
        """Builds the command string to send to the Mecademic Robot
        from the function name and arguments the command needs.

        Parameters
        ----------
        cmd : string
            Command name to send to the Mecademic Robot
        arg_list : list
            List of arguments the command requires

        Returns
        -------
        command : string
            Final command for the Mecademic Robot

        """
        command = cmd
        if(len(arg_list)!=0):
            command = command + '('
            for index in range(0, (len(arg_list)-1)):
                command = command+str(arg_list[index])+','
            command = command+str(arg_list[-1])+')'
        return command

    def _decode_msg(self, response, response_key):
        """Decrypt information from the Mecademic Robot response to useful information
        that can be manipulated.

        Parameters
        ----------
        response : string
            Response from the Mecademic Robot
        response_key : int
            Code ID of response to decrypt

        Returns
        -------
        code_list_int : list of int
            Decrypted information

        """
        code = response.replace('['+str(response_key)+'][', '').replace(']', '').replace('\x00', '')    #remove delimiters and \x00 bytes
        code_list = code.split(',')                         #split packets into their individual selves
        if(response_key == 2026 or response_key == 2027):   #if expected packet is from GetJoints (2026) or GetPose (2027), rest of packet is position data
            code_list_float = tuple((float(x) for x in code_list))      #convert position data to floats
            return code_list_float
        elif(response_key == 2029 or response_key == 2007 or response_key == 2079): #if expected packet is from GetConf (2029), GetStatusRobot (2007) or GetStatusGripper (2079), rest of packet is data
            code_list_int = tuple((int(x) for x in code_list))          #convert status data into integers
            return code_list_int
        else:
            return code                                      #nothing to decrypt or decryption not specified

    def _get_answer_list(self, command):
        """Retrieve the expected answer codes that the Mecademic Robot should
        send as feedback after a command.

        Parameters
        ----------
        command : string
            Command that is to be sent to the Mecademic Robot.

        Returns
        -------
        answer_list : list
            List of answer codes to search for in response.

        """
        if(command.find('ActivateRobot') != -1):
            return [2000,2001]
        elif(command.find('ActivateSim')!= -1):
            return [2045]
        elif(command.find('ClearMotion')!= -1):
            return [2044]
        elif(command.find('DeactivateRobot')!= -1):
            return [2004]
        elif(command.find('BrakesOn')!= -1):
            return [2010]
        elif(command.find('BrakesOff')!= -1):
            return [2008]
        elif(command.find('GetConf')!= -1):
            return [2029]
        elif(command.find('GetJoints')!= -1):
            return [2026]
        elif(command.find('GetStatusRobot')!= -1):
            return [2007]
        elif(command.find('GetStatusGripper')!= -1):
            return [2079]
        elif(command.find('GetPose')!= -1):
            return [2027]
        elif(command.find('Home')!= -1):
            return [2002,2003]
        elif(command.find('PauseMotion')!= -1):
            answer_list = [2042]
            if(self.EOM == 1):
                answer_list.append(3004)
            return answer_list
        elif(command.find('ResetError')!= -1):
            return [2005,2006]
        elif(command.find('ResumeMotion')!= -1):
            return [2043]
        elif(command.find('SetEOB')!= -1):
            return [2054,2055]
        elif(command.find('SetEOM')!= -1):
            return [2052,2053]
        else:
            answer_list = []
            if(self.EOB==1):
                answer_list.append(3012)
            if(self.EOM==1):
                for name in ['MoveJoints','MoveLin','MoveLinRelTRF','MoveLinRelWRF','MovePose','SetCartAcc','SetJointAcc','SetTRF','SetWRF']:
                    if(command.find(name) != -1):
                        answer_list.append(3004)
                        break
            return answer_list

    def ActivateRobot(self):
        """Activates the Mecademic Robot.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'ActivateRobot'
        return self.exchange_msg(cmd)

    def DeactivateRobot(self):
        """Deactivates the Mecademic Robot.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'DeactivateRobot'
        return self.exchange_msg(cmd)

    def ActivateSim(self):
        """Activates the Mecademic Robot simulation mode.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'ActivateSim'
        return self.exchange_msg(cmd)

    def DeactivateSim(self):
        """Deactivate the Mecademic Robot simulation mode.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'DeactivateSim'
        return self.exchange_msg(cmd)

    def SwitchToEtherCAT(self):
        """Places the Mecademic Robot in EtherCat mode

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'SwitchToEtherCAT'
        return self.exchange_msg(cmd)

    def SetEOB(self, e):
        """Sets End of Block answer active or inactive in the Mecademic Robot.

        Parameters
        ----------
        e : int
            Enables (1) EOB or Disables (0) EOB.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        if(e == 1):
            self.EOB = 1
        else:
            self.EOB = 0
        raw_cmd = 'SetEOB'
        cmd = self._build_command(raw_cmd,[e])
        return self.exchange_msg(cmd)

    def SetEOM(self, e):
        """Sets End of Movement answer active or inactive in the Mecademic Robot.

        Parameters
        ----------
        e : int
            Enables (1) EOM or Disables (0) EOM.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        if(e == 1):
            self.EOM = 1
        else:
            self.EOM = 0
        raw_cmd = 'SetEOM'
        cmd = self._build_command(raw_cmd,[e])
        return self.exchange_msg(cmd)

    def home(self):
        """Homes the Mecademic Robot.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        cmd = 'Home'
        return self.exchange_msg(cmd)

    def Delay(self, t):
        """Gives the Mecademic Robot a wait time before performing another action.

        Parameters
        ----------
        t : int or float
            Delay time in seconds.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        if(not isinstance(t,float)):
            t = float(t)
        raw_cmd = 'Delay'
        cmd = self._build_command(raw_cmd,[t])
        return self.exchange_msg(cmd, t*2)

    def GripperOpen(self):
        """Opens the gripper of the end-effector.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'GripperOpen'
        return self.exchange_msg(cmd)

    def GripperClose(self):
        """Closes the gripper of the end-effector.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'GripperClose'
        return self.exchange_msg(cmd)

    def MoveJoints(self, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6):
        """Moves the joints of the Mecademic Robot to the desired angles.

        Parameters
        ----------
        theta_1 : float
            Angle of joint 1
        theta_2 : float
            Angle of joint 2
        theta_3 : float
            Angle of joint 3
        theta_4 : float
            Angle of joint 4
        theta_5 : float
            Angle of joint 5
        theta_6 : float
            Angle of joint 6

        Returns
        -------
        response : string
            Returns receive decrypted response

        """
        raw_cmd = 'MoveJoints'
        cmd = self._build_command(raw_cmd,[theta_1,theta_2,theta_3,theta_4,theta_5,theta_6])
        return self.exchange_msg(cmd)

    def MoveLin(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot tool reference in a straight line to final
        point with specified direction

        Parameters
        ----------
        x : float
            Final x coordinate.
        y : float
            Final y coordinate.
        z : float
            Final z coordinate.
        alpha : float
            Final Alpha angle.
        beta : float
            Final Beta angle.
        gamma : float
            Final Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'MoveLin'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def MoveLinRelTRF(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot tool reference frame to specified coordinates and heading.

        Parameters
        ----------
        x : float
            New Reference x coordinate.
        y : float
            New Reference y coordinate.
        z : float
            New Reference z coordinate.
        alpha : float
            New Reference Alpha angle.
        beta : float
            New Reference Beta angle.
        gamma : float
            New Reference Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'MoveLinRelTRF'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def MoveLinRelWRF(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot world reference frame to specified coordinates and heading.

        Parameters
        ----------
        x : float
            New Reference x coordinate.
        y : float
            New Reference y coordinate.
        z : float
            New Reference z coordinate.
        alpha : float
            New Reference Alpha angle.
        beta : float
            New Reference Beta angle.
        gamma : float
            New Reference Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'MoveLinRelWRF'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def MovePose(self, x, y, z, alpha, beta, gamma):
        """Moves the Mecademic Robot joints to have the TRF at (x,y,z)
        with heading (alpha, beta, gamma).

        Parameters
        ----------
        x : float
            Final x coordinate.
        y : float
            Final y coordinate.
        z : float
            Final z coordinate.
        alpha : float
            Final Alpha angle.
        beta : float
            Final Beta angle.
        gamma : float
            Final Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'MovePose'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def SetBlending(self, p):
        """Sets the blending of the Mecademic Robot.

        Parameters
        ----------
        p : int
            Enable(1-100)/Disable(0) Mecademic Robot's blending.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetBlending'
        cmd = self._build_command(raw_cmd,[p])
        return self.exchange_msg(cmd)

    def SetAutoConf(self, e):
        """Enables or Disables the automatic robot configuration
        selection and has effect only on the MovePose command.

        Parameters
        ----------
        e : boolean
            Enable(1)/Disable(0) Mecademic Robot's automatic configuration selection.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetAutoConf'
        cmd = self._build_command(raw_cmd,[e])
        return self.exchange_msg(cmd)

    def SetCartAcc(self, p):
        """Sets the cartesian accelerations of the linear and angular movements of the
        Mecademic Robot end effector.

        Parameters
        ----------
        p : float
            Value between 1 and 100.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetCartAcc'
        cmd = self._build_command(raw_cmd,[p])
        return self.exchange_msg(cmd)

    def SetCartAngVel(self, w):
        """Sets the cartesian angular velocity of the Mecademic Robot TRF with respect to its WRF.

        Parameters
        ----------
        w : float
            Value between 0.001 and 180.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetCartAngVel'
        cmd = self._build_command(raw_cmd,[w])
        return self.exchange_msg(cmd)

    def SetCartLinVel(self, v):
        """Sets the cartesian linear velocity of the Mecademic Robot's TRF relative to its WRF.

        Parameters
        ----------
        v : float
            Between 0.001 and 500.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetCartLinVel'
        cmd = self._build_command(raw_cmd,[v])
        return self.exchange_msg(cmd)

    def SetConf(self, c1, c3, c5):
        """Sets the desired Mecademic Robot inverse kinematic configuration to be observed in the
        MovePose command.

        Parameters
        ----------
        c1 : int
            -1 or 1.
        c3 : int
            -1 or 1.
        c5 : int
            -1 or 1.

        Returns
        -------
        response : string
            Returns received decrypted response.

        """
        raw_cmd = 'SetConf'
        cmd = self._build_command(raw_cmd,[c1,c3,c5])
        return self.exchange_msg(cmd)

    def SetGripperForce(self, p):
        """Sets the Gripper's grip force.

        Parameters
        ----------
        p : int
            Value between 1 to 100.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        raw_cmd = 'SetGripperForce'
        cmd = self._build_command(raw_cmd,[p])
        return self.exchange_msg(cmd)

    def SetGripperVel(self, p):
        """Sets the Gripper fingers' velocity with respect to the gripper.

        Parameters
        ----------
        p : int
            value between 1 to 100.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        raw_cmd = 'SetGripperVel'
        cmd = self._build_command(raw_cmd,[p])
        return self.exchange_msg(cmd)

    def SetJointAcc(self, p):
        """Sets the acceleration of the joints.

        Parameters
        ----------
        p : int
            value between 1 to 100.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        raw_cmd = 'SetJointAcc'
        cmd = self._build_command(raw_cmd,[p])
        return self.exchange_msg(cmd)

    def SetJointVel(self, velocity):
        """Sets the angular velocities of the Mecademic Robot's joints.

        Parameters
        ----------
        velocity : int
            value between 1 to 100.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        raw_cmd = 'SetJointVel'
        cmd = self._build_command(raw_cmd,[velocity])
        return self.exchange_msg(cmd)

    def SetTRF(self, x, y, z, alpha, beta, gamma):
        """Sets the Mecademic Robot TRF at (x,y,z) and heading (alpha, beta, gamma)
        with respect to the FRF.

        Parameters
        ----------
        x : float
            Final x coordinate.
        y : float
            Final y coordinate.
        z : float
            Final z coordinate.
        alpha : float
            Final Alpha angle.
        beta : float
            Final Beta angle.
        gamma : float
            Final Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetTRF'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def SetWRF(self, x, y, z, alpha, beta, gamma):
        """Sets the Mecademic Robot WRF at (x,y,z) and heading (alpha, beta, gamma)
        with respect to the BRF.

        Parameters
        ----------
        x : float
            Final x coordinate.
        y : float
            Final y coordinate.
        z : float
            Final z coordinate.
        alpha : float
            Final Alpha angle.
        beta : float
            Final Beta angle.
        gamma : float
            Final Gamma angle.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        raw_cmd = 'SetWRF'
        cmd = self._build_command(raw_cmd,[x,y,z,alpha,beta,gamma])
        return self.exchange_msg(cmd)

    def GetStatusRobot(self):
        """Retrieves the robot status of the Mecademic Robot.

        Returns
        -------
        status : tuple
            Returns tuple with status of Activation, Homing, Simulation,
            Error, Paused, EOB and EOM.

        """
        received = None
        while received is None:
            cmd = 'GetStatusRobot'
            received = self.exchange_msg(cmd)
        code_list_int = received
        return {'Activated': code_list_int[0],
                'Homing': code_list_int[1],
                'Simulation': code_list_int[2],
                'Error': code_list_int[3],
                'Paused': code_list_int[4],
                'EOB': code_list_int[5],
                'EOM': code_list_int[6]}

    def GetStatusGripper(self):
        """Retrieves the gripper status of the Mecademic Robot.

        Returns
        -------
        status : tuple
            Returns tuple with status of Gripper enabled, Homing state, Holding part
            Limit reached, Error state and force overload

        """
        received = None
        while received is None:
            cmd = 'GetStatusGripper'
            received = self.exchange_msg(cmd)
        code_list_int = received
        return {'Gripper enabled': code_list_int[0],
                'Homing state': code_list_int[1],
                'Holding part': code_list_int[2],
                'Limit reached': code_list_int[3],
                'Error state': code_list_int[4],
                'force overload': code_list_int[5]}

    def GetConf(self):
        """Retrieves the current inverse kinematic configuration.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'GetConf'
        return self.exchange_msg(cmd)

    def GetJoints(self):
        """Retrieves the Mecademic Robot joint angles in degrees.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'GetJoints'
        return self.exchange_msg(cmd)

    def GetPose(self):
        """Retrieves the current pose of the Mecademic Robot TRF with
        respect to the WRF.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'GetPose'
        return self.exchange_msg(cmd)

    def PauseMotion(self):
        """Stops the robot movement and holds until ResumeMotion.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'PauseMotion'
        return self.exchange_msg(cmd)

    def ResumeMotion(self):
        """Resumes the robot movement after being Paused from PauseMotion
        or ClearMotion.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'ResumeMotion'
        return self.exchange_msg(cmd)

    def ClearMotion(self):
        """Stops the robot movement and deletes the rest of the robot's
        trajectory. Holds until a ResumeMotion.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'ClearMotion'
        return self.exchange_msg(cmd)

    def BrakesOn(self):
        """These commands enables the brakes of joints 1, 2 and 3,
        if and only if the robotis powered but deactivated.

        Returns
        -------
        response : string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'BrakesOn'
        return self.exchange_msg(cmd)

    def BrakesOff(self):
        """These commands disables the brakes of joints 1, 2 and 3,
        if and only if the robotis powered but deactivated.

        Returns
        -------
        response: string
            Returns the decrypted response from the Mecademic Robot.

        """
        cmd = 'BrakesOff'
        return self.exchange_msg(cmd)

    def set_queue(self, e):
        """ Enables the queueing of move commands for blending.

        Parameters
        ----------
        e : boolean
            Enables (1) Queueing or Disables (0) Queueing.

        Returns
        -------
        response : string
            Returns receive decrypted response.

        """
        if (e == 1):
            self.queue = True
            self.UserEOM = self.EOM
            self.SetEOM(0)
        else:
            self.queue = False
            self.SetEOM(self.UserEOM)
        return self.queue
