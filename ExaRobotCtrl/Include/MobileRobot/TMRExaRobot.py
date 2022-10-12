# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : TMRExaRobot.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.04.05 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import time

from numpy import pi

from TMRDiff import *

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from SerialPort import CSerialPort
from Commons import PySignal, is_system_win, MAX_SERIAL_BUFFER, search_pattern
from Commons import convert_s32_to_bytearray_be, convert_u16_to_bytearray_be, convert_bytearray_to_int_be, convert_s16_to_bytearray_be


@unique
class EnumExaCmdIdentifier(IntEnum):
    VEL_MODE = 0xF3
    POS_R_MODE = 0xF4
    ENC_READ = 0xF5

    @property
    def toInt(self):
        return int(self)


@unique
class EnumExaEncReadMode(IntEnum):
    READ_RAW = 0x01
    READ_POS = 0x02

    @property
    def toInt(self):
        return int(self)


class CTMRExaRobot(CTMRDiff):
    """CTMRExaRobot은 Exa-ROBOT에 대한 클래스이다.
    
        Exa-ROBOT에 대한 parameter 정보에 대해서 정의되어 있다.
        Args:
            adictConfig (Union[None, dict], optional): 로봇 parameter 정보. Defaults to None.
    """
    _serial: CSerialPort
    btaSerialBuff: bytearray
    strSerialPort: str
    nSerialBaud: int
    nSerialByteSize: int
    strSerialParity: str
    fSerialStopBits: float
    fRadiusLimit: float
    fPrevTime: float

    def __init__(self, adictConfig: Union[None, dict] = None):

        if adictConfig is None:
            fWheelBase = 0.38  # according to FW
            fWheelRadius = 0.08255  # wheel diameter is 6.5 inches
            nEncoderRes = 4096  # PPR = 13
            fGearRatio = 1
            fMaxVel = 0.2#0.1 #for safe ..  #0.7
            fMaxAcc = 0.1#0.4  # arbitrary, cannot find anywhere
            fMaxJerk = 0.1#0.2  # arbitrary, cannot find anywhere
            fMaxYaw = 1.9199#1.9199  # 180 deg/s (>110 deg/s : gyro performance will degrade)
            super(CTMRExaRobot, self).__init__(fWheelRadius, fWheelBase, fMaxVel, fMaxAcc, fMaxJerk, fMaxYaw)
            self.set_wheel_spec(nEncoderRes, fGearRatio)
        else:
            # todo: add configuration file, preferably JSON
            pass

        # Serial Communication Specifications
        self.nSerialBaud = 115200
        self.nSerialByteSize = 8
        self.strSerialParity = 'N'
        self.fSerialStopBits = 1

        # Serial Callbacks
        self.btaSerialBuff = bytearray()
        self._serial = CSerialPort(abIsRecvDirect = False)
        self._serial.sig_connected.connect(self.on_serial_connect)
        self._serial.sig_disconnected.connect(self.on_serial_disconnect)
        self._serial.sig_serial_error.connect(self.on_serial_error)
        self._serial.sig_send_data.connect(self.on_serial_send)
        self._serial.sig_recv_data.connect(self.on_serial_recv)

        # Radius limit: Avoid 0 denominator when calculating the radius of curvature
        self.fRadiusLimit = 0.0001

        # calculate time difference to calculate velocityps
        self.nPrevTimeStamp = 0
        self.fPrevRadR = 0
        self.fPrevRadL = 0

        self.IsSerial = True  # serial robot

    def release(self):
        self.disconnect()
        
    ### Serial Communication ###
    def connect(self, strPort: Union[None, str] = None):
        """
        Serial 통신을 연결하는 함수이다.
        
        Serial 통신의 parameter 설정한다.

        Args:
            strPort (Union[None, str], optional): Port 설정. Defaults to None.
        """
        
        
        if strPort is None:
            if is_system_win():
                self.strSerialPort = "COM1"
            else:
                self.strSerialPort = "/dev/robot"
        else:
            self.strSerialPort = str(strPort)

        self._serial.setParams(port=self.strSerialPort, baudrate=self.nSerialBaud, bytesize=self.nSerialByteSize,
                               parity=self.strSerialParity, stopbits=self.fSerialStopBits)
        self._serial.connect()

    def is_connected(self) -> bool:
        return self._serial.is_connected()

    def disconnect(self):
        if self.is_connected():
            self._serial.disconnect()

    ### CALLBACKS ###
    def on_serial_connect(self):
        self.sig_connected.emit()
        self.start_get_encoder()
        self.stCurrPos.THETA = 0. 
        self.stCurrPos.X_POS = 0. 
        self.stCurrPos.Y_POS = 0. 
        self.stCurrPos.Z_POS = 0.

    def on_serial_disconnect(self):
        self.stop_get_encoder()
        self.sig_disconnected.emit()

    def on_serial_error(self):
        self.sig_comm_error.emit()

    def on_serial_send(self, abtData: bytes):
        pass

    def on_serial_recv(self, abtData: bytes):
        if len(self.btaSerialBuff) >= MAX_SERIAL_BUFFER:
            self.btaSerialBuff.clear()

        try:
            self.btaSerialBuff.extend(bytearray(abtData))
            self.deserialize_packet()

        except Exception:
            self.btaSerialBuff.clear()

    def deserialize_packet(self):
        """Serial로 받아온 packet를 처리한다.

        """
        
        bta_stx = bytearray([0x02, 0x5B])  # STX
        n_next_packet_idx = 0

        n_idx = search_pattern(self.btaSerialBuff, bta_stx)
        if n_idx is not None:
            # length of the whole packet
            # header: 4 bytes
            # todo: actual size should be determined
            if len(self.btaSerialBuff) >= n_idx + 4:
                # todo: packet should have actual packet size
                # length of encoder raw feedback
                n_packet_len = 13
                if len(self.btaSerialBuff) >= n_idx + n_packet_len:
                    if self.btaSerialBuff[n_idx + n_packet_len - 2] == 0x5D and self.btaSerialBuff[n_idx + n_packet_len - 1] == 0x03:
                        tmp = self.btaSerialBuff[n_idx: n_idx + n_packet_len]  # start of payload
                        self.parse_packet_values(tmp)
                        n_next_packet_idx = n_idx + n_packet_len

        if len(self.btaSerialBuff) >= n_next_packet_idx:
            self.btaSerialBuff = self.btaSerialBuff[n_next_packet_idx:]

    def parse_packet_values(self, abtPayload):
        """받아온 packet을 parse한다.

        Args:
            abtPayload (str): packet
        """
        
        b_cmd = abtPayload[2]
        if b_cmd == EnumExaCmdIdentifier.ENC_READ:
            left_encoder = convert_bytearray_to_int_be(abtPayload[3:7])
            right_encoder = convert_bytearray_to_int_be(abtPayload[7:11])
            timestamp = time.time()
            # calculate position
            f_dt = (timestamp - self.nPrevTimeStamp) / 1000.
            f_rad_r = 2 * pi * (right_encoder / self.nEncRes / self.GearRatio)
            f_rad_l = 2 * pi * (left_encoder / self.nEncRes / self.GearRatio)
            f_rads_r = (f_rad_r - self.fPrevRadR) / f_dt
            f_rads_l = (f_rad_l - self.fPrevRadL) / f_dt
            if(f_rads_r > 20000 or f_rads_r < -20000 ):
                f_rads_r = 0 
                # print(f_rads_r, f_rads_l)
            if(f_rads_l > 20000 or  f_rads_l < -20000):
                f_rads_l = 0
                # print(f_rads_r, f_rads_l)

            stCurrPos, f_vel_c, f_omega_c = self.calculate_pose((f_rads_r, f_rads_l), f_dt)
           
            self.nPrevTimeStamp = timestamp
            self.fPrevRadR = f_rad_r
            self.fPrevRadL = f_rad_l
            self.sig_get_enc.emit(right_encoder, left_encoder)
            self.sig_get_vel.emit(f_rads_r, f_rads_l)
            self.sig_get_pos.emit(stCurrPos.X_POS, stCurrPos.Y_POS, stCurrPos.THETA)
        else:
            pass

    ### COMMANDS ###
    def set_velocity_control(self, afVel: float, afYawRate: float):
        f_omega_r, f_omega_l = self.calculate_joint_space_vel(afVel, afYawRate)
        self.move_joint_space(f_omega_r, f_omega_l)

    # should always be rad/s
    def move_joint_space(self, afRight: float, afLeft: float):
        tmp = bytearray([])
        f_left_rpm = (afLeft / (2 * pi)) * 60  # convert to RPM
        f_right_rpm = (afRight / (2 * pi)) * 60 # convert to RPM
        nLeft = convert_s16_to_bytearray_be(int(f_left_rpm))
        tmp.extend(nLeft)
        nRight = convert_s16_to_bytearray_be(int(f_right_rpm))
        tmp.extend(nRight)
        self.send_packet(EnumExaCmdIdentifier.VEL_MODE, tmp)

    def move_stop(self):
        self.move_joint_space(0., 0.)

    def start_get_encoder(self, anPeriod: int = 50, anType: EnumExaEncReadMode = EnumExaEncReadMode.READ_RAW):
        tmp = bytearray([])
        tmp.extend(convert_s16_to_bytearray_be(int(anType)))
        tmp.extend(convert_s16_to_bytearray_be(int(anPeriod)))
        self.send_packet(EnumExaCmdIdentifier.ENC_READ, tmp)

    def stop_get_encoder(self):
        self.start_get_encoder(0)

    def make_packet(self, anCmd: int, abtaData: bytearray) -> bytearray:
        """명령어에 대한 packet을 생성한다.

        Args:
            anCmd (int): command
            abtaData (bytearray): value

        Returns:
            bytearray: Packet
        """
        
        buff = bytearray([0x02, 0x5B])  # STX
        buff.append(int(anCmd))

        if abtaData is not None:
            buff.extend(abtaData)

        buff.extend([0x5D, 0x03])

        return buff

    def send_packet(self, anCmd: int, abtaPayLoad: bytearray = None) -> bool:
        """Serial로 명령 packet을 송신한다.

        Args:
            anCmd (int): command
            abtaPayLoad (bytearray, optional): 전달 packet. Defaults to None.

        Returns:
            bool: 송신결과
        """
        
        bta_packet = self.make_packet(anCmd, abtaPayLoad)
        if self._serial.is_connected():
            self._serial.sendData(bta_packet)
            return True
        else:
            return False


if __name__ == '__main__':
    test = CTMRExaRobot()
    test.connect('/dev/robot')
    test.get_curr_pos_t()
    test.set_velocity_control(0.05, 0)
    while 1:
        print(test.get_curr_pos_t())
        time.sleep(1)