from dataclasses import dataclass
from enum import Enum, IntEnum
from time import time, sleep
#from enum import IntEnum
import serial
import crcmod

crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0x0000)
tasks = {}
task = lambda f: tasks.setdefault(f.__name__[1:], f)

ping_delay = 5

#=====================================
#    Enumerations
#=====================================
class AMSGTYPE_E(IntEnum):
    MSG_OWAY    = 0X0
    MSG_OWAYU   = 0X1
    MSG_REQ     = 0X2
    MSG_RESP    = 0X3
    MSG_REQU    = 0X4
    MSG_RESPU   = 0X5
    MSG_REQX    = 0X6
    MSG_RESPX   = 0X7
    MSG_UNKNOWN = 0XFF

class BID_E(IntEnum):
    BEACON_ALL  = 0X00
    BEACON_ID_1 = 0X01
    BEACON_ID_2 = 0X02
    BEACON_ID_3 = 0X03
    BEACON_ID_4 = 0X04
    BEACON_ID_5 = 0X05
    BEACON_ID_6 = 0X06
    BEACON_ID_7 = 0X07
    BEACON_ID_8 = 0X08
    BEACON_ID_9 = 0X09
    BEACON_ID_10 = 0X0A
    BEACON_ID_11 = 0X0B
    BEACON_ID_12 = 0X0C
    BEACON_ID_13 = 0X0D
    BEACON_ID_14 = 0X0E
    BEACON_ID_15 = 0X0F

class APAYLOAD_E(IntEnum):
    PLOAD_PING = 0X00
    PLOAD_ECHO = 0X01
    PLOAD_NAV  = 0X02
    PLOAD_DAT  = 0X03
    PLOAD_DEX  = 0X04

class CID_E(IntEnum):
    # System Messages
    SYS_ALIVE = 0x01
    SYS_INFO = 0x02
    SYS_REBOOT = 0x03
    SYS_ENGINEERING = 0x04
    # Firmware Programming Messages
    PROG_INIT   = 0x0D
    PROG_BLOCK  = 0x0E
    PROG_UPDATE = 0x0F
    # Status Messages
    STATUS_STATE = 0x10
    STATUS_CFG_GET = 0x11
    STATUS_CFG_SET = 0x12
    # Settings Messages
    SETTINGS_GET   = 0x15
    SETTINGS_SET   = 0x16
    SETTINGS_LOAD  = 0x17
    SETTINGS_SAVE  = 0x18
    SETTINGS_RESET = 0x19
    # Calibration Messages
    CAL_ACTION   = 0x20
    AHRS_CAL_GET = 0x21
    AHRS_CAL_SET = 0x22
    # Acoustic Transceiver Messages
    XCVR_ANALYSE = 0x30
    XCVR_TX_MSG  = 0x31
    XCVR_RX_ERR  = 0x32
    XCVR_RX_MSG  = 0x33
    XCVR_RX_REQ  = 0x34
    XCVR_RX_RESP = 0x35
    XCVR_RX_UNHANDLED = 0x37
    XCVR_USBL = 0x38
    XCVR_FIX = 0x39
    XCVR_STATUS = 0x3A
    # PING Protocol Messages
    PING_SEND = 0x40
    PING_REQ  = 0x41
    PING_RESP = 0x42
    PING_ERROR = 0x43
    # ECHO Protocol Messages
    ECHO_SEND  = 0x48
    ECHO_REQ   = 0x49
    ECHO_RESP  = 0x4A
    ECHO_ERROR = 0x4B
    # NAV Protocol Messages
    NAV_QUERY_SEND     = 0x50
    NAV_QUERY_REQ      = 0x51
    NAV_QUERY_RESP     = 0x52
    NAV_ERROR          = 0x53
    NAV_QUEUE_SET      = 0x58
    NAV_QUEUE_CLR      = 0x59
    NAV_QUEUE_STATUS   = 0x5A
    NAV_STATUS_SEND    = 0x5B
    NAV_STATUS_RECEIVE = 0x5C
    # DAT Protocol Messages
    DAT_SEND         = 0x60
    DAT_RECEIVE      = 0x61
    DAT_ERROR        = 0x63
    DAT_QUEUE_SET    = 0x64
    DAT_QUEUE_CLR    = 0x65
    DAT_QUEUE_STATUS = 0x66

class CST_E(IntEnum):
    # General Status Codes
    CST_OK           = 0x00
    CST_FAIL         = 0x01
    CST_EEPROM_ERROR = 0x03
    # Command Processor Status Codes
    # Firmware Programming Status Codes
    # Acoustic Transceiver Status Codes
    CST_XCVR_BUSY          = 0x30
    CST_XCVR_ID_REJECTED   = 0x31
    CST_XCVR_CSUM_ERROR    = 0x32
    CST_XCVR_LENGTH_ERROR  = 0x33
    CST_XCVR_RESP_TIMEOUT  = 0x34
    CST_XCVR_RESP_ERROR    = 0x35
    CST_XCVR_RESP_WRONG    = 0x36
    CST_XCVR_PLOAD_ERROR   = 0x37
    CST_XCVR_STATE_STOPPED = 0x3A
    CST_XCVR_STATE_IDLE    = 0x3B
    CST_XCVR_STATE_TX      = 0x3C
    CST_XCVR_STATE_REQ     = 0x3D
    CST_XCVR_STATE_RX      = 0x3E
    CST_XCVR_STATE_RESP    = 0x3F
    
#=====================================
#    Structure definitions
#=====================================
@dataclass
class ACOMSG_T:
    MSG_DEST_ID : BID_E
    msg_src_id  : BID_E
    msg_typp    : AMSGTYPE_E
    msg_depth   : int
    MSG_PAYLOAD_ID: APAYLOAD_E
    MSG_PAYLOAD_LEN: int
    MSG_PAYLOAD : int=0

@dataclass
class ACOFIX_T:
    dest_id: int = BID_E
    src_id : int = BID_E
    flags  : int = 0
    msg_type: AMSGTYPE_E = AMSGTYPE_E.MSG_UNKNOWN
    attitude_yaw : int = 0
    attitude_pitch : int = 0
    attitude_roll : int = 0
    depth_local : int = 0
    vos : int = 0
    rssi : int = 0
    range_count : int = 0
    range_time  : int = 0
    range_dist  : int = 0
    USBL_CHANNELS: int = 0
    USBL_RSSI: int = 0
    USBL_AZIMUTH: int = 0
    USBL_ELEVATION: int = 0
    USBL_FIT_ERROR: int = 0
    POSITION_EASTING: int = 0
    POSITION_NORTHING: int = 0
    POSITION_DEPTH: int = 0
    def build(self, data):
        self.dest_id = BID_E( interpretData(data,1) ).name
        self.src_id  = BID_E( interpretData(data,1) ).name
        self.flags   = interpretData(data,1)
        self.msg_type = AMSGTYPE_E( interpretData(data,1) ).name
        self.attitude_yaw = interpretData(data,2)/10
        self.attitude_pitch = interpretData(data,2)/10
        self.attitude_roll = interpretData(data,2)/10
        self.depth_local = interpretData(data,2)
        self.vos = interpretData(data,2)/10
        self.rssi = interpretData(data,2)/10
        if(self.flags & 0x01):
            self.range_count = interpretData(data,4)
            self.range_time  = interpretData(data,4)/10000000
            self.range_dist  = interpretData(data,2)/10
        if(self.flags & 0x02):
            self.USBL_CHANNELS = interpretData(data,1)
            self.USBL_RSSI = [interpretData(data,2)/10 for i in range(self.USBL_CHANNELS)]
            self.USBL_AZIMUTH = interpretData(data,2)/10
            self.USBL_ELEVATION = interpretData(data,2,signed=True)/10
            self.USBL_FIT_ERROR = interpretData(data,2)/100
        if(self.flags & 0x04):
            self.POSITION_EASTING = interpretData(data,2)/10
            self.POSITION_NORTHING = interpretData(data,2,signed=True)/10
            self.POSITION_DEPTH = interpretData(data,2)/10
        if(self.flags & 0x08):
            print('POS ENHACED')
        if(self.flags & 0x10):
            print('POS FILTER')
    
@dataclass
class Hardware_T:
    partNumber:     int=0
    partRevision:   int=0
    serialNumber:   int=0
    flagsSys:       int=0
    flagsUser:      int=0
    def build(self,data):
        self.partNumber = interpretData(data,2)
        self.partRevision = interpretData(data,1)
        self.serialNumber = interpretData(data,4)
        self.flagsSys     = interpretData(data,2)
        self.flagsUser    = interpretData(data,2)

@dataclass
class Firmware_T:
    valid:          bool=False
    partNumber:     int=0
    versionMajor:   int=0
    versionMinor:   int=0
    versionBuild:   int=0
    checksum:       int=0
    def build(self,data):
        self.valid = interpretData(data,1)
        self.partNumber = interpretData(data,2)
        self.versionMajor = interpretData(data,1)
        self.versionMinor = interpretData(data,1)
        self.versionBuild = interpretData(data,2)
        self.checksum = interpretData(data,4)

@dataclass
class Sys_Info:
    cmd_name: str = 'SYS_INFO'
    cmd_ID: int=0
    uptime: int=0
    section: int=0
    hardware: Hardware_T=Hardware_T()
    boot: Firmware_T=Firmware_T()
    app: Firmware_T=Firmware_T()
    board_rev: int=0

#=====================================
#    Command ID Definition
#=====================================
def interpretData(data, length, signed = False):
    if length == 1:
        res = int(data[:2],16)
    else:
        res = int.from_bytes(bytes.fromhex( data[:length*2].decode() ), 'little', signed=signed)
    del data[:length*2]
    return res

#def interpretData(data):
#    return int.from_bytes(bytes.fromhex( data.decode() ), 'little', signed=False)

@task
def x02(data):
    MSG_ID = data[:2]
    del data[:2]
    UPTIME = interpretData(data,4)
    SECTION = interpretData(data,1)
    HARDWARE = Hardware_T()
    HARDWARE.build(data)
    BOOT_FIRMWARE = Firmware_T()
    BOOT_FIRMWARE.build(data)
    MAIN_FIRMWARE = Firmware_T()
    MAIN_FIRMWARE.build(data)
    board_rev = interpretData(data,1)
    print(f'{MSG_ID} {UPTIME} {SECTION}')
    print(HARDWARE)
    print(BOOT_FIRMWARE)
    print(MAIN_FIRMWARE)
    print(f'Board Revision: {board_rev}')
    #return s

@task
def x10(data):
    MSG_ID = data[:2]
    del data[:2]
    output_flags = interpretData(data,1)
    TIMESTAMP = interpretData(data,8)/1000
    print(f'OutputFlags = {output_flags}')
    print(f'TIMESTAMP = {TIMESTAMP}')
    # Environmental Fields
    if (output_flags & (1<<0)):
        ENV_SUPPLY = interpretData(data,2)/1000
        ENV_TEMP = interpretData(data,2)/10
        ENV_PRESSURE = interpretData(data,4)/1000
        ENV_DEPTH = interpretData(data,4)/10
        ENV_VOS = interpretData(data,2)/10
        print(f'ENV_SUPPLY = {ENV_SUPPLY}')
        print(f'ENV_TEMP = {ENV_TEMP}')
        print(f'ENV_PRESSURE = {ENV_PRESSURE}')
        print(f'ENV_DEPTH = {ENV_DEPTH}')
        print(f'ENV_VOS = {ENV_VOS}')
    # Attitude Fields
    if (output_flags & (1<<1)):
        ATT_YAW = interpretData(data,2,signed = True)/10
        ATT_PITCH = interpretData(data,2,signed = True)/10
        ATT_ROLL = interpretData(data,2,signed = True)/10
        print(f'ATT_YAW = {ATT_YAW}')
        print(f'ATT_PITCH = {ATT_PITCH}')
        print(f'ATT_ROLL = {ATT_ROLL}')
    # Magnetometer Calibration and Status Fields
    if (output_flags & (1<<2)):
        MAG_CAL_BUF = interpretData(data,1)
        MAG_CAL_VALID = bool(interpretData(data,1))
        MAG_CAL_AGE = interpretData(data,4)
        MAG_CAL_FIT = interpretData(data,1)
        print(f'MAG_CAL_BUF = {MAG_CAL_BUF}')
        print(f'MAG_CAL_VALID = {MAG_CAL_VALID}')
        print(f'MAG_CAL_AGE = {MAG_CAL_AGE}')
        print(f'MAG_CAL_FIT = {MAG_CAL_FIT}')

    # Accelerometer Calibration Fields
    if (output_flags & (1<<3)):
        ACC_LIM_MIN_X = interpretData(data,2,signed = True)
        ACC_LIM_MIN_Y = interpretData(data,2,signed = True)
        ACC_LIM_MIN_Z = interpretData(data,2,signed = True)
        ACC_LIM_MAX_X = interpretData(data,2,signed = True)
        ACC_LIM_MAX_Y = interpretData(data,2,signed = True)
        ACC_LIM_MAX_Z = interpretData(data,2,signed = True)
        print(f'ACC_LIM_MIN_X = {ACC_LIM_MIN_X}')
        print(f'ACC_LIM_MIN_Y = {ACC_LIM_MIN_Y}')
        print(f'ACC_LIM_MIN_Z = {ACC_LIM_MIN_Z}')
        print(f'ACC_LIM_MAX_X = {ACC_LIM_MAX_X}')
        print(f'ACC_LIM_MAX_Y = {ACC_LIM_MAX_Y}')
        print(f'ACC_LIM_MAX_Z = {ACC_LIM_MAX_Z}')
         
    # Raw AHRS Sensor Data Fields
    if (output_flags & (1<<4)):
        AHRS_RAW_ACC_X = interpretData(data,2,signed = True)
        AHRS_RAW_ACC_Y = interpretData(data,2,signed = True)
        AHRS_RAW_ACC_Z = interpretData(data,2,signed = True)
        AHRS_RAW_MAG_X = interpretData(data,2,signed = True)
        AHRS_RAW_MAG_Y = interpretData(data,2,signed = True)
        AHRS_RAW_MAG_Z = interpretData(data,2,signed = True)
        AHRS_RAW_GYRO_X = interpretData(data,2,signed = True)
        AHRS_RAW_GYRO_Y = interpretData(data,2,signed = True)
        AHRS_RAW_GYRO_Z = interpretData(data,2,signed = True)
        print(f'AHRS_RAW_ACC_X = {AHRS_RAW_ACC_X}')
        print(f'AHRS_RAW_ACC_Y = {AHRS_RAW_ACC_Y}')
        print(f'AHRS_RAW_ACC_Z = {AHRS_RAW_ACC_Z}')
        print(f'AHRS_RAW_MAG_X = {AHRS_RAW_MAG_X}')
        print(f'AHRS_RAW_MAG_Y = {AHRS_RAW_MAG_Y}')
        print(f'AHRS_RAW_MAG_Z = {AHRS_RAW_MAG_Z}')
        print(f'AHRS_RAW_GYRO_X = {AHRS_RAW_GYRO_X}')
        print(f'AHRS_RAW_GYRO_Y = {AHRS_RAW_GYRO_Y}')
        print(f'AHRS_RAW_GYRO_Z = {AHRS_RAW_GYRO_Z}')
        
    # Compensated AHRS Sensor Data Fields
    if (output_flags & (1<<5)):
        pass
        

@task
def x39(data):
    MSG_ID = data[:2]
    del data[:2]
    aco_fix = ACOFIX_T()
    aco_fix.build(data)
    return aco_fix

@task
def x3A(data):
    MSG_ID = data[:2]
    del data[:2]
    STATUS = CST_E( interpretData(data,1) )
    print(f'Transceiver Status: {STATUS.name}')
    return STATUS

@task
def x40(data):
    MSG_ID = data[1:3]
    STATUS = data[3:5]
    BEACON_ID = data[5:7]
    print(f'MSG_ID: {MSG_ID} STATUS: {STATUS} BEACON_ID:{BEACON_ID}')

@task
def x42(data):
    MSG_ID = data[:2]
    del data[:2]
    aco_fix = ACOFIX_T()
    aco_fix.build(data)
    return aco_fix


def send_data(data):
    crc = crc16(data)
    buff = '#' + data.hex().upper() + crc.to_bytes(2,'little').hex()+'\r\n'
    #print(buff)
    usbl.write(buff.encode())
    
def process_data(data):
    cmd_id = data[1:3]
    crc_rcv = interpretData(bytearray(data[-6:]),2)
    payload = bytearray(data[1:-6])

    # Calculate CRC from received data payload
    crc = crc16(bytes.fromhex( payload.decode() ))

    print(f'data: {data}')
    if crc == crc_rcv:
        print(f'Received data: {data[:-2]}')
        print(f'Received {CID_E(int(cmd_id,16)).name} command')
        test = tasks.get(payload[:2].decode(), lambda x: 'command not recognized')(payload)
        print(test)
        print('')
    else:
        print('CRC not valid')
    return test
        
def check_transceiver_state():
    CmdId = CID_E.XCVR_STATUS
    data = bytes([CmdId])
    send_data(data)
    
    data = usbl.read_until(b'\n')
    return process_data(data)

def is_transceiver_idle():
    state = check_transceiver_state()
    return state == CST_E.CST_XCVR_STATE_IDLE
    
    
#transmit a ping: '#4002040177\r\n'
def ping_beacon(dest_id):
    if (is_transceiver_idle()):
        #if True:
        CmdId = CID_E.PING_SEND
        MSGType = AMSGTYPE_E.MSG_REQU
        data = bytes([CmdId, dest_id, MSGType])
        crc = crc16(data)
        # print(f'Pinging device ID: {dest_id}\n')
        # print('#' + data.hex() + crc.to_bytes(2,'little').hex())
        # return ('#' + data.hex() + crc.to_bytes(2,'little').hex()+'\r\n')
    else:
        print('Cannot ping, Transceiver not in idle state')


usbl = serial.Serial('COM5', 115200, timeout=10.5)

# Transmit SYS_INFO
#usbl.write('#0281C1\r\n'.encode())

# Transmit PING_SENG
usbl.write('#4002040177\r\n'.encode())
#usbl.write(ping_beacon(2).encode())

ping_timer = time()
while(1):
    if (usbl.in_waiting):
        data = usbl.read_until(b'\n')
        process_data(data)
    else:
        sleep(0.01)
        act_time = time()
        if (act_time - ping_timer > ping_delay):
            ping_timer = act_time

    

"""for i in range(5):
    data = usbl.read_until(b'\n')
#data = b'$026D04000001D103066E3D000000000000FF90030106B40100000000FF910302028F088E78237D106A6D\r\n'
#data = b'$028900000001D103066E3D000000000000FF90030106B40100000000FF910302028F088E78237D10927C\r\n'
#data = b'$39020F81030000000000000000AC3BD604DE0300003D4900000E0019DE\r\n'
#data = b'$390F0287050D0AB0003D030000BD3BED04E6030000C55C00001200042A05EB04E7041B05AF04A1FED9020400F9FF0F009AE6\r\n'
    #data = b'$4000028015\r\n'
#data = b'$420F0287050D0AB0003B030000BD3BED04E3030000725500001000042A05EB04E4041D05AF04A1FEDC020400F9FF0E00D752\r\n'

    cmd_id = data[1:3]
    crc_rcv = interpretData(bytearray(data[-6:]),2)
    payload = bytearray(data[1:-6])


    # Calculate CRC from received data payload
    crc = crc16(bytes.fromhex( payload.decode() ))

    if crc == crc_rcv:
        print(f'Received data: {data[:-2]}')
        print(f'Received {CID_E(int(cmd_id,16)).name} command')
        test = tasks.get(payload[:2].decode(), lambda x: 'command not recognized')(payload)
    else:
        print('CRC not valid')
    print(test)

"""
