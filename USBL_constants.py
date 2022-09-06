from enum import IntEnum
import struct
import crcmod

# Funcion para obtener CRC
crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0x0000)

#=====================================
#    Decoding functions
#=====================================
def interpretData(data: bytearray, type: str):
    """ Extrae un número de bytes de la cadena según el tipo 
    y los decodifica. Tipos:
        <b  int8        <B  uint8
        <h  int16       <H  uint16
        <i  int32       <I  uint32
        <q  int64       <Q  uint64 
        <f  float       <d  double
    """
    length = struct.calcsize(type)
    data_str = data.decode() 
    data_bytes = bytes.fromhex(data_str[:length*2])
    del data[:length*2]
    return struct.unpack(type,data_bytes)[0]

def checksum(msg: bytearray):
    """ Calcula el checksum con crc16 para el comando y lo
    compara con el recibido en la cadena de bytes """
    received_csum = interpretData(msg[-6:-2],'<H')
    cmd = msg[:-6]
    cmd_bytes = bytes.fromhex(cmd.decode())
    csum = crc16(cmd_bytes)
    return csum == received_csum

def decode_status_state(response: bytearray):
    """ Decodifica la respuesta de estado y muestra parámetros de interés """
    status_flags = interpretData(response,'<B')
    print(f'OutputFlags = {status_flags:b}')
    timestamp = interpretData(response,'<Q')/1000
    print(f'Timestamp = {timestamp}')
    status = STATUS_T(status_flags,response)
    print(f'Status = {status.__dict__}')

def decode_ping_resp(response: bytearray):
    """ Decodifica la respuesta de estado """
    acofix = ACOFIX_T(response)
    print(f'Velocity of sound = {acofix.VOS}m/s')
    print(f'Position North = {acofix.POSITION_NORTHING}m')
    print(f'Position East = {acofix.POSITION_EASTING}m')
    print(f'Position Depth = {acofix.POSITION_DEPTH}m')
    print(f'ACOFIX object = {acofix.__dict__}')

def decode_ping_send(response: bytearray):
    status = interpretData(response,'<B')
    print(f'Status = {CST_E(status).name}')
    id = interpretData(response,'<B')
    print(f'Beacon ID = {BID_E(id).name}')

def decode_ping_error(response: bytearray):
    status = interpretData(response,'<B')
    print(f'Error = {CST_E(status).name}')
    id = interpretData(response,'<B')
    print(f'Beacon ID = {BID_E(id).name}')

#=====================================
#    Encoding functions
#=====================================
def append_checksum(msg: bytearray):
    """ Añade el checksum al bytearray msg """
    payload = msg.decode()[1:]
    cmd_bytes = bytes.fromhex(payload)
    csum = crc16(cmd_bytes)
    appendData(msg,csum,'<H')
    return msg

def appendData(msg: bytearray, data, type: str):
    """ Decodifica un dato dependiendo del tipo y lo
    añade al bytearray msg. Tipos:
        <b  int8        <B  uint8
        <h  int16       <H  uint16
        <i  int32       <I  uint32
        <q  int64       <Q  uint64 
        <f  float       <d  double
    """
    data_packed = struct.pack(type,data)
    data_str = data_packed.hex()
    msg.extend(bytearray(data_str.encode()))

def encode_ping_send(msg:bytearray, id):
    cid = CID_E.PING_SEND.value
    dest_id = BID_E(id).value
    msg_type = AMSGTYPE_E.MSG_REQU
    appendData(msg,cid,'<B')
    appendData(msg,dest_id,'<B')
    appendData(msg,msg_type,'<B')
    append_checksum(msg)
    msg.extend(b'\r\n')
    return msg

#=====================================
#    Structures
#=====================================
class ACOFIX_T:
    def __init__(self,data) -> None:
        self.DEST_ID = BID_E( interpretData(data,'<B') ).name
        self.SRC_ID  = BID_E( interpretData(data,'<B') ).name
        self.FLAGS   = interpretData(data,'<B')
        self.MSG_TYPE = AMSGTYPE_E( interpretData(data,'<B') ).name
        self.ATTITUDE_YAW = interpretData(data,'<h')/10
        self.ATTITUDE_PITCH = interpretData(data,'<h')/10
        self.ATTITUDE_ROLL = interpretData(data,'<h')/10
        self.DEPTH_LOCAL = interpretData(data,'<H')
        self.VOS = interpretData(data,'<H')/10
        self.RSSI = interpretData(data,'<h')/10
        if(self.FLAGS & 0x01):
            self.RANGE_COUNT = interpretData(data,'<I')
            self.RANGE_TIME  = interpretData(data,'<i')/10000000
            self.RANGE_DIST  = interpretData(data,'<H')/10
        if(self.FLAGS & 0x02):
            self.USBL_CHANNELS = interpretData(data,'<B')
            self.USBL_RSSI = [interpretData(data,'<h')/10 for i in range(self.USBL_CHANNELS)]
            self.USBL_AZIMUTH = interpretData(data,'<h')/10
            self.USBL_ELEVATION = interpretData(data,'<h')/10
            self.USBL_FIT_ERROR = interpretData(data,'<h')/100
        if(self.FLAGS & 0x04):
            self.POSITION_EASTING = interpretData(data,'<h')/10
            self.POSITION_NORTHING = interpretData(data,'<h')/10
            self.POSITION_DEPTH = interpretData(data,'<h')/10
        if(self.FLAGS & 0x08):
            print('POS ENHACED')
        if(self.FLAGS & 0x10):
            print('POS FILTER ERROR')

class STATUS_T:
    def __init__(self,status_flags,response):
         # Environmental Fields
        if (status_flags & (1<<0)):
            self.ENV_SUPPLY = interpretData(response,'<H')/1000
            self.ENV_TEMP = interpretData(response,'<h')/10
            self.ENV_PRESSURE = interpretData(response,'<i')/1000
            self.ENV_DEPTH = interpretData(response,'<i')/10
            self.ENV_VOS = interpretData(response,'<H')/10
        # Attitude Fields
        if (status_flags & (1<<1)):
            self.ATT_YAW = interpretData(response,'<h')/10
            self.ATT_PITCH = interpretData(response,'<h')/10
            self.ATT_ROLL = interpretData(response,'<h')/10
        # Magnetometer Calibration and Status Fields
        if (status_flags & (1<<2)):
            self.MAG_CAL_BUF = interpretData(response,'<B')
            self.MAG_CAL_VALID = bool(interpretData(response,'<B'))
            self.MAG_CAL_AGE = interpretData(response,'<I')
            self.MAG_CAL_FIT = interpretData(response,'<B')
        # Accelerometer Calibration Fields
        if (status_flags & (1<<3)):
            self.ACC_LIM_MIN_X = interpretData(response,'<h')
            self.ACC_LIM_MIN_Y = interpretData(response,'<h')
            self.ACC_LIM_MIN_Z = interpretData(response,'<h')
            self.ACC_LIM_MAX_X = interpretData(response,'<h')
            self.ACC_LIM_MAX_Y = interpretData(response,'<h')
            self.ACC_LIM_MAX_Z = interpretData(response,'<h')
        # Raw AHRS Sensor Data Fields
        if (status_flags & (1<<4)):
            self.AHRS_RAW_ACC_X = interpretData(response,'<h')
            self.AHRS_RAW_ACC_Y = interpretData(response,'<h')
            self.AHRS_RAW_ACC_Z = interpretData(response,'<h')
            self.AHRS_RAW_MAG_X = interpretData(response,'<h')
            self.AHRS_RAW_MAG_Y = interpretData(response,'<h')
            self.AHRS_RAW_MAG_Z = interpretData(response,'<h')
            self.AHRS_RAW_GYRO_X = interpretData(response,'<h')
            self.AHRS_RAW_GYRO_Y = interpretData(response,'<h')
            self.AHRS_RAW_GYRO_Z = interpretData(response,'<h')
        # Compensated AHRS Sensor Data Fields
        if (status_flags & (1<<5)):
            self.AHRS_COMP_ACC_X = interpretData(response,'<f')
            self.AHRS_COMP_ACC_Y = interpretData(response,'<f')
            self.AHRS_COMP_ACC_Z = interpretData(response,'<f')
            self.AHRS_COMP_MAG_X = interpretData(response,'<f')
            self.AHRS_COMP_MAG_Y = interpretData(response,'<f')
            self.AHRS_COMP_MAG_Z = interpretData(response,'<f')
            self.AHRS_COMP_GYRO_X = interpretData(response,'<f')
            self.AHRS_COMP_GYRO_Y = interpretData(response,'<f')
            self.AHRS_COMP_GYRO_Z = interpretData(response,'<f')

#=====================================
#    Enumerations
#=====================================
class AMSGTYPE_E(IntEnum):
    MSG_OWAY    = 0x0
    MSG_OWAYU   = 0x1
    MSG_REQ     = 0x2
    MSG_RESP    = 0x3
    MSG_REQU    = 0x4
    MSG_RESPU   = 0x5
    MSG_REQX    = 0x6
    MSG_RESPX   = 0x7
    MSG_UNKNOWN = 0xFF

class BID_E(IntEnum):
    BEACON_ALL  = 0x00
    BEACON_ID_1 = 0x01
    BEACON_ID_2 = 0x02
    BEACON_ID_3 = 0x03
    BEACON_ID_4 = 0x04
    BEACON_ID_5 = 0x05
    BEACON_ID_6 = 0x06
    BEACON_ID_7 = 0x07
    BEACON_ID_8 = 0x08
    BEACON_ID_9 = 0x09
    BEACON_ID_10 = 0x0A
    BEACON_ID_11 = 0x0B
    BEACON_ID_12 = 0x0C
    BEACON_ID_13 = 0x0D
    BEACON_ID_14 = 0x0E
    BEACON_ID_15 = 0x0F

class APAYLOAD_E(IntEnum):
    PLOAD_PING = 0x00
    PLOAD_ECHO = 0x01
    PLOAD_NAV  = 0x02
    PLOAD_DAT  = 0x03
    PLOAD_DEX  = 0x04

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