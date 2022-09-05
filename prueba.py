from USBL_constants import *
from time import time, sleep
import serial

from USBL_constants import *

PORT = 'COM9'
#ser = serial.Serial(PORT, 115200, timeout=None)
#ser.write(b'#10000DC0\r\n')    # Pregunta estado
if True:
    #ser.read_until(b'$')               # Inicio de mensaje
    #msg = ser.read_until(b'\r\n')      # Final de mensaje
    msg = b'10078D48100000000000B930C2000800000000000000480DE3FD0DFD320303FF2B0400005EF273\r\n'
    print(f'\nReceived message = {msg[:-2].decode()}')
    msg = bytearray(msg)
    if checksum(msg):
        # El mensaje ha sido recibido correctamente
        cid_bytes = msg[:2]
        cid = CID_E(int(cid_bytes,16))
        response = msg[2:]
        print(f'Received {cid.name} command')
        if cid == CID_E.STATUS_STATE:
            decode_status_state(response)
        elif cid == CID_E.PING_RESP:
            decode_ping_resp(response)
    else:
        print('Checksum error: Command was not correctly received')
