from USBL_constants import *
import serial, time

PORT = 'COM5'
ser = serial.Serial(PORT, 115200, timeout=1)

print(f'Sending CID_STATUS command')
ser.write(b'#10000DC0\r\n')

def pinging():
    print(f'\nSending PING_SEND command')
    msg = bytearray(b'#')
    encode_ping_send(msg,2)
    ser.write(msg)

start = time.time()
while True:
    msg = ser.read_until(b'\r\n')      # Final de mensaje
    if msg:
        print(f'\nReceived message = {msg[:-2].decode()}')
        msg = bytearray(msg[1:])
        if checksum(msg):
            # El mensaje ha sido recibido correctamente
            cid_num = interpretData(msg,'<B')
            cid = CID_E(cid_num)
            print(f'Received {cid.name} command')
            if cid == CID_E.STATUS_STATE:
                decode_status_state(msg)
            elif cid == CID_E.PING_RESP:
                decode_ping_resp(msg)
            elif cid == CID_E.PING_SEND:
                decode_ping_send(msg)
            elif cid == CID_E.PING_ERROR:
                decode_ping_error(msg)
            else:
                print('Cannot interpret command payload')
        else:
            print('Checksum error: Command was not correctly received')

    current = time.time()
    if current - start > 5:
        pinging()
        start = time.time()
    time.sleep(0.1)

    
