import serial
from . import crc


class Pzem:
    ADDR_DEFAULT = 0xF8
    # Read Input Registers
    CMD_RIR = 0x04
    CMD_RIR_ERR = 0x84
    # Reset.
    CMD_RST = 0x42
    CMD_RST_ERR = 0xC2
    ERR_CODE = {0x01: 'illegal function',
                0x02: 'illegal address',
                0x03: 'illegal data',
                0x04: 'slave error'}

    def __init__(self, dev, addr=ADDR_DEFAULT):
        self.addr = addr
        self.ser = serial.Serial(port=dev, baudrate=9600,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=3)

    def stats(self):
        def read_body():
            n = self.readn(1)[0]
            return self.readn(n)

        self.write([self.addr, Pzem.CMD_RIR, 0x00, 0x00, 0x00, 0x0A])
        data = self.read(Pzem.CMD_RIR, Pzem.CMD_RIR_ERR, read_body)

        return {'voltage': int.from_bytes(data[0:2],
                                          byteorder='big') / 10,
                'current': int.from_bytes(data[4:6] + data[2:4],
                                          byteorder='big') / 1000,
                'power': int.from_bytes(data[8:10] + data[6:8],
                                        byteorder='big') / 10,
                'energy': int.from_bytes(data[12:14] + data[10:12],
                                         byteorder='big'),
                'frequency': int.from_bytes(data[14:16],
                                            byteorder='big') / 10,
                'factor': int.from_bytes(data[16:18],
                                         byteorder='big') / 100,
                'alarm': int.from_bytes(data[18:20],
                                        byteorder='big') == 0xFFFF}

    def reset(self):
        def read_body():
            pass

        self.write([self.addr, Pzem.CMD_RST])
        self.read(Pzem.CMD_RST, Pzem.CMD_RST_ERR, read_body)

    def close(self):
        self.ser.close()

    def read(self, cmd, err, read_body):
        addr = self.readn(1)[0]
        if addr != self.addr:
            raise IOError(0, 'expected response for addr {}, got {}'
                          .format(self.addr, addr))

        rcmd = self.readn(1)[0]
        if rcmd != cmd:
            if rcmd == err:
                e = self.readn(1)[0]
                # Consume CRC.
                self.readn(2)
                raise IOError(e, Pzem.ERR_CODE.get(e, 'unknown error'))
            else:
                raise IOError(rcmd,
                              Pzem.ERR_CODE.get(e, 'invalid response'))

        b = read_body()
        # Consume CRC.
        self.readn(2)

        return b

    def write(self, buf):
        c = crc.crc16(buf)
        buf += [c & 0xFF]
        buf += [(c >> 8) & 0xFF]
        buflen = len(buf)

        n = self.ser.write(bytes(buf))
        if n != buflen:
            raise IOError(0, 'wrote only {} of {}'.format(n, buflen))

    def readn(self, n):
        b = self.ser.read(n)
        if len(b) == 0:
            raise IOError(0, 'read timed out')

        return b
