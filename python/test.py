#!/usr/bin/env python3
#
# python driver for I2C sensors using Bus Pirate
# verify/test sensors, prototype reading with I2C
#
# http://dangerousprototypes.com/docs/Bus_Pirate
# https://github.com/juhasch/pyBusPirateLite
# https://pybuspiratelite.readthedocs.io/en/latest/
#
# Wiring Bus Pirate to sensor pcb:
#   |PIN  |WIRE     |SENSOR_PCB
#   |GND  |(brown)  |GND
#   |5V   |(pink)   |5V
#   |MOSI |(gray)   |SDA
#   |CLK  |(purple) |SCL

import sys, time
import traceback

sys.path.append('pyBusPirateLite')
import pyBusPirateLite

class BusPirate(object):
    def __init():
        self.i2c = None
        self.bb = None

    def startup(self):
        '''detect serial port of bus pirate'''
        self.bb = pyBusPirateLite.BitBang(connect=False)
        port = self.bb.get_port()
        print('port is: ',port)
        if not port:
            sys.exit(1)
        self.bb.connect()
        '''i2c init'''
        self.i2c = pyBusPirateLite.I2C()
        self.i2c.speed = '400kHz'
        self.i2c.configure(power=True)
        #self.i2c.start()

    def shutdown(self):
        ''' shutdown bus pirate'''
        self.i2c.configure(power=False, pullup=False, aux=False, cs=False)
        self.i2c.hw_reset() # reset BIN mode, back into CLI mode
        self.i2c.disconnect() # release serial port
        print("shutdown")

    def test_mcp(self):
        ''' 0x30(0x18 W) 0x31(0x18 R)  -- MCP9804
        Pointer bits
        0000 =RFU, Reserved for Future Use (Read-Only register)
        0001 =Configuration register (CONFIG)
        0010 =Alert Temperature Upper Boundary Trip register (T UPPER )
        0011 =Alert Temperature Lower Boundary Trip register (T LOWER )
        0100 =Critical Temperature Trip register (T CRIT )
        0101 =Temperature register (T A )
        0110 =Manufacturer ID register
        0111 =Device ID/Revision register
        1000 =Resolution register
        -- read Manuf ID, Device ID, Resolution
        I2C>[0x30 6 [0x31 r r]  ==> 0x00 0x54
        I2C>[0x30 7 [0x31 r r]  ==> 0x02 0x01
        I2C>[0x30 8 [0x31 r]    ==> 0x03
        -- read temperature register
        I2C>[0x30 5][0x31 r r]
        READ: 0xC1
        READ:  ACK 0xAE
        ...
        READ: 0xC1
        READ:  ACK 0xB0
        '''
        wr = 0x18 << 1
        rd = 0x18 << 1 | 1
        print(f'MCP addr wr={hex(wr)}  rd={hex(rd)} ')
        self.i2c.write_then_read(2,0, [wr, 0x05])  # set temp register
        [msb,lsb] = self.i2c.write_then_read(1,2, [rd])        # read 2 bytes from register
        #print(f'MCP 0={hex(msb)}  1={hex(lsb)} ')
        msb &= 0x1F # clear flag bits
        #tmph = (msb<<6 | lsb>>2)
        tempc = msb*16 + lsb/16.0
        tempf = 32 + tempc * 1.8
        print(f'MCP msb={bin(msb)}  lsb={bin(lsb)} ')
        print(f'MCP tempc={tempc} tempf={tempf}  0={hex(msb)}  1={hex(lsb)} ')
        return tempc

    def test_si(self):
        ''' 0x80(0x40 W) 0x81(0x40 R)  -- Si7051
        '''
        wr = 0x40 << 1
        rd = 0x40 << 1 | 1
        print(f'Si7051 addr wr={hex(wr)}  rd={hex(rd)} ')

        self.i2c.write_then_read(2,0, [wr, 0xfe])  # FE = reset (datasheet pg. 14)
        time.sleep(0.01)

        self.i2c.start()
        self.i2c.transfer([wr, 0x84, 0xb8])  # read FIRMWARE Revision
        # no STOP, just Sr (Repeated Start)
        #print("read firmware")
        [fw] = self.i2c.write_then_read(1,1, [rd]) # read 1 byte
        print(f'Si7051 firmware={hex(fw)} ff=v1.0 20=v2.0')
        #self.i2c.stop()
        #print("done firmware")

        self.i2c.start()
        self.i2c.transfer([wr, 0xfa, 0x0f])  #  ELECTRONIC SERIAL NUMBER (pg 15)
        # no STOP, just Sr (Repeated Start)
        #print("read sna")
        sna = self.i2c.write_then_read(1,8, [rd]) # read SNA
        print(f'Si7051 SNA3={hex(sna[0])}  SNA3_crc={hex(sna[1])} ')
        print(f'Si7051 SNA2={hex(sna[2])}  SNA2_crc={hex(sna[3])} ')
        print(f'Si7051 SNA1={hex(sna[4])}  SNA1_crc={hex(sna[5])} ')
        print(f'Si7051 SNA0={hex(sna[6])}  SNA0_crc={hex(sna[7])} ')
        #print("done sna")
        # TODO use CRC to verify bytes (pg.13)
        # poly: 0x31 (x^8 + x^5 + x^4 + 1)  init=0x00
        self.i2c.start()
        self.i2c.transfer([wr, 0xfc, 0xc9])
        # no STOP, just Sr (Repeated Start)
        #print("read snb")
        snb = self.i2c.write_then_read(1,6, [rd]) # read SNB
        print(f'Si7051 SNB3={hex(snb[0])}  SNB3_crc={hex(snb[1])}  ')
        print(f'Si7051 SNB2={hex(snb[2])}  SNB2_crc={hex(snb[3])}  ')
        print(f'Si7051 SNB1={hex(snb[4])}  SNB1_crc={hex(snb[5])}  ')
        print(f'Si7051 SNB3={snb[0]} (51=Si7051)')
        #print("done snb")

        #print("reset")
        #self.i2c.write_then_read(2,0, [wr, 0xfe])  # FE = reset (datasheet pg. 14)
        #time.sleep(0.01)

        #print("read UR1")
        self.i2c.start()
        self.i2c.transfer([wr, 0xe7])
        # no STOP, just Sr (Repeated Start)
        [ur] = self.i2c.write_then_read(1,1, [rd])  # E7 = read User Register 1 (datasheet pg. 14)
        res = (ur&0x80)>>7 | (ur&0x01)
        vdds = (ur&0x40)>>6
        print(f'Si7051 user reg1 ={hex(ur)} res={hex(res)} (0=14-bit resolution) vdds={hex(vdds)} (0=OK) ')

        #print("read temp")
        self.i2c.start()
        #self.i2c.transfer([wr, 0xe3])   # E3 = get temp, hold master mode
        self.i2c.transfer([wr, 0xf3])   # F3 = get temp, NO hold master mode
        time.sleep(0.01)
        # no STOP, just Sr (Repeated Start)
        [msb,lsb] = self.i2c.write_then_read(1,2, [rd])  # set temp value
        tmp = msb<<8 | lsb
        print(f'Si7051 msb={hex(msb)}  lsb={hex(lsb)}  ')
        print(f'Si7051 tmp={hex(tmp)}  tmp={bin(tmp)}  ')
        tempc = (175.72*tmp/65536) - 46.85  # pg. 14
        tempf = 32 + 1.8 * tempc
        print(f'Si7051 tempc={tempc} tempf={tempf}')
        return tempc

    def test_sts(self):
        ''' 0x94(0x4A W) 0x95(0x4A R)  -- STS35 '''
        wr = 0x4A << 1
        rd = 0x4A << 1 | 1
        print(f'STS35 addr wr={hex(wr)}  rd={hex(rd)} ')
        self.i2c.write_then_read(3,0, [wr, 0x30, 0xa2])  # soft reset
        time.sleep(0.01)
        #self.i2c.write_then_read(3,0, [wr, 0x24, 0x0b])  # single shot mode (pg.8 4.3)
        self.i2c.write_then_read(3,0, [wr, 0x2c, 0x06])  # single shot mode (pg.8 4.3)
        time.sleep(0.01)
        [msb,lsb,crc] = self.i2c.write_then_read(1,3, [rd])  # single shot mode (pg.8 4.3)
        tf = float(msb<<8 | lsb)
        # TODO use CRC to verify msb,lsb (pg.11  4.11)
        # poly: 0x31 (x^8 + x^5 + x^4 + 1)  init=0xff
        tempc = -45.0 + 175.0 * (tf / ((1<<16)-1))  # (pg.11  4.12)
        tempf = -49.0 + 315.0 * (tf / ((1<<16)-1))
        print(f'STS35 msb={hex(msb)}  lsb={hex(lsb)}  crc={hex(crc)} ')
        print(f'STS35 tf={tf} tempc={tempc} tempf={tempf} ')
        return tempc

    def test_max(self):
        ''' 0xA0(0x50 W) 0xA1(0x50 R)  -- MAX31889
        -- write SYSTEM CONTROL, set RESET (B0)
        I2C>[0xA0 0x0c 1]
        -- set TEMP_SENSOR_SETUP = CONVERT_T
        I2C>[0xa0 0x14 1]
        -- read STATUS, check bit0 set? 
        I2C>[0xA0 0x00 [0xA1 r]
        READ: 0x01   <== B0==1, temp ready to read
        -- read FIFO DATA, read 2 bytes (MSB LSB)
        [0xa0 0x08 [0xa1 r r]
        READ: 0x15 0x48
        -- convert value to degree C, and F
        tc = 0x1548 * 0.005
        tf = 32 + 1.8 * tc
        '''
        wr = 0x50 << 1
        rd = 0x50 << 1 | 1
        print(f'MAX31889 addr wr={hex(wr)}  rd={hex(rd)} ')
        #print(f'MAX31889 reset')
        self.i2c.write_then_read(3,0, [wr, 0x0c, 0x01])  # reset
        time.sleep(0.01)

        #print(f'MAX31889 read ROM ID 1..6')
        self.i2c.write_then_read(2,0, [wr, 0x31])
        romid = self.i2c.write_then_read(1,6, [rd])
        print(f'MAX31889 rom id: {hex(romid[0])} {hex(romid[1])} {hex(romid[2])}  ')
        print(f'MAX31889 rom id: {hex(romid[3])} {hex(romid[4])} {hex(romid[5])}  ')
        self.i2c.write_then_read(2,0, [wr, 0xff])
        [partid] = self.i2c.write_then_read(1,1, [rd])
        print(f'MAX31889 part id: {hex(partid)}')

        self.i2c.write_then_read(3,0, [wr, 0x0c, 0x01])  # SYSTEM_CONTROL := RESET
        time.sleep(0.01)
        self.i2c.write_then_read(3,0, [wr, 0x14, 0x01])  # TEMP_SENSOR_SETUP := CONVERT_T
        time.sleep(0.01)
        self.i2c.start()  #  STATUS
        self.i2c.transfer([wr, 0x00])  #  STATUS
        # NOTE: no STOP before reading...
        [status] = self.i2c.write_then_read(1,1, [rd]) # read 1 byte
        #self.i2c.stop()
        print(f'MAX31889 status: {hex(status)}')

        #print("read fifo")
        time.sleep(0.01)
        #print("start")
        self.i2c.start()
        #print("xfer")
        self.i2c.transfer([wr, 0x08])  #  FIFO DATA
        # NOTE: no STOP before reading...
        #print("read msb,lsb")
        [msb,lsb] = self.i2c.write_then_read(1,2, [rd])  # status
        #print(f'MAX31889 temp msb/lsb={hex(msb)} {hex(lsb)} ')
        tempc = 0.005 * float(msb<<8 | lsb)
        tempf = 32 + 1.8 * tempc
        print(f'MAX31889 tempc={tempc} tempf={tempf}')
        return tempc

def main():
    try:
        bp = BusPirate()
        bp.startup()
        time.sleep(0.5)
        info = dict()
        tc = bp.test_mcp()
        info['mcp'] = {'tc':tc,'prec':0.20}
        tc = bp.test_si()
        info['si'] = {'tc':tc,'prec':0.1}
        tc = bp.test_sts()
        info['sts'] = {'tc':tc,'prec':0.1}
        tc = bp.test_max()
        info['max'] = {'tc':tc,'prec':0.25}

        print("temperature/precision info....")
        avg = 0
        wt = 0
        # calc avg using precision as weight
        for k in info.keys():
            w = 1.0 / info[k]['prec']
            print(f"  {k} = {info[k]} w={w} ")
            wt += w
            avg += info[k]['tc'] * w
        avg /= wt
        tf = 32.0 + 1.8 * avg
        print(f'avg avg={avg} temp F={tf} ')
        bp.shutdown()
    except Exception as e:
        print(e)
        traceback.print_tb(e)
        bp.shutdown()

if __name__=='__main__':
    main()

