""" NFC MFRC522x Reader

    Author: Roland van Straten, @rlndvnstrtn
    Date: 03-08-2016
    Version: 1.1

    Original code from github.com/mxgxw and a little of me...

    Adapted for use with micropython / PYB10 board running mpy 1.8.2

    The basic reading is working, need to test the other stuff and add the samples of mxgxw to __main__
    - Added the blue LED to show a tag is actually read.
    - Added read version of chip
    - Added reading of Receiver gain setting


    MFRC522 Reader Wiring

    SIGNAL       | PYB10 | CPU  | INFO
    ============ | ===== | ==== | =======
    NRSTPD       |   Y9  | PB10 | GPIO
    SDA / NSS    |   X5  | PA4  | NSS
    SCK          |   X6  | PA5  | SPI1
    MISO         |   X7  | PA6  | SPI1
    MOSI         |   X8  | PA7  | SPI1


    Able to read my collection of NFC cards and tags
    UID: 243,236, 67,146  NORTEC KEY TAG
    UID: 136,  2,226,  0  ST25TA02K
    UID: 136,  4,104, 57  elektor membership ultralight 4400
    UID: 136,  4,126,253  rfid ACG PHILIPS ELEKTUUR
    UID:  68,131,181, 89  mifare card
    UID:  45,136,224,140  OV-chipcard
    UID: 136,  4, 48,195  NXP mtag


    TODO: "port" it to ESP8266 mpy
          add a ctrl-break in the main function code
          add error checking

"""

# can be better :-)
from pyb import *


class MFRC522(object):
  """ class for interacting with the 'RC522' NFC reader chip """

  MI_OK       = 0
  MI_NOTAGERR = 1
  MI_ERR      = 2

  PCD_IDLE       = 0x00
  PCD_AUTHENT    = 0x0E
  PCD_RECEIVE    = 0x08
  PCD_TRANSMIT   = 0x04
  PCD_TRANSCEIVE = 0x0C
  PCD_RESETPHASE = 0x0F
  PCD_CALCCRC    = 0x03

  PICC_REQIDL    = 0x26
  PICC_REQALL    = 0x52
  PICC_ANTICOLL  = 0x93
  PICC_SElECTTAG = 0x93
  PICC_AUTHENT1A = 0x60
  PICC_AUTHENT1B = 0x61
  PICC_READ      = 0x30
  PICC_WRITE     = 0xA0
  PICC_DECREMENT = 0xC0
  PICC_INCREMENT = 0xC1
  PICC_RESTORE   = 0xC2
  PICC_TRANSFER  = 0xB0
  PICC_HALT      = 0x50

  Reserved00     = 0x00
  CommandReg     = 0x01
  CommIEnReg     = 0x02
  DivlEnReg      = 0x03
  CommIrqReg     = 0x04
  DivIrqReg      = 0x05
  ErrorReg       = 0x06
  Status1Reg     = 0x07
  Status2Reg     = 0x08
  FIFODataReg    = 0x09
  FIFOLevelReg   = 0x0A
  WaterLevelReg  = 0x0B
  ControlReg     = 0x0C
  BitFramingReg  = 0x0D
  CollReg        = 0x0E
  Reserved01     = 0x0F

  Reserved10     = 0x10
  ModeReg        = 0x11
  TxModeReg      = 0x12
  RxModeReg      = 0x13
  TxControlReg   = 0x14
  TxAutoReg      = 0x15
  TxSelReg       = 0x16
  RxSelReg       = 0x17
  RxThresholdReg = 0x18
  DemodReg       = 0x19
  Reserved11     = 0x1A
  Reserved12     = 0x1B
  MifareReg      = 0x1C
  Reserved13     = 0x1D
  Reserved14     = 0x1E
  SerialSpeedReg = 0x1F

  Reserved20        = 0x20
  CRCResultRegM     = 0x21
  CRCResultRegL     = 0x22
  Reserved21        = 0x23
  ModWidthReg       = 0x24
  Reserved22        = 0x25
  RFCfgReg          = 0x26
  GsNReg            = 0x27
  CWGsPReg          = 0x28
  ModGsPReg         = 0x29
  TModeReg          = 0x2A
  TPrescalerReg     = 0x2B
  TReloadRegH       = 0x2C
  TReloadRegL       = 0x2D
  TCounterValueRegH = 0x2E
  TCounterValueRegL = 0x2F

  Reserved30      = 0x30
  TestSel1Reg     = 0x31
  TestSel2Reg     = 0x32
  TestPinEnReg    = 0x33
  TestPinValueReg = 0x34
  TestBusReg      = 0x35
  AutoTestReg     = 0x36
  VersionReg      = 0x37
  AnalogTestReg   = 0x38
  TestDAC1Reg     = 0x39
  TestDAC2Reg     = 0x3A
  TestADCReg      = 0x3B
  Reserved31      = 0x3C
  Reserved32      = 0x3D
  Reserved33      = 0x3E
  Reserved34      = 0x3F

  MAX_LEN = 16

  serNum = []



  def __init__(self):
    ''' init the interface '''

    self.nrstpd    = pyb.Pin(pyb.Pin.cpu.B10, pyb.Pin.OUT)  # sets the pin to output
    self.nenbrc522 = pyb.Pin(pyb.Pin.cpu.A4,  pyb.Pin.OUT)  # sets the pin to output

    self.nenbrc522.high() # deselect device
    self.nrstpd.low()     # put it in power down mode

    # reader has stable data on rising edge of signal (phase 0), clock is high active (polarity 0)
    self.spi = SPI(1, SPI.MASTER, baudrate=1000000, polarity=0, phase=0, firstbit=SPI.MSB)  # default pins for SPI1 are selected

    # go configure yourself
    self.MFRC522_Init()


  def Write_MFRC522(self, addr, val):
    data = bytearray(2)
    data[0] = (addr<<1) & 0x7E
    data[1] = val
    self.nenbrc522.low()  # start the transaction
    self.spi.send(data)  # transfer two bytes to the chip
    self.nenbrc522.high() # finished it


  def Read_MFRC522(self, addr):
    data = bytearray(2)
    buf = bytearray(2)  # could reuse data instead of using buf
    data[0] = ((addr<<1)&0x7E) | 0x80
    data[1] = 0x00

    self.nenbrc522.low()            # start transaction
    self.spi.send_recv(data,buf)   # send data and read two bytes back
    self.nenbrc522.high()           # transaction ended

    return buf[1]


  def MFRC522_Reset(self):
    self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)


  def SetBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg)
    self.Write_MFRC522(reg, tmp | mask)


  def ClearBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg);
    self.Write_MFRC522(reg, tmp & (~mask))


  def AntennaOn(self):
    temp = self.Read_MFRC522(self.TxControlReg)
    if (~(temp & 0x03)):
      self.SetBitMask(self.TxControlReg, 0x03)


  def AntennaOff(self):
    self.ClearBitMask(self.TxControlReg, 0x03)


  def MFRC522_ToCard(self,command,sendData):
    backData = []
    backLen = 0
    status = self.MI_ERR
    irqEn = 0x00
    waitIRq = 0x00
    lastBits = None
    n = 0
    i = 0

    if command == self.PCD_AUTHENT:
      irqEn = 0x12
      waitIRq = 0x10
    if command == self.PCD_TRANSCEIVE:
      irqEn = 0x77
      waitIRq = 0x30

    self.Write_MFRC522(self.CommIEnReg, irqEn|0x80)
    self.ClearBitMask(self.CommIrqReg, 0x80)
    self.SetBitMask(self.FIFOLevelReg, 0x80)

    self.Write_MFRC522(self.CommandReg, self.PCD_IDLE);

    while(i<len(sendData)):
      self.Write_MFRC522(self.FIFODataReg, sendData[i])
      i = i+1

    self.Write_MFRC522(self.CommandReg, command)

    if command == self.PCD_TRANSCEIVE:
      self.SetBitMask(self.BitFramingReg, 0x80)

    i = 2000
    while True:
      n = self.Read_MFRC522(self.CommIrqReg)
      i = i - 1
      if ~((i!=0) and ~(n&0x01) and ~(n&waitIRq)):
        break

    self.ClearBitMask(self.BitFramingReg, 0x80)

    if i != 0:
      if (self.Read_MFRC522(self.ErrorReg) & 0x1B)==0x00:
        status = self.MI_OK

        if n & irqEn & 0x01:
          status = self.MI_NOTAGERR

        if command == self.PCD_TRANSCEIVE:
          n = self.Read_MFRC522(self.FIFOLevelReg)
          lastBits = self.Read_MFRC522(self.ControlReg) & 0x07
          if lastBits != 0:
            backLen = (n-1)*8 + lastBits
          else:
            backLen = n*8

          if n == 0:
            n = 1
          if n > self.MAX_LEN:
            n = self.MAX_LEN

          i = 0
          while i<n:
            backData.append(self.Read_MFRC522(self.FIFODataReg))
            i = i + 1;
      else:
        status = self.MI_ERR

    return (status,backData,backLen)


  def MFRC522_Request(self, reqMode):
    status = None
    backBits = None
    TagType = []

    self.Write_MFRC522(self.BitFramingReg, 0x07)

    TagType.append(reqMode);
    (status,backData,backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, TagType)

    if ((status != self.MI_OK) | (backBits != 0x10)):
      status = self.MI_ERR

    return (status,backBits)


  def MFRC522_Anticoll(self):
    backData = []
    serNumCheck = 0

    serNum = []

    self.Write_MFRC522(self.BitFramingReg, 0x00)

    serNum.append(self.PICC_ANTICOLL)
    serNum.append(0x20)

    (status,backData,backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,serNum)

    if (status == self.MI_OK):
      i = 0
      if len(backData)==5:
        while i<4:
          serNumCheck = serNumCheck ^ backData[i]
          i = i + 1
        if serNumCheck != backData[i]:
          status = self.MI_ERR
      else:
        status = self.MI_ERR

    return (status,backData)


  def CalulateCRC(self, pIndata):
    self.ClearBitMask(self.DivIrqReg, 0x04)
    self.SetBitMask(self.FIFOLevelReg, 0x80);
    i = 0
    while i<len(pIndata):
      self.Write_MFRC522(self.FIFODataReg, pIndata[i])
      i = i + 1
    self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)
    i = 0xFF
    while True:
      n = self.Read_MFRC522(self.DivIrqReg)
      i = i - 1
      if not ((i != 0) and not (n&0x04)):
        break
    pOutData = []
    pOutData.append(self.Read_MFRC522(self.CRCResultRegL))
    pOutData.append(self.Read_MFRC522(self.CRCResultRegM))
    return pOutData


  def MFRC522_SelectTag(self, serNum):
    backData = []
    buf = []
    buf.append(self.PICC_SElECTTAG)
    buf.append(0x70)
    i = 0
    while i<5:
      buf.append(serNum[i])
      i = i + 1
    pOut = self.CalulateCRC(buf)
    buf.append(pOut[0])
    buf.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)

    if (status == self.MI_OK) and (backLen == 0x18):
      print ("Size: " + str(backData[0]))
      return    backData[0]
    else:
      return 0


  def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
    buff = []

    # First byte should be the authMode (A or B)
    buff.append(authMode)

    # Second byte is the trailerBlock (usually 7)
    buff.append(BlockAddr)

    # Now we need to append the authKey which usually is 6 bytes of 0xFF
    i = 0
    while(i < len(Sectorkey)):
      buff.append(Sectorkey[i])
      i = i + 1
    i = 0

    # Next we append the first 4 bytes of the UID
    while(i < 4):
      buff.append(serNum[i])
      i = i +1

    # Now we start the authentication itself
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT,buff)

    # Check if an error occurred
    if not(status == self.MI_OK):
      print ("AUTH ERROR!!")
    if not (self.Read_MFRC522(self.Status2Reg) & 0x08) != 0:
      print ("AUTH ERROR(status2reg & 0x08) != 0" )

    # Return the status
    return status


  def MFRC522_StopCrypto1(self):
    self.ClearBitMask(self.Status2Reg, 0x08)


  def MFRC522_Read(self, blockAddr):
    recvData = []
    recvData.append(self.PICC_READ)
    recvData.append(blockAddr)
    pOut = self.CalulateCRC(recvData)
    recvData.append(pOut[0])
    recvData.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, recvData)
    if not(status == self.MI_OK):
      print ("Error while reading!")
    i = 0
    if len(backData) == 16:
      print ("Sector "+str(blockAddr)+" "+str(backData))


  def MFRC522_Write(self, blockAddr, writeData):
    buff = []
    buff.append(self.PICC_WRITE)
    buff.append(blockAddr)
    crc = self.CalulateCRC(buff)
    buff.append(crc[0])
    buff.append(crc[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buff)
    if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
        status = self.MI_ERR

    print (str(backLen)+" backdata &0x0F == 0x0A "+str(backData[0]&0x0F))
    if status == self.MI_OK:
        i = 0
        buf = []
        while i < 16:
            buf.append(writeData[i])
            i = i + 1
        crc = self.CalulateCRC(buf)
        buf.append(crc[0])
        buf.append(crc[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,buf)
        if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
            print ("Error while writing")
        if status == self.MI_OK:
            print ("Data written")


  def MFRC522_DumpClassic1K(self, key, uid):
    ''' dump the entire 1K of data '''
    i = 0
    while i < 64:
        status = self.MFRC522_Auth(self.PICC_AUTHENT1A, i, key, uid)
        # Check if authenticated
        if status == self.MI_OK:
            self.MFRC522_Read(i)
        else:
            print ("Authentication error")
        i = i+1


  def MFRC522_Version(self):
    ''' return version of silicon
        used reader returned 0x11 expected 0x91 (so we missed one bit?)
    '''
    val = self.Read_MFRC522(self.VersionReg) & 0x0F
    return val


  def MFRC522_ReceiverGain(self):
    ''' the receiver gain is variable from 18 to 48dB '''
    #self.Write_MFRC522(self.RFCfgReg, 0x4<<4)

    val = (self.Read_MFRC522(self.RFCfgReg) & 0x7F) >> 4

    if (val==0) or (val==2):
      val = 18
    elif (val==1) or (val==3):
      val = 23
    elif val==4:
      val = 33
    elif val==5: 
      val=38
    elif val==6:
      val=43
    else: 
      val=48
    return val


  def MFRC522_Init(self):
    ''' init the device registers '''
    self.nenbrc522.low() # assert device
    self.nrstpd.high()   # get it out of reset

    pyb.delay(500)
    self.MFRC522_Reset();

    pyb.delay(500)

    print("MFRC522 version is " + str(self.MFRC522_Version() ) )
    print("Receiver gain is " + str(self.MFRC522_ReceiverGain()) + "dB")

    self.Write_MFRC522(self.TModeReg, 0x8D)
    self.Write_MFRC522(self.TPrescalerReg, 0x3E)
    self.Write_MFRC522(self.TReloadRegL, 30)
    self.Write_MFRC522(self.TReloadRegH, 0)

    self.Write_MFRC522(self.TxAutoReg, 0x40)
    self.Write_MFRC522(self.ModeReg, 0x3D)
    self.AntennaOn()


  def MFRC522_DeInit(self):
    ''' turn it off and sleep '''
    self.AntennaOff()
    self.nenbrc522.high()
    self.nrstpd.low()



if __name__ == '__main__':
  """ some basic test code resides here """

  hell_freezes_over = True

  # let's have a reader
  MIFAREReader = MFRC522()

  # loop keeps checking for cards and tags. If one is near it will catch the UID and authenticate it
  while hell_freezes_over:

    # Scan for cards
    (status,TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)
    # If a card is found
    if status == MIFAREReader.MI_OK:
        print ("NFC card detected")

    # Get the UID of the card
    (status,uid) = MIFAREReader.MFRC522_Anticoll()

    # If we have the UID, continue
    if status == MIFAREReader.MI_OK:
        pyb.LED(4).on()
        pyb.delay(200)
        pyb.LED(4).off()
        # Print UID
        print("Card read UID: "+str(hex(uid[0]))+","+str(hex(uid[1]))+","+str(hex(uid[2]))+","+str(hex(uid[3])) )

        # This is the default key for authentication
        key = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
        # Select the scanned tag
        #MIFAREReader.MFRC522_SelectTag(uid)
        # Dump the data
        #MIFAREReader.MFRC522_DumpClassic1K(key, uid)
        # Stop
        #MIFAREReader.MFRC522_StopCrypto1()

    pyb.delay(1000)

  # handle ctrl-c
