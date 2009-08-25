#!/usr/bin/python
from bluetooth import *
import select
import fcntl
import os
import time
import sys                    
import traceback

L2CAP_PSM_HIDP_CTRL = 17
L2CAP_PSM_HIDP_INTR = 19

class uinput:
    EV_KEY = 1
    EV_REL = 2
    EV_ABS = 3
    BUS_USB = 3
    ABS_MAX = 0x3f

class uinputjoy:
    def __init__(self, buttons, axes):
        self.file = None
        for name in ["/dev/input/uinput", "/dev/misc/uinput", "/dev/uinput"]:
            try:
                self.file = os.open(name, os.O_WRONLY)
                break
            except Exception, e:
                print >> sys.stderr, "Error opening uinput: %s"%str(e)
                raise
        if self.file == None:
            raise IOError
        #id = uinput.input_id()
        #id.bustype = uinput.BUS_USB
        #id.vendor = 0x054C
        #id.product = 0x0268
        #id.version = 0
        #info = uinput.uinput_user_dev()
        #info.name = "Sony Playstation SixAxis/DS3"
        #info.id = id
        
        UI_SET_EVBIT   = 0x40045564
        UI_SET_KEYBIT  = 0x40045565
        UI_SET_RELBIT  = 0x40045566
        UI_DEV_CREATE  = 0x5501
        UI_SET_RELBIT  = 0x40045566
        UI_SET_ABSBIT  = 0x40045567
        uinput_user_dev = "80sHHHHi" + 64*4*'I'

        os.write(self.file, struct.pack(uinput_user_dev, "Sony Playstation SixAxis/DS3",
            uinput.BUS_USB, 0x054C, 0x0268, 0, 0, *([0] * (4*(uinput.ABS_MAX+1)))))
        
        fcntl.ioctl(self.file, UI_SET_EVBIT, uinput.EV_KEY)
        
        for b in buttons:
            fcntl.ioctl(self.file, UI_SET_KEYBIT, b)
        
        for a in axes:
            fcntl.ioctl(self.file, UI_SET_EVBIT, uinput.EV_ABS)
            fcntl.ioctl(self.file, UI_SET_ABSBIT, a)
        
        fcntl.ioctl(self.file, UI_DEV_CREATE)

        self.value = [None] * (len(buttons) + len(axes))
        self.type = [uinput.EV_KEY] * len(buttons) + [uinput.EV_ABS] * len(axes)
        self.code = buttons + axes
    
    def update(self, value):
        input_event = "LLHHi"
        t = time.time()
        th = int(t)
        tl = int((t - th) * 1000000)
        if len(value) != len(self.value):
            print >> sys.stderr, "Unexpected length for value in update. This is a bug."
        for i in range(0, len(value)):
            if value[i] != self.value[i]:
                os.write(self.file, struct.pack(input_event, th, tl, self.type[i], self.code[i], value[i]))
        self.value = list(value)

class decoder:
    def __init__(self):
        #buttons=[uinput.BTN_SELECT, uinput.BTN_THUMBL, uinput.BTN_THUMBR, uinput.BTN_START, 
        #         uinput.BTN_FORWARD, uinput.BTN_RIGHT, uinput.BTN_BACK, uinput.BTN_LEFT, 
        #         uinput.BTN_TL, uinput.BTN_TR, uinput.BTN_TL2, uinput.BTN_TR2,
        #         uinput.BTN_X, uinput.BTN_A, uinput.BTN_B, uinput.BTN_Y,
        #         uinput.BTN_MODE]
        #axes=[uinput.ABS_X, uinput.ABS_Y, uinput.ABS_Z, uinput.ABS_RX,
        #         uinput.ABS_RX, uinput.ABS_RY, uinput.ABS_PRESSURE, uinput.ABS_DISTANCE,
        #         uinput.ABS_THROTTLE, uinput.ABS_RUDDER, uinput.ABS_WHEEL, uinput.ABS_GAS,
        #         uinput.ABS_HAT0Y, uinput.ABS_HAT1Y, uinput.ABS_HAT2Y, uinput.ABS_HAT3Y,
        #         uinput.ABS_TILT_X, uinput.ABS_TILT_Y, uinput.ABS_MISC, uinput.ABS_RZ,
        #         ]
        buttons = range(0x100,0x111)
        axes = range(0, 20)
        self.joy = uinputjoy(buttons, axes)
        self.outlen = len(buttons) + len(axes)

    def step(self, sock): # Returns true if the packet was legal
        joy_coding = "!1B2x3B1x4B4x12B15x4H"
        rawdata = sock.recv(128)
        if len(rawdata) != 50:
            print >> sys.stderr, "Unexpected packet length (%i). Is this a PS3 Dual Axis or Six Axis?"%len(data)
            return False
        data = list(struct.unpack(joy_coding, rawdata))
        prefix = data.pop(0)
        if prefix != 161:
            print >> sys.stderr, "Unexpected prefix (%i). Is this a PS3 Dual Axis or Six Axis?"%prefix
            return False
        out = []
        for j in range(0,2):
            curbyte = data.pop(0)
            for k in range(0,8):
                out.append(int((curbyte & (1 << k)) != 0))
        out.append(data.pop(0))
        for j in range(3,7):
            out.append(data.pop(0) - 0x80)
        for j in range(7,19):
            out.append(data.pop(0))
        for j in range(19,23):
            out.append(data.pop(0) - 0x200)
        self.joy.update(out)
        return True

    def fullstop(self):
        self.joy.update([0] * self.outlen)

    def run(self, intr, ctrl):
        try:
            lastvalidtime = 0
            while True:
                (rd, wr, err) = select.select([intr], [], [], 0.1)
                curtime = time.time()
                if len(rd) + len(wr) + len(err) == 0: # Timeout
                    print "Connection is Activated."
                    ctrl.send("\x53\xf4\x42\x03\x00\x00") # Try activating the stream.
                    if lastvalidtime - curtime >= 0.1: # Zero all outputs if we don't hear a valid frame for 0.1 to 0.2 seconds
                        self.fullstop()
                    if lastvalidtime - curtime >= 5: # Disconnect if we don't hear a valid frame for 5 seconds
                        return
                else: # Got a frame.
                    #print "Got a frame at ", curtime, 1 / (curtime - lastvalidtime)
                    if self.step(intr):
                        lastvalidtime = curtime
        finally:
            self.fullstop()

def quit(i):
    os.system("/etc/init.d/bluetooth start > /dev/null 2>&1")
    exit(i)

class connection_manager:
    def __init__(self, decoder):
        self.decoder = decoder

    def prepare_socket(self, port):
        sock = BluetoothSocket(L2CAP)
        try:
            sock.bind(("", port))
        except:
            print >> sys.stderr, "Error binding to bluetooth socket."
            quit(-1)
        sock.listen(1)
        return sock

    def listen(self):
        intr_sock = self.prepare_socket(L2CAP_PSM_HIDP_INTR)
        ctrl_sock = self.prepare_socket(L2CAP_PSM_HIDP_CTRL)

        while True:
            print "Waiting for connection. Disconnect your PS3 joystick from USB and press the pairing button."
            try:
                (intr, (idev, iport)) = intr_sock.accept();
                try:
                    (rd, wr, err) = select.select([ctrl_sock], [], [], 1)
                    if len(rd) == 0:
                        print >> sys.stderr, "Got interrupt connection without control connection. Giving up on it."
                        intr.close()
                        continue
                    (ctrl, (cdev, cport)) = ctrl_sock.accept();
                    try:
                        if idev == cdev:
                            self.decoder.run(intr, ctrl)
                        else:
                            print >> sys.stderr, "Simultaneous connection from two different devices. Ignoring both."
                    finally:
                        ctrl.close()
                finally:
                    intr.close()
            except KeyboardInterrupt:
                print "CTRL+C detected. Exiting."
                quit(0)
            except Exception, e:
                traceback.print_exc()
                print >> sys.stderr, "Caught exception: %s"%str(e)

if __name__ == "__main__":
    try:
        if os.getuid() != 0:
            print >> sys.stderr, "ps3joy.py must be run as root."
            quit(1)
        os.system("/etc/init.d/bluetooth stop > /dev/null 2>&1")
        while os.system("hciconfig hci0 > /dev/null 2>&1") != 0:
            print >> sys.stderr,  "No bluetooth device found. Will retry in 5 seconds."
            time.sleep(5)
        os.system("hciconfig hci0 up > /dev/null 2>&1")
        os.system("hciconfig hci0 pscan > /dev/null 2>&1")
        os.system("modprobe uinput > /dev/null 2>&1")
        time.sleep(1) # uinput isn't ready to go right away.
        cm = connection_manager(decoder())
        cm.listen()
    except KeyboardInterrupt:
        print "CTRL+C detected. Exiting."
    quit(0)
        
