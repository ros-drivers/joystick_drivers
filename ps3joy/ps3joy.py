#!/usr/bin/python
from bluetooth import *
import select
import uinput
import fcntl
import os
import time

L2CAP_PSM_HIDP_CTRL = 17
L2CAP_PSM_HIDP_INTR = 19

class uinputjoy:
    def __init__(self, buttons, axes):
        self.file = None
        for name in ["/dev/input/uinput", "/dev/misc/uinput", "/dev/uinput"]:
            try:
                self.file = os.open(name, os.O_WRONLY)
                print self.file
                break
            except:
                print "err"
                raise
                continue
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
            print "Unexpected length for value in update"
        for i in range(0, len(value)):
            if value[i] != self.value[i]:
                os.write(self.file, struct.pack(input_event, th, tl, self.type[i], self.code[i], value[i]))
        self.value = list(value)

class decoder:
    def __init__(self):
        buttons=[uinput.BTN_SELECT, uinput.BTN_THUMBL, uinput.BTN_THUMBR, uinput.BTN_START, 
                 uinput.BTN_FORWARD, uinput.BTN_RIGHT, uinput.BTN_BACK, uinput.BTN_LEFT, 
                 uinput.BTN_TL, uinput.BTN_TR, uinput.BTN_TL2, uinput.BTN_TR2,
                 uinput.BTN_X, uinput.BTN_A, uinput.BTN_B, uinput.BTN_Y,
                 uinput.BTN_MODE]
        axes=[uinput.ABS_X, uinput.ABS_Y, uinput.ABS_Z, uinput.ABS_RX,
                 uinput.ABS_RX, uinput.ABS_RY, uinput.ABS_PRESSURE, uinput.ABS_DISTANCE,
                 uinput.ABS_THROTTLE, uinput.ABS_RUDDER, uinput.ABS_WHEEL, uinput.ABS_GAS,
                 uinput.ABS_HAT0Y, uinput.ABS_HAT1Y, uinput.ABS_HAT2Y, uinput.ABS_HAT3Y,
                 uinput.ABS_TILT_X, uinput.ABS_TILT_Y, uinput.ABS_MISC, uinput.ABS_RZ,
                 ]
        self.joy = uinputjoy(buttons, axes)

    def step(self, sock):
        joy_coding = "!3x3B1x4B4x12B15x4H"
        data = struct.unpack(joy_coding, sock.recv(128))
        #print data,
        out = [0] * (17 + 20)
        i = 0
        for j in range(0,2):
            for k in range(0,8):
                out[i] = int((data[j] & (1 << k)) != 0)
                i = i + 1
        out[i] =data[2] 
        i = i + 1
        for j in range(3,7):
            out[i] = data[j] - 0x80
            i = i + 1
        for j in range(7,19):
            out[i] = data[j]
            i = i + 1
        for j in range(19,23):
            out[i] = data[j] - 0x200
            i = i + 1
        #print out
        self.joy.update(out)

    def run(self, intr, ctrl):
        start = time.time()
        frames = 0
        while True:
            (rd, wr, err) = select.select([intr], [], [], 0.1)
            if len(rd) + len(wr) + len(err) == 0: # Timeout
                print "Activating connection."
                ctrl.send("\x53\xf4\x42\x03\x00\x00") # Try activating the stream.
            else: # Got a frame.
                frames = frames + 1
                curtime = time.time()
                print "Got a frame at ", curtime, frames / (curtime - start)
                self.step(intr)

class connection_manager:
    def __init__(self, decoder):
        self.devicesockets = {} # Stores dev to socket pair mapping
        self.intrdev = {} # Stores socket to dev mapping
        self.ctrldev = {} # Stores socket to dev mapping
        self.socknames = [ "intr", "ctrl" ]
        self.sockdev = [ self.intrdev, self.ctrldev ]
        self.decoder = decoder

    def acceptsock(self, index, servsock):
        (sock, (dev, port)) = servsock.accept()
        print "Adding", self.socknames[index], "from", dev
        try:
            devsock = self.devicesockets[dev]
        except KeyError:
            devsock = [None, None]
        if devsock[index] != None:
            print "addintr: Already had an", self.socknames[index], "socket for ", dev
        devsock[index] = sock
        self.devicesockets[dev] = devsock
        self.sockdev[index][sock] = dev
        print devsock
        print self.devicesockets
        if devsock[0] != None and devsock[1] != None:
            print "Both connections are open"
            try:
                self.decoder.run(devsock[0], devsock[1])
            except: # Normal exit point upon disconnect.
                print "Joystick disconnected or errored out."
                devsock[0].close()
                devsock[1].close()
                            
    def acceptintr(self, sock):
        self.acceptsock(0, sock)
    
    def acceptctrl(self, sock):
        self.acceptsock(1, sock)

    def prepare_socket(self, port):
        sock = BluetoothSocket(L2CAP)
        sock.bind(("", port))
        sock.listen(1)
        return sock

    def listen(self):
        intr_sock = self.prepare_socket(L2CAP_PSM_HIDP_INTR)
        ctrl_sock = self.prepare_socket(L2CAP_PSM_HIDP_CTRL)

        while True:
            (rd, wr, err) = select.select([intr_sock, ctrl_sock], [], [], 60)
            for s in rd:
                #print "Socket ", s, "is in read list."
                if s == intr_sock:
                    cm.acceptintr(s)
                elif s == ctrl_sock:
                    cm.acceptctrl(s)
                else:
                    print "Unexpected socket after select"

if __name__ == "__main__":
    cm = connection_manager(decoder())
    cm.listen()
