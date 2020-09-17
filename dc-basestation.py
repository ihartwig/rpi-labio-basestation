#! /usr/bin/python3

import RPi.GPIO as GPIO
import logging
import socket
import threading

DCCPP_DEFAULT_PORT = 2560   # from JMRI
DCCPP_MAX_CMD_LEN = 30      # from DCCPP source
DCCPP_THROTTLE_SCALE = 1.27 # convert from 0-127 to 0-100

LABIO_M3_GPIO = 19
LABIO_M4_GPIO = 18
LABIO_M3_M4_EN_GPIO = 13  # BCM GPIO num
LABIO_PWM_FREQ = 1000

logging.basicConfig()
LOG = logging.getLogger(__name__)
LOG.setLevel(logging.DEBUG)


class DcLayout(object):
  def __init__(self):
    self.power_state = 1    # off 0, on 1
    self.throttle_pos = 0.0 # 0-100
    self.throttle_dct = 1   # reverse 0, forward 1
    # setup gpio hw
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LABIO_M3_GPIO, GPIO.OUT)
    GPIO.setup(LABIO_M4_GPIO, GPIO.OUT)
    GPIO.setup(LABIO_M3_M4_EN_GPIO, GPIO.OUT)
    self.m3_m4_en_pwm = GPIO.PWM(LABIO_M3_M4_EN_GPIO, LABIO_PWM_FREQ)
    self.set_hw()

  def set_hw(self):
    if self.power_state == 0:
      self.m3_m4_en_pwm.stop()
    else:
      self.m3_m4_en_pwm.start(self.throttle_pos)
      GPIO.output(LABIO_M3_GPIO, GPIO.HIGH if self.throttle_dct else GPIO.LOW)
      GPIO.output(LABIO_M4_GPIO, GPIO.LOW if self.throttle_dct else GPIO.HIGH)

  def status(self):
    """
    Returns: Track power status,Throttle status, Turn-out status, and a version number.
    """
    return (self.power_state, 1, int(self.throttle_pos), self.throttle_dct)

  def power(self, state):
    """
    Returns: < p0 > Power to tracks OFF.
    """
    self.power_state = 1 if state else 0 # force 0 or 1
    self.set_hw()
    return (self.power_state,)

  def cab_throttle(self, reg, cab, pos, dct):
    # emergency stop gives us a special value to rewrite
    if pos == "-1":
      pos = "0"
    self.throttle_pos = int(pos) / DCCPP_THROTTLE_SCALE
    self.throttle_dct = int(dct)
    self.set_hw()
    return (1, int(self.throttle_pos), self.throttle_dct)


class DccPlusPlusCommandServer(threading.Thread):
  def __init__(self, clientsocket, layout):
    threading.Thread.__init__(self)
    self.clientsocket = clientsocket
    self.layout = layout

  def run(self):
    """
    read in DCC++ commands and dispatch responses
    """
    command = []
    while True:
      c = str(clientsocket.recv(1), 'utf-8')
      if "<" == c:
        command = []
      elif ">" == c:
        self.do_command("".join(command).strip())
      else:
        command.append(c)

  def do_command(self, command):
    LOG.debug("recv: {}".format(command))
    command_type = command[0]
    # dispatch by command type
    resp = None
    if command_type == "s":
      status = self.layout.status()
      resp = "<p{:d}><T {:d} {:d} {:d}>".format(*status)
    elif command_type == "0" or command_type == "1":
      status = self.layout.power(int(command_type))
      resp = "<p{:d}>".format(*status)
    elif command_type == "t":
      command_tup = command.split(" ")
      status = self.layout.cab_throttle(*command_tup[1:])
      resp = "<T {:d} {:d} {:d}>".format(*status)
    # send response
    if resp != None:
      LOG.debug("send: {}".format(resp))
      self.clientsocket.send(bytes(resp, 'utf-8'))
    else:
      LOG.debug("(no resp)")


if __name__ == "__main__":
  # create the physical layout interface
  layout = DcLayout()
  # create an INET, STREAMing socket
  serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # bind the socket to a public host, and a well-known port
  serversocket.bind(('localhost', DCCPP_DEFAULT_PORT))
  # become a server socket
  serversocket.listen(1)
  while True:
    # accept connections from outside
    (clientsocket, address) = serversocket.accept()
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    cs = DccPlusPlusCommandServer(clientsocket, layout)
    cs.run()
