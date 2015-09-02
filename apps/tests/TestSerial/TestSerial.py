# http://wiesel.ece.utah.edu/redmine/projects/hacks/wiki/How_to_use_Python_and_MIG_to_interact_with_a_TinyOS_application
"""
 Python application for testing serial port communication.
"""
import sys
import time

import TestSerialMsg
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

class TestSerial:
  def __init__(self, motestring):
    self.mif = MoteIF.MoteIF()
    self.tos_source = self.mif.addSource(motestring)
    self.mif.addListener(self, TestSerialMsg.TestSerialMsg)

  def receive(self, src, msg):
    if msg.get_amType() == TestSerialMsg.AM_TYPE:
      m = TestSerialMsg.TestSerialMsg(msg.dataGet())
      print 'Received packet #:', m.get_counter(), '\n'

  def send(self):
    counter = 0
    payload = TestSerialMsg.TestSerialMsg()
    while True:
      time.sleep(1)
      print 'Sending packet #:', counter
      payload.set_counter(counter)
      self.mif.sendMsg(self.tos_source, 0xFFFF, payload.get_amType(), 0, payload)
      counter += 1

def main():
  if '-h' in sys.argv or len(sys.argv) < 2:
    print 'Usage: ...'
    sys.exit()
  dl = TestSerial(sys.argv[1])
  dl.send()

if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    pass
