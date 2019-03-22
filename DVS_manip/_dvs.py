from ctypes import CDLL
from ctypes import CFUNCTYPE, POINTER
from ctypes import c_int32, c_float, c_void_p
import time
import random
from itertools import starmap
#import rclpy
#from rclpy.node import Node
#from std_msgs.msg import Int32MultiArray

class DVS(object):
    def __init__(self):
 #       rclpy.init()
        self._start_c = CDLL('./_dvsfiledump_4').read
        #self._start_c = CDLL('./_dvs').read
        self._event_callback = None
  #      self.node = rclpy.create_node('talker_dvs')
  #      self.pub = self.node.create_publisher(Int32MultiArray, 'chatter')
  #      self.msg=Int32MultiArray() 


    def _event(self, events, count):
        None
#        self.msg.data=[events[i] for i in range(count*4)]
#        self.pub.publish(self.msg)
        #map(lambda i: self._event_callback(events[4*i], events[4*i+1], events[4*i+2]), range(count))
        
    def start(self, event_callback, period_callback=None, fps=60.):
        """
        event_callback: int(*f)(int*, int)
        period_callback: bool(*f)(int)
        """
        
        self._event_callback = event_callback

        buffer_type = POINTER(c_int32)

        event_sig = CFUNCTYPE(c_void_p, buffer_type, c_int32)
        period_sig = CFUNCTYPE(c_int32, c_int32)

        event_c_func = event_sig(self._event)
        period_c_func = period_sig(period_callback)

        fps_c_float = c_float(fps)

        self._start_c(fps_c_float, event_c_func, period_c_func)
