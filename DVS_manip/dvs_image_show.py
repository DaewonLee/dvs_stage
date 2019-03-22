import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import queue as Queue
import threading
import cv2
import numpy as np

class Visualizer(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(Int32MultiArray, 'chatter', self.chatter_callback)
        self.q = Queue.Queue(0)
        self.thread = threading.Thread(target=self.roslisten)
       # cv2.startWindowThread()
        cv2.namedWindow('window')
        self.img = 128 * np.ones((320,480),dtype=np.uint8)

    def chatter_callback(self, msg):
        self.q.put(msg.data) 

    def roslisten(self):
        try:
            rclpy.spin(self)
        finally:
            self.destroy_node()
            rclpy.shutdown()

    def empty_img(self):    
        self.img = 128 * np.ones((320,480),dtype=np.uint8)
    def set_pixel(self,event):
        self.img[event[0],event[1]]=event[2]


def main(args=None):
    
    rclpy.init(args=args)
    listener = Visualizer()
    listener.thread.start()

    counter = 1
    while(True):
        if counter%5 is not 0:
            qgot=listener.q.get()
            [listener.set_pixel(qgot[i:i+4]) for i in range(0,len(qgot),4)]
        else:
            cv2.imshow('window',listener.img)
            cv2.waitKey(1)
            listener.empty_img()
    
        counter+=1
        

if __name__ == '__main__':
    main()
