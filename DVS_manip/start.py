from _dvs import DVS
from dvs_accumulator import DVSAccumulator
import sys
import rclpy
if __name__ == '__main__':

    # Create a DVS interface
    dvs_ = DVS()
    #rclpy.init()

    # Create an accumulator to build binned images
    acc = DVSAccumulator(dvs_, float(sys.argv[1]))

    def on_frame_update(frame, idx):
        # Do nothing
        return True

    ##############################
    # Start the accumulator      #
    # with our callback function #
    ##############################
    acc.start(on_frame_update)
    ##############################
