import cv2
from LIDARVisualizer import *
import numpy as np
import rosbag
import sys

from PyQt4.QtCore import Qt
from PyQt4.QtGui import QApplication, QWidget, QImage, QPixmap, QLabel, QGridLayout, QSlider


class BagVisualizer(QWidget):
    def __init__(self, parent, bag):
        super(BagVisualizer, self).__init__(parent)
        with rosbag.Bag(bag) as bag:
            right_image_msgs = [msg for msg in bag.read_messages(
                                          topics=[
                                                  '/right_camera'
                                                + '/pg_16492281/' 
                                                + 'image_color_flipped'
                                                + '/compressed'])]
            print(sys.getsizeof(right_image_msgs))
            lidar_msgs = [msg for msg in bag.read_messages(topics=['/velodyne_points'])]
            print(sys.getsizeof(lidar_msgs))
       
        # For LIDAR
        self.lidar_vis = LIDARVisualizer(self)

        # For displaying video
        self.right_image_label = QLabel(self)

        # Scrubber
        self.scrubber = QSlider(Qt.Horizontal, self)
        
        # Process images
        print("Loading images")
        self.right_image_timestamps = [msg.timestamp for msg in right_image_msgs]
        self.right_images = [self.process_image_message(msg.message.data) for msg in right_image_msgs]

        self.current_frame = 0

        # Load LIDAR
        print("Loading LIDAR data")
        self.lidar_readings = [msg.message.data for msg in lidar_msgs]
        self.lidar_timestamps = [msg.timestamp for msg in lidar_msgs]
 
        self.scrubber.setTickInterval(1)
        self.scrubber.setMinimum(0)
        self.scrubber.setMaximum(len(self.right_images) - 1)
        self.scrubber.setValue(0)
        self.scrubber.valueChanged.connect(self.set_frame)

    def set_frame(self):
        i = self.scrubber.value()
        self.load_image(i)
        lidar_index = min(i / 3, len(self.lidar_readings) - 1)
        
        #lidar_timestamp = self.lidar_timestamps[lidar_index]
        #image_timestamp = self.right_image_timestamps[i]
        #if lidar_timestamp > image_timestamp:
        #    while lidar_index != 0:
        #        lidar_index -= 1 
        #        new_lidar_timestamp = self.lidar_timestamps[lidar_index] 
        #        if new_lidar_timestamp < image_timestamp:
        #            break

        #elif lidar_timestamp < image_timestamp:
        #    while lidar_index != len(self.lidar_timestamps) - 1:
        #        lidar_index += 1 
        #        new_lidar_timestamp = self.lidar_timestamps[lidar_index] 
        #        if new_lidar_timestamp > image_timestamp:
        #            lidar_index -= 1
        #            break
        

        self.lidar_vis.load_xyzi_pc_and_render(self.lidar_readings[lidar_index])
        self.current_frame = self.scrubber.value()

    def process_image_message(self, img):
        im = cv2.imdecode(np.fromstring(img, np.uint8), 
                                 cv2.IMREAD_COLOR)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

        qim = QImage(im.data, im.shape[1], im.shape[0], im.strides[0], QImage.Format_RGB888)
        return qim

    def load_image(self, i):
        right_height = self.right_image_label.height()
        right_width  = self.right_image_label.width()
        right_size = min(right_height, right_width)
        
        qim = self.right_images[i] 
        pix = QPixmap.fromImage(qim).scaled(right_size, right_size, Qt.KeepAspectRatio)
        self.right_image_label.setPixmap(pix)


    def resize(self, x, y):
        super(BagVisualizer, self).resize(x, y)
        self.setGeometry(0, 0, x, y)

        self.scrubber.setGeometry(0, 0, x, 30)
        self.scrubber.move(0, 0)
        
        self.right_image_label.resize(x // 2, y)
        self.right_image_label.move(0, 30)
        self.load_image(self.current_frame)

        self.lidar_vis.resize(x // 2, y)
        self.lidar_vis.move(x // 2, 30)


    def resizeEvent(self, event):
        self.resize(event.size().width(), event.size().height())

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print('Please pass in a bag file name.')
    
    app = QApplication(sys.argv[:1])

    bag_vis = BagVisualizer(None, sys.argv[1])

    bag_vis.resize(1280, 720)
    bag_vis.show()

    sys.exit(app.exec_())

