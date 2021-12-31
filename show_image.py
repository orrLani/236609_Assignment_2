#!/usr/bin/env python
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import numpy
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import numpy as np
import argparse
import cv2
import imutils
import time
import pandas as pd
import time
import pickle


# define the lower and upper boundaries of the "blue"
BlueLower = (100, 150, 0)
BlueUpper = (140,255,255)


RedLower = (0,50,50)
RedUppder = (10,255,255)


PIXSELS_TO_BIG_BALL = 30
PIXSELS_TO_BIG_BALL_LOWER = 50
PIXSELS_TO_BIG_BALL_UPPER = 150



class Ball_Identification:

    def __init__(self):
        # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
        rospy.init_node('opencv_example', anonymous=True)
        self.model = pickle.load(open("/home/orr/my_ws/src/MRS_236609/scripts/model.pkl", 'rb'))
        self.is_first_time = True
        self.see_circle = False


        self.distance = None
        self.pixel_counter = None
        self.movie = None
        self.df = pd.DataFrame({'distance': [], 'radius': [], 'pixel_count': [], 'class': []})
        cv2.namedWindow("Image Window", 1)


        # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
        # rospy.loginfo("Hello ROS!")

        sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        time.sleep(1)



    def show_image(self,img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    def show_red(self):
        pass

    def image_callback(self,img_msg):
        # log some info about the image topic
        # rospy.loginfo(img_msg.header)

        image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        # resize the image
        image = imutils.resize(image, width=1800)
        
        # Converts an image from one color space to another
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if self.is_first_time == True:
            self.movie = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc(*'mp4v'), 10, (1350, 1800),True)
            self.is_first_time = False



        # set Gaussia 
        self.blurred = cv2.GaussianBlur(image, (11, 11), 0)

        # find the red images
        image = self.find_circle(image, "red")

        # find the blue image
        image = self.find_circle(image, "blue")

        # diplay more ditles
        if image is None:
            return
        print(image.shape)
        self.movie.write(image)
        self.movie.release()
        self.show_image(image)

    def scan_callback(self,msg):

        ranges = msg.ranges
        pixel_counter = 1

        # calculate right from middle
        i = 359

        while ( 0<=(ranges[i-1] - ranges[i]) and (ranges[i-1] - ranges[i]) < 0.1 and i > 290):
            i = i - 1
            pixel_counter += 1

        # calculate left from middle
        i = 0
        while (0<=(ranges[i-1] - ranges[i]) and (ranges[i-1] - ranges[i]) < 0.1 and i < 70):
            i = i + 1

            pixel_counter += 1


        distanse = (ranges[0] + ranges[1] + ranges[-1])/3

        self.distance = distanse
        self.pixel_counter = pixel_counter

        # rospy.loginfo(pixel_counter)
        # rospy.loginfo(distanse)


        # if self.see_circle:
        #     predict = self.model.predict(np.array([distanse, pixel_counter]).reshape(1, -1))
        #     predict = predict[0]
        #     if predict == 1:
        #         rospy.loginfo('see small ball')
        #     else:
        #         rospy.loginfo('see big ball')

        #new_row = {'distance': distanse, 'pixel_counter': pixel_counter}
        #self.df = self.df.append(new_row, ignore_index=True)
        # print(self.df)
        # self.df.to_csv('~/my_ws/src/MRS_236609/scripts/wow.csv')
    def find_circle(self,image,ball_color):

        # add gaussian blur


        # image = cv2.cvtColor( image, cv2.COLOR_BGR2RGB)


        hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)

        if ball_color == 'blue':
            colorUpper  = BlueUpper
            colorLower =  BlueLower
        else:
            colorUpper = RedUppder
            colorLower = RedLower
            
        msk = cv2.inRange(hsv, colorLower, colorUpper)
        msk = cv2.erode(msk, None, iterations=2)
        msk = cv2.dilate(msk, None, iterations=2)
        # find the blue image

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(msk.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)
        # find red or blue ball base on the color
        if len(cnts) > 0:
            
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            self.see_circle = True
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            print()
            
            cv2.circle(image, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)

            cv2.circle(image, center, 5, (0, 0, 255), -1)

            font = cv2.FONT_HERSHEY_SIMPLEX

            if not self.distance and not self.pixel_counter:
                return
            new_row = {'distance': self.distance, 'radius': radius,'pixel_count':self.pixel_counter,'class':1}
            self.df = self.df.append(new_row, ignore_index=True)
            #print(self.df)
            self.df.to_csv('~/my_ws/src/MRS_236609/scripts/wow.csv')

            if str(self.distance) =='inf':
                predict = 'cant'
            else:
                predict = self.model.predict(np.array([self.distance,radius]).reshape(1, -1))[0]
                #predict = 1
            size = None
            if predict ==1:
                size = 'big'
            elif predict ==0:
                size = 'small'
            else:
                size = 'dont see ball'

            fontScale = 1

            # print('thr r is' +str(radius))
            if ball_color =='blue':
                color = (255, 0, 0)
                org = (100, 200)
            else:
                color = (0, 0, 255)
                org = (100, 300)
            thickness = 2

            # print('the shape is {0}'.format(image.shape))
            # print('the ration between them is {0} and {1}'.format(center[0],image.shape[1]-center[0]))

            ration = (center[0],image.shape[1]-center[0])
            # need
            if ration[0] > ration[1]:
                self.move = 'RIGHT'
            else:
                self.move = 'LEFT'

            epsilon = 500
            if abs(ration[0] - ration[1]) < epsilon:
                self.move = 'CENTER'

            show = 'find  {0} and {1} ball in distance {2} rad is {3} pixel conut {4}' \
                   ' {5}'.format(size,ball_color,str(self.distance),str(radius),str(self.pixel_counter),self.move)


            # print('the ration between them is {0} and {1}'.format(center[0],image.shape[1]-center[0]))

            image = cv2.putText(image, show, org, font,fontScale, color, thickness, cv2.LINE_AA)
        else:
            self.see_circle = False
        return image


if __name__ == '__main__':


    # Initialize the CvBridge class
    bridge = CvBridge()

    start = time.time()

    bi = Ball_Identification()

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()

    # Define a function to show the image in an OpenCV Window


