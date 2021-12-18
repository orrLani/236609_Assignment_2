#!/usr/bin/env python

# license removed for brevity



import rospy

import numpy as np

import matplotlib.pyplot as plt

from nav_msgs.srv import GetMap



from nav_msgs.msg import OccupancyGrid

from map_msgs.msg import OccupancyGridUpdate



# Brings in the SimpleActionClient

import actionlib

# Brings in the .action file and messages used by the move base action

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal , MoveBaseFeedback









class MapService(object):



    def __init__(self):

        """

        Class constructor

        """

        rospy.wait_for_service('static_map')

        static_map = rospy.ServiceProxy('static_map', GetMap)

        self.map_data = static_map().map

        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])

        self.shape = self.map_data.info.height, self.map_data.info.width

        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(self.shape)

        self.resolution = self.map_data.info.resolution



    def show_map(self, point=None):

        plt.imshow(self.map_arr)

        if point is not None:

            plt.scatter([point[0]], [point[1]])

        plt.show()



    def position_to_map(self, pos):

        return (pos - self.map_org) // self.resolution



    def map_to_position(self, indices):

        return indices * self.resolution + self.map_org





def movebase_client(x,y):



   # Create an action client called "move_base" with action definition file "MoveBaseAction"

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

 

   # Waits until the action server has started up and started listening for goals.

    client.wait_for_server()



   # Creates a new goal with the MoveBaseGoal constructor

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"

    goal.target_pose.header.stamp = rospy.Time.now()

   # Move to position 0.5 on the x axis of the "map" coordinate frame 

    goal.target_pose.pose.position.x = x

    # Move to position 0.5 on the y axis of the "map" coordinate frame 

    goal.target_pose.pose.position.y = y

   # No rotation of the mobile base frame w.r.t. map frame

    goal.target_pose.pose.orientation.w = 1.0



   # Sends the goal to the action server.

    client.send_goal(goal)

    rospy.loginfo("New goal command received!")



   # Waits for the server to finish performing the action.

    wait = client.wait_for_result()

   # If the result doesn't arrive, assume the Server is not available

    if not wait:

        rospy.logerr("Action server not available!")

        rospy.signal_shutdown("Action server not available!")

    else:

    # Result of executing the action

        return client.get_result()   













class CostmapUpdater:

    def __init__(self):

        self.cost_map = None

        self.shape = None

        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)

        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

	rospy.Subscriber('move_base_msgs/MoveBaseFeedback', MoveBaseFeedback , self.update_curr_location)



    def update_curr_location(self, msg):

	pose = msg.base_position.pos

	print("our location is :" + pose.x)



    def init_costmap_callback(self, msg):

        print('only once')  # For the student to understand

        self.shape = msg.info.height, msg.info.width

        self.cost_map = np.array(msg.data).reshape(self.shape)



    def costmap_callback_update(self, msg):

       # print('periodically')  # For the student to understand

        shape = msg.height, msg.width

        data = np.array(msg.data).reshape(shape)

        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data

      # self.show_map()  # For the student to see that it works



    def show_map(self):

        if not self.cost_map is None:

            plt.imshow(self.cost_map)

            plt.show()











from geometry_msgs.msg import PoseWithCovarianceStamped

from sensor_msgs.msg import LaserScan



class DistCheck:

	def __init__(self):

		self.cost_map = None

		self.shape = None

	

	#	rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)

	#	rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

		rospy.Subscriber('/scan',LaserScan, self.distcallback)

		rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose)

		self.biggest_ball_size = 0





	def initial_pose(self,msg):

		self.initialpose = msg.pose.pose

		print("initial pose is")

		print("X=" + str(self.initalpose.position.x))

		print("y=" + str(self.initalpose.position.y))



		

		

		

	def distcallback(self,msg):

		tmp =  PoseWithCovarianceStamped()

	

		ranges = msg.ranges

		print(len(msg.ranges))	

		#if self.is_ball:

		self.ball_distance = msg.ranges[0]

		pixel_counter =1

		# calculate right from middle

		i=359

		while(abs(ranges[i] - ranges[i-1]) <1 and i > 270):

			i = i-1

			pixel_counter += 1

			

		# calculate left from middle

		i=0

		while(abs(ranges[i] - ranges[i+1]) <1 and i < 90):

			i = i+1

			pixel_counter += 1

		if pixel_counter > self.biggest_ball_size:

			self.biggest_ball_size = pixel_counter		

		print("the size is:" + str(pixel_counter))

		print("ball_distnce =" + str(self.ball_distance))





# If the python node is executed as main process (sourced directly)

if __name__=='__main__':

	rospy.init_node('map_node')

	ms = DistCheck()

	rospy.spin()