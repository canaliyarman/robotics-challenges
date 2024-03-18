#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap, GetMapResponse
from datetime import datetime
from move_base_client import GoalSender
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus
import math
# Random Explorer Class
class RandomExplorer:

    def __init__(self):
        self.previous_goals = []
        # Define a Simple Goal Client
        self.bad_statuses = [4, 5 , 8 , 9]
        self.goal_sender = GoalSender()
        self.currently_moving = False
        self.run_once = True
        self.latest_map_msg = None
        self.latest_map = None
        self.previous_goal_map_msg = None
        self.previous_goal_map = None
        self.previous_cells_to_explore = None
        self.previous_idx = None
        self.cost_map = []
        self.point_count = 256
        self.previous_goals_max_size = 10
        # self.subscriber_costmap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.update_costmap)
        self.subscriber_map = rospy.Subscriber('/map', OccupancyGrid, self.update_map_subscribed)
        self.subscriber_move_base = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.printFeedback)
        # Define a Service for Map
        #rospy.wait_for_service('/dynamic_map')
        #self.get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)

    
    def printFeedback(self, feedback_data):
        if feedback_data.status in self.bad_statuses:
            # rospy.loginfo("WE DUN GOOFED, CANCELLING CURRENT GOAL MAKING NEW ONE")
            # rospy.loginfo(feedback_data.status)
            rospy.loginfo("BAD GOAL ABORT MISSION")
            goal_sender.cancel_goal()         
            # self.goal_sender.currently_moving

    def update_map_subscribed(self, map_data):
        if map_data.info.width <= 64:  #Default size of map before any input (64x64)
            return
        self.latest_map_msg = map_data
        self.latest_map = np.array(self.latest_map_msg.data)
        rospy.loginfo("Got new map message, currently moving value: " + str(self.currently_moving))  
        if not self.goal_sender.currently_moving:
            goal_msg = self.process_map()  
            self.goal_sender.send_goal(goal_msg)
            self.previous_goal_map = self.latest_map
            self.previous_goal_map_msg = self.latest_map_msg
        else:
            self.risk_check()
    # def update_costmap(self, map_data):
    #     self.cost_map = np.array(map_data.data)
    #     rospy.loginfo("-------------------------------------")
    #     rospy.loginfo(self.cost_map)       

    def update_map(self):

        try:
            map_resp = self.get_map_srv()
            assert isinstance(map_resp, GetMapResponse)

            if map_resp.map.info.width <= 64:  #Default size of map before any input (64x64)
                return False

            self.latest_map_msg = map_resp.map
            self.latest_map = np.array(self.latest_map_msg.data)
            # print(self.latest_map_msg.info)

        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed to get map: %s" % e)
            return False

        return True

    def risk_check(self):
        current_goal = self.previous_goals[-1]
        rospy.loginfo("Current goal is " + str(current_goal))
        cells_explored = np.count_nonzero(self.latest_map > -1)
        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        res = self.latest_map_msg.info.resolution
        
        cells = 0
        cells_to_pick = np.zeros((cells_explored, 2))
        
        previous_idx_of_goal = int((self.previous_goal_map_msg.info.width * current_goal[1]) + current_goal[0])
        rospy.loginfo("PREV IDX OF GOAL " + str(previous_idx_of_goal))
        rospy.loginfo("ORIGIN OF CURRENT MAP " + str([self.latest_map_msg.info.origin.position.x, self.latest_map_msg.info.origin.position.y]))
        rospy.loginfo("ORIGIN OF PREVIOUS MAP " + str([self.previous_goal_map_msg.info.origin.position.x, self.previous_goal_map_msg.info.origin.position.y]))
        
        for y in xrange(0, height):
            for x in xrange(0, width):
                idx = x + y * width

                if self.latest_map[idx] > -1:
                    cells_to_pick[cells][0] = x
                    cells_to_pick[cells][1] = y
                        # cells_to_pick[cells][2] = self.euclidean_distance(self.previous_goals[0],cells_to_pick[cells][0], cells_to_pick[cells][1])
                    cells = cells + 1
        rospy.loginfo("value of previous grid in current map " + str(self.latest_map[previous_idx_of_goal]))
        if self.latest_map[previous_idx_of_goal] != 0:
            rospy.loginfo("BAD GOAL ABORT MISSION")
            self.goal_sender.cancel_goal()
        else:
            rospy.loginfo("GOOD GOAL GO AHEAD")
    def process_map(self):

        # Get Map Metadata
        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        res = self.latest_map_msg.info.resolution
        map_origin = np.array([self.latest_map_msg.info.origin.position.x, self.latest_map_msg.info.origin.position.y])
        map_frame_id = self.latest_map_msg.header.frame_id
        stamp = self.latest_map_msg.header.stamp

        # Pick Goal and Create Msg
        cells_to_pick = self.get_valid_cells(height, self.latest_map, width)
        goal = None
        if self.run_once or len(self.previous_goals) < 2:
            goal = self.get_goal(cells_to_pick, map_origin, res)
            self.run_once = False
        else:
            goal = self.get_better_goal(cells_to_pick, map_origin, res)

        goal_msg = self.make_goal_msg(goal, map_frame_id)

        return goal_msg

    # Return explored cells
    def get_valid_cells(self, height, gridmap, width):
        # Get Number of Explored Cells
        cells_explored = np.count_nonzero(gridmap > -1)
        rospy.loginfo("Cells Explored %i", cells_explored)

        # Create an Array with Cells to Pick
        cells = 0
        cells_to_pick = np.zeros((cells_explored, 2))
        for y in xrange(0, height):
            for x in xrange(0, width):
                idx = x + y * width

                if gridmap[idx] > -1 and gridmap[idx] < 5:
                    cells_to_pick[cells][0] = x
                    cells_to_pick[cells][1] = y
                        # cells_to_pick[cells][2] = self.euclidean_distance(self.previous_goals[0],cells_to_pick[cells][0], cells_to_pick[cells][1])
                    cells = cells + 1
        
        return cells_to_pick

    # Select Random Valid Cell
    def get_goal(self, cells_to_pick, map_origin, res):
        # Pick Cell
        distance = 0
        best_goal = None
        # for cell in cells_to_pick:
        #     current_distance = self.euclidean_distance([0.0,0.0],cell)
        #     if current_distance > distance:
        #         goal = cell
        rand_idx = np.random.randint(0, len(cells_to_pick))
        goal = cells_to_pick[rand_idx]
        rospy.loginfo("---------------------")
        
        self.previous_goals.append(np.array(goal))
        rospy.loginfo(self.previous_goals)
        goal = (goal * res) + map_origin
        return goal

    
    def euclidean_distance(self, p1, p2):
        distance = math.sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        return distance
    def get_better_goal(self, cells_to_pick, map_origin, res):
        rospy.loginfo("GETTING BETTER GOAL")
        rospy.loginfo("PREVIOUS GOALS : " + str(self.previous_goals))
        rospy.loginfo(str(cells_to_pick))
        rospy.loginfo(cells_to_pick.shape)
        best_goal = None
        best_distance = 0
        # mean = np.mean(cells_to_pick, axis=2)
        # treshold_dist = np.max(cells_to_pick, axis=2) * 0.75
        # for index, x in np.ndenumerate(cells_to_pick):
        #     if x[2] > mean and x[2] < treshold_dist:
        #         best_goal = x
        possible_goals = []
        for prev_goal in self.previous_goals[:-1]:
            for cell in cells_to_pick:
                # ADD SELF.LATESTMAP CHECK FOR VALUES HERE
                current_distance_to_grid = self.euclidean_distance(cell, prev_goal)
                
                if  current_distance_to_grid > best_distance:
                    rospy.loginfo("nBEST = "+ str(current_distance_to_grid) + " BEST GOAL = " + str(cell) + "for Prev Goal = " + str(prev_goal))
                    best_goal = cell
                    best_distance = current_distance_to_grid
                possible_goals.append((current_distance_to_grid, cell))
        possible_goals = sorted(possible_goals, key= lambda x:x[0], reverse=True)
        possible_goals = possible_goals[:self.point_count]
        rospy.loginfo(possible_goals)
        closest = 99999999
        best_goal = None
        for possible_goal in possible_goals:
            current_distance_to_point = self.euclidean_distance(possible_goal[1], self.previous_goals[-1])
            if current_distance_to_point < closest:
                best_goal = possible_goal[1]
                
        best_goal = possible_goals[0][1]
        
        # # rand_idx = np.random.randint(0, len(cells_to_pick))
        
        # goal = cells_to_pick[rand_idx]
        goal = best_goal
        self.previous_goals.append(goal)
        goal = (goal * res) + map_origin
        if len(self.previous_goals) > self.previous_goals_max_size:
            self.previous_goals.pop(0)
        
        return goal
    def make_goal_msg(self, goal, frame_id="map"):
        # Get Message
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1
        goal_msg.header.frame_id = frame_id
        goal_msg.header.stamp = rospy.Time.now()
        return goal_msg

    def send_goal(self, goal):
        raise NotImplementedError()

    def loop(self):
        start_time = datetime.now()
        while not rospy.is_shutdown():
            rospy.loginfo("New Goal...")
            if self.cost_map == []:
                continue
            if self.update_map():
                goal_msg = self.process_map()
                self.goal_sender.send_goal(goal_msg)

if __name__ == '__main__':
    rospy.init_node('random_explorer')
    explorer = RandomExplorer()
    #explorer.loop()
    rospy.spin()
