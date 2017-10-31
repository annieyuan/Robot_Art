#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import sys
from _pyvicon_kuka import ViconStreamer
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose

class waypoint_navigation:
    
    def __init__(self):
        
        print("Waypoint following node initialized.")
        
        # Initialize vicon streaming
        print("Initializing Vicon Streamer.")
        self.s = ViconStreamer()
        self.s.connect("128.84.189.209", 800)
    
        #if len(sys.argv) > 1 and sys.argv[1] in ["-l", "--list"]:
            #s.printStreamInfo()
            #sys.exit(0)
    
        self.streams = self.s.selectStreams(["Time", "KUKAyouBot2:main_body <t-X>", "KUKAyouBot2:main_body <t-Y>", "KUKAyouBot2:main_body <a-Z>"])
        self.s.startStreams(verbose=False)
        
        # Initialize cmd_vel publisher
        print("Initializing kuka command publisher")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize waypoint list subscriber
        print("Initializing waypoint list subscriber")
        self.list_sub = rospy.Subscriber('/waypoint_list',PoseArray ,self.callback, queue_size = 10)

        self.vMax = 0.15
        self.omegaMax = 0.5
        self.epsilon = 0.3
        self.epMat = np.array([[1,0],[0,1/self.epsilon]])
        
        self.distanceThreshold = 400
        
        self.list_counter = 0
        
        

    def callback(self,waypoint_list):
        
        print("New waypoint list received, executing waypoint following ...")
        
        #print(waypoint_list)

        self.list_counter = self.list_counter + 1
        
        
        currentPose = np.array([0,0,0]) # x,y,theta
        while self.s.getData() is None:
            pass
        (t, x, y, o) = self.s.getData()
        print("Initial position:" + str(x) + "," + str(y))
        currentPose[0] = x
        currentPose[1] = y
        currentPose[2] = o
        
        waypointIndex = 0
        numWaypoints = len(waypoint_list.poses)
        
        currentWaypoint = Pose()
        
        currentWaypoint = waypoint_list.poses[waypointIndex]
        nextWaypoint = waypoint_list.poses[waypointIndex + 1]
        
        print("First waypoint:")
        print(currentWaypoint)
        
        # Execute waypoint navigation for new list
        while waypointIndex < numWaypoints:
            newCmd = Twist()
            newCmd.linear.x = 0
            newCmd.linear.y = 0
            newCmd.linear.z = 0   
            
            newCmd.angular.x = 0
            newCmd.angular.y = 0
            newCmd.angular.z = 0
                        
            # get current position
            (t, x, y, o) = self.s.getData()
            currentPose[0] = x
            currentPose[1] = y
            currentPose[2] = o
            
            # euclidean distance to next goal
            diffx = currentWaypoint.position.x - x
            diffy = currentWaypoint.position.y - y
            goalDist = math.sqrt(math.pow(diffx,2) + math.pow(diffy,2))
            
            wpDx = nextWaypoint.position.x - currentWaypoint.position.x
            wpDy = nextWaypoint.position.y - currentWaypoint.position.y
            wpAng = np.arctan2(wpDy,wpDx)
            desOmega = wpAng - o
            
            if goalDist < self.distanceThreshold:
                waypointIndex = waypointIndex + 1
                if waypointIndex == numWaypoints:
                    diffx = 0
                    diffy = 0
                    desOmega = 0
                else:
                    currentWaypoint = waypoint_list.poses[waypointIndex]
                    if waypointIndex < (numWaypoints - 1):
                        nextWaypoint = waypoint_list.poses[waypointIndex + 1]
                        
                    print("Next waypoint:")
                    print(currentWaypoint)
                    diffx = currentWaypoint.position.x - x
                    diffy = currentWaypoint.position.y - y
                
            # Determine Vx, Omega commands
            #vDes = [[diffx],[diffy]]
            
            #angMat = [[math.cos(o), math.sin(o)],[-math.sin(o),math.cos(o)]]
            
            #print(self.epMat)
            #print(angMat)
            #print(vDes)
              
            desLocalX = math.cos(o)*diffx + math.sin(o)*diffy
            desLocalY = -math.sin(o)*diffx + math.cos(o)*diffy
            
            #desOmega = desLocalY/self.epsilon  # tunable parameter 
            
            #print("Desired Local X/Y Velocities:" + str(desLocalX) + ", " + str(desLocalY))
            
            if abs(desLocalX) > self.vMax:
                newCmd.linear.x = self.vMax*np.sign(desLocalX)
            else:
                newCmd.linear.x = desLocalX
                
            if abs(desLocalY) > self.vMax:
                newCmd.linear.y = self.vMax*np.sign(desLocalY)
            else:
                newCmd.linear.y = desLocalY
                
            if abs(desOmega) > self.omegaMax:
                newCmd.angular.z = self.omegaMax
            else:
                newCmd.angular.z = desOmega
                
                
            #newCmd.linear.y = 0
            #newCmd.angular.z = 0
                
                    
            # send command to robot
            self.cmd_pub.publish(newCmd)
            #print(newCmd)
        
            
    def test_waypoints(self):
        #waypoint_list = np.array([[3551,1060],[21,3167]])  

        waypoint_list = np.array([[200,100],[300,100],[400,100],[500,100],[600,100],[700,100],[800,100],[900,100],[1000,100],[1100,100],[1200,100],[1300,100],[1400,100],[1500,100],[1500,200],[1500,300],[1500,400],[1500,500],[1500,600],[1500,700],[1500,800],[1500,900],[1500,1000],[1500,1100],[1600,1100],[1600,1200],[1600,1300],[1600,1400],[1700,1400],[1700,1500],[1800,1500],[1800,1600],[1800,1700],[1900,1700],[1900,1800],[2000,1800],[2000,1900],[2100,1900],[2100,2000],[2200,2000],[2300,2000],[2300,2100],[2400,2100],[2400,2200],[2500,2200],[2600,2200],[2700,2200],[2700,2300],[2800,2300],[2900,2300],[3000,2300],[3000,2400],[3000,2500],[3000,2600],[3000,2700],[3000,2800],[3000,2900],[3000,3000],[3000,3100],[3000,3200],[3000,3300],[3000,3400],[3000,3500],[3000,3600],[3000,3700]])
        
        currentPose = np.array([0,0,0]) # x,y,theta
        (t, x, y, o) = self.s.getData()
        currentPose[0] = x
        currentPose[1] = y
        currentPose[2] = o
        
        waypointIndex = 0
        numWaypoints = waypoint_list.shape[0]
        
        currentWaypoint = waypoint_list[waypointIndex]
        
        # Execute waypoint navigation for new list
        while waypointIndex < numWaypoints:
            newCmd = Twist()
            newCmd.linear.x = 0
            newCmd.linear.y = 0
            newCmd.linear.z = 0   
            
            newCmd.angular.x = 0
            newCmd.angular.y = 0
            newCmd.angular.z = 0
                        
            # get current position
            (t, x, y, o) = self.s.getData()
            currentPose[0] = x
            currentPose[1] = y
            currentPose[2] = o
            
            # euclidean distance to next goal
            diffx = currentWaypoint[0] - x
            diffy = currentWaypoint[1] - y
            goalDist = math.sqrt(math.pow(diffx,2) + math.pow(diffy,2))
            
            if goalDist < self.distanceThreshold:
                waypointIndex = waypointIndex + 1
                if waypointIndex == numWaypoints:
                    diffx = 0
                    diffy = 0
                else:
                    currentWaypoint = waypoint_list[waypointIndex]
                    print(currentWaypoint)
                    diffx = currentWaypoint[0] - x
                    diffy = currentWaypoint[1] - y
                
            # Determine Vx, Omega commands
            vDes = [[diffx],[diffy]]
            
            #angMat = [[math.cos(o), math.sin(o)],[-math.sin(o),math.cos(o)]]
            
            #print(self.epMat)
            #print(angMat)
            #print(vDes)
            
            '''
            desiredMotion = np.dot(self.epMat,angMat)
            desiredMotion = np.dot(desiredMotion,vDes)
            
            if desiredMotion[0] > self.vMax:
                newCmd.linear.x = self.vMax
            else:
                newCmd.linear.x = desiredMotion[0]
                
            if desiredMotion[1] > self.omegaMax:
                newCmd.angular.z = self.omegaMax
            else:
                newCmd.angular.z = desiredMotion[1]
                '''
                
            desLocalX = math.cos(o)*diffx + math.sin(o)*diffy
            desLocaly = -math.sin(o)*diffx + math.cos(o)*diffy
            
            if abs(desLocalX) > self.vMax:
                newCmd.linear.x = self.vMax*np.sign(desLocalX)
            else:
                newCmd.linear.x = desLocalX
                
            if abs(desLocaly) > self.vMax:
                newCmd.linear.y = self.vMax*np.sign(desLocaly)
            else:
                newCmd.linear.y = desLocaly
            
                    
            # send command to robot
            #self.cmd_pub.publish(newCmd)
            #print(newCmd)
        
    def follow_waypoints(self):
        
        #self.test_waypoints()
              
        while not rospy.is_shutdown():
            print("Number of lists received:", self.list_counter)
            time.sleep(1)

            
    '''
    def clean_shutdown(self):
        print("\nExiting thingy...")
        #return to normal
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True
        '''
            
            
def main(args):

    rospy.init_node('waypoint_navigation')
    implemented_waypoints = waypoint_navigation()
    #implemented_waypoints.follow_waypoints()
    #print "main started"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
 

if __name__ == '__main__':
    main(sys.argv)