#!/usr/bin/env python

import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
import tf

'''
Before running this code, the following packages must be installed:

apriltags_ros
freenect_launch

And the following launch files must be running:

freenect_launch freenect.launch
apriltags_ros example.launch


Note: to see the image with the tag detections, run 
rosrun image_view image_view image:=/tag_detections_image
'''

#keeps track of positions in the KUKA's arm frame
#key is the location according to tag_translation
kuka_positions = {}

#keeps track of tag positions in camera frame
(x, y, z) = (-10, -10, -10)   #dummies so we know invalid positions
tag_positions = {
	"tag_13": (x, y, z)	#attached to robot base
} #the rest of the dictionary is populated in main

#keeps track of which tag is at which position in physical space
tag_translation = {
	"tag_12": "upperL",
	"tag_14": "upperR",
	"tag_20": "lowerR",
	"tag_11": "lowerL",
	"tag_15": "paint1",
	"tag_16": "paint2",
	"tag_17": "paint3",
	"tag_18": "paint4",
	"tag_19": "paint5",
	"tag_21": "paint6",
	"tag_9": "water",
	"tag_7": "towel",
	"tag_4": "test",
}

#x and z distances of tag_13 from arm base in meters
#note that the y_dist should be 0 if tag_13 is placed appropriately
x_dist = 0.25
z_dist = 0

def main():
    print "in main"
    rospy.init_node("paint_positions", anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    listener = tf.TransformListener()

    publishers = {}

    for tag in tag_translation:
	tag_positions[tag] = (x, y, z)
	pub_name = tag_translation[tag] + "_pos"
	publishers[tag] = rospy.Publisher(pub_name, Point, queue_size=10)
	

    while not rospy.is_shutdown():
	try:
	    #calculates the position of all tags in the frame of tag_13
	    for tag in tag_positions:
		if (tag_positions[tag] != (-10, -10, -10) and tag_positions['tag_13'] != (-10, -10, -10)):
	    	    (trans, rot) = listener.lookupTransform(tag, 'tag_13', rospy.Time(0))
		    if (tag != "tag_13"):
			print trans
			#(x, y, z) for trans becomes (y, -x, -z) in arm frame for some reason
			#keep internal track of location relative the base
			kuka_positions[tag_translation[tag]] = (-trans[1] - x_dist, trans[0], -trans[2] - z_dist)
			[x_pos, y_pos, z_pos] = [kuka_positions[tag_translation[tag]][0], kuka_positions[tag_translation[tag]][1], kuka_positions[tag_translation[tag]][2]]
			#publish the information 
			publishers[tag].publish(x_pos, y_pos, z_pos)


	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    continue



def tag_callback(data):
    for i in range(len(data.detections)):
	
    	br = tf.TransformBroadcaster()

	#position to the center of the tag in the frame of the camera
    	position = data.detections[i].pose.pose.position
     	orientation = data.detections[i].pose.pose.orientation

	#print orientation.w

    	tag_name = "tag_" + str(data.detections[i].id)
	tag_positions[tag_name] = position

	#broadcast all positions of tags relative camera frame to tf in order to do transform
	#br.sendTransform((position.x, position.y, position.z), tf.transformations.quaternion_from_euler(0, 0, orientation.w), rospy.Time.now(), tag_name, "camera")
	br.sendTransform((position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), tag_name, "camera")


def get_positions():
    return kuka_positions



if __name__=='__main__':
    try:
	main()
	rospy.spin()
	print "terminating positions.py"
	print kuka_positions
    except rospy.ROSInterruptException:
	pass



