#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
import roslib
import rospkg
import numpy
import rosbag
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32, String

current_position = numpy.zeros((35.0, 35.0, 4.0))
mar = Marker()
position_tagging = numpy.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
roslib.load_manifest("ros_pa3")
rospack=rospkg.RosPack()
path=rospack.get_path('ros_pa3')
path=path+'/scripts'
#bag=rosbag.Bag(path)
fi = open(path + "/estimation.txt", "w")

#publishing_the_tags displaying markers on rviz
def publish_tags(show):
	
#marking the position on the grid with the mapping values provided through the gird.bag
	for i in numpy.arange(position_tagging.shape[0]):
		mar = Marker()
		mar.header.frame_id = "/map"
		mar.header.stamp = rospy.Time.now()
		mar.ns = "tag_rviz"
		mar.id = (i+1)
		mar.type = Marker.CUBE
		mar.pose.position.x = position_tagging[i,0]/100.0 -4
		mar.pose.position.y = position_tagging[i,1]/100.0 -4
		mar.pose.position.z = 0
		mar.scale.x = 0.1
		mar.scale.y = 0.1
		mar.scale.z = 0.1 
		mar.color.r = 1.0
		mar.color.g = 1.0
		mar.color.b = 0.0
		mar.color.a = 1.0
		mar.action = Marker.ADD
		show.publish(mar)
		while (show.get_num_connections() < 1):
			yoo = 1
		while (show.get_num_connections() < 1):
			yoo = 1

		for i in numpy.arange(position_tagging.shape[0]):
			mar = Marker()
			mar.header.frame_id = "/map"
			mar.header.stamp = rospy.Time.now()
			mar.ns = "tag_rviz"
			mar.type = Marker.CUBE			
			mar.id = (i+1)
			mar.pose.position.x = position_tagging[5-i,0]/100.0 -4
			mar.pose.position.y = position_tagging[5-i,1]/100.0 -4
			# print("x_pose",m.pose.position.x) 
			# print("y_pose",m.pose.position.y)
			mar.pose.position.z = 0
			mar.color.r = 15.0
			mar.color.g = 12.0
			mar.color.b = 10.0
			mar.color.a = 21.0				
			mar.scale.x = 0.1
			mar.scale.y = 0.1
			mar.scale.z = 0.1 
			mar.action = Marker.ADD
			show.publish(mar)
			while (show.get_num_connections() < 1):
				yoo = 1
			while (show.get_num_connections() < 1):
				yoo = 1
def init(): 
	#initiliazing the tags
	change_rate = rospy.Rate(10)
	bag = rosbag.Bag('/home/apollo/catkin_ws/src/ros_pa3/scripts/grid.bag','r')
	current_position[11,27,2] = 1
	show = rospy.Publisher('tag_marker', Marker, queue_size=150)
	
	publish_tags(show)
	try:
		for topic, msg, time_stamp in bag.read_messages(topics=['Movements', 'Observations']):
			if topic == 'Movements':
				rotation1 = numpy.degrees((euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w]))[2])
				rotation2 = numpy.degrees((euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w]))[2])
				new_pos(rotation1, msg.translation*100, rotation2)
			else: 
				radian = msg.bearing
				radian = numpy.degrees((euler_from_quaternion([radian.x, radian.y, radian.z, radian.w]))[2])
				distance = msg.range * 100				
				pos_obs(msg.tagNum, distance, radian)
	finally:
		bag.close()

def return_pos(i, j, k):
	radian = -135 + k * 90
	# print("r", radian)
	x = i * 20 + 10
	xfinal = x/100.0
	# print("x",x)
	y = j * 20 + 10
	yfinal = y/100.0
	# print("y",y)
	return radian, x, y


def new_pos(rot1, trans, rot2):
	global current_position, temp_pos
	temp_pos = current_position
	current_position = numpy.copy(temp_pos)
	final_p = 0
	for temp_i in numpy.arange(current_position.shape[0]):
		for temp_j in numpy.arange(current_position.shape[1]):
			for temp_k in numpy.arange(current_position.shape[2]):
				if temp_pos[temp_i, temp_j, temp_k] < 0.1:
					continue
				for i in numpy.arange(current_position.shape[0]):
					for j in numpy.arange(current_position.shape[1]):
						for k in numpy.arange(current_position.shape[2]):
							rot1_tmp, trans_tmp, rot2_tmp = calculating(i, j, k, temp_i, temp_j, temp_k)
							#print("", rot1_tmp, "", trans_tmp,"", rot2_tmp)
							trans_p = (1.0/(numpy.sqrt(2*numpy.pi)*10))*numpy.power(numpy.e,-1.0*(((trans_tmp-trans)**2)/(2.0*10**2)))
							rot1_p = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot1_tmp-rot1)**2)/(2.0*45**2)))
							rot2_p = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot2_tmp-rot2)**2)/(2.0*45**2)))

							value = temp_pos[temp_i, temp_j, temp_k] * trans_p * rot1_p * rot2_p
							current_position[i, j, k] = current_position[i, j, k] + value
							final_p = final_p + value
	current_position = current_position / final_p
	index = numpy.argmax(current_position)
	disp_angle = index % current_position.shape[2]
	#print("angle after:", disp_angle)
	index = index / current_position.shape[2]
	disp_y = index % current_position.shape[1]
	#print("y after:", disp_y)
	index = index / current_position.shape[1]
	disp_x = index % current_position.shape[0]
	#print("x after:", disp_x)
	#print "P:(",disp_x,"," ,disp_y,"," ,disp_angle,")"
	rviz_display(disp_x, disp_y, disp_angle)


"""def roundDegree(r):
    if(r < -180):
        r = r + 360
    elif(r > 180):
        r = r - 360
    return r"""


def pos_obs(tagging_with_numbers, trans, rot):
	global current_position, temp_pos
	temp_pos = current_position
	current_position = numpy.copy(temp_pos)
	final_p = 0
	for i in numpy.arange(current_position.shape[0]):
		for j in numpy.arange(current_position.shape[1]):
			for k in numpy.arange(current_position.shape[2]):
				rot_tmp, trans_tmp = Observation(i, j, k, tagging_with_numbers)
				rot_prb = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot_tmp-rot)**2)/(2.0*45**2)))
				trans_prb = (1.0/(numpy.sqrt(2*numpy.pi)*10))*numpy.power(numpy.e,-1.0*(((trans_tmp-trans)**2)/(2.0*10**2)))
				value = temp_pos[i, j, k] * trans_prb * rot_prb
				current_position[i, j, k] = value
				final_p = final_p + value
	current_position = current_position / final_p
	index = numpy.argmax(current_position)
	index_angle = index % current_position.shape[2]
	#print("angle", index_angle)
	index = index / current_position.shape[2]
	index_y = index % current_position.shape[1]
	#print("y:", index_y)
	index = index / current_position.shape[1]
	index_x = index % current_position.shape[0]
	#print("x:", index_x)
	#print"U:(",index_x,"," ,index_y,"," ,index_angle,")"
	rviz_display_final(index_x, index_y, index_angle)


"""def ComputeParams(x, y, z, x1, y1, z1):
    r = math.degrees(math.atan2((y1-y), (x1-x)))
    trans = math.sqrt((x1 - x)**2 + (y1 -y)**2)
    r1 = roundDegree(r - z)
    r2 = roundDegree(z1 - z - r1)
    return trans, r1, r2"""
def rviz_display(x, y, z):
	global current_position,mar
	show = rospy.Publisher('line_marker', Marker, queue_size=150)
	mar.header.frame_id = "/map"
	mar.header.stamp = rospy.Time.now()
	mar.id = 0
	mar.ns = "line_rviz"	
	mar.type = Marker.LINE_STRIP
	mar.color.r = 15.0
	mar.color.g = 19.0
	mar.color.b = 70.0
	mar.color.a = 11.0	
	mar.scale.x = 0.05
	mar.scale.y = 0.0
	mar.scale.z = 0.0 
	point = Point()
	display_angle, display_x, display_y = return_pos(x, y, z)
	print "P:", "(",display_x/100.0, ", ", display_y/100.0," , " ,display_angle,")"
	point.x = display_x/100.0 -4.0
	point.y = display_y/100.0 -4.0
	#print("X :", display_x/100.0)
	#print("Y :", display_y/100.0)
	point.z = 0
	mar.points.append(point)
	mar.action = Marker.ADD
	show.publish(mar)
	while (show.get_num_connections() < 1):
		yo = 1
	

def rviz_display_final(x, y, z):
	global current_position,mar
	show = rospy.Publisher('line_marker', Marker, queue_size=150)
	mar.header.frame_id = "/map"
	mar.header.stamp = rospy.Time.now()
	mar.id = 0
	mar.ns = "line_rviz"	
	mar.type = Marker.LINE_STRIP
	mar.color.r = 15.0
	mar.color.g = 19.0
	mar.color.b = 70.0
	mar.color.a = 11.0	
	mar.scale.x = 0.05
	mar.scale.y = 0.0
	mar.scale.z = 0.0 
	point = Point()
	display_angle, display_x, display_y = return_pos(x, y, z)
	print"U:","(", display_x/100.0," , ",display_y/100.0," , ", display_angle, ")"
	point.x = display_x/100.0 -4.0
	point.y = display_y/100.0 -4.0
	#print("X :", display_x/100.0)
	#print("Y :", display_y/100.0)
	point.z = 0
	mar.points.append(point)
	mar.action = Marker.ADD
	show.publish(mar)
	while (show.get_num_connections() < 1):
		yo = 1

def Observation(x, y, z, tagging_with_numbers):
	global position_tagging
	rotations, trans_x, trans_y = return_pos(x, y, z)
	angle = numpy.degrees(numpy.arctan2(position_tagging[tagging_with_numbers,1]-trans_y, position_tagging[tagging_with_numbers,0] - trans_x))	
	trans = numpy.sqrt((trans_x - position_tagging[tagging_with_numbers,0]) ** 2 + (trans_y - position_tagging[tagging_with_numbers,1]) ** 2)
	radian = angle - rotations
	if radian > 180:
		radian = radian - 360
	elif radian < -180:
		radian = radian + 360
	return radian, trans

def calculating(x, y, z, temp_x, temp_y, temp_z):
	radian1, t1_x, t1_y = return_pos(x, y, z)
	radian2, t2_x, t2_y = return_pos(temp_x, temp_y, temp_z)
	trans = numpy.sqrt((t1_x - t2_x) ** 2 + (t1_y - t2_y) ** 2)
	angle = numpy.degrees(numpy.arctan2(t1_y-t2_y, t1_x - t2_x))
	rotations1 = radian1 - angle	
	rotations2 = angle - radian2
	if rotations2 > 180:
		rotations2 = rotations2 - 360
	elif rotations2 < -180:
		rotations2 = rotations2 + 360		
	if rotations1 > 180:
		rotations1 = rotations1 - 360
	elif rotations1 < -180:
		rotations1 = rotations1 + 360
	return rotations2, trans, rotations1

if __name__ == '__main__':
	try:
		rospy.init_node('pa3')
		init()
	except rospy.ROSInterruptException:
		pass
