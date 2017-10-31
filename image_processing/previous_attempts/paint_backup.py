import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
from artKit import *
import sys
import json
from collections import OrderedDict
import re
import scipy.misc as smp
import time
import math
import Queue

def paint(img):
	"given an image (one layer), output the json cmds for painting this image"
	return json_cmds

def filter_contours(cnt, DIFF = 0.1, MIN = 20):

	area = [cv2.contourArea(c) for c in cnt]
	cnt, area = reorder([cnt, area], np.argsort(area)[::-1])
	res = []
	i,j = 0,1
	while (j < len(cnt)):
		if len(res)==0:
			if area[i] > MIN:
				res.append(cnt[i])
			else:
				i+=1
				j+=1

		elif abs(area[i]-area[j]) > DIFF*area[i] and area[j] > MIN:
			res.append(cnt[j])
			i+=1
			j+=1
		else:
			j+=1
	return res

def paint_order(h):
	res = []
	q = Queue.Queue()
	q.put(0)
	while not q.empty():
		i = q.get()
		res.append(i)
		child = h[i][2]
		next_node = h[i][0]
		if next_node > 0:
			q.put(next_node)
		if child > 0:
			q.put(child)
	return res

def find_contours(imgray):
	# print np.shape(imgray)
	
	thresh = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2) 
	
	cv2.imwrite('threshold_G.jpg', thresh)

	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	cv2.drawContours(imgray, contours, -1, (0,255,0), 3)
	cv2.imwrite('all_contours.jpg', imgray)

	cnt_discrete = [None]*len(contours)
	# grey_scale = [0.0]*len(contours)
	for k in range(len(contours)):
		
		cnt = contours[k]
		mask = np.zeros(imgray.shape,np.uint8)	
		cv2.drawContours(mask,[cnt],0,255,-1)
		
		cv2.imwrite(str(k)+ '_mask.jpg', mask)

		
		thresh = cv2.adaptiveThreshold(mask, 255 ,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,11,2)
		cv2.imwrite(str(k)+ '_threshold.jpg', thresh)
		cnt2, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		cnt_discrete[k] = cnt2[0]
	
	return cnt_discrete
	# grey_scale = [0.0]*len(contours)
	# for i in range(len(contours)):
	# 	cnt = contours[i]
	# 	mask = np.zeros(imgray.shape,np.uint8)
	# 	cv2.drawContours(mask,[cnt],0,255,-1)
	# 	grey_scale[i] = cv2.mean(imgray, mask = mask)[0]

	# grey_scale.sort(reverse=True) #sort by descending order, from light -> dark 
	# print grey_scale

	# cnt_discrete = [None]*len(grey_scale)
	# for i in range(len(grey_scale)):
	# 	# ret,thresh = cv2.threshold(imgray, grey_scale[i], 255,cv2.THRESH_BINARY)
	# 	th2 = cv2.adaptiveThreshold(imgray, 255 ,cv2.ADAPTIVE_THRESH_MEAN_C,\
 #            cv2.THRESH_BINARY,11,2)
	# 	cv2.imwrite(str(i)+ '_threshold.jpg', thresh)
	# 	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	# 	cnt_discrete[i] = contours[1]
	# return cnt_discrete

def get_contours(file_name):
	"return the contours of the image file give, each contour is a list of point vectors."
	#cnt in clockwise x-y order
	im = cv2.imread(file_name)

	# im = cv2.medianBlur(im,5)
	
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	# cv2.imwrite('gray.jpg', imgray)
	th2 = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,11,2)

	# ret,thresh = cv2.threshold(imgray,100,255,cv2.THRESH_BINARY)
	# cv2.imwrite('threshold_trial.jpg', thresh)

	contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	
	# thresh = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
 #            cv2.THRESH_BINARY,11,2) 

	# # ret,thresh = cv2.threshold(imgray,127,255,0)
	# contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	# contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	# contours = filter_contours(contours)
	# area = [cv2.contourArea(c) for c in contours]
	# print area 

	cv2.drawContours(im, contours, -1,(0,255,0),3), 
	cv2.imwrite('all_contour.jpg', im)


	order = paint_order(hierarchy[0])
	# print order
	contours = np.array(contours)
	contours = contours[order]
	


	colors = [float]*len(contours)
	for i in range(len(contours)):
		cnt = contours[i]
		mask = np.zeros(imgray.shape,np.uint8)
		cv2.drawContours(mask,[cnt],0,255,-1, hierarchy = hierarchy, maxLevel = 0)
		cv2.imwrite(str(i) + '_mask.jpg', mask)
		colors[i] = cv2.mean(im, mask = mask)
	
	for i in range(len(contours)):	
		img = im	
		cv2.drawContours(img, contours,i,(0,255,0), -1), 
		cv2.imwrite(str(i) + '_contour.jpg', img)

	return contours, colors

def paint_contours(cnt, colors, dirc=const.HORZ, brushes=const.FLAT_HARD_SMALL):
	"""
	cnt: array of size n, cnt[i]: the pixels position of each contour
	colors: n x 3
	given a list of contours, output the painting command lists each contour with the color given
	dir: option argument, if not given, all contours will be filled with horizontal strokes:
	left to right

	IDEAS:
	figure out why currently the curves contours do no work -> find contours return points
	in a clockwise direction, go around the boundry, not exactly what we want,
	but we can write a function that figure out if a point is in the contour
	semi-randomly generate strokes ( 30% flat, 40% vertical, 30% circle)
	as long as all values are in the contour
	"""
	if dirc == const.HORZ:
		dirc = [const.HORZ]*len(colors)
	if brushes == const.FLAT_HARD_SMALL:
		brushes = [const.FLAT_HARD_SMALL]*len(colors)
	
	# order = color.sort_by_brightness(colors)
	
	#redundant 
	# order = sort_by_area(cnt)
	cnt, colors, dirc, brushes = reorder([cnt, colors, dirc, brushes], list(range(len(cnt))))
	
	strokes = []
	for i in range(len(cnt)):
		w, c, b = Brush.width_to_pixel(brushes[i]), colors[i], brushes[i] #paint width, color, brush type
		
		#horizontal stroke
		if dirc[i] == const.HORZ:
			bdry = np.vstack(cnt[i]) #the bounrdry points
			# bdry = bdry[bdry[:,1].argsort(kind='mergesort')] # sort by the y coordinate: top -> bottom

			x_table, y_min = build_xtable(bdry)
			x_table = nearest_neighbor(x_table)

			# if i == 1:
			# 	print bdry[:100]
			# 	print x_table[:100]
			j = 0
			while j < len(x_table):
				y = j + y_min
				xl, xr = x_table[j]
				path = ((xl,y), (xr,y))
				strokes.append(Stroke(c,b,path))
				j+=w/2

		else:  #NOT WORKING
			bdry = np.vstack(cnt[i]) #the bounrdry points
			
			print "*****************before dot"
			for j in bdry[:50]:
				print j
			a = math.radians(dirc[i])
			M = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]) 
			M_inv = M.transpose()
			
			bdry = np.rint((np.dot(M, bdry.transpose())).transpose()) #this is not right 
			bdry.astype(int)
			#bdry = [np.rint(np.dot(M,pt)) for pt in bdry] #CAN BE OPTIMIZED
			print "******************after dot"
			for x in bdry[:50]:
				print x

			# bdry = bdry[bdry[:,1].argsort(kind='mergesort')] # sort by the y coordinate: top -> bottom

			x_table, y_min = build_xtable(bdry)
			x_table = nearest_neighbor(x_table)

			# if i == 1:
			# 	print bdry[:100]
			# 	print x_table[:100]
			j = 0
			while j < len(x_table):
				y = j + y_min
				xl, xr = x_table[j]
				print xl, xr, y
				xl, y2 = np.rint(np.dot(M_inv, (xl,y)))
				xr, y3 = np.rint(np.dot(M_inv, (xr,y)))
				print xl, xr, y2
				path = ((int(xl),int(y2)), (int(xr),int(y3)))
				strokes.append(Stroke(c,b,path))

				j+=w/2

	return strokes

def sort_by_area(cnt):
	area = [-cv2.contourArea(c) for c in cnt] 
	return np.argsort(area) #area in descending order

#helper: order everylist in the input list by the order indice given
def reorder(parameters, order):
	result = []
	for p in parameters:
		p = np.array(p)[order]
		result.append(p)
	return result


#CAN BE OPTIMIZED
def nearest_neighbor(x_table):
	"""fill in blank by getting the closest neighbor above the point"""
	for i in range(len(x_table)):
		x_left, x_right = x_table[i]
		if x_left < 0:
			x_table[i] = x_table[i-1]
	return x_table

#NOT IMPLEMENTED
def linear_interpolate(x_table):
	"""fill in blank of x_value with linear interpolation"""
	return None

def build_xtable(cnt):
	"""
	cnt: contour boundries points sorted by y
	get x_left and x_right for each contour
		return a 2d array, cnt[i]= (x_left, x_right) at y_min + i"""
	# print "*************build xtable debug: cnt***************"
	# print cnt

	cnt, y_span, y_min = set_y_zero(cnt)
	x_table = [[] for i in range(int(y_span))]
	for i in range(len(cnt)):
		y = int(cnt[i][1])
		x = int(cnt[i][0])
		x_table[y].append(x)

	x_table = [ [min(i or [-1]), max(i or [-1])] for i in x_table]

	# for i in range(len(x_table)):
	# 	cur = x_table[i]
	# 	if (len(cur) > 1):
	# 		x_table[i] = [min(cur), max(cur)]
	# 	elif (len(cur) == 1):
	# 		x = cur[0]
	# 		prev_l, prev_r = x_table[i-1]
	# 		if x > prev_l:
	# 			x_table[i] = [prev_l, x]
	# 		else:
	# 			x_table[i] = [x, prev_r]
	# 	else:
	# 		x_table[i] = x_table[i-1]
			
	# print "*************build xtable debug: xtable***************"
	# for i in range(min(20,len(x_table))):
	# 	print x_table[i]

	cnt[:,1]+=y_min #restore the y_coordinates
	return x_table,y_min

def set_y_zero(cnt):
	cnt_sorted = cnt[cnt[:,1].argsort(kind='mergesort')]
	# print "*************build xtable debug: cnt_sorted***************"
	# print cnt_sorted
	
	y_min = cnt_sorted[0][1]
	y_max = cnt_sorted[-1][1]
	y_span = y_max-y_min + 1
	cnt[:,1]-=y_min
	return cnt, y_span, y_min

#helper for horizontal painting: find the next (start,end) given the brush width, cur pos
def next_stroke(w,bdry,j,y_next):
	# print('next_stroke({} {})'.format(w,j))
	y_max = bdry[-1][1]
	y = bdry[j][1]
	# print ("y cur  is {}".format(y))
	# print ("y next is {}".format(y_next))
	for k in range(j, len(bdry),2):
		if bdry[k][1] > y_next:
			return (((bdry[k-2][0], y_next), (bdry[k-1][0],y_next)),j, y_next+w/2)
		elif bdry[k][1] == y_next:
			return ((bdry[k],bdry[k+1]),j, y_next+w/2)
		else:
			continue
	return (None, -1, None)

#sample json command
#{"01": [{"brush": "FLAT_HARD_SMALL", "color": ["56.24705882", "98.88235294", "141.1"], "path": "(151,111)(486,111)"}]"
#note the rgb value is in fact bgr value...
def format_json(strokes):
	"formats list of strokes into json format and outputs a commands.json file"
	commands = {}
	for i in range(len(strokes)):
		stroke = str(strokes[i]).split()
		color = (str(strokes[i].color).split())[1:4]
		brush = str(strokes[i].brush.name)
		path = stroke[1]
		cmd = i+1
		# if len(cmd)<=1:
		# 	cmd = '0'+cmd
		commands[cmd] = []
		commands[cmd].append ({
			'brush': brush,
			'path': path,
			'color': color
		})
		with open('commands.json','w') as outfile:
			json.dump(commands,outfile,sort_keys=True)


#for now imgsize is set to(900,900)
#for now brush = FLAT_HARD_SMALL defined in constants
#for now only handles horizontal strokes
def test(jsonfilename,imgsize = (900,900),b = const.FLAT_HARD_SMALL):
	"for testing the cmds, given the json cmds, output the painting"
	img = np.zeros( (900,900,3), dtype=np.uint8) #initiates a 900*900*3 array of 8 bit unsigned int

	#loading json
	with open(jsonfilename) as datafile:
		commands = json.load(datafile,object_pairs_hook=OrderedDict)

	for cmd in commands:
		color = commands[cmd][0]['color']
		# print type(color[0])
		color = map(float,color)
		tmp = color[0]
		color[0]=color[2]
		color[2]=tmp
		#http://stackoverflow.com/questions/8494514/converting-string-to-tuple
		tmp = re.findall('\([^)]*\)',commands[cmd][0]['path']) #list of string tuples in the path
		points_on_path = [tuple(int(el2) for el2 in el.strip('()').split(',')) for el in tmp]

		brushsize = Brush.width_to_pixel(b)

		# only start and endpoints and only horizontal strokes
		startx,starty = points_on_path[0]
		endx,endy = points_on_path[1]
		brstarty = starty-(brushsize/2)
		bendy = starty+(brushsize/2)
		img[brstarty:bendy,startx:endx] = color

	im = smp.toimage(img)       # Create a PIL image
	im.show()

def main():
	# print sys.version
	file = str(sys.argv[1])
	cnt, colors = get_contours(file)
	dirc = [90]*len(colors)
	# strokes = paint_contours(cnt, colors, dirc)
	# strokes = paint_contours(cnt, colors)
	# format_json(strokes)
	# test('commands.json')
	# print "*************Brush Strokes***************"
	# for i in strokes:
	# 	print i
	# print "End"

if __name__ == "__main__":
	main()