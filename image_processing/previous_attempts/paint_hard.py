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

def check_convex(cnt):
	#instead of just taking the convex hull
	#trianglate the convex shape into triangles
	#TODO
	for i in range(len(cnt)):
		c = cnt[i]
		if not cv2.isContourConvex(c):
			cnt[i] = cv2.convexHull(c)
	return cnt

def paint_order(h):
	res = []
	q = Queue.Queue()
	q.put(0)
	while not q.empty():
		i = q.get()
		res.append(i)
		child = h[i][2]
		next_node = h[i][0]
		parent = h[i][3]
		if next_node > 0:
			q.put(next_node)
		if child > 0:
			q.put(child)
	return res

def select_discrete(h):
	res = []
	q = Queue.Queue()
	q.put(0)
	while not q.empty():
		i = q.get()
		next_node = h[i][0]
		prev_node = h[i][1]
		child = h[i][2]
		parent = h[i][3]
		if parent < 0 or child < 0 or (next_node < 0 and prev_node < 0):
			res.append(i)
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

def get_contours(file_name, approx_convex):
	"return the contours of the image file give, each contour is a list of point vectors."

	orig = cv2.imread(file_name)
	
	im = cv2.cvtColor(orig,cv2.COLOR_BGR2GRAY)
	thresh = im
	# thresh = cv2.adaptiveThreshold(im,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
 #            cv2.THRESH_BINARY,11,2) 

	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	cv2.drawContours(orig, contours, -1,(0,255,0),3), 	
	cv2.imwrite('all_contour.jpg', orig)

	contours = np.array(contours)
	contours = contours[select_discrete(hierarchy[0])]
	contours = filter_contours(contours)
	
	if approx_convex:
		contours = check_convex(contours)

	res = []
	colors = [float]*len(contours)
	for i in range(len(contours)):
		cnt = contours[i]
		mask = np.zeros(im.shape,np.uint8)
		cv2.drawContours(mask,[cnt],0,255,-1)
		
		# cv2.imwrite(str(i) + '_mask.jpg', mask)		
		cnts, h = cv2.findContours(mask, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
		res.append(cnts[0])

		# img = orig[::]
		# cv2.drawContours(img, cnts, -1,(0,255,0),3),
		# cv2.imwrite(str(i) + '_approxNone.jpg', img)

		colors[i] = cv2.mean(orig, mask = mask)

	return res, colors

def paint_contours(cnt, colors, dirc=const.HORZ, brushes=const.FLAT_HARD_SMALL):
	if dirc == const.HORZ:
		dirc = [const.HORZ]*len(colors)
	if brushes == const.FLAT_HARD_SMALL:
		brushes = [const.FLAT_HARD_SMALL]*len(colors)
	
	# order = color.sort_by_brightness(colors)
	# order = sort_by_area(cnt)
	#redundant but needed to work
	cnt, colors, dirc, brushes = reorder([cnt, colors, dirc, brushes], list(range(len(cnt))))
	
	strokes = []
	for i in range(len(cnt)):
		w, c, b = Brush.width_to_pixel(brushes[i]), colors[i], brushes[i] #paint width, color, brush type
		#horizontal stroke
		if dirc[i] == const.HORZ:
			bdry = np.vstack(cnt[i]) #the bounrdry points
			x_table, y_min = build_xtable(bdry)
			x_table = nearest_neighbor(x_table)
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
	
	# print "*************build xtable debug: nearest neighbour***************"
	# for i in range(len(x_table)):
	# 	print x_table[i]

	return x_table

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

	# print "*************build xtable debug: xtable***************"
	# for i in range(len(x_table)):
	# 	if x_table[i][0] > 0:
	# 		print x_table[i]

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
	approx_convex = int(sys.argv[2])
	cnt, colors = get_contours(file, approx_convex)
	dirc = [90]*len(colors)
	# strokes = paint_contours(cnt, colors, dirc)
	strokes = paint_contours(cnt, colors)
	format_json(strokes)
	test('commands.json')
	# print "*************Brush Strokes***************"
	# for i in strokes:
	# 	print i
	# print "End"

if __name__ == "__main__":
	main()