import cv2
import numpy as np
from artKit import *
import json
import sys
import math
import Queue
import random
from pprint import pprint
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
import time

PERCENTAGE = 0.9
BATCH_SIZE = 100

def pointillism(file_name,colors, brush=const.FLAT_HARD_SMALL):
	orig = cv2.imread(file_name)
	x,y,_ = orig.shape

	size = brush.width_to_pixel(brush)
	orig = cv2.resize(orig, (y/size, x/size))
	colors = np.array(colors)
	img = np.array(orig)
	img = reduce_colors(orig, colors)
	cv2.imwrite("reduced.jpg", img)
	colors = colors[color.sort_by_brightness(colors)]
	strokes = []
	for c in colors:
		sub = get_one_color_pixels(img, c)
		selected_pixels_index = random.sample(range(len(sub)), int(PERCENTAGE*len(sub)))
		sub = sub[selected_pixels_index]
		point_strokes = [Stroke(c, brush, (x*size,x*size)) for x in sub]
		strokes.append(point_strokes)
	strokes = [y for x in strokes for y in x]
	return strokes

def reduce_colors(orig, colors):
	for i in range(len(orig)):
		for j in range(len(orig[0])):
			this_color = orig[i][j]
			if this_color not in colors:
				orig[i][j] = sorted(colors, key=lambda x: np.linalg.norm(x-this_color))[0]
				# print "before: %s after: %s" %(str(temp), str(orig[i][j]))
	return orig

#COULD BE OPTIMIZED
def get_one_color_pixels(img, c):
	x,y,_=img.shape
	res = np.zeros((x,y))
	for i in range(x):
		for j in range(y):
			if np.array_equal(img[i][j],c):
				res[i][j] = 1
	return np.transpose(np.nonzero(res))

def format_json(strokes, layer):
	# "formats list of strokes into json format and outputs a commands.json file"
	commands = {}
	n = len(strokes)
	batch = range(0,n, BATCH_SIZE)
	i = 0
	j = i+1
	while j < len(batch):
		# print("formatting json for batch (%i,%i)" % (batch[i],batch[j]))
		for k in range(batch[i],batch[j]):
			stroke = str(strokes[k]).split()
			color = str(strokes[k].color)
			r,g,b = strokes[k].color
			color = '[%s,%s,%s]' %(str(r),str(g), str(b))
			brush = str(strokes[k].brush.name)
			path = stroke[1]
			cmd = k+1
			commands[cmd] = []
			commands[cmd].append ({
				'brush': brush,
				'path': path,
				'color': color
			})
		i = i+1
		j = i+1

	# print("formatting json for batch (%i,%i)" % (batch[-1],n))
	for k in range(batch[-1],n):
		stroke = str(strokes[k]).split()
		color = str(strokes[k].color)
		r,g,b = strokes[k].color
		color = '[%s,%s,%s]' %(str(r),str(g), str(b))
		brush = str(strokes[k].brush.name)
		path = stroke[1]
		cmd = k+1
		commands[cmd] = []
		commands[cmd].append ({
			'brush': brush,
			'path': path,
			'color': color
		})

	json_file = 'layer_%i_commands.json' %layer
	with open(json_file,'w') as outfile:
		json.dump(commands,outfile,sort_keys=True)

def test(imgsize = (900,900), brush = const.FLAT_HARD_SMALL, layer=0, l_file = ""):
	cmd_data=open('commands.json').read()
	data = json.loads(cmd_data)
	if not layer:
		print "creating a new image"
		img = np.zeros((imgsize[0],imgsize[1],3))
		img[:,:] = [255,255,255]
	else:
		print "layering on %s" % l_file
		img = cv2.imread(l_file)
	for x in range(1,len(data)):
		if x < 10:
			info_num = "0" + str(x)
		else:
			info_num = str(x)
		color, point = parse_cmd(data,info_num)
		size = brush.width_to_pixel(brush)
		x, y = point
		# print color
		for i in range(-size/2, size/2):
			for j in range(-size/2, size/2):
				x2 = min(max(0,x+i),imgsize[0])
				y2 = min(max(0,y+j),imgsize[1])
				img[x2,y2] = color
	file = 'layer_%i_img.png' %layer
	cv2.imwrite(file, img)

def parse_cmd(data,num):
	num = str(int(num))
	path_info = json.dumps(data[num][0]["path"])
	point = (int(path_info[2:path_info.find(",")]),int(path_info[path_info.find(",")+1:path_info.find(")")]))
	color = json.dumps(data[num][0]["color"])
	color = color[2:-2].split(',')
	color = map(float,color)
	return (color,point)

def base(file,clist):
  img = np.array(cv2.imread(file))
  w,h,c = img.shape
  res = img

  # res = [closest_c(j,clist) for i in img for j in i]
  for i in range(w):
	for j in range(h):
	  res[i,j]=closest_c(img[i,j],clist)

  cv2.imwrite('base.png',res)

def closest_c(color,clist):
		closer = sorted(clist, key=lambda c: np.linalg.norm(c-color))
		return closer[0]

def dominantcolors(file,K):
  img = cv2.imread(file)
  # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  w,h,c = img.shape
  img = img.reshape((w * h, 3))

  # Clusters the pixels
  clt = KMeans(n_clusters = K)
  clt.fit(img)
  # clt.fit(img)

  numLabels = np.arange(0, len(np.unique(clt.labels_)) + 1)
  (hist, _) = np.histogram(clt.labels_, bins = numLabels)

  # normalize the histogram, such that it sums to one
  hist = hist.astype("float")
  hist /= hist.sum()

  centroid = clt.cluster_centers_
  clist = centroid.tolist()
  return np.round(clist)

if __name__ == "__main__":

	start_time = time.time()
	file = str(sys.argv[1])
	K = int(sys.argv[2])
	layering = int(sys.argv[3])
	colors = dominantcolors(file,K)
	base(file,colors)
	print("%s seconds for finding dominant colors" % (time.time() - start_time))
	img = cv2.imread('base.png')

	for layer in range(0, layering+1):
		random.seed(hash(time.time()))
		print ("----------------layer %i-------------------------" %layer)	
		start_time = time.time()	
		
		strokes = pointillism(file, colors, brush = const.FLAT_HARD_SUPER_SMALL)

		print("%s seconds for outputing strokes" %(time.time() - start_time))
		print("%i number of strokes: " % (len(strokes)))
		
		start_time = time.time()
		format_json(strokes, layer)
		
		print("%s seconds for formatting json" % (time.time() - start_time))

		start_time = time.time()
		x,y,_ =img.shape
		layer_file = 'layer_%i_img.png' %(layer-1)
		test(imgsize=(x,y), brush = const.FLAT_HARD_SUPER_SMALL, layer = layer, l_file = layer_file)
		print("%s seconds for testing" % (time.time() - start_time))






