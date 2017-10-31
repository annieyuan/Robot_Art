import cv2
import numpy as np
import sys
from sklearn.cluster import KMeans
import math
from matplotlib import pyplot as plt

def base(file,clist):
	img = cv2.imread(file)
	w,h,c = img.shape
	res = img
	for i in range(w):
		for j in range(h):
			res[i,j]=closest_c(img[i,j],clist)
	cv2.imwrite('base.png',res)

def closest_c(color,clist):
	b,g,r = color

	closest = [0.0,0.0,0.0]
	min_dis = 1000000.0
	for c in clist:
		b2,g2,r2 = c
		distance = ((r2-r)*0.30)**2 + ((g2-g)*0.59)**2 + ((b2-b)*0.11)**2
		if distance<min_dis:
			min_dis = distance
			closest = c

	return closest



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
	for i in range(len(clist)):
		for j in range(len(clist[0])):
			clist[i][j] = int(round(clist[i][j]))

	return clist

def main():
		file = str(sys.argv[1])
		K = int(sys.argv[2])
		clist = dominantcolors(file,K)
		base(file,clist)
		print clist

if __name__ == "__main__":
	main()