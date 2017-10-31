import math
import numpy as np
import webcolors

def sort_by_brightness(colors):
	#sort colors according to light to dark: 
	#sort by brightness = sqrt( .299 R2 + .587 G2 + .114 B2 )
	#refer to http://alienryderflex.com/hsp.html
	#input colors: (r,g,b,alpha)
	#return an array of indices (the order of which each contour should be painted)
	brightness = [-math.sqrt(0.299*color[0] + 0.587*color[1] + 0.114*color[2]) for color in colors]
	return np.argsort(brightness)


# from http://stackoverflow.com/questions/9694165/convert-rgb-color-to-english-color-name-like-green
def closest_colour(requested_colour):
    min_colours = {}
    for key, name in webcolors.css3_hex_to_names.items():
        r_c, g_c, b_c = webcolors.hex_to_rgb(key)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        min_colours[(rd + gd + bd)] = name
    return min_colours[min(min_colours.keys())]

def get_colour_name(requested_colour):
    try:
        closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
    except ValueError:
        closest_name = closest_colour(requested_colour)
        actual_name = None
    return actual_name, closest_name

