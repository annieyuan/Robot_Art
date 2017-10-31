from . import color
class Stroke:
	#given the brush type, path is interpreted different
	#straight painting: horizontal, vertical, diagnoal (no curve): path is just a pair (start_pt, end_pt)
	#for curve lines: [FOR NOW] a list of points along the path ((x1,y1), (x2,y2), etc)
	def __init__(self, color, brush, path):
		self.color = color
		self.brush = brush
		self.path = path

	def __repr__(self):
		c = color.get_colour_name(self.color)[1]
		path = ""
		for i in self.path:
			path = path + "({},{})".format(i[0],i[1])
		return "{} {} {}".format(self.brush, path, c)