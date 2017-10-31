class Brush:
	# actual painting width = width * pressure
	# 0 < pressure < 1
	# type: [flat, round], size: [0,6,8,..]
	def __init__(self, name, pressure, type, size):
		self.name = name
		self.pressure = pressure
		self.type = type
		self.size = size

	def width_to_pixel(brush, canvas=(900,900), img=(900,900)):
		#placeholder for now
		#should calculate from brush size, canvas size and img
		return brush.size

	def __repr__(self):
		return self.name

