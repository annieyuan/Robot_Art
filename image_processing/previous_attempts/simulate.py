from artKit import graphics
from artKit import parser
import json
import time

BRUSH_SIZE = 10
def simulate(data):
	"""currently does not work because mac does not support tk"""
	#open graphics window

	win = graphics.GraphWin("Painting Sim", 900, 900)
	#getting json data, start and end hard coded
	for x in range(1,len(data)):
		if x < 10:
			info_num = "0" + str(x)
		else:
			info_num = str(x)

		line_info = parser.parse_cmd(data,info_num)
		path_s = line_info[1]
		path_f = line_info[2]

		#drawings
		line = graphics.Line(graphics.Point(path_s[0],path_s[1]),graphics.Point(path_f[0],path_f[1]))
		line.setWidth(BRUSH_SIZE)
		line.setFill(graphics.color_rgb(line_info[0][0],line_info[0][1],line_info[0][2]))
		line.draw(win)
		time.sleep(0.01)

	win.getMouse() # Pause to view result

if __name__ == "__main__":
	cmd_data=open('commands.json').read()
	data = json.loads(cmd_data)
	simulate(data)