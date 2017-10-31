import json
from pprint import pprint

def parse_cmd(data,num):
	num = str(int(num))


	path_info = json.dumps(data[num][0]["path"])
	path_s = (int(path_info[2:path_info.find(",")]),int(path_info[path_info.find(",")+1:path_info.find(")")]))

	beg_sec = path_info.find("(",2)
	path_f = (int(path_info[beg_sec+1:path_info.find(",",beg_sec)]),int(path_info[path_info.find(",",beg_sec)+1:len(path_info)-2]))

	color = json.dumps(data[num][0]["color"])
	# print type(color)
	# print color
	# print color[2:-2]
	color = color[2:-2].split(',')
	color = map(float,color)
	col_b, col_g, col_r = color
	# pprint(((col_r,col_g,col_b),path_s,path_f))
	return ((col_r,col_g,col_b),path_s,path_f)
	

