import json
from pprint import pprint
import time


def parse_cmd(data,num):
    num = str(int(num))

    col_info = json.dumps(data[num][0]["color"])
    col_rinfo = (col_info[1:col_info.find(",")])
    temp_info = col_info[len(col_rinfo)+2:len(col_info)]
    col_ginfo = (temp_info[0:temp_info.find(",")])
    temp_info = col_info[len(col_rinfo+col_ginfo)+2:len(col_info)]
    col_binfo = (temp_info[1:temp_info.find("]")])


    path_info = json.dumps(data[num][0]["path"])
    #path_s = (int(path_info[2:path_info.find(",")]),int(path_info[path_info.find(",")+1:path_info.find(")")]))
    path_s = (int(path_info[path_info.find(",")+1:path_info.find(")")]),int(path_info[2:path_info.find(",")]))
    beg_sec = path_info.find("(",2)
    #path_f = (int(path_info[beg_sec+1:path_info.find(",",beg_sec)]),int(path_info[path_info.find(",",beg_sec)+1:len(path_info)-2]))
    path_f = (int(path_info[path_info.find(",",beg_sec)+1:len(path_info)-2]),int(path_info[beg_sec+1:path_info.find(",",beg_sec)]))
    col_r = float(col_binfo[0:-1])
    col_g = float(col_ginfo[0:-1])
    col_b = float(col_rinfo[1:-1])
    #print (path_s, path_f)

    return ((col_r,col_g,col_b),path_s,path_f)
    
def main(file_path):
    cmd_data=open(file_path).read()
    data = json.loads(cmd_data)
    M = []
    for x in range(1,len(data)+1):
        info_num = str(x)
        M.append(parse_cmd(data,info_num))
    #print M
    return M


if __name__ == '__main__':
    main()