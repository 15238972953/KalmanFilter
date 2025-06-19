import matplotlib.pyplot as plt
import re

# 开关变量：控制是否打印
TRUE_DATA                = 1    
CAM_DATA                 = 0
CAM_KALMANFILTER_DATA    = 0
RADAR_DATA               = 1
RADAR_KALMANFILTER_DATA  = 1
FUSION_DATA              = 0
FUSION_KALMANFILTER_DATA = 0

def parse_line(line):
    # 提取一行中的所有 (x,y)
    matches = re.findall(r'\(([^,]+),([^)]+)\)', line)
    return [(float(x), float(y)) for x, y in matches]

def read_all_groups(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    groups = [parse_line(line.strip()) for line in lines]
    return groups

true_data, cam_data, cam_kalmanfilter_data, radar_data, radar_kalmanfilter_data, \
            fusion_data, fusion_kalmanfilter_data = read_all_groups("cmake-build-debug/data.txt") # 文件路径根据自己实际情况修改

# 提取 x, y
def extract_xy(group):
    x = [pt[0] for pt in group]
    y = [pt[1] for pt in group]
    return x, y

# 提取数据
if TRUE_DATA: 
    x1, y1 = extract_xy(true_data)
    plt.plot(x1, y1, '-r', label='true_pos')  # 红色连线
if CAM_DATA: 
    x2, y2 = extract_xy(cam_data)
    plt.plot(x2, y2, '-g', label='cam_pos')
if CAM_KALMANFILTER_DATA: 
    x3, y3 = extract_xy(cam_kalmanfilter_data)
    plt.plot(x3, y3, '-b', label='cam_pos_kalmanfilter')
if RADAR_DATA: 
    x4, y4 = extract_xy(radar_data)
    plt.plot(x4, y4, '-m', label='radar_pos')
if RADAR_KALMANFILTER_DATA: 
    x5, y5 = extract_xy(radar_kalmanfilter_data)
    plt.plot(x5, y5, '-y', label='radar_pos_kalmanfilter')
if FUSION_DATA:
    x6, y6 = extract_xy(fusion_data)
    plt.plot(x6, y6, '-c', label='fused_pos')
if FUSION_KALMANFILTER_DATA:
    x7, y7 = extract_xy(fusion_kalmanfilter_data)
    plt.plot(x7, y7, '-k', label='fused_pos_kalmanfilter')


plt.xlabel('X')
plt.ylabel('Y')
plt.title('Line Plot of 7 Groups')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
