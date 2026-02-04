import networkx as nx
import yaml
import os
import math
import rospy
from nav_msgs.msg import Odometry 

# 1. Định nghĩa đường dẫn file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPH_FILE = os.path.join(BASE_DIR, "track.graphml")
YAML_FILE = os.path.join(BASE_DIR, "runs.yaml")

# 2. Nạp bản đồ vào biến toàn cục
G = nx.read_graphml(GRAPH_FILE)

# Biến toàn cục để lưu vị trí xe
current_x, current_y = None, None

def odom_callback(msg):
    global current_x, current_y
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

def find_nearest_node(G, x, y):
    nearest_node = None
    min_dist = float('inf')
    for node, data in G.nodes(data=True):
        n_x = float(data.get('x', 0))
        n_y = float(data.get('y', 0))
        dist = math.sqrt((n_x - x)**2 + (n_y - y)**2)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

def get_optimized_order(nodes, start_node):
    unvisited = nodes.copy()
    # Nếu start_node gần xe không nằm trong danh sách blue_nodes, ta vẫn bắt đầu từ nó
    if start_node in unvisited:
        unvisited.remove(start_node)
    
    current_node = start_node
    optimized_path = [current_node]

    while unvisited:
        # Tìm nút gần nhất dựa trên trọng số length (độ dài mét)
        next_node = min(unvisited, key=lambda node: nx.shortest_path_length(G, current_node, node, weight='length'))
        optimized_path.append(next_node)
        unvisited.remove(next_node)
        current_node = next_node
    return optimized_path

def find_full_path(targets):
    full_node_list = []
    for i in range(len(targets) - 1):
        try:
            path = nx.shortest_path(G, source=targets[i], target=targets[i+1], weight='length')
            full_node_list.extend(path if i == 0 else path[1:])
        except nx.NetworkXNoPath:
            print(f"Lỗi: Không tìm thấy đường từ {targets[i]} đến {targets[i+1]}")
            return None
    return full_node_list

def run_planner():
    global current_x, current_y
    
    rospy.init_node('global_planner_script', anonymous=True)
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    print("Đang đợi dữ liệu vị trí từ EKF...")
    while current_x is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if rospy.is_shutdown(): return

    print(f"Đã nhận vị trí EKF: x={current_x:.2f}, y={current_y:.2f}")
    
    # BƯỚC 1: Tìm node gần xe nhất làm điểm xuất phát
    start_node = find_nearest_node(G, current_x, current_y)
    print(f"Xe bắt đầu tại node: {start_node}")

    # BƯỚC 2: Danh sách blue_nodes
    blue_nodes = ['n182', 'n183', 'n93', 'n96', 'n88', 'n144', 'n122', 'n406', 
                  'n423', 'n462', 'n331', 'n341', 'n318', 'n55', 'n243', 'n223', 'n75']

    # BƯỚC 3: Tối ưu hóa thứ tự dựa trên start_node thực tế
    print("Đang tối ưu hóa thứ tự các điểm...")
    optimized_blue_nodes = get_optimized_order(blue_nodes, start_node)
    print(f"Thứ tự tối ưu: {optimized_blue_nodes}")

    # BƯỚC 4: Tìm lộ trình chi tiết
    path_result = find_full_path(optimized_blue_nodes)

    if path_result:
        clean_path = [int(node.replace('n', '')) for node in path_result]
        
        try:
            with open(YAML_FILE, 'r') as f:
                current_data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            current_data = {}

        current_data['runGreedyPath'] = clean_path

        with open(YAML_FILE, 'w') as f:
            yaml.dump(current_data, f, default_flow_style=False)
        
        print(f"Thành công! Đã cập nhật {len(clean_path)} node vào {YAML_FILE}")

if __name__ == '__main__':
    try:
        run_planner()
    except rospy.ROSInterruptException:
        pass