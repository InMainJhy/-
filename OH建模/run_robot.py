try:
    import mujoco
except ImportError:
    print("错误: 未找到 'mujoco' 库。")
    print("请运行以下命令安装: pip install mujoco")
    exit(1)

import time
import math
import random

# Load the model
model_path = "cafe_building.xml"
try:
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading model: {e}")
    exit(1)

# 机器人控制参数
WHEEL_RADIUS = 0.05
MAX_SPEED = 2.0  # 弧度/秒
TURN_SPEED = 1.0
OBSTACLE_THRESHOLD = 0.5 # 米

# 获取执行器 ID
try:
    id_wheel_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_wheel_l")
    id_wheel_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_wheel_r")
    
    # 获取传感器 ID
    id_range_front = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_front")
    id_range_left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_left")
    id_range_right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_right")
except:
    print("无法找到机器人的执行器或传感器。请确保 cafe_building.xml 已更新。")
    exit(1)

def get_sensor_value(sensor_id):
    # 传感器数据地址
    adr = model.sensor_adr[sensor_id]
    # 测距仪返回 1 个值
    return data.sensordata[adr]

print("正在启动扫地机器人仿真...")
print("按 Ctrl+C 停止。")

# Initialize viewer (if available) or run headless
# Since we are in a script, we'll try to use the native viewer if installed,
# otherwise just run the physics loop.
# Note: In a real desktop env, user runs `python run_robot.py`.
# We will use the 'viewer' handle if possible, but standard mujoco python 
# usually requires `mujoco.viewer.launch_passive` or similar.

try:
    import mujoco.viewer
    viewer_available = True
except ImportError:
    viewer_available = False
    print("未找到 MuJoCo 查看器。正在以无头模式运行。")

def controller(model, data):
    # 读取传感器
    dist_front = get_sensor_value(id_range_front)
    dist_left = get_sensor_value(id_range_left)
    dist_right = get_sensor_value(id_range_right)
    
    # 逻辑：类 Braitenberg 避障
    # 如果前方有障碍物，随机转向或背向侧面障碍物
    
    # 如果范围内没有击中任何物体，测距仪返回 -1（通常默认 10m）
    if dist_front == -1: dist_front = 10.0
    if dist_left == -1: dist_left = 10.0
    if dist_right == -1: dist_right = 10.0
    
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED
    
    if dist_front < OBSTACLE_THRESHOLD:
        # 前方受阻
        print(f"前方障碍物! ({dist_front:.2f}m)")
        if dist_left < dist_right:
            # 右转
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED
        else:
            # 左转
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED
            
    elif dist_left < 0.3:
        # 离左墙太近，稍微右转
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED * 0.5
        
    elif dist_right < 0.3:
        # 离右墙太近，稍微左转
        left_speed = MAX_SPEED * 0.5
        right_speed = MAX_SPEED
        
    # 应用控制
    data.ctrl[id_wheel_l] = left_speed
    data.ctrl[id_wheel_r] = right_speed

if viewer_available:
    mujoco.viewer.launch(model, data, loader=None)
else:
    # 无头模式循环
    start = time.time()
    while time.time() - start < 60: # 运行 60 秒演示
        controller(model, data)
        mujoco.mj_step(model, data)
        # 睡眠以大致匹配实时（如果我们在可视化）
        time.sleep(model.opt.timestep)
    print("仿真结束。")
