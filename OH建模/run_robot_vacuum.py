import mujoco
import time
import math
import sys
import random

# 使用 robot_vacuum_v2.xml
MODEL_PATH = "robot_vacuum_v2.xml"

# 控制参数 (Motor Torque)
FORWARD_TORQUE = 0.5
TURN_TORQUE = 0.3
BACKUP_TORQUE = 0.4
OBSTACLE_DIST = 0.5
WALL_DIST = 0.4
CLIFF_THRESHOLD = 0.05  # 地面高度约 0.02，超过 0.05 视为悬崖

# 状态定义
STATE_CRUISE = 0
STATE_AVOID = 1
STATE_BACKUP = 2
STATE_CLIFF = 3

class RobotController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.state = STATE_CRUISE
        self.state_timer = 0
        self.turn_direction = 1 # 1 for left, -1 for right

        # 获取 ID
        try:
            self.id_act_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_l")
            self.id_act_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_r")
            
            self.id_rng_f = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_front")
            self.id_rng_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_left")
            self.id_rng_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_right")
            self.id_clf_f = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "cliff_front")
        except Exception as e:
            print(f"初始化控制器失败: {e}")
            sys.exit(1)

    def get_sensors(self):
        d_front = self.data.sensordata[self.model.sensor_adr[self.id_rng_f]]
        d_left = self.data.sensordata[self.model.sensor_adr[self.id_rng_l]]
        d_right = self.data.sensordata[self.model.sensor_adr[self.id_rng_r]]
        d_cliff = self.data.sensordata[self.model.sensor_adr[self.id_clf_f]]
        
        # 修正 -1 为 10.0 (无穷远)
        if d_front < 0: d_front = 10.0
        if d_left < 0: d_left = 10.0
        if d_right < 0: d_right = 10.0
        if d_cliff < 0: d_cliff = 10.0
        
        return d_front, d_left, d_right, d_cliff

    def step(self):
        d_front, d_left, d_right, d_cliff = self.get_sensors()
        
        ctrl_l = 0.0
        ctrl_r = 0.0

        # 状态机逻辑
        if self.state == STATE_CRUISE:
            # 默认前进
            ctrl_l = FORWARD_TORQUE
            ctrl_r = FORWARD_TORQUE
            
            # 1. 悬崖检测 (优先级最高)
            if d_cliff > CLIFF_THRESHOLD:
                print("检测到悬崖！紧急后退")
                self.state = STATE_CLIFF
                self.state_timer = 30 # 持续 30 步
                
            # 2. 前方避障
            elif d_front < OBSTACLE_DIST:
                print(f"前方障碍 ({d_front:.2f}m)，开始避障")
                self.state = STATE_AVOID
                self.state_timer = 50 # 转向持续时间
                # 决定转向方向：向空旷的一侧转
                if d_left > d_right:
                    self.turn_direction = 1 # 左转
                else:
                    self.turn_direction = -1 # 右转
            
            # 3. 沿墙修正 (仅微调，不切换状态)
            elif d_left < WALL_DIST:
                 ctrl_l = FORWARD_TORQUE
                 ctrl_r = FORWARD_TORQUE * 0.5
            elif d_right < WALL_DIST:
                 ctrl_l = FORWARD_TORQUE * 0.5
                 ctrl_r = FORWARD_TORQUE

        elif self.state == STATE_AVOID:
            # 原地转向
            if self.turn_direction == 1: # 左转
                ctrl_l = -TURN_TORQUE
                ctrl_r = TURN_TORQUE
            else: # 右转
                ctrl_l = TURN_TORQUE
                ctrl_r = -TURN_TORQUE
                
            self.state_timer -= 1
            
            # 如果前方空旷了，提前结束避障
            if d_front > OBSTACLE_DIST * 1.5 and self.state_timer < 40:
                self.state = STATE_CRUISE
            
            if self.state_timer <= 0:
                self.state = STATE_CRUISE

        elif self.state == STATE_CLIFF:
            # 后退
            ctrl_l = -BACKUP_TORQUE
            ctrl_r = -BACKUP_TORQUE
            self.state_timer -= 1
            if self.state_timer <= 0:
                # 后退完后，旋转一下以离开危险区
                self.state = STATE_AVOID
                self.state_timer = 40
                self.turn_direction = 1 # 默认左转离开

        # 应用控制
        self.data.ctrl[self.id_act_l] = ctrl_l
        self.data.ctrl[self.id_act_r] = ctrl_r

def main():
    print(f"正在加载模型: {MODEL_PATH}")
    try:
        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"加载模型失败: {e}")
        return

    # 初始化控制器
    robot = RobotController(model, data)

    # 尝试启动查看器
    viewer = None
    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    except ImportError:
        print("未找到查看器，将在无头模式下运行。")

    print("开始自动巡航...")
    start_time = time.time()
    
    while True:
        if viewer and not viewer.is_running():
            break
            
        # 机器人思考并行动
        robot.step()
        
        # 物理模拟
        mujoco.mj_step(model, data)
        
        if viewer:
            viewer.sync()
            time.sleep(model.opt.timestep)
        else:
             # 无头模式每秒输出一次状态
             if int((time.time() - start_time) * 10) % 20 == 0:
                 d_f, d_l, d_r, d_c = robot.get_sensors()
                 print(f"状态: {robot.state} | 前: {d_f:.2f} | 悬崖: {d_c:.3f}")
                 time.sleep(0.05)

if __name__ == "__main__":
    main()