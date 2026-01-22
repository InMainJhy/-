import mujoco
import time
import math
import sys

# 使用新的 V2 模型
MODEL_PATH = "robot_vacuum_v2.xml"

# 控制参数 (Motor Torque)
# Motor control input is usually -1 to 1 (mapped to torque via gear)
FORWARD_TORQUE = 0.5
TURN_TORQUE = 0.3
OBSTACLE_DIST = 0.5
WALL_DIST = 0.4

def main():
    print(f"Loading model: {MODEL_PATH}")
    try:
        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    try:
        id_act_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_l")
        id_act_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_r")
        
        id_rng_f = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_front")
        id_rng_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_left")
        id_rng_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "range_right")
    except:
        print("Error getting IDs")
        return

    # Try viewer
    viewer = None
    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    except ImportError:
        print("Running headless.")

    print("Starting simulation loop...")
    
    start_time = time.time()
    while True:
        if viewer and not viewer.is_running():
            break
        
        # 1. Get Sensor Data
        d_front = data.sensordata[model.sensor_adr[id_rng_f]]
        d_left = data.sensordata[model.sensor_adr[id_rng_l]]
        d_right = data.sensordata[model.sensor_adr[id_rng_r]]
        
        # Fix -1
        if d_front < 0: d_front = 10.0
        if d_left < 0: d_left = 10.0
        if d_right < 0: d_right = 10.0

        # 2. Control Logic
        # Default: Move Forward
        ctrl_l = FORWARD_TORQUE
        ctrl_r = FORWARD_TORQUE
        
        # Obstacle Avoidance
        if d_front < OBSTACLE_DIST:
            # Turn
            if d_left > d_right:
                # Turn Left (Left wheel reverse, Right wheel forward)
                ctrl_l = -TURN_TORQUE
                ctrl_r = TURN_TORQUE
            else:
                # Turn Right
                ctrl_l = TURN_TORQUE
                ctrl_r = -TURN_TORQUE
        
        # Wall Following
        elif d_left < WALL_DIST:
             # Steer Right
             ctrl_l = FORWARD_TORQUE
             ctrl_r = FORWARD_TORQUE * 0.5
        elif d_right < WALL_DIST:
             # Steer Left
             ctrl_l = FORWARD_TORQUE * 0.5
             ctrl_r = FORWARD_TORQUE

        # 3. Apply Control
        data.ctrl[id_act_l] = ctrl_l
        data.ctrl[id_act_r] = ctrl_r

        # 4. Step
        mujoco.mj_step(model, data)
        
        if viewer:
            viewer.sync()
            time.sleep(model.opt.timestep)
        else:
             if int((time.time() - start_time) * 10) % 20 == 0:
                 print(f"F: {d_front:.2f} | L: {d_left:.2f} | R: {d_right:.2f}")
                 time.sleep(0.05)

if __name__ == "__main__":
    main()