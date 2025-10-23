import time
import numpy as np
import torch
import csv
import datetime
import pandas as pd
import serial
from DNN_torch_ram import DNNRam, LSTMNetwork, load_nn


# ------------ 模拟 IMU：读取 Mocap 数据 ---------------------
def read_GeorgiaTech_dataset(speed):
    if speed == "0.6":
        angle_file = "./normal_walk_1_0-6/AB12_normal_walk_1_0-6_angle.csv"
        velocity_file = "./normal_walk_1_0-6/AB12_normal_walk_1_0-6_velocity.csv"
    elif speed == "1.2":
        angle_file = "./normal_walk_1_1-2/AB12_normal_walk_1_1-2_angle.csv"
        velocity_file = "./normal_walk_1_1-2/AB12_normal_walk_1_1-2_velocity.csv"
    elif speed == "1.8":
        angle_file = "./normal_walk_1_1-8/AB12_normal_walk_1_1-8_angle.csv"
        velocity_file = "./normal_walk_1_1-8/AB12_normal_walk_1_1-8_velocity.csv"
    else:
        raise ValueError(f"Invalid speed: {speed}")

    df_angle = pd.read_csv(angle_file)
    df_vel = pd.read_csv(velocity_file)
    qT_R = np.array(df_angle['hip_flexion_r'])
    qT_L = np.array(df_angle['hip_flexion_l'])
    dqT_R1 = np.array(df_vel['hip_flexion_velocity_r'])
    dqT_L1 = np.array(df_vel['hip_flexion_velocity_l'])

    data = {
        "qT_L": qT_L,
        "qT_R": qT_R,
        "dqT_L1": dqT_L1,
        "dqT_R1": dqT_R1,
    }
    return data

# ---------------- Loopback 串口发送函数 --------------------
def float_to_uint(val, min_val, max_val, bits=16):
    span = max_val - min_val
    return int((val - min_val) * ((2**bits - 1) / span))

def send_and_verify_serial_output(L_cmd, R_cmd, ser):
    try:


        B1 = float_to_uint(L_cmd, -20, 20)
        B2 = float_to_uint(R_cmd, -20, 20)

        b1 = (B1 >> 8) & 0xFF
        b2 = B1 & 0xFF
        b3 = (B2 >> 8) & 0xFF
        b4 = B2 & 0xFF

        msg = bytearray([0x40, 0x41, 0x42, b1, b2, b3, b4, 0x43])
        ser.write(msg)
        time.sleep(0.01)
        n = ser.in_waiting
        reply = ser.read(n) if n > 0 else b''


        print(f"TX: {list(msg)} | RX: {list(reply)}")
    except Exception as e:
        print(f"[WARN] Serial send failed: {e}")
# -----------------------------------------------------------


ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

# Load NN
kp = 50
kd = 0.5 * np.sqrt(kp)
dnn = load_nn(
    saved_policy_path = "./nn_para/lstm/current_exo.pt",
    nn_type = 'lstm',
    kp = kp,
    kd = kd
)

# Load dataset
dataset = read_GeorgiaTech_dataset("1.2")
qT_L = dataset["qT_L"]
qT_R = dataset["qT_R"]
dqT_L = dataset["dqT_L1"]
dqT_R = dataset["dqT_R1"]

# Save CSV
date = time.localtime(time.time())
csv_filename = f"./data/Lily/simulated_{date.tm_year:04}{date.tm_mon:02}{date.tm_mday:02}-{date.tm_hour:02}{date.tm_min:02}{date.tm_sec:02}.csv"
with open(csv_filename, 'w', newline='') as csvfile:
    fieldnames = ['Time', 'L_IMU_Ang', 'R_IMU_Ang', 'L_IMU_Vel', 'R_IMU_Vel', 'L_Cmd', 'R_Cmd']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    start = time.time()
    for i in range(len(qT_L)):
        now = time.time() - start
        L_IMU_angle = qT_L[i]
        R_IMU_angle = qT_R[i]
        L_IMU_vel   = dqT_L[i]
        R_IMU_vel   = dqT_R[i]

        dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel)

        L_Cmd = dnn.hip_torque_L * 0.1
        R_Cmd = dnn.hip_torque_R * 0.1
        L_Cmd = np.clip(L_Cmd, -5.0, 5.0)
        R_Cmd = np.clip(R_Cmd, -5.0, 5.0)

        send_and_verify_serial_output(L_Cmd, R_Cmd, ser)

        data = {
            'Time': now,
            'L_IMU_Ang': L_IMU_angle,
            'R_IMU_Ang': R_IMU_angle,
            'L_IMU_Vel': L_IMU_vel,
            'R_IMU_Vel': R_IMU_vel,
            'L_Cmd': L_Cmd,
            'R_Cmd': R_Cmd
        }
        writer.writerow(data)
        csvfile.flush()

        time.sleep(0.01)
