#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi5 ↔ Teensy (RX8/TX8) – 4 个 IMU 量 → NN 推理 → 2 个扭矩回写
2025-08-06、
Pi TX (GPIO14) → Teensy RX8 (pin 34)

Pi RX (GPIO15) ← Teensy TX8 (pin 35)

记得共地。
"""

import serial, struct, time, csv, datetime
import numpy as np
import torch
#from DNN_torch_ram import load_nn      # 你现有的加载函数
from DNN_torch_end2end import  load_nn  
# ========= 通讯配置 =========
SER_DEV      = '/dev/serial0'   # GPIO14/15 默认 UART0
BAUDRATE     = 115200
TIMEOUT      = 0.01             # 20 ms
PI_USE_BINARY = 1               # 1=二进制帧 (A5 5A), 0=文本CSV

# ========= NN / 实验参数 =========
kp, kd          = 50, 0.5*np.sqrt(50)

dnn             = load_nn('./nn_para/end2end/run/max_exo.pt', nn_type='lstm', kp=kp, kd=kd)

CMD_MIN, CMD_MAX = -15.0, 15.0
L_Ctl, R_Ctl    = 1.0, 1.0      # 左/右腿比例
kcontrol        = 0.1           # 额外缩放

# ========= 日志文件 =========
ts   = datetime.datetime.now()
root = './Lily/s1x75-'
logf = (root + ts.strftime('%Y%m%d-%H%M%S') + '.csv')
csv_header = ['Time','L_IMU_Ang','R_IMU_Ang','L_IMU_Vel','R_IMU_Vel',
              'L_Cmd','R_Cmd']

# ========= 串口工具函数 =========
def cksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF

def read_packet(ser: serial.Serial):
    """
    返回 (RTx_deg, LTx_deg, LTx_vel, RTx_vel) 或 None
    二进制模式：帧格式
      A5 5A LEN(17) TYPE(01) [4×float32] CHKSUM
    文本模式：'%.6f,%.6f,%.6f,%.6f\\n'
    """
    if not PI_USE_BINARY:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return None
        try:
            return tuple(map(float, line.split(',')))
        except ValueError:
            return None

    # ---------- 二进制 ----------
    while True:
        h = ser.read(1)

        if not h:
            return None
        if h == b'\xA5' and ser.read(1) == b'\x5A':
            break
    ln = ser.read(1)
    if not ln or ln[0] != 17:
        return None
    typ = ser.read(1)
    if not typ or typ[0] != 0x01:
        return None
    payload = ser.read(16)
    if len(payload) != 16:
        return None
    chk = ser.read(1)
    if not chk or chk[0] != cksum8(typ + payload):
        return None
    return struct.unpack('<ffff', payload)

def send_torque(ser: serial.Serial, tau_L: float, tau_R: float):
    """
    向 Teensy 发送 2×float32 (小端) 扭矩命令
    """
    ser.write(struct.pack('<ff', tau_L, tau_R))

# ========= 主程序 =========
def main():
    ser = serial.Serial(SER_DEV, BAUDRATE, timeout=TIMEOUT)
    ser.reset_input_buffer()
    start = time.time()

    with open(logf, 'w', newline='') as fcsv:
        wr = csv.DictWriter(fcsv, fieldnames=csv_header)
        wr.writeheader()

        while True:
            pkt = read_packet(ser)

            if pkt is None:
            
                continue
            Lpos, Rpos, Lvel, Rvel = pkt  # ★ 不再叫 *_deg
           
            #Lpos = np.degrees(Lpos)
            #Rpos = np.degrees(Rpos)
            #Lvel = np.degrees(Lvel)  # deg/s
            #Rvel = np.degrees(Rvel)

            now = time.time() - start
            # ----- NN 推理 -----
 
            dnn.generate_assistance(Lpos, Rpos, Lvel, Rvel)  # ★ 与模型输入顺序一致
            L_cmd = -float(np.clip(L_Ctl * dnn.hip_torque_L , CMD_MIN, CMD_MAX))
            R_cmd = float(np.clip(R_Ctl * dnn.hip_torque_R , CMD_MIN, CMD_MAX))

            # ----- 发送给 Teensy -----
            send_torque(ser, L_cmd, R_cmd)
            last_flush = time.time()
   

            # ----- 记录 -----
            wr.writerow({
                'Time'       : f'{now:.3f}',
                'L_IMU_Ang'  : f'{Lpos:.3f}',
                'R_IMU_Ang'  : f'{Rpos:.3f}',
                'L_IMU_Vel'  : f'{Lvel:.3f}',
                'R_IMU_Vel'  : f'{Rvel:.3f}',
                'L_Cmd'      : f'{L_cmd:.3f}',
                'R_Cmd'      : f'{R_cmd:.3f}',
            })
            fcsv.flush()
            if time.time() - last_flush > 0.5:
                fcsv.flush()
                last_flush = time.time()
            # ----- 控制台输出 -----
            print(f'| time:{now:6.2f}s | Lθ:{Lpos:7.2f}° | Rθ:{Rpos:7.2f}° | '
                f'Lω:{Lvel:7.2f} | Rω:{Rvel:7.2f} | '
                f'τL:{L_cmd:6.2f} | τR:{R_cmd:6.2f} |')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nQuit.')
