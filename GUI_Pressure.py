import serial
from serial.tools import list_ports
import struct
import time
from math import *
import csv
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import os
from datetime import datetime
import numpy as np
"""
Hip‑Exo GUI – **refactored for clarity**
--------------------------------------------------
Key runtime‑tunable parameters (sent over BLE → Teensy)
------------------------------------------------------
| Symbol | Spin‑box | What it means | How it enters the equation |
|--------|----------|--------------|----------------------------|
| **k_gain** | k‑scale | Overall gain. <br>Final motor torque τ is proportional to it. | τ = k_gain · (k_flex/k_ext) · sin Δ |
| **k_flex** | Flex‑assist | Extra scaling while **swing‑leg flexing** (hip flexion). |
| **k_ext** | Ext‑assist | Extra scaling while **stance‑leg extending** (hip extension). |
| **delay** | Phase delay | Index 0‑99 → ~0‑990 ms phase shift applied to Δ. |

Users can hover each spin‑box to see the same description.
Realtime values (angles & torques) are now also shown numerically next to the plots.
"""
from serial.tools import list_ports

def find_available_ports():
    connected_ports = []
    available_ports = list(list_ports.comports())

    for port, desc, hwid in available_ports:
        try:
            ser = serial.Serial(port)
            if ser.readable():
                connected_ports.append(port)
            ser.close()
        except serial.SerialException:
            pass

    return connected_ports

# -------------  helpers unchanged (find_available_ports, saturation …) -------------

#  (due to space, helpers are identical to the previous version)
#  …  << KEEP THE ORIGINAL HELPER FUNCTIONS HERE >>


class MainWindow(QWidget):
    """Main application window – cleaned & documented."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pressure insole")

        # === Serial setup vars ===
        self.ser = None
        self.connected = False

        # === Realtime data buffers ===
        self.win_size = 150
        self.t_buffer = [0]*self.win_size

        self.t0 = time.time()
        self.t_buffer = [0]*200
        self.pressure_buf = np.zeros((200, 2))
        self._build_layout()
        self._build_pressure_curves()

        log_dir = "data"
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        self.csv_path = os.path.join(log_dir, filename)

        # # === 创建 CSV 文件并写入表头 ===
        # with open(self.csv_path, "w", newline="") as f:
        #     writer = csv.writer(f)
        #     # 表头：时间戳 + R数据 + L数据
        #     writer.writerow([
        #         "timestamp",
        #         "R_angle(θ)", "R_Torque Cmd", "R_Torque Est",
        #         "L_angle(θ)", "L_Torque Cmd", "L_Torque Est",
        #         "Lθ Display", "Rθ Display", "Lτ Display", "Rτ Display"
        #     ])
        # ------------  timer  ------------
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)    # 50 Hz refresh
        self.timer.timeout.connect(self._update_everything)
        self.timer.start()

    # --------------------------------------------------------------------- UI helpers

        
    def _build_layout(self):
        main = QHBoxLayout(self)
        left = QVBoxLayout();  main.addLayout(left)
        plots = QVBoxLayout(); main.addLayout(plots, 1)

        # --- COM & connect row ---
        top = QHBoxLayout(); left.addLayout(top)
        top.addWidget(QLabel("Port:"))
        self.cmb_port = QComboBox(); self.cmb_port.addItems(find_available_ports()); top.addWidget(self.cmb_port)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self._connect_clicked)
        top.addWidget(self.btn_connect)
        self.btn_log = QPushButton("Start Logging"); top.addWidget(self.btn_log)

        # --- parameter box ---
        grp = QGroupBox("Torque law parameters")
        grid = QGridLayout(grp); left.addWidget(grp)

   

        # 右侧图区域
        self.plot_layout = plots

    def _build_pressure_curves(self):
        """Build scrolling 18-channel foot-pressure plot."""
        self.pressure_plot = pg.PlotWidget(background='w')
        self.pressure_plot.setTitle("Foot pressure over time (g)")
        self.pressure_plot.setLabel('bottom', 'Time (s)')
        self.pressure_plot.setLabel('left', 'Pressure (g)')
        self.pressure_plot.showGrid(x=True, y=True)
        self.legend = self.pressure_plot.addLegend(offset=(10, 10))
      

        self.pressure_lines = []
        for i in range(2):
            color = pg.intColor(i, hues=2)
            name = 'toe pressure' if i==0 else 'heel pressure'
            line = self.pressure_plot.plot(pen=pg.mkPen(color, width=2),name=name)
            self.pressure_lines.append(line)

        font = QFont()
        font.setPointSize(15)   # adjust to desired size
        font.setBold(True)

        for sample, label in self.legend.items:
            label.setFont(font)  # works! label is a QGraphicsTextItem

        self.plot_layout.addWidget(self.pressure_plot, 1)



    # ------------------------------------------------------------------ serial logic
    def _connect_clicked(self):
        port = self.cmb_port.currentText()
        try:
            self.ser = serial.Serial(port,115200,timeout=0)
            self.connected = True
            self.btn_connect.setText("Connected")
            self.btn_connect.setStyleSheet("background:#4caf50;color:white")
        except serial.SerialException:
            QMessageBox.critical(self,"Error",f"Cannot open {port}")


    # ------------------------------------------------------------- main update loop
    def _update_everything(self):
        if not self.connected: return
        self._read_serial()

    def _read_serial(self):
   
        # Need 32‑byte packet: A5 5A len(32) + 29 data

        if self.ser.read(1)!=b'\xAA': return
        if self.ser.read(1)!=b'\x02': return
  
        payload = self.ser.read(36)

        if len(payload)!=36: return
        data = struct.unpack('>' + 'h'*(36//2), payload)  
        pressures = [int(x) for x in data]  # 单位：g
        
        #exit()
        t = time.time()
        self.t_buffer = self.t_buffer[1:] + [t - self.t0]
        pressures_toe = (pressures[17]+pressures[13]+pressures[7]+pressures[1])/4
        pressures_heel = (pressures[9]+pressures[3]+pressures[8]+pressures[2])/4

        self.pressure_buf = np.vstack([self.pressure_buf[1:], [pressures_toe,pressures_heel]])
      
        # --- update plots ---
        for i, line in enumerate(self.pressure_lines):
            line.setData(self.t_buffer, self.pressure_buf[:, i])

        # --- 数据保存到 CSV ---
        # --- save to CSV (注意 R/L 对调) ---
        # with open(self.csv_path, "a", newline="") as f:
        #     writer = csv.writer(f)
        #     writer.writerow([
        #         t,                       # timestamp
        #         R_angle,                 # R Angle (θ)
        #         L_tau_d,                 # R Torque Cmd
        #         L_tau,                   # R Torque Est
        #         L_angle,                 # L Angle (θ)
        #         R_tau_d,                 # L Torque Cmd
        #         R_tau,                   # L Torque Est
        #         f"{L_angle:.1f}",        # Lθ Display
        #         f"{R_angle:.1f}",        # Rθ Display
        #         f"{L_tau:.1f} (cmd {L_tau_d:.1f})",
        #         f"{R_tau:.1f} (cmd {R_tau_d:.1f})"
        #     ])


        

# -------------------------------------------------------------------------- main
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, foreground='k')  # smoother plots + black text
    w = MainWindow(); w.show()
    sys.exit(app.exec_())