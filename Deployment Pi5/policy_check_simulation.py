import numpy as np   
from math import pi, tan, sin, cos    
import matplotlib.pyplot as plt    
import csv    
import torch     
import torch.nn as nn    
from DNN_torch_ram import DNNRam, LSTMNetwork      
import copy as cp    
import argparse    
import seaborn as sns    

from scipy.signal import butter, filtfilt, find_peaks   
from scipy.interpolate import interp1d    
from scipy.signal import resample   

sns.set(palette="muted", font_scale=1.4, color_codes=True)     
custom_params = {"axes.spines.right": False, "axes.spines.top": False}  
sns.set_style("white")   


# Function to generate Butterworth low-pass filter coefficients
def butter_lowpass(cutoff, fs, order=2):    
    nyq = 0.5 * fs                # Nyquist frequency  
    normal_cutoff = cutoff / nyq  # Normalize cutoff frequency
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a   


def plot_evaluated_results(
    data=None, 
    data_index=2, 
    data_label_list=None, 
    start_index=1000, 
    end_index=1500, 
    save_path='original_data' 
):  
    fig, axs = plt.subplots(data_index, 2, figsize=(data_index * 7, 8))  

    time_list = data['time_list']   
    
    L_actual_angle = data['L_IMU_angle']   
    L_ref_angle = data['L_ref_angle']    
    
    R_actual_angle = data['R_IMU_angle']    
    R_ref_angle = data['R_ref_angle']    
    
    L_actual_velocity = data['L_IMU_vel']      
    L_ref_velocity = data['L_ref_vel']      
    
    R_actual_velocity = data['R_IMU_vel']     
    R_ref_velocity = data['R_ref_vel']       
    
    L_Cmd_torque = data['L_exo_torque']        
    R_Cmd_torque = data['R_exo_torque']           
    
    axs[0, 0].plot(time_list[start_index:end_index], L_actual_angle[start_index:end_index], label='Actual Position', color='blue')  
    # axs[0, 0].plot(time_list[start_index:end_index], L_ref_angle[start_index:end_index], label='Reference Position', color='black')    
    
    axs[0, 0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3)  
    axs[0, 0].set_xlabel('Time (s)')   
    axs[0, 0].set_ylabel('Left Hip Position (Deg)')   
    
    axs[0, 1].plot(time_list[start_index:end_index], R_actual_angle[start_index:end_index], label='Actual Position', color='blue')
    # axs[0, 1].plot(time_list[start_index:end_index], R_ref_angle[start_index:end_index], label='Reference Position', color='black')  

    axs[0, 1].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3)   
    axs[0, 1].set_xlabel('Time (s)')   
    axs[0, 1].set_ylabel('Right Hip Position (Deg)')    

    axs[1, 0].plot(time_list[start_index:end_index], L_actual_velocity[start_index:end_index], label='Actual Velocity', color='blue')  
    axs[1, 0].plot(time_list[start_index:end_index], L_ref_velocity[start_index:end_index], label='Filtered Velocity', color='black')   
    # axs[1, 0].plot(time_list[start_index:end_index], -1/0.008/14.14 * np.ones(end_index-start_index) * L_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    axs_left = axs[1, 0].twinx()
    # axs_left.plot(time_list[start_index:end_index], 0.1 * np.ones(end_index-start_index) * L_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    axs[1, 0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3) 
    axs[1, 0].set_xlabel('Time (s)')  
    axs[1, 0].set_ylabel('Left Hip Velocity (Deg/s)')      

    axs[1, 1].plot(time_list[start_index:end_index], R_actual_velocity[start_index:end_index], label='Acctual Velocity', color='blue')  
    axs[1, 1].plot(time_list[start_index:end_index], R_ref_velocity[start_index:end_index], label='Filtered Velocity', color='black')  
    axs_right = axs[1, 1].twinx()
    # axs[1, 1].plot(time_list[start_index:end_index], -1/0.008/14.14 * np.ones(end_index-start_index) * R_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    # axs_right.plot(time_list[start_index:end_index], 0.1 * np.ones(end_index-start_index) * R_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    axs[1, 1].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3)     
    axs[1, 1].set_xlabel('Time (s)')  
    axs[1, 1].set_ylabel('Right Velocity (Deg/s)')   
    
    axs[2, 0].plot(time_list[start_index:end_index], L_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    # axs[2, 0].plot(time_list[start_index:end_index], L_ref_velocity[start_index:end_index], label='Reference Velocity', color='black')   
    axs[2, 0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3)  
    axs[2, 0].set_xlabel('Time (s)')  
    axs[2, 0].set_ylabel('Left Assistive Torque (Nm)')    

    axs[2, 1].plot(time_list[start_index:end_index], R_Cmd_torque[start_index:end_index], label='Assistive Torque', color='red')  
    # axs[2, 1].plot(time_list[start_index:end_index], R_ref_velocity[start_index:end_index], label='Right Torque', color='black')  
    axs[2, 1].legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=3) 
    axs[2, 1].set_xlabel('Time (s)')   
    axs[2, 1].set_ylabel('Right Assistive Torque (Nm)')   
     
    # plt.tight_layout()   
    plt.savefig(save_path)   
      
    plt.show()   
    

def load_nn(
        saved_policy_path = "current_exo.pt",  
        nn_type='nn',
        kp=None, 
        kd=None   
    ):  
    hip_nn = None  
    if nn_type == 'lstm':   
        hip_nn = LSTMNetwork(n_input=4, n_layer_1=256, num_layers=2, n_output=2, 
                             kp=kp, kd=kd, 
                             b=np.array([0.02008337, 0.04016673, 0.02008337]), a=np.array([1.,-1.56101808, 0.64135154])
                             )        
        hip_nn.load_saved_policy(torch.load(saved_policy_path, map_location=torch.device('cpu')))     
    else: 
        hip_nn = DNNRam(18, 128, 64, 2, saved_policy_path=saved_policy_path, kp=kp, kd=kd)     
    return hip_nn      


def load_data(
    file_path=None
):  
    data_total = {}      
    
    # Initialize lists for each column
    time_list = []
    L_torque = []
    R_torque = []
    L_cmd = []
    R_cmd = []
    R_IMU_angle = []
    L_IMU_angle = []
    L_IMU_vel = []  
    R_IMU_vel = []  
    L_Ref_Ang = []   
    R_Ref_Ang = []   
    L_Ref_Vel = []   
    R_Ref_Vel = []   
    
    L_IMU_vel_filter = []   
    R_IMU_vel_filter = []   
    
    L_IMU_Angle_filter = []   
    R_IMU_Angle_filter = []     

    # Read the CSV file
    with open(file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        
        # Skip header row
        next(csvreader)
        
        # Extract data row by row
        for row in csvreader:  
            
            # print("length :", len(row))
            # time_list.append(float(row[11]))    
            # L_IMU_angle.append(float(row[0]))   
            # R_IMU_angle.append(float(row[1]))    
            # L_IMU_vel.append(float(row[2])) 
            # R_IMU_vel.append(float(row[3])) 
            
            time_list.append(float(row[0]))    
            L_IMU_angle.append(float(row[1]))   
            R_IMU_angle.append(float(row[2]))    
            L_IMU_vel.append(float(row[3])) 
            R_IMU_vel.append(float(row[4])) 
            
            # L_cmd.append(-1 * float(row[4]))       
            # R_cmd.append(float(row[5]))     
            # L_Ref_Ang.append(float(row[6]))
            # R_Ref_Ang.append(float(row[7]))
            # L_Ref_Vel.append(float(row[8]))
            # R_Ref_Vel.append(float(row[9]))    
    
    # L_IMU_vel_filter = apply_filter(np.array(L_IMU_vel), b, a)
    # R_IMU_vel_filter = apply_filter(np.array(R_IMU_vel), b, a)
    # L_IMU_vel_filter = np.squeeze(L_IMU_vel_filter)   
    # R_IMU_vel_filter = np.squeeze(R_IMU_vel_filter)   
    
    # print(time_list)  
    data_total['time_list']   =  time_list 
    data_total['L_IMU_angle'] =  L_IMU_angle  
    data_total['R_IMU_angle'] =  R_IMU_angle     
    data_total['L_IMU_vel']   =  L_IMU_vel   
    data_total['R_IMU_vel']   =  R_IMU_vel     
    
    # L_IMU_Angle_filter = butterworth_filter(np.array(L_Ref_Ang), cutoff, 60, order=4, filter_type='low')
    # R_IMU_Angle_filter = butterworth_filter(np.array(R_Ref_Ang), cutoff, 60, order=4, filter_type='low') 
    return data_total    


def forward_calculation(
    data_total=None, 
    hip_dnn   =None,
    k_control =0.2  
):  
    L_IMU_angle = data_total['L_IMU_angle']   
    R_IMU_angle = data_total['R_IMU_angle']     
    L_IMU_vel   = data_total['L_IMU_vel']   
    R_IMU_vel   = data_total['R_IMU_vel']    
    
    # L_IMU_angle = data_total['R_IMU_angle']   
    # R_IMU_angle = data_total['L_IMU_angle']     
    # L_IMU_vel   = data_total['R_IMU_vel']   
    # R_IMU_vel   = data_total['L_IMU_vel']     
     
    L_Ref_angle    = []   
    R_Ref_angle    = []   
    
    L_Ref_velocity = []   
    R_Ref_velocity = []   
    
    L_Cmd_torque = []    
    R_Cmd_torque = []    
    
    data = {}   
    # kp = 50  
    # kd = 14.14  
    # # print(L_IMU_angle.shape)
    for i in range(len(L_IMU_angle)):   
    # for i in range(L_IMU_angle.shape[0]):    
        # print(f"Time when running NN = {time_list[i]:^8.3f}")  
        # qHr_L, qHr_R, filter_vel_L, filter_vel_R = nn_model.get_predicted_action(L_IMU_angle[i][0], R_IMU_angle[i][0], L_IMU_vel[i][0], R_IMU_vel[i][0])     
        # print("imu:", L_IMU_angle[i], R_IMU_angle[i]) 
        hip_dnn.generate_assistance(L_IMU_angle[i], R_IMU_angle[i], L_IMU_vel[i], R_IMU_vel[i])      
        
        L_Ref_angle.append(180/np.pi * float(hip_dnn.qHr_L))      
        R_Ref_angle.append(180/np.pi * float(hip_dnn.qHr_R))      
        
        L_Ref_velocity.append(180/np.pi * float(hip_dnn.dqTd_filtered_L)) 
        R_Ref_velocity.append(180/np.pi * float(hip_dnn.dqTd_filtered_R))      
        
        L_Cmd_torque.append(hip_dnn.hip_torque_L*k_control)    
        R_Cmd_torque.append(hip_dnn.hip_torque_R*k_control)     
        
        # L_Ref_angle.append(L_IMU_angle[i])     
        # R_Ref_angle.append(R_IMU_angle[i])   
        # L_Ref_velocity.append(float(filter_vel_L))    
        # R_Ref_velocity.append(float(filter_vel_R))        
        # hip_torque_L = ((hip_dnn.qHr_L - L_IMU_angle[i]* np.pi/180.0) * kp + hip_dnn.dqTd_filtered_L * kd * (-1.0)) * 0.08
        # hip_torque_R = ((hip_dnn.qHr_R - R_IMU_angle[i]* np.pi/180.0) * kp + hip_dnn.dqTd_filtered_L * kd * (-1.0)) * 0.08
        # hip_torque_L = - L_IMU_vel[i] * kd * np.pi/180.0    
        # hip_torque_R = - R_IMU_vel[i] * kd * np.pi/180.0     
        # hip_torque_L = (qHr_L * kp + filter_vel_L * kd * (-1.0)) * 0.008
        # hip_torque_R = (qHr_R * kp + filter_vel_R * kd * (-1.0)) * 0.008

        # self.hip_torque_L = ((self.qHr_L-self.qTd_L) * self.kp2 + self.dqTd_L * self.kd2 * (-1.0)) * 0.008
        # self.hip_torque_R = ((self.qHr_R-self.qTd_R) * self.kp3 + self.dqTd_R * self.kd3 * (-1.0)) * 0.008
        # qHr_L = np.clip(qHr_L, -np.pi, np.pi)  
        # qHr_R = np.clip(qHr_R, -np.pi, np.pi)   
        
        # hip_torque_L = (qHr_L - L_IMU_angle[i][0]* np.pi/180.0) * kp - filter_vel_L * kd * np.pi/180.0    
        # hip_torque_R = (qHr_R - R_IMU_angle[i][0]* np.pi/180.0) * kp - filter_vel_R * kd * np.pi/180.0    

        # hip_torque_L = qHr_L - L_IMU_angle[i] * np.pi/180.0
        # hip_torque_R = qHr_R - R_IMU_angle[i] * np.pi/180.0
        
        # hip_torque_L = L_IMU_angle[i]
        # hip_torque_R = R_IMU_angle[i]  
        
        # hip_torque_L = qHr_L
        # hip_torque_R = qHr_R
        
        # hip_torque_L = (qHr_L - L_IMU_angle[i]* np.pi/180.0) * kp - L_IMU_vel[i] * kd * np.pi/180.0    
        # hip_torque_R = (qHr_R - R_IMU_angle[i]* np.pi/180.0) * kp - R_IMU_vel[i] * kd * np.pi/180.0    
        
        # hip_torque_L = 0.0   
        # hip_torque_R = 0.0   
    
    data['time_list']         = data_total['time_list'] 
    # print(data['time_list']) 
    
    data['L_IMU_angle']  = L_IMU_angle  
    data['L_ref_angle']  = L_Ref_angle 
    data['L_IMU_vel']    = L_IMU_vel  
    data['L_ref_vel']    = L_Ref_velocity   
    
    data['R_IMU_angle']  = R_IMU_angle      
    data['R_ref_angle']  = R_Ref_angle   
    data['R_IMU_vel']    = R_IMU_vel  
    data['R_ref_vel']    = R_Ref_velocity 
    
    # print(L_Cmd_torque)
    data['L_exo_torque'] = cp.deepcopy(L_Cmd_torque)
    data['R_exo_torque'] = cp.deepcopy(R_Cmd_torque)     
    
    return data


if __name__ == "__main__":   
    b, a = butter_lowpass(5, 100, order=2)  
    # low_pass_filter = LPF(a=a, b=b)    
    print("b :", b, "a : ", a)    
    
    # # 加载参数字典  
    # state_dict = torch.load("./nn_para/lstm/0_exo.pt", map_location=torch.device('cpu'))

    # # 打印参数名和形状
    # for name, param in state_dict.items():
    #     print(f"{name}: {param.shape}")
    #     # print(param)
        
    kp = 50
    kd = 0.5 * np.sqrt(kp)       

    # dnn = DNN(18, 128, 64, 2)  # depends on training network  
    dnn = load_nn(
        saved_policy_path = "./nn_para/lstm/current_exo.pt",  
        nn_type           = 'lstm', 
        # saved_policy_path = "./nn_para/mlp/30_exo.pt",   
        # nn_type='nn', 
        kp = kp, 
        kd = kd 
    ) 
        
    #     # 加载参数字典  
    # state_dict = torch.load("./nn_para/mlp/current_exo.pt", map_location=torch.device('cpu'))

    # # 打印参数名和形状
    # for name, param in state_dict.items():
    #     print(f"{name}: {param.shape}")
    #     # print(param)  
    
    data_total = load_data(
        file_path = './data/Jimmy/20241010-102336-Quinn-Walk-S1-Trail01.csv'  
    )   
    
    data_plot = forward_calculation(
        data_total=data_total,  
        hip_dnn   =dnn,
        k_control =0.3  
    )   
    
    plot_evaluated_results(
        data=data_plot, 
        data_index=3,  
        data_label_list=[], 
        start_index = 500, 
        end_index = 2000,
        save_path='simulation_data.pdf'
    )   