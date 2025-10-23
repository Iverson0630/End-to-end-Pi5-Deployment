import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import torch
import torch.nn as nn
import numpy as np


def load_nn(
        saved_policy_path = "current_exo.pt",  
        nn_type='nn',
        kp=None, 
        kd=None, 
        input_b=np.array([0.02008337, 0.04016673, 0.02008337]), 
        input_a=np.array([1.,-1.56101808, 0.64135154])  
    ):  
    hip_nn = None  
    if nn_type == 'lstm':   
        print('load lstm model')
        hip_nn = LSTMNetwork(
            n_input=4, n_layer_1=256, num_layers=2, n_output=2, 
            kp=kp, kd=kd, 
            b=input_b, a=input_a
        )        
        hip_nn.load_saved_policy(torch.load(saved_policy_path, map_location=torch.device('cpu')))     
        print('load parameters successfully!!')
    else: 
        hip_nn = DNN(
            18, 128, 64, 2, 
            saved_policy_path=saved_policy_path, 
            kp=kp, kd=kd, 
            b=input_b, a=input_a  
        )     
    return hip_nn      


class LPF(object):
    def __init__(self, a=None, b=None):   
        self.a = a   
        self.b = b   
        
        self.value_1 = np.zeros(3)   
        self.value_2 = np.zeros(3)  
         
        self.filtered_value = 0   
    
    def cal_scalar(self, input_scalar=None):     
        self.value_1[1:3] = self.value_1[0:2]        
        self.value_1[0] = input_scalar   
        self.value_2[1:3] = self.value_2[0:2]    
        
        self.value_2[0] = np.dot(self.value_1, self.b) - np.dot(self.value_2[2:0:-1], self.a[2:0:-1]) 
        self.filtered_value = self.value_2[0]   
        return self.filtered_value   
    

class Network(nn.Module):
    def __init__(self,n_input=18, n_layer_1=128, n_layer_2=64, n_output=2) -> None:
        super(Network,self).__init__()   
        
        # create neural network 
        self.fc1 = nn.Linear(n_input,n_layer_1)    
        self.fc2 = nn.Linear(n_layer_1,n_layer_2)  
        self.fc3 = nn.Linear(n_layer_2,n_output)    
    
    def forward(self,x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)  
        return x  
    
    def load_saved_policy(self,state_dict):
        self.fc1.weight.data = state_dict['p_fc1.weight']
        self.fc1.bias.data = state_dict['p_fc1.bias']
        self.fc2.weight.data = state_dict['p_fc2.weight']
        self.fc2.bias.data = state_dict['p_fc2.bias']  
        self.fc3.weight.data = state_dict['p_fc3.weight']     
        self.fc3.bias.data = state_dict['p_fc3.bias']   
        

class DNN:  
    def __init__(self, 
                 n_input, n_layer_1, n_layer_2, n_output, 
                 saved_policy_path, 
                 kp, kd, 
                 b=np.array([0.06745527, 0.13491055, 0.06745527]), a=np.array([ 1., -1.1429805, 0.4128016])
                 ) -> None:
        
        self.n_input = n_input   
        self.n_layer_1 = n_layer_1        
        self.n_layer_2 = n_layer_2      
        self.n_output = n_output      
        
        self.saved_policy_path = saved_policy_path
        self.network = Network(self.n_input, self.n_layer_1, self.n_layer_2, self.n_output)     
        self.network.load_saved_policy(torch.load(self.saved_policy_path, map_location=torch.device('cpu')))  

        self.b = b  
        self.a = a 
        
        self.left_vel_filter  = LPF(a=self.a, b=self.b)       
        self.right_vel_filter = LPF(a=self.a, b=self.b)      
        
        self.left_ref_filter  = LPF(a=self.a, b=self.b)       
        self.right_ref_filter = LPF(a=self.a, b=self.b)      


        self.in_1  = np.ones(4)  
        self.in_2  = np.ones(4)   
        self.out_3 = np.ones(2)   
        self.out_2 = np.ones(2)   
        self.out_1 = np.ones(2)   
        self.input_data = np.zeros(self.n_input)     
        
        self.qTd_L = 10
        self.qTd_R = 10
        self.dqTd_L = 0
        self.dqTd_R = 0  
        
        self.qHr_L = 0 
        self.qHr_R = 0   

        self.kp2 = kp 
        self.kp3 = kp  
        self.kd2 = kd    
        self.kd3 = kd   

        self.dqTd_history_L = np.zeros(3)  
        self.dqTd_filtered_history_L = np.zeros(3)
        self.dqTd_filtered_L = 0
        self.dqTd_history_R = np.zeros(3)
        self.dqTd_filtered_history_R = np.zeros(3)  
        self.dqTd_filtered_R = 0  
        
        self.hip_torque_L = 0
        self.hip_torque_R = 0    
    
    def generate_assistance(self, LTx, RTx, LTAVx, RTAVx):  
        self.qTd_L  = LTx * np.pi/180.0     
        self.qTd_R  = RTx * np.pi/180.0     
        self.dqTd_L = LTAVx * np.pi/180.0             
        self.dqTd_R = RTAVx * np.pi/180.0      
          
        # self.LTx = LTx  
        # self.RTx = RTx  
        # self.LTAVx = LTAVx  
        # self.RTAVx = RTAVx       
        ###############################################
        # # filter dqTd_L
        # self.dqTd_history_L[1:3] = self.dqTd_history_L[0:2]
        # self.dqTd_history_L[0] = self.LTAVx
        # self.dqTd_filtered_history_L[1:3] = self.dqTd_filtered_history_L[0:2]
        # self.dqTd_filtered_history_L[0] = np.sum(np.dot(self.dqTd_history_L, self.b)) - np.sum(np.dot(self.dqTd_filtered_history_L[2:0:-1], self.a[2:0:-1]))
        # self.dqTd_filtered_L = self.dqTd_filtered_history_L[0]  
        # # filter dqTd_R   
        # self.dqTd_history_R[1:3] = self.dqTd_history_R[0:2]
        # self.dqTd_history_R[0] = self.RTAVx
        # self.dqTd_filtered_history_R[1:3] = self.dqTd_filtered_history_R[0:2]
        # self.dqTd_filtered_history_R[0] = np.sum(np.dot(self.dqTd_history_R, self.b)) - np.sum(np.dot(self.dqTd_filtered_history_R[2:0:-1], self.a[2:0:-1]))
        # self.dqTd_filtered_R = self.dqTd_filtered_history_R[0]    
        
        ### velocity ###
        self.dqTd_filtered_L = self.left_vel_filter.cal_scalar(input_scalar=self.dqTd_L)   
        self.dqTd_filtered_R = self.right_vel_filter.cal_scalar(input_scalar=self.dqTd_R)     

        self.input_data = np.concatenate((self.in_2, self.in_1, self.qTd_L, self.qTd_R, self.dqTd_filtered_L, self.dqTd_filtered_R, self.out_3, self.out_2, self.out_1), axis=None)
        self.in_2  = np.copy(self.in_1)
        self.in_1  = np.array([self.qTd_L, self.qTd_R, self.dqTd_filtered_L, self.dqTd_filtered_R])  
        self.out_3 = np.copy(self.out_2)     
        self.out_2 = np.copy(self.out_1)     
        
        input_data_tensor = torch.tensor(self.input_data, dtype=torch.float32)
 
        output_tensor = self.network(input_data_tensor)   
        output_data = output_tensor.detach().numpy()   
        
        self.qHr_L, self.qHr_R = output_data  
        self.qHr_L_ori, self.qHr_R_ori = output_data  
        
        self.qHr_L = self.left_ref_filter.cal_scalar(input_scalar=self.qHr_L_ori)    
        self.qHr_R = self.right_ref_filter.cal_scalar(input_scalar=self.qHr_R_ori)    
        self.out_1 = np.array([self.qHr_L, self.qHr_R])   
        
        # self.x_L[1:3] = self.x_L[0:2]
        # self.x_L[0] = self.qHr_L
        # self.y_L[1:3] = self.y_L[0:2]
        # self.y_L[0] = np.sum(np.dot(self.x_L, self.b)) - np.sum(np.dot(self.y_L[2:0:-1], self.a[2:0:-1]))
        # self.qHr_L = self.y_L[0] * 0.1   

        # self.x_R[1:3] = self.x_R[0:2]  
        # self.x_R[0] = self.qHr_R  
        # self.y_R[1:3] = self.y_R[0:2]  
        # self.y_R[0] = np.sum(np.dot(self.x_R, self.b)) - np.sum(np.dot(self.y_R[2:0:-1], self.a[2:0:-1]))
        # self.qHr_R = self.y_R[0] * 0.1    
        
        # hip torque  
        self.hip_torque_L = (self.qHr_L * self.kp2 + self.dqTd_filtered_L * self.kd2 * (-1.0))
        self.hip_torque_R = (self.qHr_R * self.kp3 + self.dqTd_filtered_R * self.kd3 * (-1.0))

        #self.hip_torque_L = ((self.qHr_L-self.qTd_L) * self.kp2 + self.dqTd_L * self.kd2 * (-1.0)) * 0.008
        #self.hip_torque_R = ((self.qHr_R-self.qTd_R) * self.kp3 + self.dqTd_R * self.kd3 * (-1.0)) * 0.008
        
        #self.hip_torque_R = self.qHr_R-self.qTd_R
        # print(f"qHr_L={self.qHr_L}, qHr_R={self.qHr_R}, hip_torque_L={self.hip_torque_L}, hip_torque_R={self.hip_torque_R}")
        return self.hip_torque_L, self.hip_torque_R, self.qHr_L*self.kp2, self.dqTd_filtered_L* self.kd2, self.qHr_R* self.kp2, self.dqTd_filtered_R* self.kd2
    
    def get_predicted_action(self, LTx, RTx, LTAVx, RTAVx): 
        self.LTx = LTx
        self.RTx = RTx
        self.LTAVx = LTAVx
        self.RTAVx = RTAVx  

        self.qTd_L = LTx * np.pi / 180.0    
        self.qTd_R = RTx * np.pi / 180.0   
        self.dqTd_L = LTAVx * np.pi / 180.0        
        self.dqTd_R = RTAVx * np.pi / 180.0     

        self.input_data = np.concatenate((self.in_2, self.in_1, self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R, self.out_3, self.out_2, self.out_1), axis=None)
        self.in_2 = np.copy(self.in_1)
        self.in_1 = np.array([self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R])
        self.out_3 = np.copy(self.out_2)   
        self.out_2 = np.copy(self.out_1)     

        self.para_first[:]  = 0  
        self.para_second[:] = 0   
        self.para_third[:]  = 0  
        
        input_data_tensor = torch.tensor(self.input_data, dtype=torch.float32)
        output_tensor = self.network(input_data_tensor)
        self.qHr_L, self.qHr_R = output_tensor.detach().numpy()  
        self.qHr_L_ori, self.qHr_R_ori = self.qHr_L, self.qHr_R    
        self.out_1 = np.array([self.qHr_L, self.qHr_R])   
        
        self.x_L[1:3] = self.x_L[0:2]
        self.x_L[0] = self.qHr_L 
        self.y_L[1:3] = self.y_L[0:2]  
        self.y_L[0] = np.sum(np.dot(self.x_L, self.b)) - np.sum(np.dot(self.y_L[2:0:-1], self.a[2:0:-1]))
        self.qHr_L = self.y_L[0] * 0.1   
        # self.qHr_L = self.y_L[0]    
        
        self.x_R[1:3] = self.x_R[0:2]  
        self.x_R[0] = self.qHr_R  
        self.y_R[1:3] = self.y_R[0:2]  
        self.y_R[0] = np.sum(np.dot(self.x_R, self.b)) - np.sum(np.dot(self.y_R[2:0:-1], self.a[2:0:-1]))
        self.qHr_R = self.y_R[0] * 0.1    
        # self.qHr_R = self.y_R[0] 
        
        # filter dqTd_L
        self.dqTd_history_L[1:3] = self.dqTd_history_L[0:2]
        self.dqTd_history_L[0] = self.LTAVx
        self.dqTd_filtered_history_L[1:3] = self.dqTd_filtered_history_L[0:2]
        self.dqTd_filtered_history_L[0] = np.sum(np.dot(self.dqTd_history_L, self.b)) - np.sum(np.dot(self.dqTd_filtered_history_L[2:0:-1], self.a[2:0:-1]))
        self.dqTd_filtered_L = self.dqTd_filtered_history_L[0]  
        
        # filter dqTd_R
        self.dqTd_history_R[1:3] = self.dqTd_history_R[0:2]
        self.dqTd_history_R[0] = self.RTAVx
        self.dqTd_filtered_history_R[1:3] = self.dqTd_filtered_history_R[0:2]
        self.dqTd_filtered_history_R[0] = np.sum(np.dot(self.dqTd_history_R, self.b)) - np.sum(np.dot(self.dqTd_filtered_history_R[2:0:-1], self.a[2:0:-1]))
        self.dqTd_filtered_R = self.dqTd_filtered_history_R[0]
        
        return self.qHr_L, self.qHr_R, self.dqTd_filtered_L, self.dqTd_filtered_R 


class LSTMNetwork(nn.Module):  
    def __init__(self, 
                 n_input=4, n_layer_1=256, num_layers=2, n_output=2, 
                 kp=50.0, kd=1.0, b=np.array([0.06745527, 0.13491055, 0.06745527]), a=np.array([ 1., -1.1429805, 0.4128016])
        ) -> None:
        super(LSTMNetwork,self).__init__()   
        
        self.p_lstm1 = nn.LSTM(n_input, n_layer_1, num_layers, batch_first=True)   # for LSTM network
        self.p_fc3 = nn.Linear(n_layer_1, n_output)   # for LSTM network, originally was num_h2
        
        self.qTd_L = 10
        self.qTd_R = 10
        self.dqTd_L = 0
        self.dqTd_R = 0   
        
        self.qHr_L = 0.0 
        self.qHr_R = 0.0 
          
        self.kp    = kp 
        self.kd    = kd     
        
        self.b     = b 
        self.a     = a 
        
        self.left_vel_filter  = LPF(a=self.a, b=self.b)       
        self.right_vel_filter = LPF(a=self.a, b=self.b)       
        self.left_ref_filter  = LPF(a=self.a, b=self.b)       
        self.right_ref_filter = LPF(a=self.a, b=self.b)      
        
    def forward(self,x):   
        # x_feature = torch.relu(self.p_lstm1(x))
        # x_output = torch.relu(self.p_fc3(x_feature))
        # x_output = self.p_fc3(x_output)     
        p_out, _ = self.p_lstm1(x)    
        if p_out.dim() == 2:
            p_out = p_out.unsqueeze(1)

        p_out = p_out[:, -1, :]   
        p_out = torch.relu(p_out)   
        p_out = self.p_fc3(p_out)      
        return p_out.detach().numpy().squeeze()    
    
    def get_predicted_action(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel): 
        state = np.array([L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel])   
        
        state_tensor = torch.tensor(state[np.newaxis, :], dtype=torch.float32)  
        action = self.forward(state_tensor)  
        return action[0], action[1]  
    
    def generate_assistance(self, L_IMU_angle, R_IMU_angle, L_IMU_Vel, R_IMU_Vel):
        self.qTd_L  = L_IMU_angle * np.pi/180.0      
        self.qTd_R  = R_IMU_angle * np.pi/180.0       
        self.dqTd_L = L_IMU_Vel * np.pi/180.0              
        self.dqTd_R = R_IMU_Vel * np.pi/180.0         
        
        self.dqTd_filtered_L = self.left_vel_filter.cal_scalar(input_scalar=self.dqTd_L)   
        # self.dqTd_filtered_R = self.right_vel_filter.cal_scalar(input_scalar=self.dqTd_R)    
        # print(self.qTd_L, self.qTd_R)
        action = self.get_predicted_action(self.qTd_L, self.qTd_R, self.dqTd_L, self.dqTd_R)  
        self.qHr_L = self.qTd_L+0.1*action[0]
        self.qHr_R = self.qTd_R+0.1*action[1]
           
        
        #self.hip_torque_L = 0.1 * self.qHr_L * self.kp + self.dqTd_L * self.kd * (-1.0)   
        #self.hip_torque_R =  0.1 * self.qHr_R * self.kp + self.dqTd_R * self.kd * (-1.0) 
        
        self.hip_torque_L = -0.1 *action[0] * self.kp + self.dqTd_L * self.kd * (-1.0)   
        self.hip_torque_R =  0.1 * action[1] * self.kp + self.dqTd_R * self.kd * (1.0) 
       
        return self.hip_torque_L, self.hip_torque_R  
        
    def load_saved_policy(self,state_dict):  
        self.p_lstm1.weight_ih_l0.data = state_dict['p_lstm1.weight_ih_l0']    
        self.p_lstm1.weight_hh_l0.data = state_dict['p_lstm1.weight_hh_l0']    
        
        self.p_lstm1.bias_ih_l0.data = state_dict['p_lstm1.bias_ih_l0']    
        self.p_lstm1.bias_hh_l0.data = state_dict['p_lstm1.bias_hh_l0']     
        
        self.p_lstm1.weight_ih_l1.data = state_dict['p_lstm1.weight_ih_l1']     
        self.p_lstm1.weight_hh_l1.data = state_dict['p_lstm1.weight_hh_l1']     
        
        self.p_lstm1.bias_ih_l1.data = state_dict['p_lstm1.bias_ih_l1']      
        self.p_lstm1.bias_hh_l1.data = state_dict['p_lstm1.bias_hh_l1']    
        
        self.p_fc3.weight.data = state_dict['p_fc3.weight']    
        self.p_fc3.bias.data = state_dict['p_fc3.bias']   
        

