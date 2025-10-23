//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "IMU_Adapter.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring>  
#include <FlexCAN_T4.h>   
#include "Motor_Control_Tmotor.h"   

#define LOG_FILENAME "2024-12-17-Hip-Walking_Powered_06.csv" 

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h" 

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"
#include "sdlogger.h"
SdLogger logger(BUILTIN_SDCARD, F("walking_log_"), F(".csv"));



SdFs sd;
FsFile file;  
const float DEG2RAD = PI / 180.0f;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;    

// ads1292r torque_sensor1;                                      //Create torque sensor object see ads1292r.h

// Data logging
int isLogging = 0;   
int L_load_cell_torque = 0;  
int R_load_cell_torque = 0;    

/*Filter*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);     
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   
int CAN_ID = 3;  

int Sig_Motor_ID_1 = 1;     
int Sig_Motor_ID_2 = 0;       

double torque_command = 0;  
double velocity_command = 0;  
double position_command = 0;  

double M1_torque_command = 0;  
double M2_torque_command = 0;   

double RL_torque_command_1 = 0.0;    
double RL_torque_command_2 = 0.0;   

double MAX_torque_command = 8;   
double MIN_torque_command = -8;    
float M1_torque_meas = 0.0f;
float M2_torque_meas = 0.0f;
int LimitInf = -18;    
int LimitSup = 18;    

float p_des = 0;   
float v_des = 0;    
float kp = 0;   
float kd = 0;   
float t_ff = 0;   

/*MOTOR*/    
float initial_pos_1 = 0;       
float initial_pos_2 = 0;       

Motor_Control_Tmotor sig_m1(0x002, CAN_ID);   
Motor_Control_Tmotor sig_m2(0x001, CAN_ID);   
/*MOTOR*/  

/*Isra Serial Class Setup*/  
Serial_Com Serial_Com;   

/*Sensors Setup*/ 
IMU_Adapter imu;

/*Serial Send*/  
size_t Send_Length = 11; 
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };   

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;  
uint16_t R_IMUX_int = 0x00;   

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00; 

uint16_t L_CMD_int16 = 0x7fff;  
float L_CMD_serial   = 0.0;  

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial   = 0.0;

float IMUX_float = 0;   
float IMU11 = 0;   
float IMU22 = 0;   
float IMU33 = 0;  
float IMU44 = 0;  

/* Time control*/
// unsigned long Delta_T1 = 35;  //Looks like increasing this improves stability, but mkaes the torque less smooth
// unsigned long t_i, t_pr1;
// unsigned long beginning = 0;
double t;   
double next_t;    
double delta_t;    

//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;    // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;     // [Hz] Bluetooth sending data frequency 
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
uint8_t data_rs232_rx[60] = {0};   // <-- char → uint8_t

int L_leg_IMU_angle = 0;       
int R_leg_IMU_angle = 0;  
int L_motor_torque  = 0;   
int R_motor_torque  = 0;  
int L_motor_torque_desired = 0;    
int R_motor_torque_desired = 0;   
int t_teensy = 0;  
int M_Selected = 0;  
int CtrlMode_Selected = 0;  

int GUI_force_cmd     = 0;  
int GUI_stiffness_cmd = 0;  
int GUI_damping_cmd   = 0;  
int GUI_assistive_ratio_cmd = 0;     

int GUI_pos_ampl_cmd    = 0;       
int GUI_pos_fre_cmd     = 0;       
int GUI_force_ampl_cmd  = 10;        
int GUI_force_fre_cmd   = 10;       

double GUI_force = 1.0;    
double GUI_K_p   = 1.0;    
double GUI_K_d   = 0.1;    

double assistive_ratio = 0.08;   

int L_pos_int_d = 0;     
int L_pos_int_a = 0;     
int L_vel_int_d = 0;       
int L_vel_int_a = 0;      
 
int R_pos_int_d = 0;     
int R_pos_int_a = 0;       
int R_vel_int_d = 0;         
int R_vel_int_a = 0;       
//**************************

double cmd_ampl = 1.0;        
double cmd_fre  = 1.0;      
float pos_ampl  = 0.0;      
float pos_fre   = 0.5;      

float l_pos_des = 0.0;    
float l_vel_des = 0.0;    
float r_pos_des = 0.0;     
float r_vel_des = 0.0;     

float ref_force_ampl = 0.2;    
float ref_force_fre = 0.5;    

float l_ref_tau    = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau    = 0.0;     
float r_ref_tau_dt = 0.0;    

float l_leg_angle    = 0.0;  
float r_leg_angle    = 0.0;  
float l_leg_velocity = 0.0;   
float r_leg_velocity = 0.0;  

//***Impedance Control Test***//  
float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float dt = 0.01;    

float tau_t_1 = 0.0;  
float tau_t_1_last = 0.0;    
float tau_t_2 = 0.0;    
float tau_t_2_last = 0.0;   

float tau_dt_1 = 0.0;    
float tau_dt_2 = 0.0;     

float tau_ff_1 = 0.0;      
float tau_ff_2 = 0.0;   

//*** Motor Mode Set ***//   
int ctl_method = 1;    // 0 for using RL controller, 1 for using other normal controller  
int ctl_mode = 0;      // 0 for torque control, 1 for mit control    
int ctl_type = 0;      // 0 for motion, 1 for force tracking, 2 for direct torque   

int sensor_type = 0;   // 0 for using IMU, 1 for using encoder   
int l_ctl_dir = 1;      //确实是左脚，1是向上
int r_ctl_dir = 1;     //确实是右脚，1是向上
float torque_cmd_scale = 20.0;   
//*** Motor Mode Set ***//    

int doi          = 0;   
int currentpoint = 0;    
int delayindex   = 0;    
// double Rescaling_gain    = 0.01; // 1.6 for max 4 Nm

double LTx_filtered      = 0;
double LTx_filtered_last = 0;
double RTx_filtered      = 0;
double RTx_filtered_last = 0;
double torque_filtered   = 0;
double RLTx              = 0;
double RLTx_filtered     = 0;
double RLTx_filtered_last = 0;  // ✅ 这个变量你还没定义

double RLTx_delay[100]   = {};
double torque_delay[100] = {};  

int L_motor_torque_command = 0;
int R_motor_torque_command = 0;

double Rescaling_gain   = 3;
double Flex_Assist_gain = 2;
double Ext_Assist_gain  = 2;

/* ======= ① 新增全局参数 ======= */
int8_t phase_offset_L = 0;   // << 你可以开 BLE 命令实时改
int8_t phase_offset_R = 0;
/* ============================== */


// double Rescaling_gain    = 0;
// double Flex_Assist_gain  = 0;
// double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;  

double S_torque_command_left = 0.0;    
double S_torque_command_right = 0.0;   


/* ---------- 新增：标定 / 死区 / 滞环 ---------- */
float L0 = 0.0f, R0 = 0.0f;          // 开机零点
uint32_t nCal = 0;                   // 累计样本数
const float DEAD_TOR = 0.03f;        // 死区 (rad) 约 2°
const float CROSS_DEG = 20.0f;        // 滞环翻边阈值 (deg)
int8_t side = 0;                     // 当前步相位 -1/+1

// 把角度 low-pass 打包成函数，方便以后调系数
inline double lowpass(double x, double &y)
{
    y = 0.95 * y + 0.05 * x;
    return y;
}

// ≈8 byte 结构：0xAA 0x55  len  cmd  AngleX_L  AngleX_H  crc_L  crc_H
const uint8_t PKT_LEN = 8;

bool readPkt(HardwareSerial& port, float &angleDeg){
    static uint8_t buf[PKT_LEN];
    static uint8_t idx = 0;

    while (port.available()){
        uint8_t b = port.read();
        if (idx==0 && b!=0xAA)      continue;         // 找帧头
        if (idx==1 && b!=0x55){ idx=0; continue; }

        buf[idx++] = b;
        if (idx < PKT_LEN) continue;

        idx = 0;                                     // 准备下次

        /* —— 可选 CRC 校验 —— */

        int16_t raw = (buf[4] | (buf[5]<<8));        // little-endian
        angleDeg = raw * 0.01f;                      // 固件默认 0.01°
        return true;
    }
    return false;
}

// ===== Pi5 <-> Teensy over Serial8 (RX8/TX8) =====
#define PI_SERIAL      Serial8        // Teensy 4.1: RX8=pin34, TX8=pin35
#define PI_BAUD        115200
#define PI_USE_BINARY  1              // 1=二进制帧, 0=文本CSV

volatile float  tau_pi_L = 0.0f;     // Pi 侧给左腿   (→ M2)
volatile float  tau_pi_R = 0.0f;     // Pi 侧给右腿   (→ M1)
static float    tau_pi_L_prev = 0.0f, tau_pi_R_prev = 0.0f;   // 防止读包被切
// -------------------------------------------


static inline uint8_t cksum8(const uint8_t* p, size_t n){
  uint8_t s = 0; while (n--) s += *p++; return s;
}

// 发送给 Pi5：顺序 = Lpos(rad), Rpos(rad), Lvel(rad/s), Rvel(rad/s)
void sendIMUToPi(float Lpos_rad, float Rpos_rad, float Lvel, float Rvel) {
  uint8_t pkt[4 + 16 + 1];
  pkt[0]=0xA5; pkt[1]=0x5A;
  pkt[2]=17;         // TYPE(1)+ 4*float
  pkt[3]=0x01;       // IMU
  memcpy(&pkt[4],  &Lpos_rad,4);
  memcpy(&pkt[8],  &Rpos_rad,4);
  memcpy(&pkt[12], &Lvel,4);
  memcpy(&pkt[16], &Rvel,4);
  pkt[20] = cksum8(&pkt[3], 1+16);
  PI_SERIAL.write(pkt, sizeof(pkt));
}


// 期望 Teensy ← Pi: 纯 8 字节 —— 小端 float32 ×2
// 如果你在 Pi 端用 struct.pack('<ff', τL, τR)，这里就直接读 8 B
bool readTorqueFromPi(float& tauL, float& tauR){
    if (PI_SERIAL.available() < 8) return false;
    PI_SERIAL.readBytes((char*)&tauL, 4);
    PI_SERIAL.readBytes((char*)&tauR, 4);
    return true;
}



//// setup can and motors ////
void setup() {
  delay(3000);   

  Serial.begin(115200);     //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);    //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  PI_SERIAL.begin(PI_BAUD);   // <== 新增：Pi5 串口

  Serial_Com.INIT();    

  //#################
  // Serial.println("SETUP DONE");  
  // Serial.print("Controller executed at ");   
  // Serial.print(String(cyclespersec_ctrl));   
  // Serial.println(" Hz");   
  // Serial.print("BT com executed at ");   
  // Serial.print(String(cyclespersec_ble));   
  // Serial.println(" Hz");   
  //#################### 

  // torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_offset_calibration();              

  delay(1000);  
  memset(RLTx_delay,   0, sizeof(RLTx_delay));
  memset(torque_delay, 0, sizeof(torque_delay));

  SPI1.setMOSI(26);  
  SPI1.setMISO(1);  
  SPI1.setSCK(27);   

  initial_CAN();    
  Serial.println("Can bus setup done...");  

  initial_Sig_motor();   
  Serial.println("initial_Sig_motor bus setup done...");  

  IMUSetup();   
  Serial.println("IMUSetup bus setup done...");  
 
  delay(100);  

  if (logger.begin()) {
    Serial.print(F("SD Logging file: "));
    Serial.println(logger.filename());
    logger.println(F("Time,imu_RTx,imu_LTx,RLTx_delay,torque_delay,"
                     "tau_raw_L,tau_raw_R,S_torque_command_left,"
                     "S_torque_command_right,M1_torque_command,"
                     "M2_torque_command,Rescaling_gain,Flex_Assist_gain,"
                     "Ext_Assist_gain,Assist_delay_gain"));
    logger.flush();
  } else {
    Serial.println(F("SD card init or file create failed!"));
  }

  t_0 = micros();    
}  

void initial_Sig_motor() {
  // 进入控制模式
  sig_m1.enter_control_mode();
  delay(50);
  sig_m2.enter_control_mode();
  delay(50);



  // 发送一次“零扭矩”的命令，让对端有一帧有效回复，便于读取初始 pos/spe/torque
  sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // 读一段时间，直到拿到有效回复
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    CAN_message_t mr;
    while (Can3.read(mr)) {
      // tmotor 的回复统一走 MIT 回复帧
      sig_m1.unpack_reply(mr);
      sig_m2.unpack_reply(mr);
    }
  }

  // 记录初始零位，后续用相对位姿可以自己相减
  initial_pos_1 = sig_m1.pos;
  initial_pos_2 = sig_m2.pos;

  // 初始化命令
  M1_torque_command = 0.0;
  M2_torque_command = 0.0;

  Serial.println("tmotor torque-only (t_ff) path ready.");
}



// ==== Params (可由 BLE 覆盖) ====
volatile float  alpha_tau     = 0.85f;   // 一阶低通
volatile float  torque_rate   = 150.0f;  // Nm/s


// ---- Gate 参数（默认与仿真一致，可 BLE 改）----
volatile float  gate_k       = 6.0f;   // tanh 的 k
volatile float  gate_p_on    = 0.0f;   // power-on 阈值 (τ·ω 的阈)
volatile uint8_t gate_lead_ms = 20;    // ★ 默认 20ms（可由 BLE 改）

// ---- Gate 输入预测需要的状态（每腿各一个）----
static float xL_prev = 0.0f;
static float xR_prev = 0.0f;


float tau_raw_L = 0.0f, tau_raw_R = 0.0f;

float tau_cmd_L_filt = 0.0f, tau_cmd_R_filt = 0.0f;
float M1_prev = 0.0f, M2_prev = 0.0f;

float clip_torque(float t){
    const float max_torque = 12.0f;
    return (t >  max_torque) ?  max_torque :
           (t < -max_torque) ? -max_torque : t;
}
// 0.5*(tanh(k*(x - p_on)) + 1)  带阈值的软门
inline float smooth_gate_p(float x, float k, float p_on){
  return 0.5f * (tanhf(k * (x - p_on)) + 1.0f);
}

// 一阶线性预测：用当前斜率估计 lead_s 后的 x
inline float lead_predict(float x, float x_prev, float lead_ms, float Ts){
  const float lead_s = 0.001f * lead_ms;      // ms -> s
  const float dx     = (x - x_prev) / Ts;     // 斜率
  return x + lead_s * dx;                     // 预测 x(t+lead_s)
}


inline float slew(float target, float prev, float rate, float Ts){
    float diff = target - prev, maxDiff = rate * Ts;
    if (fabs(diff) > maxDiff) target = prev + copysignf(maxDiff, diff);
    return target;
}
/****************************************/

void loop()
{
  imu.READ();
  Serial_Com.READ2();

  current_time = micros() - t_0;
  t = current_time / 1e6f;

  if (current_time - previous_time > Tinterval_ctrl_micros) {
    previous_time = current_time;          // ★ 必须加上这一句

    const float Ts = Tinterval_ctrl_micros / 1e6f;   // 控制周期

    /* BLE */
    if (current_time - previous_time_ble > Tinterval_ble_micros){
        Receive_ble_Data();
        Transmit_ble_Data();
        previous_time_ble = current_time;
    }

    /* --- 滤波角度 --- */
    RLTx = imu.RTx - imu.LTx;

    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = 0.95f * LTx_filtered_last + 0.05f * imu.LTx;

    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = 0.95f * RTx_filtered_last + 0.05f * imu.RTx;

    Serial.printf("BLE: Angle Left=%.2f Angle Right=%.2f\n",
      LTx_filtered, RTx_filtered);
    /* --- 角速度(°/s) ---> rad/s 更物理，不过比例因子无关宏旨 --- */
    const float LTx_vel = (LTx_filtered - LTx_filtered_last) / Ts;
    const float RTx_vel = (RTx_filtered - RTx_filtered_last) / Ts;
    /*send to python*/

    sendIMUToPi(LTx_filtered, RTx_filtered, LTx_vel, RTx_vel);

    static uint32_t pi_last_us = 0;
    const  uint32_t PI_TIMEOUT_US = 100000;
    
    float tauL_tmp, tauR_tmp;
    if (readTorqueFromPi(tauL_tmp, tauR_tmp)) {
        tau_pi_L = tauL_tmp; tau_pi_R = tauR_tmp;
        pi_last_us = current_time;
    } 
    if ((current_time - pi_last_us) > PI_TIMEOUT_US) {
        tau_pi_L = tau_pi_R = 0.0f;
    }
    
     /* === Pi → Teensy 扭矩覆盖 === */
     S_torque_command_left  = tau_pi_L;   // 左腿  (→ M2)
     S_torque_command_right = tau_pi_R;   // 右腿  (→ M1)


     M1_torque_command = S_torque_command_right * r_ctl_dir;
     M2_torque_command = S_torque_command_left  * l_ctl_dir;

     Serial.printf("BLE: M1=%.2f M2=%.2f\n",
      M1_torque_command, M2_torque_command);
    if (ctl_mode == 1){
        sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M1_torque_command);
        sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M2_torque_command);
        receive_mit_ctl_feedback();
    }else{
        for (int i=0;i<4;++i) receive_mit_ctl_feedback();
        sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M1_torque_command);
        sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M2_torque_command);
    }
    previous_time = current_time;
    // == 日志写入（100Hz）==
    if (logger.isOpen()) {
      logger.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        current_time / 1000,   // Time in ms
        imu.RTx,
        imu.LTx,
        RLTx_delay[doi],        // 当前帧
        torque_delay[doi],      // 当前帧
        tau_raw_L,
        tau_raw_R,
        S_torque_command_left,
        S_torque_command_right,
        M1_torque_command,
        M2_torque_command,
        Rescaling_gain,
        Flex_Assist_gain,
        Ext_Assist_gain,
        Assist_delay_gain
      );
      // logger.flush(); // 可每N次flush，或1s flush一次以保护SD卡寿命
    }
    static int log_flush_count = 0;
    if (++log_flush_count >= 10) {
      logger.flush();
      log_flush_count = 0;
    }
}
}







void IMUSetup() {
  imu.INIT();   
  delay(1500);     
  imu.INIT_MEAN();     
}  




void initial_CAN() {
  Can3.begin();
  // Can3.setBaudRate(1000000);  
  Can3.setBaudRate(1000000);  
  delay(400);  
  Serial.println("Can bus setup done...");  
  delay(200);  
}   

float Sig_torque_control(float force_des, float dt_force_des, float force_t, float dt_force_t, float kp, float kd, float tau_ff)  
{
  float tor_cmd = 0;   

  tor_cmd = kp * (force_des - force_t) + kd * (dt_force_des - dt_force_t) + tau_ff; 

  return tor_cmd;   
}  

float Sig_motion_control(float pos_des, float vel_des, float pos_t, float vel_t, float kp, float kd, float tau_ff)  
{
  float pos_ctl_cmd = 0;   

  pos_ctl_cmd = kp * (pos_des - pos_t) + kd * (vel_des - vel_t) + tau_ff; 

  return pos_ctl_cmd;   
}  

void receive_mit_ctl_feedback() {
  CAN_message_t mr;
  while (Can3.read(mr)) {
    sig_m1.unpack_reply(mr);
    sig_m2.unpack_reply(mr);
  }

  // 同步实测扭矩（Nm）
  M1_torque_meas = sig_m1.torque;
  M2_torque_meas = sig_m2.torque;

  // 如果你要相对位姿：
  // float pos1_rel = sig_m1.pos - initial_pos_1;
  // float pos2_rel = sig_m2.pos - initial_pos_2;
}   

const uint16_t ID_M1_POSVEL = (0x002<<5) | 0x009;  // 0x049
const uint16_t ID_M1_TORQUE = (0x002<<5) | 0x01C;  // 0x05C
const uint16_t ID_M2_POSVEL = (0x001<<5) | 0x009;  // 0x029
const uint16_t ID_M2_TORQUE = (0x001<<5) | 0x01C;  // 0x03C
const uint16_t ID_M1_IQ = (0x002<<5) | 0x014;  // 0x064
const uint16_t ID_M2_IQ = (0x001<<5) | 0x014;  // 0x034
#define KT_1  0.43f   
#define KT_2  0.43f   


void SendIMUSerial()
{
  L_IMUX_int = Serial_Com.float_to_uint(l_leg_angle, -180, 180, 16);     
  R_IMUX_int = Serial_Com.float_to_uint(r_leg_angle, -180, 180, 16);     

  L_IMUV_int = Serial_Com.float_to_uint(l_leg_velocity, -800, 800, 16);     
  R_IMUV_int = Serial_Com.float_to_uint(r_leg_velocity, -800, 800, 16);         
  
  Send[0] = 0x31;  
  Send[1] = 0x32;  
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;  
  Send[4] = R_IMUX_int >> 8;  
  Send[5] = R_IMUX_int & 0xFF;  
  Send[6] = L_IMUV_int >> 8;  
  Send[7] = L_IMUV_int & 0xFF;  
  Send[8] = R_IMUV_int >> 8;  
  Send[9] = R_IMUV_int & 0xFF;  
  Send[10] = 0x33;   
}


void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}

bool findFrameHeader()
{
    static uint8_t h[3]={0};
    while (Serial5.available()) {
        h[0]=h[1];  h[1]=h[2];  h[2]=Serial5.read();
        if(h[0]==165 && h[1]==90 && h[2]==20) return true;
    }
    return false;
}

void Receive_ble_Data()
{
    while (findFrameHeader()) {
        if (Serial5.available() < 17)   // 这里 17 已够用（我们最多用到 index 13）
            return;

        Serial5.readBytes(data_rs232_rx, 17);

        Rescaling_gain    = int16_t(data_rs232_rx[0] | data_rs232_rx[1] << 8) / 100.0f;
        Flex_Assist_gain  = int16_t(data_rs232_rx[2] | data_rs232_rx[3] << 8) / 100.0f;
        Ext_Assist_gain   = int16_t(data_rs232_rx[4] | data_rs232_rx[5] << 8) / 100.0f;

        Rescaling_gain    = constrain(Rescaling_gain,    0.0f, 10.0f);
        Flex_Assist_gain  = constrain(Flex_Assist_gain,  0.0f, 10.0f);
        Ext_Assist_gain   = constrain(Ext_Assist_gain,   0.0f, 10.0f);

        Assist_delay_gain = data_rs232_rx[6];
        if (Assist_delay_gain > 99) Assist_delay_gain = 99;

        // === New: 相位补偿（int8）
        phase_offset_L = int8_t(data_rs232_rx[7]);  // e.g., -20~+20 帧（10ms/帧）
        phase_offset_R = int8_t(data_rs232_rx[8]);

        // === New: Gate 参数 ===
        int16_t k100   = int16_t(data_rs232_rx[9]  | data_rs232_rx[10] << 8);
        int16_t pon100 = int16_t(data_rs232_rx[11] | data_rs232_rx[12] << 8);
        uint8_t leadms = data_rs232_rx[13];

        gate_k        = constrain(k100   / 100.0f, 0.1f, 20.0f);  // 0.1~20
        gate_p_on     = pon100 / 100.0f;                          // 自行根据数据范围设定
        gate_lead_ms  = (leadms > 200) ? 200 : leadms;            // 0~200ms

        // （可选）调试输出
        Serial.printf("BLE: Res=%.2f Flex=%.2f Ext=%.2f | Assist=%u phaseL=%d phaseR=%d | k=%.2f p_on=%.2f lead=%ums\n",
                      Rescaling_gain, Flex_Assist_gain, Ext_Assist_gain,
                      Assist_delay_gain, phase_offset_L, phase_offset_R,
                      gate_k, gate_p_on, gate_lead_ms);
    }
}



void Transmit_ble_Data() {

  t_teensy        = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU_angle = imu.RTx * 100;
  L_motor_torque  = sig_m1.torque * 100;
  R_motor_torque  = sig_m2.torque * 100;
  L_motor_torque_command = M1_torque_command *100;
  R_motor_torque_command = M2_torque_command *100;

  // L_load_cell_torque = torque_sensor1.torque[0] * 100;  
  // R_load_cell_torque = torque_sensor1.torque[1] * 100;    

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = L_leg_IMU_angle;
  data_ble[6]  = L_leg_IMU_angle >> 8;
  data_ble[7]  = R_leg_IMU_angle;
  data_ble[8]  = R_leg_IMU_angle >> 8;
  data_ble[9]  = L_load_cell_torque;
  data_ble[10] = L_load_cell_torque >> 8;  
  data_ble[11] = R_load_cell_torque;  
  data_ble[12] = R_load_cell_torque >> 8;  
  data_ble[13] = L_motor_torque_command;
  data_ble[14] = L_motor_torque_command >> 8;
  data_ble[15] = R_motor_torque_command;
  data_ble[16] = R_motor_torque_command >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);


}

