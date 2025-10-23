/* -------------  IMU_Adapter.cpp ------------- */
#include "IMU_Adapter.h"

float deriv(float cur, float dt)
{
    static float prev = 0.0f;
    float vel = (cur - prev) / dt;
    prev = cur;
    return vel;
}

/* ------- 1. 初始化 -------- */
void IMU_Adapter::INIT() {
    SerialImuLeft.begin(460800);
    SerialImuRight.begin(460800);

    /* 唤醒 & 设置参数（左右各一条） */
    Cmd_03_Left();             delay(200);
    Cmd_03_Right();            delay(200);
/* 左右 IMU 统一改为 */
/* 只改 Tag：0x44  */
/* 保留角速度，库默认支持 10 B 帧 */
Cmd_12_Left (5,255,0,0,3,100, 2,4,9, 0x44);  // Euler-X + Gyro-X
Cmd_12_Right(5,255,0,0,3,100, 2,4,9, 0x44);


        delay(200);

    Cmd_19_Left();             delay(200);
    Cmd_19_Right();            delay(200);
}

/* ------- 2. 静止取平均做零漂 ------- */
void IMU_Adapter::INIT_MEAN() {
    const int N = 300;
    float sumL=0, sumR=0;

    for (int i=0;i<N;i++){
        READ();
        sumL += AngleXLeft;
        sumR += AngleXRight;
        delay(5);
    }
    offL = sumL/N;
    offR = sumR/N;
}

/* ------- 3. 每次循环更新 ------- */
/* ------- 3. 每次循环更新 ------- */
void IMU_Adapter::READ(){
    while(SerialImuLeft.available())
        pollOneByte(SerialImuLeft , Cmd_GetPkt_Left);
    while(SerialImuRight.available())
        pollOneByte(SerialImuRight, Cmd_GetPkt_Right);

    LTx   = AngleXLeft  - offL;
    RTx   = AngleXRight - offR;
    LTAVx = VelXLeft;
    RTAVx = VelXRight;

// #if 1          // ← 改成 1 才打印！默认关
//     static uint8_t dbg = 0;          // 每 10 帧打印一次
//     if (++dbg >= 100) {
//         dbg = 0;
//         if (Serial.availableForWrite() > 64) {      // **非阻塞**
//             Serial.printf("Lθ %.1f , Rθ %.1f | Lω %.1f , Rω %.1f\r\n",
//                           LTx, RTx, LTAVx, RTAVx);
//         }
//     }
// #endif
}


/* ------- 辅助：一字节进状态机 ------- */
void IMU_Adapter::pollOneByte(HardwareSerial& port,
                              uint8_t (&pktFunc)(uint8_t))
{
    U8 byte = port.read();
    pktFunc(byte);         // im948_CMD 自带解包状态机
}
