#include <Arduino.h>
#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
#include <MadgwickAHRS.h>
#include <rc.hpp>
#include <telemetry.hpp>

UnitRollerI2C RollerI2C_RIGHT;  // Create a UNIT_ROLLERI2C object
UnitRollerI2C RollerI2C_LEFT;
//I2Cドライバの多重初期化を防ぐためのフラグ
bool UnitRollerI2C::initialized = false;

float Pitch_ahrs, Roll_ahrs, Yaw_ahrs, Roll_bias, Roll;
float Gyro_x, Gyro_y, Gyro_z;
float Acc_x, Acc_y, Acc_z;
int32_t Imu_time,_Imu_time, Imu_dtime;
int32_t Current_ref_r, Current_ref_l;
int32_t Current_r, Current_l;
int32_t Pos_r, Pos_l, Pos_bias_r, Pos_bias_l;
int32_t Speed_r, Speed_l;
int32_t Voltage_r,Voltage_l;
int32_t St, _St, Et, Dt;
float f1,f2,f3,f4;
float k1;
float U0, U_yaw, U_v;
uint8_t Start_flag = 0;
//TaskHandle_t xHandle = NULL;

void dummyTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        //printf("time:%6.3fms roll:%7.3f pitch:%7.3f yaw:%7.3f C_r:%7d C_l:%7d P_r:%8.3f P_l:%8.3f S_r:%8.3f S_l:%8.3f\n",
        //(float)(Dt)/1.0e3, Roll_ahrs, Pitch_ahrs, Yaw_ahrs,
        //Current_r, Current_l, (float)Pos_r/100.0, (float)Pos_l/100.0, (float)Speed_r/100.0, (float)Speed_l/100.0);

        M5.Display.setCursor(0, 10);
        M5.Display.printf("%5.1f", Roll);
        M5.Display.setCursor(0, 50);
        M5.Display.printf("%4.1fV", (float)Voltage_r/100.0);
        M5.Display.setCursor(0, 90);
        M5.Display.printf("%4.1f", (float)Dt/1.0e3);

        M5.update();
        if(M5.BtnA.wasPressed()){
            //Star_flagをトグル 
            Start_flag = !Start_flag;
            Roll_bias = Roll;
            Pos_bias_r = Pos_r;
            Pos_bias_l = Pos_l;
        }
        TelemetryData.Roll = Roll;
        TelemetryData.Pitch = Pitch_ahrs;
        TelemetryData.Yaw = Yaw_ahrs;
        TelemetryData.RollRate = Gyro_x;
        TelemetryData.PitchRate = Gyro_y;
        TelemetryData.YawRate = Gyro_z;
        TelemetryData.CurrentR = (float)Current_r/100.0;
        TelemetryData.CurrentL = (float)Current_l/100.0;
        TelemetryData.CurrentRefR = (float)Current_ref_r/100.0;
        TelemetryData.CurrentRefL = (float)Current_ref_l/100.0;
        TelemetryData.PosR = (float)Pos_r/100.0;
        TelemetryData.PosL = (float)Pos_l/100.0;
        TelemetryData.SpeedR = (float)Pos_bias_r/100.0;
        TelemetryData.SpeedL = (float)Pos_bias_l/100.0;
        TelemetryData.VoltageR = (float)Voltage_r/100.0;
        TelemetryData.Dt = (float)Dt/1.0e3;

        telemetry();

        #if 0
        printf(">Dt:%5.2f\n", (float)Dt/1.0e3);
        printf(">Roll:%f\n", Roll_ahrs);
        printf(">R_R:%f\n", Gyro_x);
        printf(">C_r:%f\n", (float)Current_r/100.0);
        printf(">C_l:%f\n", (float)Current_l/100.0);
        printf(">Cref_r:%f\n", (float)Current_ref_r/100.0);
        printf(">Cref_l:%f\n", (float)Current_ref_l/100.0);
        printf(">P_r:%f\n", (float)(Pos_r - Pos_bias_r)/100.0);
        printf(">P_l:%f\n", (float)(Pos_l - Pos_bias_l)/100.0);
        printf(">S_r:%f\n", (float)Speed_r/100.0);
        printf(">S_l:%f\n", (float)Speed_l/100.0);
        printf(">V_r:%f\n", (float)Voltage_r/100.0);
        //printf(">V_l:%f\n", (float)Voltage_l/100.0);
        #endif

    }
}

void taskFunction(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms の周期

    // 初期化
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        _St = St;
        St = micros();
        M5.Imu.update();
        auto imudata = M5.Imu.getImuData();
        Gyro_x = imudata.gyro.x;
        Gyro_y = imudata.gyro.y;
        Gyro_z = imudata.gyro.z;
        Acc_x = imudata.accel.x;
        Acc_y = imudata.accel.y;
        Acc_z = imudata.accel.z;
        MadgwickAHRSupdateIMU(Gyro_x * DEG_TO_RAD, Gyro_y * DEG_TO_RAD, Gyro_z * DEG_TO_RAD, Acc_x, Acc_y, Acc_z, &Pitch_ahrs, &Roll_ahrs, &Yaw_ahrs);
        k1 = 0.3;
        Roll = k1*Roll +(1-k1)*Roll_ahrs;
        Current_r =  RollerI2C_RIGHT.getCurrentReadback();
        Current_l = -RollerI2C_LEFT.getCurrentReadback();
        Pos_r =  RollerI2C_RIGHT.getPosReadback();
        Pos_l = -RollerI2C_LEFT.getPosReadback();
        Speed_r =  RollerI2C_RIGHT.getSpeedReadback();
        Speed_l = -RollerI2C_LEFT.getSpeedReadback();
        Voltage_r = RollerI2C_RIGHT.getVin();
        //Voltage_l = RollerI2C_LEFT.getVin();

        // current control
        if(Start_flag==1){
            f1 = 7500.0;//4200.0;//振子の角度に比例して電流を制御
            f2 = 150.0;//200.0;//振子の角速度に比例して電流を制御
            f3 = 0.0;//モータの角度に比例して電流を制御 0.1
            f4 = 0.0;//モータの角速度に比例して電流を制御
            float yaw_ref = -360.0*Stick[RUDDER];
            float yaw_err = yaw_ref - Gyro_z;
            U_yaw = yaw_err * 300.0;
            U_v = -Stick[THROTTLE] * 200000.0;
            //State feedback control
            U0 = (-f1 * (Roll-Roll_bias) - f2 * Gyro_x - f3 * ((float)(Pos_r-Pos_bias_r)/1.0) - f4 * Speed_r);
            Current_ref_r = (int32_t)(U0 + U_v + U_yaw);
            Current_ref_l = (int32_t)(U0 + U_v - U_yaw);
            //limit current
            if (Current_ref_r>120000)Current_ref_r=120000;
            else if (Current_ref_r<-120000)Current_ref_r=-120000;
            if (Current_ref_l>120000)Current_ref_l=120000;
            else if (Current_ref_l<-120000)Current_ref_l=-120000;

            RollerI2C_RIGHT.setCurrent(Current_ref_r);
            RollerI2C_LEFT.setCurrent(-Current_ref_l);
        }
        else{
            RollerI2C_RIGHT.setCurrent(0);
            RollerI2C_LEFT.setCurrent(0);
        }
        Et = micros();
        Dt = Et - St;

    }
}

void setup(){

    Roll_bias = 0.0;
    auto cfg = M5.config();     
    M5.begin(cfg);
    M5.Display.setTextSize(4);               // テキストサイズを変更
    //M5.Display.setCursor(0, 20);
    //M5.Display.printf("%5.1f", Roll_ahrs);       // 画面にHello World!!と1行表示
    rc_init();
    delay(2000);
    printf("Start\n");
    if(RollerI2C_RIGHT.begin(0x64, 2, 1, 400000)==true){
        printf("RollerI2C_RIGHT begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT begin failed\n");
    }
    if(RollerI2C_LEFT.begin(0x65, 2, 1, 400000)==true){
        printf("RollerI2C_LEFT begin success\n");
    }
    else{
        printf("RollerI2C_LEFT begin failed\n");
    }

    RollerI2C_RIGHT.setMode(3);
    RollerI2C_LEFT.setMode(3);
    RollerI2C_RIGHT.setCurrent(0);
    RollerI2C_LEFT.setCurrent(0);
    RollerI2C_RIGHT.setOutput(1);
    RollerI2C_LEFT.setOutput(1);
    delay(1000);


    // FreeRTOSタスクの作成
    BaseType_t result = xTaskCreateUniversal(
        taskFunction,
        "5ms Periodic Task",
        8192,
        NULL,
        5,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }

    result = xTaskCreateUniversal(
        dummyTask,
        "Dummy Task",
        8192,
        NULL,
        1,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }
}

void loop() {
    // FreeRTOSを使用する場合、loop関数は空にします
    delay(1);
}