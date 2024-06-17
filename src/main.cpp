#include <Arduino.h>
#include <config/pin.h>
#include <lib/wheels/wheels.h>
#include <lib/encoder/encoder.h>
// #include <lib/i2c/scani2c.h>
#include <MPU6050.h>

#include <Wire.h>
// #include <MPU6050_6Axis_MotionApps20.h>

void MoveBase_Forward(int Speed);
void MoveBase_SideRight(int Speed);
void MoveBase_SideLeft(int Speed);
void MoveBase_Stop();

void AppendMSG(int pos, String msg);
void DebugData();
bool ControlRobot(float target);
void startMotion();
void Scani2c();
void InitMPU();
void ReadGyroData();
void CalculateAngle();
void CalibrateMPU();

Wheel M1(M1_PIN_IN1, M1_PIN_IN2, M1_PIN_PWM, -4);
Wheel M2(M2_PIN_IN1, M2_PIN_IN2, M2_PIN_PWM, 0);
Wheel M3(M3_PIN_IN1, M3_PIN_IN2, M3_PIN_PWM, -4);
Wheel M4(M4_PIN_IN1, M4_PIN_IN2, M4_PIN_PWM, 0);

String valuedebug[10];
int valuedebug_len = 4;

void updateEncoder1();

Encoder Position(ENCODER1_A_PIN, ENCODER1_B_PIN, updateEncoder1);

float integral = 0;
float prevError = 0;
unsigned long rampUpDuration = 1000;
unsigned long startTime = 0;

// TwoWire Wire2(PB11, PB10);
// MPU6050 mpu;

const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;
unsigned long last_time = 0;
float gyro_z_angle = 0.0;
char tmp_str[7];

float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

char *convert_int16_to_str(int16_t i)
{
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
HardwareSerial Serial3(PB11, PB10);
void setup()
{
  Serial.begin(115200);
  Serial3.begin(9600);
  // mpu.initialize();

  // Serial.println(mpu.getDeviceID());
  // Serial.println("Ready");
  //   calibrateMPU6050();
  // InitMPU();
  //  CalibrateMPU();

  M1.Init();
  M2.Init();
  M3.Init();
  M4.Init();

  startMotion();
}
String data = "";

int Sensor_L = 0;
int Sensor_R = 0;

int Ultra_L1 = 0;
int Ultra_L2 = 0;

int Ultra_R1 = 0;
int Ultra_R2 = 0;

int Mode = 0;

bool State_Blue_1 = true;
bool State_Blue_2 = true;

void Read_DataSerial()
{
  while (Serial3.available() > 0)
  {
    char inByte = Serial3.read();
    if (inByte == '\n')
    {

      Sensor_L = data.substring(0, 1).toInt();
      Sensor_R = data.substring(2, 3).toInt();

      Ultra_L1 = data.substring(4, 9).toInt();
      Ultra_L2 = data.substring(10, 15).toInt();
      Ultra_R1 = data.substring(16, 21).toInt();
      Ultra_R2 = data.substring(22, 27).toInt();
      Mode = data.substring(28, 29).toInt();

      Serial.print(Sensor_L);
      Serial.print(",");
      Serial.print(Sensor_R);
      Serial.print(",");
      Serial.print(Ultra_L1);
      Serial.print(",");
      Serial.print(Ultra_L2);
      Serial.print(",");
      Serial.print(Ultra_R1);
      Serial.print(",");
      Serial.print(Ultra_R2);
      Serial.print(",");
      Serial.println(Mode);
      data = "";
    }
    else if (inByte == '\r')
    {
    }
    else
    {
      data += inByte;
    }
    // Serial.write(inByte);
  }
}

void loop()
{
  /*..


   M1.ControlMotorSpeed(50);
   M2.ControlMotorSpeed(50);
   M3.ControlMotorSpeed(50);
   M4.ControlMotorSpeed(50);

  */
  Read_DataSerial();

  if (Mode == 0)
  {
  }
  else if (Mode == 1)
  {
    while (State_Blue_1)
    {
      Read_DataSerial();
      if (Ultra_R1 >= 4 || Ultra_R2 >= 4)
      {
        MoveBase_SideRight(80);
        // MoveBase_SideLeft(80);
      }
      else
      {
        MoveBase_Forward(80);
      }
      if (!Sensor_L && !Sensor_R)
      {
        State_Blue_1 = false;
        State_Blue_2 = true;

        M1.ControlMotorSpeed(0);
        M2.ControlMotorSpeed(0);
        M3.ControlMotorSpeed(0);
        M4.ControlMotorSpeed(0);

        MoveBase_Stop();
        delay(1000);
      }
    }
    while (State_Blue_2)
    {
      Read_DataSerial();
      //MoveBase_SideRight(60);
      MoveBase_SideLeft(60);

      if (Sensor_L && Sensor_R)
      {
        State_Blue_1 = false;
        State_Blue_2 = false;
        delay(1000);
        MoveBase_Stop();

        M1.ControlMotorSpeed(0);
        M2.ControlMotorSpeed(0);
        M3.ControlMotorSpeed(0);
        M4.ControlMotorSpeed(0);

        delay(1000);

        MoveBase_Forward(100);

        delay(5000);
        M1.ControlMotorSpeed(0);
        M2.ControlMotorSpeed(0);
        M3.ControlMotorSpeed(0);
        M4.ControlMotorSpeed(0);
      }
    }
  }

  // ReadGyroData();
  // CalculateAngle();
  // delay(1);
  //  Scani2c();
  /*
  float curent = (((Position.read() / 1266.0) * 100.0));

  AppendMSG(0, String(Position.read()));
  AppendMSG(1, String(curent));
  DebugData();
  */

  // ControlRobot(100);

  // Scani2c();
}

void updateEncoder1()
{
  Position.update();
}

void AppendMSG(int pos, String msg)
{
  valuedebug[pos] = msg;
}

void DebugData()
{

  for (int a = 0; a < valuedebug_len; a++)
  {
    Serial.print(valuedebug[a]);
    Serial.print(",");
  }
  Serial.println();
}

void startMotion()
{
  delay(1000);
  startTime = millis(); // บันทึกเวลาที่เริ่มต้น
  last_time = millis();
}

bool ControlRobot(float target)
{

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float sensor = gx / 131.0; // แปลงค่าจาก raw data

  float curent = (((Position.read() / 1166.0) * 100.0));
  float error = target - curent;

  integral += error;

  float output = Kp * error + Ki * integral + Kd * (error - prevError);

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  float rampFactor = 1.0;

  if (elapsedTime < rampUpDuration)
  {
    rampFactor = (float)elapsedTime / rampUpDuration; // คำนวณ factor การเพิ่มความเร็ว
  }

  output = (output * Gain) * rampFactor;

  AppendMSG(0, String(Position.read()));
  AppendMSG(1, String(curent));
  AppendMSG(2, String(error));
  AppendMSG(3, String(gz));
  // DebugData();

  MoveBase_Forward(error);

  return true;
}

void MoveBase_Forward(int Speed)
{
  M1.ControlMotorSpeed(-Speed);
  M2.ControlMotorSpeed(Speed);
  M3.ControlMotorSpeed(-Speed);
  M4.ControlMotorSpeed(Speed);
}

void MoveBase_SideRight(int Speed)
{
  M1.ControlMotorSpeed(-Speed);
  M2.ControlMotorSpeed(-Speed);
  M3.ControlMotorSpeed(Speed);
  M4.ControlMotorSpeed(Speed);
}
void MoveBase_SideLeft(int Speed)
{
  M1.ControlMotorSpeed(Speed);
  M2.ControlMotorSpeed(Speed);
  M3.ControlMotorSpeed(-Speed);
  M4.ControlMotorSpeed(-Speed);
}

void MoveBase_Stop()
{
  M1.ControlMotorSpeed(0);
  M2.ControlMotorSpeed(0);
  M3.ControlMotorSpeed(0);
  M4.ControlMotorSpeed(0);
}
