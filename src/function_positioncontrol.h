#include <Arduino.h>

#include <lib/encoder/encoder.h>
#include <lib/wheels/wheels.h>
#include <config/pin.h>
#include <lib/i2c/scani2c.h>
#include <lib/kalmanfilter/kalmanfilter.h>

// TwoWire Wire2(PB11, PB10);

// #include <config/parameter.h>

void updateEncoder1();
void updateEncoder2();
void calibrateMPU6050();

Encoder PositionX(ENCODER1_A_PIN, ENCODER1_B_PIN, updateEncoder1);
Encoder PositionY(ENCODER2_A_PIN, ENCODER2_B_PIN, updateEncoder2);

Wheel M1(M1_PIN_IN1, M1_PIN_IN2, M1_PIN_PWM, 0);
Wheel M2(M2_PIN_IN1, M2_PIN_IN2, M2_PIN_PWM, -13);
Wheel M3(M3_PIN_IN1, M3_PIN_IN2, M3_PIN_PWM, 0);
Wheel M4(M4_PIN_IN1, M4_PIN_IN2, M4_PIN_PWM, -13);

KalmanFilter kalmanX;
KalmanFilter kalmanY;

unsigned long timer_kalmanfilter = 0;

float prevErrorX = 0;
float prevErrorY = 0;
float integralX = 0;
float integralY = 0;

float DPP = (0.1596 / 100.0);

unsigned long startTime = 0;
unsigned long rampUpDuration = 1000;

unsigned long EndPosition = 0;


// ตัวแปรสำหรับการชดเชยมุมเอียงเริ่มต้น
float initialAngleX = 0;
float initialAngleY = 0;

float RotationCompensationFactor = 0.05;




unsigned long timer = 0;
float elapsedTime, currentTime, previousTime;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccAngleX, AccAngleY;
float GyroAngleX, GyroAngleY, GyroAngleZ;
float AngleX, AngleY, AngleZ;
float ForceMagnitudeApprox;



void InitRobot()
{
    M1.Init();
    M2.Init();
    M3.Init();
    M4.Init();
    PositionX.begin();
    PositionY.begin();

    HardwareTimer *MyTim1 = new HardwareTimer(TIM2);
    MyTim1->setPrescaleFactor(0); // Prescaler = 1
    MyTim1->setOverflow(143);     // Overflow = 720 for 50 kHz
    MyTim1->resume();

    // Configure Timer 3 for PA2 and PA3
    HardwareTimer *MyTim2 = new HardwareTimer(TIM3);
    MyTim2->setPrescaleFactor(0); // Prescaler = 1
    MyTim2->setOverflow(143);     // Overflow = 720 for 50 kHz
    MyTim2->resume();

    I2cinit();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // calibrateMPU6050();

    delay(1000);
    pinMode(PC13, OUTPUT);
    for (int a = 0; a < 10; a++)
    {
        digitalWrite(PC13, 1);
        delay(100);
        digitalWrite(PC13, 0);
        delay(100);
    }

    previousTime = millis();
}

void DebugPosition()
{
    if (_Debug)
    {
        Serial.print("x = ");
        Serial.print(PositionX.read());
        Serial.print(" : y = ");
        Serial.println(PositionY.read());
    }
}

void startMotion()
{
    delay(100);
    startTime = millis(); // บันทึกเวลาที่เริ่มต้น

    timer_kalmanfilter = millis();
}

// ตัวแปรสำหรับเก็บค่า offset
float gyroXOffset, gyroYOffset, gyroZOffset;
float accXOffset, accYOffset, accZOffset;

float gyrovalue = 0;

void calibrateMPU6050()
{
    const int sampleSize = 1000;
    long axSum = 0, aySum = 0, azSum = 0;
    long gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < sampleSize; i++)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        // อ่านค่าจากเซ็นเซอร์
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // รวมค่า
        axSum += ax;
        aySum += ay;
        azSum += az;
        gxSum += gx;
        gySum += gy;
        gzSum += gz;

        delay(3); // หน่วงเวลาเล็กน้อยเพื่อให้การอ่านค่ามีความต่อเนื่อง
    }

    // คำนวณค่า offset
    accXOffset = axSum / sampleSize;
    accYOffset = aySum / sampleSize;
    accZOffset = azSum / sampleSize;
    gyroXOffset = gxSum / sampleSize;
    gyroYOffset = gySum / sampleSize;
    gyroZOffset = gzSum / sampleSize;
}

void ReadGyro()
{
    unsigned long currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0; // แปลงเป็นวินาที
  previousTime = currentTime;

  // อ่านค่าจาก accelerometer และ gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // แปลงค่า accelerometer เป็น g (แรงโน้มถ่วง)
  AccX = (float)ax / 16384.0;
  AccY = (float)ay / 16384.0;
  AccZ = (float)az / 16384.0;

  // แปลงค่า gyroscope เป็น degrees/sec
  GyroX = (float)gx / 131.0;
  GyroY = (float)gy / 131.0;
  GyroZ = (float)gz / 131.0;

  // คำนวณมุมจาก accelerometer
  AccAngleX = (atan2(AccY, AccZ) * 180.0 / PI);
  AccAngleY = (atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI);

  // คำนวณมุมจาก gyroscope โดยการบูรณาการ
  GyroAngleX += GyroX * elapsedTime;
  GyroAngleY += GyroY * elapsedTime;
  GyroAngleZ += GyroZ * elapsedTime;

  // Complementary filter - รวมค่าจาก accelerometer และ gyroscope
  AngleX = 0.96 * (AngleX + GyroX * elapsedTime) + 0.04 * AccAngleX;
  AngleY = 0.96 * (AngleY + GyroY * elapsedTime) + 0.04 * AccAngleY;
  AngleZ += GyroZ * elapsedTime; // Assuming the Z angle (yaw) is only determined by the gyroscope

  
  Serial.println(AngleZ);

  // ไม่มีการใช้ delay เพื่อให้ลูปรันได้เร็วขึ้น
}

bool RobotControl(float targetX, float targetY)
{
    float curentX = (-((PositionX.read() / 1090.0) * 100.0));
    float curentY = ((PositionY.read() / 1090.0) * 100.0);

    float errorX = targetX - curentX;
    float errorY = targetY - curentY;

    // Define this factor based on your requirements
    // errorX -= rotationCompensation;
    // errorY -= rotationCompensation;

    integralX += errorX;
    integralY += errorY;

    // คำนวณ PID Control
    float outputX = Kp * errorX + Ki * integralX + Kd * (errorX - prevErrorX);
    float outputY = Kp * errorY + Ki * integralY + Kd * (errorY - prevErrorY);

    // คำนวณเวลา ramp-up
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    float rampFactor = 1.0;

    if (elapsedTime < rampUpDuration)
    {
        rampFactor = (float)elapsedTime / rampUpDuration; // คำนวณ factor การเพิ่มความเร็ว
    }

    outputX = (outputX * Gain) * rampFactor;
    outputY = (outputY * Gain) * rampFactor;

    // outputX = (outputX * Gain);
    // outputY = (outputY * Gain);

    float speed1 = outputX - outputY;
    float speed2 = outputX + outputY;
    float speed3 = -outputX - outputY;
    float speed4 = -outputX + outputY;
    /*
        Serial.print(speed1);
        Serial.print(" , ");
        Serial.print(speed2);
        Serial.print(" , ");
        Serial.print(speed3);
        Serial.print(" , ");
        Serial.print(speed4);
        Serial.print(" , ");

        Serial.print(errorX);
        Serial.print(" , ");
        Serial.print(errorY);
        Serial.print(" , ");
        Serial.print(abs(errorX) <= 0.01);
        Serial.print(" , ");
        Serial.print(abs(errorY) <= 0.01);
        Serial.print(" , ");
        Serial.print(PositionX.read());
        Serial.print(" , ");
        Serial.print(PositionY.read());
        Serial.print("\n");
*/
    M1.ControlMotorSpeed(speed1);
    M2.ControlMotorSpeed(speed2);
    M3.ControlMotorSpeed(speed3);
    M4.ControlMotorSpeed(speed4);

    prevErrorX = errorX;
    prevErrorY = errorY;

    if (abs(errorX) <= 1 && abs(errorY) <= 1)
    {
        if ((millis() - EndPosition) > 500)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        EndPosition = millis();
        return false;
    }
}
void RobotControlStop()
{
    M1.ControlMotorSpeed(0);
    M2.ControlMotorSpeed(0);
    M3.ControlMotorSpeed(0);
    M4.ControlMotorSpeed(0);
}
void updateEncoder1()
{
    PositionX.update();
}

void updateEncoder2()
{
    PositionY.update();
}