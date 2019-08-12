#include "LSM6DSLSensor.h"
#include "LIS2MDLSensor.h"
#include "src/MadgwickAHRS/MadgwickAHRS.h"

DevI2C *i2c;
LSM6DSLSensor *sensor;
LIS2MDLSensor *lis2mdl;
Madgwick *madgwick;

// Madgwick wants:
//   accl -> m/s2
//   gyro -> degrees/s

// Sensor outputs:
//   accl -> cm/s2
//   gyro -> ???

// Sensor range per datasheet:
//   accl -> 2-16 g
//   gyro -> 125 -> 2000 gps

// Max seen values:
//   accl -> 2-16 g
//   gyro -> 483700

int FPS = 10;
float TOTAL_DELAY_TIME = 1000.0 / FPS;

void setup()
{
    // init Gyroscope/Accelerator
    i2c = new DevI2C(D14, D15);
    sensor = new LSM6DSLSensor(*i2c, D4, D5);
    lis2mdl = new LIS2MDLSensor(*i2c);
    lis2mdl->init(NULL);
    sensor->init(NULL);
    sensor->enableAccelerator();
    sensor->enableGyroscope();

    // init Madgwick filter
    madgwick = new Madgwick();
    madgwick->begin(1.0 / TOTAL_DELAY_TIME);
}

unsigned long start;
unsigned long end;
unsigned long execution_time = 0;
int print_counter = 0;
void loop()
{
    start = millis();
    print_counter++;
    doStuff();
    end = millis();
    execution_time = end - start;
    int time_to_delay = TOTAL_DELAY_TIME - execution_time;
    if (time_to_delay < 0)
    {
        time_to_delay = 0;
        Serial.printf("WARNING: DID NOT DELAY\n");
    }
    delay(time_to_delay);
}

void doStuff()
{
    // Accelerometer
    int asi[3];
    sensor->getXAxes(asi);
    float as[3];
    for (int i = 0; i < 3; i++)
    {
        // Convert from cm/s2 to m/s2
        as[i] = asi[i] / 100.0;
    }

    float ax = as[0];
    float ay = as[1];
    float az = as[2];

    // Gyroscope
    int gsi[3];
    sensor->getGAxes(gsi);
    float gs[3];
    for (int i = 0; i < 3; i++)
    {
        // Convert from rad/? to deg/s
        gs[i] = (gsi[i] * 57.2958) / 100000.0;
    }

    float gx = gs[0];
    float gy = gs[1];
    float gz = gs[2];

    // Magnetometer
    int msi[3];
    lis2mdl->getMAxes(msi);
    float ms[3];
    for (int i = 0; i < 3; i++)
    {
        // Convert from mGauss to uTesla
        ms[i] = msi[i] / 10.0;
    }

    float mx = ms[0];
    float my = ms[1];
    float mz = ms[2];

    bool should_print = print_counter % FPS == 0;

    if (should_print)
    {
        Serial.printf("execution_time: %d\n", execution_time);
        Serial.printf("Accl: x: %5.2f\ty: %5.2f\tz: %5.2f\n", ax, ay, az);
        Serial.printf("Gyro: x: %5.2f\ty: %5.2f\tz: %5.2f\n", gx, gy, gz);
        Serial.printf("Magn: x: %5.2f\ty: %5.2f\tz: %5.2f\n", mx, my, mz);
    }

    madgwick->update(gx, gy, gz, ax, ay, az, mx, my, mz);

    float roll = madgwick->getRoll();
    float pitch = madgwick->getPitch();
    float yaw = madgwick->getYaw();

    if (should_print)
    {
        Serial.printf("\nroll: %.2f, pitch: %.2f, yaw: %.2f\n\n\n", roll, pitch, yaw);
    }
}
