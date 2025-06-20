
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

float angX,angY,dangZ1,dangZ2,dangZ3,angZ;

//Kalman filter data
float f11,f12,f21,f22,fc11,fc12,fc21,fc22;
float d11,d12,d21,d22;

//states
float xh1k,xh2k,xh1k_1,xh2k_1;
float s1,s2; //observations, angular position and angular velocity from  accelerometer angles and gyro info

//H matrix is identity

//Filter gain matrix
float k11,k12,k21,k22;

//sampling freq and interval
float fs,Ts;

// for sampling freq
int sample_pin=16;

boolean running = false;

Adafruit_MPU6050 mpu;

void setup(void) {

  pinMode(sample_pin,OUTPUT);

  sample_pin=false;

  ///////////////////////////
  
  //sampling frequency
  
  fs=10000.0;
  
  //sampling interval
  Ts=1/fs;
  
  
  f11=1;
  f12=-Ts;
  
  f21=0.0;
  f22=1.0;
  
  d11=Ts;
  d12=-Ts*Ts*0.5;
  
  d21=0.0;
  d22=Ts;

  //Kalman gains have been calculated offline for Q=I,R=I*1e-5;
  
  k11=0.0311;
  k12=-5.1556e-5;
  
  k21=-4.8444e-5;
  k22=0.0311;
   
  //initialise state estimates
  
  xh1k=0.0;
  xh2k=0.0;
  
  xh1k_1=0.0;
  xh2k_1=0.0;
  
  ///////

  //calculate Kalman filter Fc closed loop F matrix. Note H=I identity matrix Fc=F-KH (K has been found offline)

  fc11=f11-k11;
  fc12=f12-k12;

  fc21=f21-k21;
  fc22=f22=k22;

  /////

  Serial.begin(115200);

  while (!Serial)

    //delay(10); // will pause Zero, Leonardo, etc until serial console opens

    //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!

  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);

}

void loop() {

  /* Get new sensor events with the readings */

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 /* Print out the values */

 /*

  Serial.print("Acceleration X: ");

  Serial.print(a.acceleration.x);

  Serial.print(", Y: ");

  Serial.print(a.acceleration.y);

  Serial.print(", Z: ");

  Serial.print(a.acceleration.z);

  Serial.println(" m/s^2");*/

  //dangZ1=sqrt(a.acceleration.z*a.acceleration.z+a.acceleration.x*a.acceleration.x);

  dangZ2=sqrt(a.acceleration.z*a.acceleration.z + a.acceleration.y*a.acceleration.y);

  //dangZ3=sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y);

  //angX=57.29*atan2(a.acceleration.y,dangZ1);

  //Serial.println(angX); //roll

  angY=57.29*atan2(a.acceleration.x,dangZ2);

 // Serial.println(angY); //pitch angle in degrees only

  //angZ=57.29*atan2(a.acceleration.z,dangZ3);

  //Serial.println(angZ); //Pitch wrt 90 degrees



  //Kalman filter here

  s1= angY; //angular position from accelerometer calculation

  s2=g.gyro.y;//from gyro angular velocity



  //shuffle regressors of the states

  xh1k_1=xh1k;

  xh2k_1=xh2k;



  xh1k=fc11*xh1k_1 +fc12*xh2k_1+k11*s1+k12*s2;

  xh2k=fc21*xh1k_1 +fc22*xh2k_1+k21*s1+k22*s2;

/*

Serial.print("\t");

  Serial.print(s1); //noisy angle estimate of pitch

  Serial.print(" "); */

  Serial.println(xh1k); //KF pitch angle estimate

  // Serial.println(xh2k); //KF pitch angular velocity estimate


  // Sets a flag at a precise time.

   //////

// To measure the sample rate from pin 21

  digitalWrite(sample_pin,running);

  running=!running;

  //delay(5);

}
