
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <BasicLinearAlgebra.h> 
using namespace BLA;

// -------GLOBALS------ //
  const float F0 = 100.0;   //0.5;   // f0: cutoff frequency (Hz)
  const float Fs = 1000.0;  // fs: sample frequency (Hz)
  const int motor_2_pin = 27;
  const int motor_5_pin = 30;
  const float Pi=3.1415926536;

  long prevT = 0;
  const long interval = 10000;  // in uS
  int posPrev[4] = { 0, 0, 0, 0 };
  volatile long prevT_i = 0;

  float wFilt[3] = { 0, 0, 0 };
  float T_fan=.02718; // Fan thrust is 0.05436 N, Lver arm is half a meter
  float targetRoll = 0.0;
  float targetPitch = 0.0;
  float targetYaw = 0.0;
  float quatRef[4];

  // Identity Matrix
  BLA::Matrix <3,3> I_matrix3={1,0,0,0,1,0,0,0,1};

  //Quat mult
  BLA::Matrix <4,1> q1q2; 

  // Star Tracker
  float q0, q1, q2, q3;
  float q0_filt,q1_filt,q2_filt,q3_filt;
  bool justCameBackFromZero = false;
  String inputLine = "";
   
  BLA::Matrix<4,1> q_st = {1,0,0,0};

  //IMU
  BLA::Matrix<3, 1> w_vec={0,0,0};
  BLA::Matrix<4,1> q_imu = {1,0,0,0};

  //Command and Control
  float u=0.0;
  BLA:: Matrix<6> ref= {0.0,0.0,0.0,0.0,0.0,0.0};
  BLA:: Matrix <6> state= {0.0,0.0,0.0,0.0,0.0,0.0};
  BLA:: Matrix<1,2> error = {0,0};
  BLA::Matrix<3,3> P;
  BLA::Matrix<3,3> Q;
  BLA::Matrix<3,3> R_st;
  BLA::Matrix<3,3> R_imu;
  BLA::Matrix<4,1> q_measured;
  BLA::Matrix<4,1> q_old={1.0, 0.0, 0.0, 0.0};
  BLA::Matrix<4,1> q_new;

  //Booleans
  volatile bool new_imu;
  volatile bool new_st;

  //Bang Bang
  float u_star;

// ------DEFINITIONS------//

  #define BNO08X_CS 10
  #define BNO08X_INT 9
  #define BNO08X_RESET -1
  #define MAX_CHAR_BUFFER 20  // Adjust this based on maximum expected data length
  char data_string[MAX_CHAR_BUFFER];


//------Initialize Controller------ //
  BLA::Matrix<2,1> K={10,5};
  float eta_old = 0.0;
  float dead_band=.15;
  // const float Is = 0.0004148; // kg*m^2
  const float invIs = 2410.8;


  Adafruit_BNO08x bno08x(BNO08X_RESET);
  sh2_SensorValue_t sensorValue;

  struct euler_t {
    float yaw;
    float pitch;
    float roll;
  } ypr;

  float qr;
  float qi;
  float qj;
  float qk;
  float wx;
  float wy;
  float wz;


//------ FUNCTIONS ------ //
  //Command Functions
    void input_Euler_Angles() {
      if (Serial8.available() > 0) {
        int i = 0;
        char incoming_char;

        // Read data until newline character or buffer full
        while (Serial8.available() > 0 && i < MAX_CHAR_BUFFER - 1) {
          incoming_char = Serial8.read();
          if (incoming_char != '\n') {
            data_string[i] = incoming_char;
            i++;
          }
        }
        data_string[i] = '\0';  // Add null terminator for string conversion

        // Extract float values using sscanf
        sscanf(data_string, "[%f,%f,%f]", &targetRoll, &targetPitch, &targetYaw);
      }
    }

    void eulerToQuat(float roll, float pitch, float yaw, float* quatRef) {
      // Calculate sin and cos of half angles for efficiency
      float halfRoll = roll * 0.5;
      float sinHalfRoll = sin(halfRoll);
      float cosHalfRoll = cos(halfRoll);

      float halfPitch = pitch * 0.5;
      float sinHalfPitch = sin(halfPitch);
      float cosHalfPitch = cos(halfPitch);

      float halfYaw = yaw * 0.5;
      float sinHalfYaw = sin(halfYaw);
      float cosHalfYaw = cos(halfYaw);

      // Calculate quaternion elements
      quatRef[0] = cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw;  // w
      quatRef[1] = sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw;  // x
      quatRef[2] = cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw;  // y
      quatRef[3] = cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw;  // z
    }

    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
      float sqr = sq(qr);
      float sqi = sq(qi);
      float sqj = sq(qj);
      float sqk = sq(qk);

      ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
      ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

      if (degrees) {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
      }
    }
  // IMU Funcitons
    void setReports(void) {
      // Serial.println("Setting desired reports");
      if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,5000)) {
        Serial8.println("Could not enable gyroscope");
      }
    }

    void getIMU(BLA::Matrix<4,1>& q_old, BLA::Matrix<4,3>& G, float deltaT) {
      if (!bno08x.getSensorEvent(&sensorValue)) {
      return;
      }

      switch (sensorValue.sensorId) {
        case SH2_GYROSCOPE_CALIBRATED:
          wx = sensorValue.un.gyroscope.x;
          wy = sensorValue.un.gyroscope.y;
          wz = sensorValue.un.gyroscope.z;
          break;
        
        }

      w_vec = { wx, wy, wz };

      q_imu=q_old + G * w_vec * deltaT * float(0.5);


    }

  // Star Tracker Functions`
    QuatResult new_quat(char c) {
      QuatResult result;
      result.new_st= false;
      

      if (c== '\n'){
        // End of line: process the complete message
        result.new_st= true; // Notify update system that a new ST quaternion is ready
        inputLine.trim();  // Remove leading/trailing spaces
        inputLine.replace("[", "");
        inputLine.replace("]", "");

        float a,b,c1,d;
        if (sscanf(inputLine.c_str(),"%f %f %f %f", &a, &b, &c1, &d) == 4){
          q0_filt = a; q1_filt = b; q2_filt = c1; q3_filt = d; //Pull out indvidual strings

          float norm = sqrt(q0_filt*q0_filt
                    + q1_filt*q1_filt
                    + q2_filt*q2_filt
                    + q3_filt*q3_filt);
            
          q0 = q0_filt / norm;
          q1 = q1_filt / norm;
          q2 = q2_filt / norm;
          q3 = q3_filt / norm;
          
          bool isZeroFrame = (q0 == 0.0 && q1 == 0.0 && q2 == 0.0 && q3 == 0.0);
          if (isZeroFrame) {
            justCameBackFromZero = true;
            Serial.println("Quaternion: 0.0, 0.0, 0.0, 0.0");
          } else if (justCameBackFromZero) {
            Serial.println("[INFO] First valid frame after tag reacquisition ignored.");
            justCameBackFromZero = false;
          } else {
            result.q_measured={q0,q1,q2,q3};
          
          }
        } else {
          Serial.println("[WARN] Malformed frame ignored: " + inputLine);
        }

        inputLine = ""; // Reset for next line
      } else {
        inputLine += c;  // Build the line character by character
      }


        
          return result;
        }

      



  // quaternion math functions
    
   BLA::Matrix<4,1> quat_mult(const BLA::Matrix<4,1>& q1,
    const BLA::Matrix<4,1>&q2){
    // Used for quaterion multiplicaiton
    BLA::Matrix<4,1> q1q2={
    q1(0)*q2(0)-q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3),
    q1(0)*q2(1)+q1(1)*q2(0)+q1(2)*q2(3)-q1(3)*q2(2),
    q1(0)*q2(2)-q1(1)*q2(3)+q1(2)*q2(0)+q1(3)*q2(1),
    q1(0)*q2(3)+q1(1)*q2(2)-q1(2)*q2(1)+q1(3)*q2(0)
    };
    return q1q2; 
    }
 
    BLA::Matrix<4,1> quatConj(BLA::Matrix <4,1> q){
    //Computes quaternion conjgate
    BLA::Matrix <4,1> quat_conj={q(0),-q(1),-q(2),-q(3)};
    return quat_conj;
    }
  // Data Fusion Functions
    //Predicts next quaternion
    void predict(BLA::Matrix<3,3> P, BLA::Matrix<3,3> Q){
    P=P+Q;
    }
    //update old quaternion from the measured
    void update_w_quat(BLA::Matrix<3,3> P,BLA:: Matrix <4,1> q_old,BLA:: Matrix <4,1> q_measured,BLA::Matrix<3,3> R){
      BLA::Matrix<4,1> quat_conj={q_old(0),-q_old(1),-q_old(2),-q_old(3)};
      BLA::Matrix<4,1> q_err=quat_mult(q_measured,quat_conj);
      // Find the magnitude of error
      mag_q_err=sqrt(pow(q_err(0),2)+pow(q_err(1),2)+pow(q_err(2),2)+pow(q_err(3),2));
      // Normallize q_err
      q_err_norm=q_err/mag_q_err;

      // Ensure Scalar part is pos
      if (q_err_norm[0]<0){
        q_err_norm=-q_err;
      }

      // Small-angle innovation
      BLA::Matrix<3,1>z={2*q_err[1],2*q_err[2],2*q_err[3]};

      //Kalman update
      BLA::Matrix<3,3> S=P+R;
      BLA::Matrix<3,3> K=P*Invert(S);

      //Correction Vector
      BLA::Matrix<3,1> dtheta=K*z;
      //Update Covariance matrix P
      P=(I_Matrix3-K)*P

      // --- 8. Convert dtheta -> delta quaternion ---
      float dq0 = 1.0f;
      float dq1 = 0.5f * dtheta(0);
      float dq2 = 0.5f * dtheta(1);
      float dq3 = 0.5f * dtheta(2);

      // Greate the dq quaternion
      BLA::Matrix<4,1> dq = {dq0, dq1, dq2, dq3 };
      float mag_dq = sqrt(dq(0)*dq(0) + dq(1)*dq(1) + dq(2)*dq(2) + dq(3)*dq(3));
      dq = dq / mag_dq;

      //Apply Correction
      BLA::Matrix<4,1> q_new = quat_mult(BLA::Matrix<4,1> dq,BLA::Matrix<4,1> q_old);
      float mag_q = sqrt(q_new(0)*q_new(0) +
                        q_new(1)*q_new(1) +
                        q_new(2)*q_new(2) +
                        q_new(3)*q_new(3));
      q_new = q_new / mag_q;

      return q_new;
    }
    //Sequence for updating
    void quat_update_seq(BLA::Matrix<3,3> P,BLA::Matrix<3,3> Q,BLA::Matrix<4,1> q_old,BLA::Matrix<4,1> q_imu,BLA::Matrix<4,1> q_st,BLA::Matrix<3,3> R_st,BLA::Matrix<3,3> R_imu,bool new_st){
      BLA Matrix <3,3> P=predict(P,Q)
      if (new==true){
        q_new=update_w_quat(q_old,q_st,R_st);
      } 
      else if (new !=true){
        q_new=update_w_quat(q_old,q_imu,R_imu);
      }

      new_st=false; //Reset bool

  
    }




void setup() {
  // ------INITIALIZE KALMAN FILTER------ //
  BLA::Matrix<3,3> P={1,0,0,0,1,0,0,0,1};
  P=P*0.01f;
  BLA::Matrix<3,3> R_imu={1,0,0,0,1,0,0,0,1};
  R_imu=R_imu*sqrtf(3*Pi/180);
  BLA::Matrix<3,3> R_st={1,0,0,0,1,0,0,0,1};
  R_st=R_st*sqrtf(.2*Pi/180);
  BLA::Matrix<4,1> q_measured={0,0,0,0};

//------ INITIALIZE FAN PINS------ //
  digitalWrite(motor_2_pin,HIGH);// HIGH is off
  digitalWrite(motor_5_pin,HIGH); //HIGH is off
  pinMode(motor_2_pin, OUTPUT);
  pinMode(motor_5_pin, OUTPUT);

  //Set up Star Tracker
  Serial.begin(115200);
  delay(1000);
  Serial.println("Bridge started.");

  Serial1.begin(115200);
  delay(2000);
  Serial1.println("RUN_SCRIPT");
  

  // Set up IMU
  Serial8.begin(9600);
  // while (!Serial) delay(10);  // wait for serial port to open!
  Wire.setClock(400000);
  /* Initialise the sensor */
  if (!bno08x.begin_I2C()) {
    Serial8.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial8.println("BNO08x Found!");
  setReports();

  Serial8.println("Reading events");
  delay(100);


  


}

void loop() {
  long currT = micros();
  if (Serial1.available()){ //Check if it is availible if not, can rely on IMU
    while(Serial1.available()){ 
      char c=Serial1.read(); //Collect quaternions
      QuatRsult st = new_quat(c);

      //Update Q 4x3 
      G = { -q_old(1), -q_old(2), -q_old(3), q_old(0), -q_old(3), q_old(2), q_old(3), q_old(0), -q_old(1), -q_old(2), q_old(1), q_old(0)};
      
      if (currt-prevT>= interval){
        float deltaT = ((float)(currT - prevT)) / 1.0e6;
        prevT = currT;
        getIMU(q_old,G,deltaT);
        new_imu= true;
      }
      
      // Do Data Fusion
      // Prediction Step
      P=predict(P,Q);
      if (st.new_st){
        q_st=st.q_measured;
        q_new=update_w_quat(P,q_old,q_st,R_st);

      } else if (new_st!= true && new_imu==true){
          q_new=update_w_quat(P,q_old,q_imu,R_imu);
      }
      // Reset Boolean values 
      new_st= false;
      new_imu= false;
      q_old=q_new;

      //Command Actions

        input_Euler_Angles();
        //Convert Degrees to Radians 
        float targetRollRad = targetRoll / 57.2958;
        float targetPitchRad = targetPitch / 57.2958;
        float targetYawRad = targetYaw / 57.2958;
        eulerToQuat(targetRollRad, targetPitchRad, targetYawRad, quatRef);
        ref = { 0.0, 0.0, 0.0, quatRef[1], quatRef[2], quatRef[3] };
        state = { wx, wy, wz, qi, qj, qk };
        error = {-wz,quatRef[3]-qk};
        u_star =error*-K;
        u=u_star(0)

        if(u>T_fan && u>dead_band){
  
          //Pos on
          digitalWrite(motor_2_pin,LOW);
          digitalWrite(motor_5_pin,HIGH);
              Serial8.print("POS on");
              Serial8.print(" ");
              Serial8.print(u);
              Serial8.print(" ");

          }else if(u<dead_band && u>-dead_band){
            digitalWrite(motor_2_pin,HIGH);
            digitalWrite(motor_5_pin,HIGH);
            Serial8.print("None on");
            Serial8.print(" ");
            Serial8.print(u);
            Serial8.print(" ");
          }else if(u<-T_fan && u<-dead_band){
            digitalWrite(motor_2_pin,HIGH);
            digitalWrite(motor_5_pin,LOW);
            Serial8.print("Neg on");
            Serial8.print(" ");
            Serial8.print(u);
            Serial8.print(" ");
          }

      //--- NO STAR TRACKER -------//
    } else{ // RELY on IMU only
      if (currT - prevT >= interval) {
        float deltaT = ((float)(currT - prevT)) / 1.0e6;
        prevT = currT;
        getIMU(q_old,G,deltaT);

        qr = q_new(0);
        qi = q_new(1);
        qj = q_new(2);
        qk = q_new(3);
      }

     // Update q_old to this time step
      q_old(0) = qr;
      q_old(1) = qi;
      q_old(2) = qj;
      q_old(3) = qk;

      /* convert quaternion to Euler angles */
      quaternionToEuler(qr, qi, qj, qk, &ypr, true);

      // get new target orientation
      input_Euler_Angles();

      /*  Convert Degrees to Radians */
      float targetRollRad = targetRoll / 57.2958;
      float targetPitchRad = targetPitch / 57.2958;
      float targetYawRad = targetYaw / 57.2958;

      /* Convert Euler Angles to Quaternions */
      eulerToQuat(targetRollRad, targetPitchRad, targetYawRad, quatRef);

      BLA::Matrix<6> ref = { 0.0, 0.0, 0.0, quatRef[1], quatRef[2], quatRef[3] };
      BLA::Matrix<6> state = { wx, wy, wz, qi, qj, qk };
      BLA::Matrix<1,2> error = {-wz,quatRef[3]-qk};
      BLA::Matrix<1> u_star =error*-K;
      float u=u_star(0);
    

      // write fan controller code here:
      // only use the two yaw fans
      // Deadband bang-bang control for Yaw
      if(u>T_fan && u>dead_band){
        
      //Pos on
      digitalWrite(motor_2_pin,LOW);
      digitalWrite(motor_5_pin,HIGH);
          Serial8.print("POS on");
          Serial8.print(" ");
          Serial8.print(u);
          Serial8.print(" ");

      }

      else if(u<dead_band && u>-dead_band){
      digitalWrite(motor_2_pin,HIGH);
      digitalWrite(motor_5_pin,HIGH);
        Serial8.print("None on");
        Serial8.print(" ");
        Serial8.print(u);
        Serial8.print(" ");
      }
      else if(u<-T_fan && u<-dead_band){

      digitalWrite(motor_2_pin,HIGH);
      digitalWrite(motor_5_pin,LOW);
        Serial8.print("Neg on");
        Serial8.print(" ");
        Serial8.print(u);
        Serial8.print(" ");
      }


    }
        


  


}
