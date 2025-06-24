/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

   #ifdef USE_BASE
   
   #ifdef POLOLU_VNH5019
     /* Include the Pololu library */
     #include "DualVNH5019MotorShield.h"
   
     /* Create the motor driver object */
     DualVNH5019MotorShield drive;
     
     /* Wrap the motor driver initialization */
     void initMotorController() {
       drive.init();
     }
   
     /* Wrap the drive motor set speed function */
     void setMotorSpeed(int i, int spd) {
       if (i == LEFT) drive.setM1Speed(spd);
       else drive.setM2Speed(spd);
     }
   
     // A convenience function for setting both motor speeds
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
   #elif defined POLOLU_MC33926
     /* Include the Pololu library */
     #include "DualMC33926MotorShield.h"
   
     /* Create the motor driver object */
     DualMC33926MotorShield drive;
     
     /* Wrap the motor driver initialization */
     void initMotorController() {
       drive.init();
     }
   
     /* Wrap the drive motor set speed function */
     void setMotorSpeed(int i, int spd) {
       if (i == LEFT) drive.setM1Speed(spd);
       else drive.setM2Speed(spd);
     }
   
     // A convenience function for setting both motor speeds
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
   #elif defined L298_MOTOR_DRIVER
     void initMotorController() {
       digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
       digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
     }
     
     void setMotorSpeed(int i, int spd) {
       unsigned char reverse = 0;
     
       if (spd < 0)
       {
         spd = -spd;
         reverse = 1;
       }
       if (spd > 255)
         spd = 255;
       
       if (i == LEFT) { 
         if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
         else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
       }
       else /*if (i == RIGHT) //no need for condition*/ {
         if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
         else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
       }
     }
     
     void setMotorSpeeds(int leftSpeed, int rightSpeed) {
       setMotorSpeed(LEFT, leftSpeed);
       setMotorSpeed(RIGHT, rightSpeed);
     }
   #elif defined ZKBM1_MOTOR_DRIVER
   
      void initMotorController() {
        pinMode(DRIVE_PWM_IN1, OUTPUT);
        pinMode(DRIVE_PWM_IN2, OUTPUT);
        pinMode(STEER_PWM_IN3, OUTPUT);
        pinMode(STEER_PWM_IN4, OUTPUT);
      }

      void setMotorSpeed(int spd) {
        unsigned char reverse = 0;
    
        if (spd < 0)
        {
          spd = -spd;
          reverse = 1;
        }
        if (spd > 255)
          spd = 255;

        // Inform encoder hall driver of direction
        if (reverse == 0)
            updateDriveDirection(100);  // forward
        else
            updateDriveDirection(-100); // reverse

        if (reverse == 0) {
            analogWrite(DRIVE_PWM_IN1, spd);
            analogWrite(DRIVE_PWM_IN2, 0);
          } else {
            analogWrite(DRIVE_PWM_IN1, 0);
            analogWrite(DRIVE_PWM_IN2, spd);
        }
      }

      // TODO STEERING MOTOR DRIVER PART
      // void setSteeringMotorSpeed(int spd) {

      //  }
   #else
     #error A motor driver must be selected!
   #endif
   
   #endif