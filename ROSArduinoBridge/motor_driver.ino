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
   
      // Steering control parameters - adjust these to tune steering behavior
      long STEERING_TOLERANCE = 4;    // Encoder counts tolerance for stopping
      long STEERING_DEAD_ZONE = 2;    // Dead zone to prevent micro-adjustments
      long STEERING_SLOW_ZONE = 20;   // Zone where speed reduction starts
      int STEERING_MIN_PWM = 30;      // Minimum PWM to ensure motor can turn
      int STEERING_MAX_PWM = 255;     // Maximum PWM for full speed
      bool STEERING_EXPONENTIAL = true; // Use exponential vs linear speed reduction

      void initMotorController() {
        pinMode(DRIVE_PWM_IN1, OUTPUT);
        pinMode(DRIVE_PWM_IN2, OUTPUT);
        pinMode(STEER_PWM_IN3, OUTPUT);
        pinMode(STEER_PWM_IN4, OUTPUT);
      }

      void setMotorSpeed(int spd) {
        bool reverse = false;
    
        if (spd < 0)
        {
          spd = -spd;
          reverse = true;
        }
        if (spd > 255)
          spd = 255;

        // Inform encoder driver of direction
        // Pass 0 for stop condition to trigger inertia-aware direction handling
        if (spd == 0) {
          updateEncoderDirection(DRIVE, 0); // Signal stop condition
        } else {
          updateEncoderDirection(DRIVE, reverse ? -1 : 1);
        }

        if (!reverse) {
          analogWrite(DRIVE_PWM_IN1, spd);
          analogWrite(DRIVE_PWM_IN2, 0);
        } else {
          analogWrite(DRIVE_PWM_IN1, 0);
          analogWrite(DRIVE_PWM_IN2, spd);
        }
      }

      void setSteeringDirection(int target_position) {
        long current_position = readEncoder(STEER);
        long error = target_position - current_position;

        // Check if we're within dead zone (no movement needed)
        if (abs(error) <= STEERING_DEAD_ZONE) {
          analogWrite(STEER_PWM_IN3, 0);
          analogWrite(STEER_PWM_IN4, 0);
          return;
        }

        // Calculate PWM based on distance from target
        int pwm_value;
        if (abs(error) <= STEERING_SLOW_ZONE) {
          // In slow zone - use exponential decay for smoother deceleration
          // This creates a more gradual speed reduction as we approach the target
          float progress = (float)abs(error) / STEERING_SLOW_ZONE;
          if (STEERING_EXPONENTIAL) {
            pwm_value = STEERING_MIN_PWM + (STEERING_MAX_PWM - STEERING_MIN_PWM) * (progress * progress);
          } else {
            // Linear speed reduction as alternative
            pwm_value = STEERING_MIN_PWM + (STEERING_MAX_PWM - STEERING_MIN_PWM) * progress;
          }
        } else {
          // Outside slow zone - full speed
          pwm_value = STEERING_MAX_PWM;
        }

        // Debug output (can be commented out in production)
        // Serial.print("Target:"); Serial.print(target_position);
        // Serial.print(" Current:"); Serial.print(current_position);
        // Serial.print(" Error:"); Serial.print(error);
        // Serial.print(" PWM:"); Serial.println(pwm_value);

        if (error > 0) {
          // Turn steering right
          updateEncoderDirection(STEER, 1);
          analogWrite(STEER_PWM_IN3, pwm_value);
          analogWrite(STEER_PWM_IN4, 0);
        } else {
          // Turn steering left
          updateEncoderDirection(STEER, -1);
          analogWrite(STEER_PWM_IN3, 0);
          analogWrite(STEER_PWM_IN4, pwm_value);
        }
      }

      void updateSteeringParameters(long tolerance, long dead_zone, long slow_zone, int min_pwm, int max_pwm) {
        // Validate and update steering parameters
        if (tolerance > 0) STEERING_TOLERANCE = tolerance;
        if (dead_zone >= 0) STEERING_DEAD_ZONE = dead_zone;
        if (slow_zone > dead_zone) STEERING_SLOW_ZONE = slow_zone;
        if (min_pwm >= 0 && min_pwm < max_pwm) STEERING_MIN_PWM = min_pwm;
        if (max_pwm > min_pwm && max_pwm <= 255) STEERING_MAX_PWM = max_pwm;
      }
   #else
     #error A motor driver must be selected!
   #endif
   
   #endif