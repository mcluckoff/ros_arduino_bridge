/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

// Driving motors encoder drivers: 
#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(ARDUINO_HC89_COUNTER)
  volatile long enc_count[2] = {0L, 0L}; // DRIVE = 0, STEER = 1
  volatile int enc_direction[2] = {1, 1}; // +1 = forward/right, -1 = reverse/left

  void initEncoders() {
    pinMode(DRIVE_ENC_PIN, INPUT_PULLUP);
    pinMode(STEER_ENC_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_PIN), driveISR, FALLING); // or RISING, CHANGE
    attachInterrupt(digitalPinToInterrupt(STEER_ENC_PIN), steerISR, FALLING);
  }

  void updateEncoderDirection(int enc, int dir) {
    if (enc == DRIVE || enc == STEER) {
      if (dir == 1 || dir == -1) {
        enc_direction[enc] = dir;
      }
    }
  }

  void driveISR() {
    enc_count[DRIVE] += enc_direction[DRIVE];
  }

  void steerISR() {
    enc_count[STEER] += enc_direction[STEER];
  }

  long readEncoder(int i) {
    if (i == DRIVE) return enc_count[DRIVE];
    else if (i == STEER) return enc_count[STEER];
    else return 0L;
  }

  void resetEncoder(int i) {
    if (i == DRIVE) {
      enc_count[DRIVE] = 0L;
    } else if (i == STEER) {
      enc_count[STEER] = 0L;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

void resetEncoders() {
  resetEncoder(DRIVE);
  resetEncoder(STEER);
}

#endif