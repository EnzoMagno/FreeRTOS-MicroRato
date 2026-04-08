#include <Arduino.h>
#include "IRLine.h"
#include "state_machines.h"
#include "robot.h"
#include "config.h"
#include "path_handler.h"

//#define DEBUG



void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);

volatile long enc_left = 0;
volatile long enc_right = 0;

void enc_left_ISR() {
    int A = digitalRead(ENC1_A);
    int B = digitalRead(ENC1_B);
    
    if (A == B) {
        enc_left++;
    } else {
        enc_left--;
    }
}

void enc_right_ISR() {
    int A = digitalRead(ENC2_A);
    int B = digitalRead(ENC2_B);
    
    if (A == B) {
        enc_right++;
    } else {
        enc_right--;
    }
}


void setup()
{

  Serial.begin();
  
  pinMode (START_BUTTON,INPUT_PULLUP);
  pinMode (RESET_BUTTON,INPUT_PULLUP);

  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc_right_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc_right_ISR, CHANGE);


 

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
 
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);


   // ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  
  //init_PIO_dual_encoders(ENC1_A, ENC2_A);

  
  analogReadResolution(10);

  //Initialize the robot stopped
 
  robot.stop();
 
}


void loop() {
		
    
  // Read and print sensors
    
    robot.IRLine.readIRSensors();
    //robot.IRLine.printIRLine();
    robot.IRLine.detectNode();
    robot.setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    robot.setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);
  
    //Encoder reading
	  // edge_detection();
    // Serial.print("L = ");
    // Serial.print(enc_left);
    // Serial.print("   R = ");
    // Serial.println(enc_right);
    //delay(150);


  /**
   * State Machines Handlers
   */
    Main_FSM_Handler();
    Map_FSM_Handler();
    Solve_FSM_Handler();
    //Test_FSM_Handler();
    
    //FodaseFMSHandler();
  //** End of State Machines Handlers
  

  #ifdef DEBUG
    // Serial.printf("PWM1: %d\n",robot.PWM_1);
    // Serial.printf("PWM2%d\n",robot.PWM_2);
  #endif
} 