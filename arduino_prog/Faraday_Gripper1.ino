/* Fontys@Work RoboCup @Work 2018
   controlling the gripper and it's light.
   Recieves commands for the distance of the fingers and the light

   Values between 0 and 120 open/close the gripper (0 is open)
   Value 200 turns light ON
   Value 210 turns light OFF

   Modified on May 28th 2018
   by Dave van der Meer, Karsten Vink and Bjarne Wildschut
*/

#include <Servo.h>
Servo Lservo; // define the left servo
Servo Rservo; // define the right servo
/***************************************************************************************************************************************/
// Constant variables
#define Light 5 //Light
#define ledPin 13 //buildin led
#define left_servo_pin 5
#define right_servo_pin 6

#define StepSize 1

// Variables

int SerialData = 0;
char value = 0;

int angleL = 0;
int angleR = 0;

int servo_position = 0 ;
int step_size = 0 ;

bool led_on = false ;
/***************************************************************************************************************************************/

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(Light, OUTPUT);
  // start serial port at 9600 bps:
  Serial.begin(115200);

  while (!Serial)
  {
    // wait for serial port to connect. Needed for native USB port only
    Serial.println("Faraday_Gripper");
  }

  // Move to initial position and turn off servos

  Lservo.attach(left_servo_pin);
  Rservo.attach(right_servo_pin);

  MoveServo(170,50);

  Lservo.detach() ;
  Rservo.detach() ;
}

/***************************************************************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  /**/
  while (Serial.available() > 0)
  {
    value = Serial.read();
    Serial.println(value);

    if (value <= 199){
      if (value < 0)
      {
        value = 0;
      }
      else
      {
        if (value > 120 && value <= 199)
        {
          value = 120 ;
        }
      }
      move_to_goal((int) value);
    }
    
  
    else
    {
      if (value == 200)
      {
        led_on = true ;
        digitalWrite(Light, HIGH) ;
        digitalWrite(ledPin, HIGH) ;
      }
  
      else
      {
        if (value == 210)
        {
          led_on = false ;
          digitalWrite(Light, LOW) ;
          digitalWrite(ledPin, LOW) ;
        }
      }
    }
  }
}

void move_to_goal(int value)
{
  if ( !(Lservo.attached() || Rservo.attached()) )
  {
    Lservo.attach(left_servo_pin);
    Rservo.attach(right_servo_pin);
  }

  angleL = 50 + value ;
  angleR = 170 - value;

  //servo_position = Lservo.read() ;
  while(angleL != Lservo.read())
  {
    if (Lservo.read() < angleL)
    {
      step_size = StepSize ;
    }
    else
    {
      step_size = -StepSize ;
    }
  delay(15) ;
  //Serial.println("working");

  Lservo.write(Lservo.read() + step_size);
  Rservo.write(Rservo.read() - step_size);
  MoveServo(Rservo.read(), Lservo.read());
  
  }
  delay(200) ;

  if (value <= 10)
  {
    Lservo.detach() ;
    Rservo.detach() ;
    //Serial.println("detached");
  }
  
  MoveServo(250,250);
  
}

void MoveServo(char goalR, char goalL){
  char Poss[2] = {goalL, goalR};
  Serial.write(Poss, 2);
}
  


/***************************************************************************************************************************************/
