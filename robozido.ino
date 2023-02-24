#include <ESP8266WiFi.h>    // Including libraries for wifi
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Servo.h>          // Including library for Servo
#include <NewPing.h>

/* Defining motor control pins */
#define left_motor_forward    D1   // D1->IN1
#define left_motor_backward   D2   // D2->IN2
#define right_motor_backward  D3   // D3->IN3
#define right_motor_forward   D4   // D4->IN4

#define left_ir_sensor    A0    // Infrared sensor pins
#define right_ir_sensor   D0

#define trigger_pin  D7     // Ultrasonic sensor pins
#define echo_pin     D8

#define MAX_DISTANCE 250

String command;   //  String to store app command state

Servo ultrasonic_servo_motor;
Servo arm_servo_motor;

int arm_angle = 0;        // This will store the arm angle
float distance = 100.0;   // This will store the distance measured by the ultrasonic sensor

const char* ssid = "Robocell Car";    // Wifi Name, set as your team name
const char* password = "12345678";    // Set password
ESP8266WebServer server(80);          // Starting the Web-Server at port:80

void setup()
{
  pinMode(left_motor_backward, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);

  pinMode(left_ir_sensor, INPUT);
  pinMode(right_ir_sensor, INPUT);

  ultrasonic_servo_motor.attach(D6);
  ultrasonic_servo_motor.write(90);   // U.S. sensor faces front

  NewPing sonar(trigger_pin, echo_pin, MAX_DISTANCE);

  arm_servo_motor.attach(D5);
  delay(1000);
  arm_servo_motor.write(arm_angle);

  Serial.begin(9600);   // Setting the up the Serial Monitor

  WiFi.mode(WIFI_AP);   // Setting up the Wifi
  WiFi.softAP(ssid, password);

  /* Starting the Web-Server */
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on ( "/", HTTP_handleRoot );
  server.onNotFound ( HTTP_handleRoot );
  server.begin();

  Stop();  
}

/*************** LINE FOLLOWER MODE ****************/

void line_follower_mode()
{
  while(command!="ST" and command!="m" and command!="o"){
    // Serial.println("Line follower mode ");
    if (analogRead(left_ir_sensor) < 512  and digitalRead(right_ir_sensor) == LOW)
      Serial.println("Forward"), Forward(80);    // If both sensors are on white, go forward

    if (analogRead(left_ir_sensor) >= 512  and digitalRead(right_ir_sensor) == LOW)
      Serial.println("Left"), Left(150);    // Left sensor on black and right sensor on white, go left then

    if (analogRead(left_ir_sensor) < 512  and digitalRead(right_ir_sensor) == HIGH)
      Serial.println("Right"), Right(150);  // Left sensor on white and right sensor on black, go right then

    if (analogRead(left_ir_sensor) >= 512  and digitalRead(right_ir_sensor) == HIGH)
      Serial.println("Stop"), Stop();   // Both sensor on black, stop then.
    
    server.handleClient();    // Get command from mobile app
    command = server.arg("State");
  }
}

/*************** OBSTACLE AVOIDANCE MODE ***************/

void obstacle_avoidance_mode()
{
  while(command!="ST" and command!="m" and command!="l")
  {
    distance = measure_distance();

    while(distance<=30 and command!="ST" and command!="m" and command!="l")
    { /*  If forward distance is less than 30 cms, then
          wait for 50 ms and check distance at right side and update the distance.
          If the distance is greater than 30 cms, then turn to right side
          else if distance is greater than 5 cms, turn to left side,
          else go backwards. */                         
      delay(50);
      ultrasonic_servo_motor.write(0);    // Ultrasonic sensor faces right side
      delay(150);
      distance = measure_distance();
   
      ultrasonic_servo_motor.write(90);   // Ultrasonic sensor is back to position i.e., faces front
      delay(150);
     
      if(distance>30)   // If right distance is greater than 30 cm than turn right
      {
        Right(180);
        delay(275);
      }
      else if( distance>5 and distance<30)    // If right distance is not greater than 30 cm it turns left
      {
        Left(180);
        delay(275);
      }
      else{   // If distance is less than 5 cms, it goes backwards
        Backward(180);
        delay(200);
      }
    }

    distance = measure_distance();

    while(distance>30 and command!="ST" and command!="m" and command!="l")  
    { /* Untill the forward distance is greater than 30 cms it goes forward */
      ultrasonic_servo_motor.write(90);   // U.S. sensor is at original position ,i.e faces front
      delay(150);
      Forward(125);
      distance = measure_distance();    // update the distance
    }
  }
}

float measure_distance()    // This function measures the distance of obstacle from the Ultrasonic sensor
{
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  float duration = pulseIn(echo_pin, HIGH);   // Calculates after how much time we are getting the response
  float distance = (duration * 0.034) / 2;    // Speed of Sound=340m/s i.e. 0.034cm/microseconds

  Serial.print("Distance=");
  Serial.print(distance);
  Serial.print(" ");

  server.handleClient();    // Updating command receiving from the mobile app 
  command = server.arg("State");

  return distance;
}

/***************** ARM CODE *****************/

void arm_up()
{  /* If the arm angle goes below zero, make it zero. */
  if(arm_angle<=0)
    arm_angle = 0;
  arm_angle -= 5;
  Serial.println("Arm Up");
  arm_servo_motor.write(arm_angle);
}

void arm_down()
{ /* If the arm angle goes above 180, make it 180. */
   if(arm_angle>=180)
    arm_angle=180;
  arm_angle += 5;
  Serial.println("Arm Down");
  arm_servo_motor.write(arm_angle);
}

bool wifi = 0, line = 0, obs = 0;

void loop() 
{
  server.handleClient();    // Get the command from the app
  command = server.arg("State");

  if (command == "m")
  { /* If command is for wifi mode, make it true and rest false */
    Stop();
    wifi  = 1;
    line = 0;
    obs = 0;
  }

  else if (command == "l")
  { /* If command is for line follower mode, make it true and rest false */
    Stop();
    wifi = 0;
    line = 1;
    obs = 0;
  }

  else if (command == "o")
  { /* If command is for obstacle avoidance mode, make it true and rest false */
    Stop();
    wifi = 0;
    line = 0;
    obs = 1;
  }

  else if (command == "ST")
  { /* If the command is to stop, then stop the bot */
    Stop();
  }

/***************** WIFI BOT ******************/
  if (wifi)
  { /**/
    if (command == "F") 
      Forward(512);
    else if (command == "B") 
      Backward(512);
    else if (command == "L") 
      Left(512);
    else if (command == "R") 
      Right(512);
    else if (command == "V")  
      Stop();
    else if (command == "P") 
      arm_up();
    else if (command == "S") 
      arm_down();
  }
  if(line)
    line_follower_mode();
  if(obs)
    obstacle_avoidance_mode();

}

void HTTP_handleRoot(void)
{
  if ( server.hasArg("State") )
    Serial.println(server.arg("State"));
  
  server.send ( 200, "text/html", "" );

  delay(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************** MOTOR FUNCTIONS ****************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************** FORWARD *****************/

void Forward(int speed)
{
  analogWrite(left_motor_forward,speed);
  analogWrite(right_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  Serial.println("Forward//////");
}

/***************** BACKWARD ******************/
void Backward(int speed)
{
  analogWrite(left_motor_backward,speed);
  analogWrite(right_motor_backward,speed);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
  Serial.println("Backward//////");
}

/***************** TURN LEFT *****************/
void Left(int speed)
{
  analogWrite(right_motor_forward,speed);
  analogWrite(left_motor_backward,speed);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  Serial.println("Left//////");
}

/***************** TURN RIGHT ******************/
void Right(int speed)
{
  analogWrite(left_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  analogWrite(right_motor_backward,speed);
  Serial.println("Right//////");
}
/****************** STOP *******************/
void Stop()
{
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  digitalWrite(right_motor_backward,LOW);
}