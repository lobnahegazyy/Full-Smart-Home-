#include <Servo.h>
#include <LiquidCrystal_I2C.h>

/////////////////////////////////////////////////////////////////////
//PIR - Garage
int pirPin = 12; // Arduino pin the PIR sensor is connected to
int servoPin = 9; //Arduino pin the servo is connected to
int motionStatus = 0; // variable to store the PIR sensor's motion status (high or low)
int pirState = 0; // variable to track the state change
Servo scaryServo; // create servo object to control our servo

/////////////////////////////////////////////////////////////////////
//Gas Sensor

int Input = A0;
int Buzzer = 4;
int val;
int MAX = 250;

////////////////////////////////////////////////////////////////////////
//Ultrasonic - room

const int trigPin = 10;
const int echoPin = 11;
const int led1 = 13;
long duration;
int distance;

//////////////////////////////////////////////////////////////////////////
//Water level - lcd

LiquidCrystal_I2C lcd(0x27, 20, 4);

//////////////////////////////////////////////////////////////////////////////
//ultrasonic - main gate
Servo myservo;   
int cm = 0;

long readUltrasonicDistance(int triggerPin, int echoPin) {
    pinMode(triggerPin, OUTPUT); 
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    return pulseIn(echoPin, HIGH);
}


void setup() {
  //////////////////////////////////////////////////////////////////
  /// PIR
  Serial.begin(9600); // initialize the serial monitor
  pinMode(pirPin, INPUT); // set the Arduino pin that PIR sensor is connected to as an INPUT
  scaryServo.attach(servoPin); // attaches the servo on pin 9 to the servo object
  scaryServo.write(0); // start the servo at 0 degrees
  delay(100); // give time for the PIR sensor to calibrate (30-60secs is best)

////////////////////////////////////////////////////////////////////////////////
/// GAS detector 


  pinMode(Input ,INPUT);
  pinMode(Buzzer ,OUTPUT);

//////////////////////////////////////////////////////////////////////////////////
//ultrasonic - room
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(led1, OUTPUT);

///////////////////////////////////////////////////////////////////////////////////////
//water level - lcd
  lcd.begin();
  lcd.backlight();

//////////////////////////////////////////////////////////////////////////////
//ultrasonic - main gate
  myservo.attach(8); 


}

void loop() {
  motionStatus = digitalRead(pirPin); // read the PIR pin's current output (is it HIGH or LOW?)
   
    // if the motion status is HIGH
    if (motionStatus == HIGH) { 
      
      scaryServo.write(90); // rotate the servo to 90 degrees
      
      if (pirState == LOW ) {
        Serial.println("Motion Detected"); // print result to the serial monitor
        pirState = HIGH; // update the previous state to HIGH
      }
    }
    
    //or else if motion status is low
    else { 
      
      scaryServo.write(0); // rotate the servo back to 0 degrees
      
      if (pirState == HIGH) {
        Serial.println ("Motion Ended"); // print result to the serial monitor
        pirState = LOW; // update the previous state to LOW
      }
    }


///////////////////////////////////////////////////////////////////////////////////////
/// Gas detector
 val = analogRead(A0);
  Serial.print("Analog Reading: ");
  Serial.println(val);
  if (val >= MAX) {
    digitalWrite(Buzzer ,HIGH);
    Serial.println("GAS LEAKING");
  }
  else {
    digitalWrite(Buzzer ,LOW);
    Serial.println("NORMAL");
  }
  delay(1000); // Add a delay to make the output easier to read


///////////////////////////////////////////////////////////////
//Ultrasonic - room

// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);

// Calculating the distance
distance= duration*0.034/2;

if (distance <= 6){
  digitalWrite(led1, HIGH);

}
else{
  digitalWrite(led1, LOW);

}

// Prints the distance on the Serial Monitor
Serial.print("Distance: ");
Serial.println(distance);

///////////////////////////////////////////////////////////////////////////////////////
//water level - lcd
  int value = analogRead(A1);
  lcd.setCursor(0, 0);
  lcd.print("Value :");
  lcd.print(value);
  lcd.print("   ");
  Serial.println(value);
  lcd.setCursor(0, 1);
  lcd.print("W Level :");


  if (value == 0) {
    lcd.print("Empty ");
  } else if (value > 1 && value < 350) {
    lcd.print("LOW   ");
  } else if (value > 350 && value < 510) {
    lcd.print("Medium");
  } else if (value > 510){
    lcd.print("HIGH");
  }

//////////////////////////////////////////////////////////////////////////////
//ultrasonic - main gate


    cm = 0.01723 * readUltrasonicDistance(7, 6); // Corrected pin assignment

    if (cm < 2) {
        Serial.print(cm);
        Serial.println(" cm");
        
        // Move servo to 120 degrees
        for (int pos = 0; pos <= 120; pos += 1) { 
            myservo.write(pos);             
            delay(10); // Delay for smooth movement
        }
        delay(500);

        // Move servo back to 0 degrees
        for (int pos = 120; pos >= 0; pos -= 1) { 
            myservo.write(pos);
            delay(10); // Delay for smooth movement
        }
        delay(500);
    } else {
        myservo.write(0); // Stop servo movement if distance exceeds 11 cm
      }

}


















