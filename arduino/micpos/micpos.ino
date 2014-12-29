#include <Bridge.h>
#include <Console.h>
#include <FileIO.h>
#include <HttpClient.h>
#include <Mailbox.h>
#include <Process.h>
#include <YunClient.h>
#include <YunServer.h>

#include <NewPing.h>
#include <string.h>

// Motor 1
#define MOTOR_PIN1_X  2
#define MOTOR_PIN2_X  3

// Motor 2
#define MOTOR_PIN1_Y   0
#define MOTOR_PIN2_Y   1

// Motor 3
#define MOTOR_PIN1_Z   4
#define MOTOR_PIN2_Z   5
#define MOTOR_PINPWM   6

#define ECHO_X      A0  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_X   A1  // Adrduino pin tied to trigger pin on the ultrasonic sensor.

#define ECHO_Y      A2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_Y   A3  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define ECHO_Z      A4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_Z   A5  // Arduino pin tied to trigger pin on the ultrasonic sensor.

// SET THIS TO 1000 WHEN TESTING DIRECTION!!!!!!!
#define MAX_MOVE_TIME  35000 // Max time in ms that a motor can move
#define MAX_SPEED      255
#define SLOW_SPEED     75
#define SLOW_DISTANCE  50

class Motor {

    int dirPin1 = 0;
    int dirPin2 = 0;
    int speed = 0;
    int pwmPin = 0;

  public:

    Motor(int directionPin1, int directionPin2, int pwmPin = 20) {
      pinMode(directionPin1, OUTPUT);
      pinMode(directionPin2, OUTPUT);
      pinMode(pwmPin, OUTPUT);

      dirPin1 = directionPin1;
      dirPin2 = directionPin2;
      this->pwmPin = pwmPin;
    }

    void move(boolean forward, int speed) {
      this->speed = speed;
      analogWrite(pwmPin, speed);
      digitalWrite(dirPin1, (forward) ? LOW : HIGH);
      digitalWrite(dirPin2, (forward) ? HIGH : LOW);
    }

    void moveForward(int speed = MAX_SPEED) {
      move (true, speed);
    }

    void moveBackward(int speed = MAX_SPEED) {
      move(false, speed);

    }

    void setSpeed( int speed) {
      this->speed = speed;
      analogWrite(pwmPin, speed);
    }

    int   getSpeed() {
      return speed;
    }

    void stop() {
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, LOW);
    }

};



class Slider {

    int setPosition = 0;
    int minDistance = 10;
    int maxDistance = 400;
    int pingDivisor = 2;
    long startTime = 0;
    boolean movingForward = true;
    boolean moving = false;

    Motor* motor;
    NewPing* sonar;

  public:
    Slider(Motor* motor, NewPing* sonar, int minDistance = 2, int maxDistance = 600, int divisor = 2 ) {
      this->motor = motor;
      this->sonar = sonar;
      setPosition = getCurrentPosition();
      this->minDistance = minDistance;
      this->maxDistance = maxDistance;
      this->pingDivisor = divisor;
    }


    // Sets position to the max value.
    void moveForward() {
      moving = true;
      movingForward = true;
      startTime = millis();
      setPosition = maxDistance;

      motor->moveForward();
    }

    // Sets position to the min value.
    void moveBackward() {
      moving = true;
      movingForward = false;
      startTime = millis();
      setPosition = minDistance;
      motor->moveBackward();

    }

    void moveTo(int newPosition) {

      // Protect us from going to far... ever
      if (newPosition > maxDistance) {
        newPosition = maxDistance;
      }
      else if (newPosition < minDistance) {
        newPosition = minDistance;
      }

      setPosition = newPosition;
      int currentPosition = getCurrentPosition();
      if (setPosition > currentPosition) {
        moving = true;
        movingForward = true;
        motor->moveForward();

      } else if (setPosition < currentPosition) {
        moving = true;
        movingForward = false;
        motor->moveBackward();
      }
      startTime = millis();
    }

    int getCurrentPosition() {
      int timeThereAndBack = sonar->ping_median(10);
      delay(29);  // Not sure this is needed
      return timeThereAndBack / pingDivisor;
    }

    void stop() {
      motor->stop();
      moving = false;
    }

    int getSpeed(int currentPosition, int setPosition) {

      if (motor->getSpeed() == MAX_SPEED &&
          abs(currentPosition - setPosition) < SLOW_DISTANCE) {
        return SLOW_SPEED;
      }
      return MAX_SPEED;
    }

    void process() {

      if (moving) {
        // Stop if we have gone for TOO long of time.
        if ((millis() - startTime) > MAX_MOVE_TIME) {
          motor->stop();
          moving = false;
          return;
        }

        int currentPosition = getCurrentPosition();
        // Commented out as we do not need it now.
        // Serial.print("SetPosition: ");
        // Serial.print(setPosition);
        // Serial.print(" Current Position: ");
        // Serial.println(currentPosition);


        // If we are moving at max speed and we are close, then slow down
        motor->setSpeed(getSpeed(currentPosition, setPosition));


        if (movingForward && (currentPosition >= setPosition  ||
                              currentPosition >= maxDistance)) {



          motor->stop();
          moving = false;
          int currentPosition = getCurrentPosition();
          Serial.print("SetPosition: ");
          Serial.print(setPosition);
          Serial.print(" Current Position: ");
          Serial.println(currentPosition);

        } else if (movingForward == false && (currentPosition <= setPosition  ||
                                              currentPosition <= minDistance)) {
          motor->stop();
          moving = false;
          int currentPosition = getCurrentPosition();
          Serial.print("SetPosition: ");
          Serial.print(setPosition);
          Serial.print(" Current Position: ");
          Serial.println(currentPosition);

        }
      }
    }
};


NewPing* sonarX ;
NewPing* sonarY ;
NewPing* sonarZ ;

Motor* motorX;
Motor* motorY;
Motor* motorZ;

Slider* sliderX;
Slider* sliderY;
Slider* sliderZ;


// These are for calculating how many loops we process in a second
long rateStartTime = 0L;
long loopCount = 0L;

// Listen on default port 5555, the webserver on the Yún
// will forward there all the HTTP requests for us.
YunServer server;

void setup() {
  //
  motorX = new Motor(MOTOR_PIN1_X, MOTOR_PIN2_X);
  sonarX = new NewPing(TRIGGER_X, ECHO_X, 400);
  sliderX = new Slider(motorX, sonarX);

  motorY = new Motor(MOTOR_PIN1_Y, MOTOR_PIN2_Y);
  sonarY = new NewPing(TRIGGER_Y, ECHO_Y, 400);
  sliderY = new Slider(motorY, sonarY);

  motorZ = new Motor(MOTOR_PIN1_Z, MOTOR_PIN2_Z, MOTOR_PINPWM);
  sonarZ = new NewPing(TRIGGER_Z, ECHO_Z, 400);
  sliderZ = new Slider(motorZ, sonarZ);

  // Bridge startup
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Bridge.begin();
  digitalWrite(13, HIGH);

  // Listen for incoming connection only from localhost
  // (no one from the external network could connect)
  server.listenOnLocalhost();
  server.begin();

  // initialize serial communication:
  Serial.begin(9600);

  rateStartTime = millis();
}

void loop() {

  // Get clients coming from server
  YunClient client = server.accept();

  // There is a new client?
  if (client) {

    // Process request
    processRequest(client);

    // Close connection and free resources.
    client.stop();
  }

  sliderX->process();
  sliderY->process();
  sliderZ->process();

  loopCount++;

  // Poll every 50ms, might be able to take this out.
  //We should have enough of a delay with all the processing
  delay(50);
}


void processStep(YunClient client, String subCommand) {
  if (subCommand == "up") {
    sliderY->moveTo(sliderY->getCurrentPosition() +  client.parseInt());
  } else if (subCommand == "down") {
    sliderY->moveTo(sliderY->getCurrentPosition() - client.parseInt());
  } else if (subCommand == "right") {
    sliderX->moveTo(sliderX->getCurrentPosition() +  client.parseInt());
  } else if (subCommand == "left") {
    sliderX->moveTo(sliderX->getCurrentPosition() - client.parseInt());
  } else if (subCommand == "in") {
    sliderZ->moveTo(sliderZ->getCurrentPosition() +  client.parseInt());
  } else if (subCommand == "out") {
    sliderZ->moveTo(sliderZ->getCurrentPosition() - client.parseInt());
  }
}

void processMove(YunClient client, String subCommand) {

  if (subCommand == "up") {
    sliderY->moveForward();
  } else if (subCommand == "down") {
    sliderY->moveBackward();
  } else if (subCommand == "right") {
    sliderX->moveForward();
  } else if (subCommand == "left") {
    sliderX->moveBackward();
  } else if (subCommand == "in") {
    sliderZ->moveForward();
  } else if (subCommand == "out") {
    sliderZ->moveBackward();
  }
}

void outputClientStatus(YunClient client) {
  char buf[64];
  char digit[10];
  float loopRate = loopCount / ((rateStartTime - millis()) / 1000);
  dtostrf(loopRate,4,2, digit);
  rateStartTime = millis();
  loopCount = 0;

  client.println("HTTP/1.1 200 OK");
  client.println("Status: 200");
  client.println("Content-type: application/json; charset=utf-8");
  client.println(); //mandatory blank line
  sprintf(buf, "{\"rate\":\"%s\",\"x\":\"%d\",\"y\":\"%d\",\"z\":\"%d\" }",
          digit,
          sliderX->getCurrentPosition(), sliderY->getCurrentPosition(), sliderZ->getCurrentPosition());

  client.print(buf);
}

//dtostrf
/**
/status - returns status of the sensors
/move/{left|right|up|down|in|out}
/step/{left|right|up|down|in|out}/#
/stop - stops all motors
/setpos/x/y/z = set position of xyz
**/

void processRequest(YunClient client) {
  // read the command
  String command = client.readStringUntil('/');
  command.trim();
  Serial.println("Processing: " + command);

  if (command == "status") {
    outputClientStatus(client);
  } else if (command == "move") {

    String subCommand = client.readStringUntil('/');
    subCommand.trim();
    processMove(client, subCommand);

  } else if (command == "step") {

    String subCommand = client.readStringUntil('/');
    subCommand.trim();
    processStep(client, subCommand);

  } else if (command == "stop") {
    sliderX->stop();
  } else if (command == "setpos") {
    int x = 0;
    int y = 0;
    int z = 0;

    x = client.parseInt();
    if (client.read() == '/') {
      y = client.parseInt();
    }
    if (client.read() == '/') {
      z = client.parseInt();
    }

    sliderX->moveTo(x);
    sliderY->moveTo(y);
    sliderZ->moveTo(z);
  }


}



