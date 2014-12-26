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

#define ECHO_X      11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_X   12  // Adrduino pin tied to trigger pin on the ultrasonic sensor.

#define ECHO_Y      10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_Y   11  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define ECHO_Z      8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_Z   9  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define MAX_MOVE_TIME  35000 // Max time in ms that a motor can move

class Motor {

    int dirPin1 = 0;
    int dirPin2 = 0;
    int speed = 75;
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

    void move(boolean forward, int speed = 255) {
      analogWrite(pwmPin, speed);
      digitalWrite(dirPin1, (forward) ? LOW : HIGH);
      digitalWrite(dirPin2, (forward) ? HIGH : LOW);
    }

    void moveForward(int speed = 255) {
      move (true, speed);
    }

    void moveBackward(int speed = 255) {
      move(false, speed);

    }

    void setSpeed( int speed) {
      analogWrite(pwmPin, speed);
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
      delay(29);
      return timeThereAndBack / pingDivisor;
    }

    void stop() {
      motor->stop();
      moving = false;
    }

    void process() {
      if (moving) {
        // Stop if we have gone for TOO long of time.
        if ((millis() - startTime) > MAX_MOVE_TIME) {
          motor->stop();
          moving = false;
          Serial.println("Stopping - timed out ");

          return;
        }

        int currentPosition = getCurrentPosition();
        // Serial.print("SetPosition: ");
        // Serial.print(setPosition);
        // Serial.print(" Current Position: ");
        // Serial.println(currentPosition);

        if (abs(currentPosition - setPosition) < 40) {
          motor->setSpeed(75);
        }

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
}

void loop() {

  // Get clients coming from server
  YunClient client = server.accept();

  // There is a new client?
  if (client) {
    Serial.println("http request");

    // Process request
    processRequest(client);

    // Close connection and free resources.
    client.stop();

  }

  sliderX->process();
  sliderY->process();
  sliderZ->process();
  delay(50); // Poll every 50ms


}


// THis is not working for some reason???
int readInt(YunClient client) {
  int value = client.parseInt();
  client.read();
  return value;
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

  // is "digital" command?
  if (command == "status") {
    // processStatus
    Serial.println("Return Status");
    int currentPosition = sliderZ->getCurrentPosition();
    Serial.print("Slider Position: ");
    Serial.print(currentPosition);
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

    Serial.print("x: ");
    Serial.print(x);
    Serial.print( " y: ");
    Serial.print(y);
    Serial.print( " z: ");
    Serial.println(z);
  }


}



