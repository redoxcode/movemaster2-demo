// control a 6DOF robot (customized for movemaster2 with teensy4.1)
// hastily coded. Lot of room for improvements here.
// Like propper PID control and more efficient encoder readout

static int joints = 6;

const int motorPinsB[] = {0, 2, 4, 7, 8, 10};
const int motorPinsA[] = {1, 3, 5, 6, 9, 11};
const int encoderPinsA[] =    {22, 20, 17, 14, 38, -1};
const int encoderPinsB[] =    {23, 19, 16, 41, 39, -1};
const int limitSwitchPins[] = {21, 18, 15, 40, 37, -1};
const int plugCheckPin = 33;
int encoderCount[] = {0, 0, 0, 0, 0, 0};
byte encoderLastState[] = {00, 00, 00, 00, 00, 00};


int target[] = {0, 0, 0, 0, 0, 0};




void HBridge(int i, float p) {
  if (p > 0.0) {
    if (p > 1.0)p = 1.0;
    int v = abs(int(p * 255.0));
    analogWrite(motorPinsA[i], v);
    analogWrite(motorPinsB[i], 0);
  }
  else if (p < 0.0) {
    if (p < -1.0)p = -1.0;
    int v = abs(int(p * 255.0));
    analogWrite(motorPinsA[i], 0);
    analogWrite(motorPinsB[i], v);
  }
  else if (p == 0.0) {
    analogWrite(motorPinsA[i], 0);
    analogWrite(motorPinsB[i], 0);
  }
}

void resetCounters() {
  for (int i = 0; i < joints; i++) {
    target[i] = 0;
    encoderCount[i] = 0;
    byte state = 0;
    if (digitalRead(encoderPinsA[i])) state += 1;
    if (digitalRead(encoderPinsB[i])) state += 10;
    encoderLastState[i] = state;
  }
}

void calibrate_joint(int i) {
  while (!digitalRead(limitSwitchPins[i])) HBridge(i, -1.0);
  HBridge(i, 0.0);
}

void calibrate_joints() {
  calibrate_joint(1);
  calibrate_joint(2);
  calibrate_joint(0);

  while (!digitalRead(limitSwitchPins[3])) {
    HBridge(3, -0.7);
    HBridge(4, -0.7);
  }

  while (!digitalRead(limitSwitchPins[4])) {
    if (digitalRead(limitSwitchPins[3])) {
      HBridge(3, 1.0);
    } else {
      HBridge(3, 0.7);
    }

    HBridge(4, -0.85);
  }

  HBridge(3, 0.0);
  HBridge(4, 0.0);

  HBridge(5, 0.8);
  delay(3000);
  HBridge(5, 0.0);
  resetCounters();

}


void counters() {
  for (int i = 0; i < joints; i++) {
    byte state = 0;
    if (digitalRead(encoderPinsA[i])) state += 1;
    if (digitalRead(encoderPinsB[i])) state += 10;
    if (state != encoderLastState[i]) {
      if (encoderLastState[i] == 00 and state == 01) encoderCount[i] += 1;
      else if (encoderLastState[i] == 01 and state == 11) encoderCount[i] += 1;
      else if (encoderLastState[i] == 11 and state == 10) encoderCount[i] += 1;
      else if (encoderLastState[i] == 10 and state == 00) encoderCount[i] += 1;

      else if (encoderLastState[i] == 00 and state == 10) encoderCount[i] -= 1;
      else if (encoderLastState[i] == 10 and state == 11) encoderCount[i] -= 1;
      else if (encoderLastState[i] == 11 and state == 01) encoderCount[i] -= 1;
      else if (encoderLastState[i] == 01 and state == 00) encoderCount[i] -= 1;

      else {
        Serial.print("STEP ERROR JOINT ");
        Serial.println(i);
      }
      encoderLastState[i] = state;
    }

  }


}


void pid() {
  for (int i = 0; i < joints; i++) {
    int dif = target[i] - encoderCount[i];
    float p = float(dif) * 0.1;

    HBridge(i, p);

  }
}




void process_data (char * data) {
  //Serial.println(data);
  if (data[2] == '?') {
    int i = getChannelFromData(data, 1);
    if (data[0] == 'C') Serial.print("C" + String(i) + "=" + String(encoderCount[i]));
    if (data[0] == 'T') Serial.print("T" + String(i) + "=" + String(target[i]));
  } else if (data[2] == '=') {
    int i = getChannelFromData(data, 1);
    if (data[0] == 'T') target[i] = longFromData(data, 3);
    if (data[0] == 'C') encoderCount[i] = longFromData(data, 3);
  } else if (data[1] == '?') {
    //if (data[0] == 'B') Serial.print("B=" + String(busy));
  } else if (data[0] == 'R') {
    calibrate_joints();
  }

  Serial.print (";");
}


int getChannelFromData(char * data, int char_pos) {
  char data1[1] = {data[char_pos]};
  int i =  atoi(data1);
  if (i >= joints) i = 0;
  return i;
}

long longFromData(char * data, int start_offset) {
  for (int i = 0; i < start_offset; i++) data[i] = ' ';
  return atol(data);
}



#define MAX_INPUT 32
void processIncomingBytes() {
  while (Serial.available() > 0) {
    counters();
    const byte inByte = Serial.read();
    static char input_line [MAX_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte)
    {

      case ';':   // end of text
        input_line [input_pos] = 0;  // terminating null byte

        // terminator reached! process input_line here ...
        process_data (input_line);

        // reset buffer for next time
        input_pos = 0;
        break;

      case '\r':   // discard carriage return
        break;

      case '\n':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (input_pos < (MAX_INPUT - 1))
          input_line [input_pos++] = inByte;
        break;

    }
  }
}


void setup() {
  Serial.begin(500000);
  for (int i = 0; i < joints; i++) {
    pinMode(motorPinsA[i], OUTPUT);
    pinMode(motorPinsB[i], OUTPUT);
    analogWriteFrequency(motorPinsA[i], 585937.5);
    analogWriteFrequency(motorPinsB[i], 585937.5);
    pinMode(encoderPinsA[i], INPUT);
    pinMode(encoderPinsB[i], INPUT);
    pinMode(limitSwitchPins[i], INPUT);
  }
  pinMode(plugCheckPin, INPUT_PULLUP);
  resetCounters();
}

void connectorCheck() {
  while (digitalRead(plugCheckPin)) {
    for (int i = 0; i < joints; i++)HBridge(i, 0.0);
    Serial.println("CONNECTOR ERROR");
    delay(500);
  }
}

void loop() {

  counters();
  pid();
  processIncomingBytes();
  connectorCheck();
}
