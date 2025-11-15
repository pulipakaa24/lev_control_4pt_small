#include <Arduino.h>

// PIN MAPPING

const int distPin = A1;
const int pwmPin = 6;
const int dirPin = 7;
const int dir2Pin = 4;
const int pwm2Pin = 3;
const int dist2Pin = A2;


const int range2Pin = A4;
const int rangePin = A3;

// variables

int dist_raw, oor, t0, tcurr, tprior, telapsed, pwm, pwm2, dist2_raw, oor2;
float dist,ecurr, eprior, derror, ecum, ff,dist2,ecurr2, eprior2, derror2, ecum2, ff2;

const float sensor_gain = 10.0 / (712-200.0);
const float offset = 8;

const int CAP = 200;


// CONTROLLER CONSTANTS

float kp, ki, kd, K;
float MAX_INTEGRAL_TERM = 1e4;

const float ref = 21.0; //14.9;
const float ref2 = 22.0; //14.9;

const float K_current = 1;
const float ki_current = 0;

//// FOR MC 1
//const float K_f = 50; // gain for when we want to fall (ref > dist or error > 0)
//const float ki_f = 0.01;
//const float kd_f = 8; // 25
//
//const float K_a = 20;
//const float ki_a = ki_f;
//const float kd_a = 10; //30;

// FOR MC 1
const float K_f = 40; // gain for when we want to fall (ref > dist or error > 0)
const float ki_f = 0.01;
const float kd_f = 7; // 25

const float K_a = 20;
const float ki_a = ki_f;
const float kd_a = 20; //30;

// FOR MC 2

const float K2_f = K_f;// gain for when we want to fall (ref > dist or error > 0)
const float ki2_f = ki_f;
const float kd2_f = kd_f;

const float K2_a = K_a;
const float ki2_a = ki_a;
const float kd2_a = kd_a;

const int sampling_rate = 1000;
const int dt_micros = 1e6/sampling_rate;

#define LEV_ON

int ON = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);

  
  pinMode(distPin, INPUT);
  pinMode(rangePin, INPUT);

  t0 = micros();
  tprior = t0;

  ecum = 0;
  ecum2 = 0;

  // positive pwm is A
  // negative is B

  // ATTRACT IS B  // REPEL IS A


  //when error is negative, I want to attract.
  send_pwm1(0);
  send_pwm2(0);

}



void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');  // Read the full input
    inputString.trim();  // Remove leading/trailing whitespace, including \n and \r

    // Conditional pipeline
    if (inputString == "0") {
        ON=0;
    } 
    else {
        ON=1;
    }
  }
  
  telapsed = micros() - tprior;

  if (telapsed >= dt_micros){
    // put your main code here, to run repeatedly:
    dist_raw = analogRead(distPin);
    dist = float_map(dist_raw, 189, 950, 16, 26); // 189->950, 16->26
    Serial.print(dist);
    Serial.print(", ");

    dist2_raw = analogRead(dist2Pin);
    dist2 = float_map(dist2_raw, 189, 950, 16, 26);
    Serial.print(dist2);
    Serial.print(", ");

    oor = (dist_raw <= 195); // naive oor 
    oor2 = (dist2_raw <= 195); // naive oor
    
//
    ecurr = ref - dist;
    //ecurr = dist - ref;
    derror = ecurr - eprior;

    ecurr2 = ref2 - dist2;
    //ecurr2 = dist2 - ref2;
    derror2 = ecurr2 - eprior2;

    
    if (ON) {
      int pwm1 = levitate(ecurr, derror, ecum, oor);
      int pwm2 = levitate2(ecurr2, derror2, ecum2, oor2);
      Serial.print(pwm1);
      Serial.print(", ");
      Serial.print(pwm2);
      Serial.print(", ");
    }
    else {
      send_pwm1(0);
      send_pwm2(0);
      Serial.print(0);
      Serial.print(", ");
      Serial.print(0);
      Serial.print(", ");
    }

    
    Serial.print(ON);
    
    
//    
//    
//
    tprior = micros();
    eprior = ecurr;
    eprior2 = ecurr2;
//    //Serial.print(ecurr); Serial.print(","); Serial.print(oor); Serial.print(","); Serial.print(derror); Serial.print(","); Serial.print(pwm); Serial.print(";  "); Serial.print(ecurr2); Serial.print(","); Serial.print(oor2); Serial.print(","); Serial.print(derror2); Serial.print(","); Serial.print(pwm2);
//    Serial.print(ecurr); Serial.print(","); Serial.print(ecurr2); Serial.print(","); Serial.print(ecum); Serial.print(",");Serial.print(ecum2); Serial.print(",");
//    
    Serial.println();
  }

  //Serial.println(telapsed);
}

int levitate(float e, float de, float ecum, int oor){
  if (oor){
    pwm = 0;
  }
  else{
    if (e < 0) { // this means that dist > ref so we gotta attract to track now
      kd = kd_a;
      ki = ki_a;
      K = K_a;
    }
    else{
      kd = kd_f;
      ki = ki_f;
      K = K_f;
    }

    pwm = constrain(K*(e + ki*ecum + kd*de), -CAP,CAP);

  }
  send_pwm1(pwm);
  return (int)pwm;
}

int levitate2(float e, float de, float ecum, int oor){
  if (oor){
    pwm2 = 0;
  }
  else{
    if (e < 0) { // this means that dist > ref so we gotta attract to track now
      kd = kd2_a;
      ki = ki2_a;
      K = K2_a;
    }
    else{
      kd = kd2_f;
      ki = ki2_f;
      K = K2_f;
    }

    pwm2 = constrain(K*(e + ki*ecum + kd*de), -CAP,CAP);

  }
  send_pwm2(pwm2);
  return (int)pwm2;

}

void send_pwm1(int val){
  if (val > 0) {
    digitalWrite(dirPin, LOW);
  }
  else{
    digitalWrite(dirPin,HIGH);
  }

  analogWrite(pwmPin,abs(val));

}

void send_pwm2(int val){
  if (val > 0) {
    digitalWrite(dir2Pin, LOW);
  }
  else{
    digitalWrite(dir2Pin,HIGH);
  }

  analogWrite(pwm2Pin,abs(val));

}

float float_map(int x, int in_min, int in_max, int out_min, int out_max){
        float top = (float)((x-in_min) * (out_max-out_min));
        return top / (float)(in_max - in_min) + (float)out_min;
     

  //return dist_raw * sensor_gain + offset;
}
