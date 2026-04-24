/*
 * Magdi Laoun, 13.07.2025
 * Balance of a bar moved by a motor with a PID controller
 * Use of TMC5160 stepper motor driver
 * Use of encoder to measure the position of the bar 400 lines
 * Boards: ESP32-C3 super mini, TMC5160
*/
#include <Arduino.h>
#include <Pendulum.h>

Encoder encoder(CHA, CHB); //Create encoder object
TMC tmc(SCK, MOSI, MISO, CS, EN); // Create TMC object with pin definitions
Pendulum pendulum(10000, 200, 128, 0.04f); // Create Pendulum object with parameters
MoveMode moveMode = STANDBY; //Initial move mode
uint64_t lastTime = 0; //Variable to store time
uint8_t phaseCounter = 0; //Counter
void standby();
void oscillation();
void invertedBalance();
void balance();
void looping(uint8_t direction);
void dumping();
void resetTMC();
void resetEncoder();
void checkSerial();
void transmitFloat(float value, byte address);
float getFloatValue(byte *input);
void setMoveMode(MoveMode mode);
void setup(){
    Serial.begin(115200); //Initialize serial communication at 115200 baud rate
    resetTMC(); //Reset TMC5160
}
void loop(){
    checkSerial(); //Check for serial commands
    pendulum.updateAngle(encoder); //Update angle and angular velocity of pendulum
    pendulum.updatePosition(tmc); //Update position and velocity of cart
    if (micros() - lastTime > 100000) { //Update computer every 0.1 seconds
        transmitFloat(pendulum.angle, 0x00); //Transmit angle
        transmitFloat(pendulum.angularVelocity, 0x01); //Transmit angular velocity
        transmitFloat(pendulum.position, 0x02); //Transmit position
        transmitFloat(pendulum.velocity, 0x03); //Transmit velocity
        lastTime = micros();
    }
    switch (moveMode) {
        case STANDBY: standby(); break;
        case OSCILLATION: oscillation(); break;
        case BALANCE: balance(); break;
        case INVERTEDBALANCE: invertedBalance(); break;
        case DUMPING: dumping(); break;
        case LOOPINGCW: looping(CW); break;
        case LOOPINGCCW: looping(CCW); break;
    }
}
void checkSerial(){
    if (Serial.available() > 0) { //If data is available on serial port
        byte *data = (byte *) malloc(5);
        Serial.readBytes(data, 5); //Read 5 bytes from serial port
        float value = getFloatValue(data);
        switch (data[0]) {
            case 0x00: pendulum.setMagnitude(value); break;
            case 0x01: pendulum.setSpeed(tmc, value); break;
            case 0x02: pendulum.setAcceleration(tmc, value); break;
            case 0x03: pendulum.setThreshold(value); break;
            case 0x06: pendulum.setBalancePos(value); break;
            case 0x07: pendulum.setLimit(value); break;
            case 0x08: pendulum.setLoopImpulse1(value); break;
            case 0x09: pendulum.setLoopImpulse2(value); break;
            case 0x0A: pendulum.setLoopAngle(value); break;
            case 0x20: pendulum.setKpa(value); break;
            case 0x21: pendulum.setKda(value); break;
            case 0x22: pendulum.setKpm(value); break;
            case 0x23: pendulum.setKdm(value); break;
            case 0x50: setMoveMode(static_cast<MoveMode>(data[1])); break;
            case 0x51: resetTMC(); break;
            case 0x52: resetEncoder(); break;
        }
        free(data);
    }
}
void standby(){
    moveMode = STANDBY;
    tmc.setRampMode(0); //Set to position mode
    tmc.targetPosition(0); //Set target position to zero
    pendulum.updateSpeed(tmc); //Update speed
    pendulum.updateAcceleration(tmc); //Update acceleration
}
void oscillation(){
    tmc.setRampMode(0); //Set to position mode
    if (pendulum.overThreshold()) {
        setMoveMode(INVERTEDBALANCE);
        return;
    }
    pendulum.oscillate(tmc, &phaseCounter);
    if (phaseCounter>12){setMoveMode(STANDBY);}
}
void invertedBalance(){
    if (pendulum.overLimit()) {setMoveMode(STANDBY); return;}
    //Control algorithm for inverted balance
    pendulum.invertedBalance(tmc);
}
void balance() {
    if (pendulum.overLimit()) {setMoveMode(STANDBY); return;}
    pendulum.balance(tmc);
}
void looping(uint8_t direction = CW) {
    int8_t sign = (direction == CW) ? -1 : 1;
    switch (phaseCounter) {
        case 0:
            pendulum.impulse1(tmc, sign);
            if (pendulum.loopingTest1()) {phaseCounter = 1;}
            break;
        case 1:
            pendulum.impulse2(tmc, sign);
            if (pendulum.loopingTest2()) {phaseCounter = 2;}
            break;
        case 2:
            if (pendulum.loopingTest3()) {setMoveMode(INVERTEDBALANCE);}
            break;
    }
    
}
void dumping() {
    switch (phaseCounter) {
        case 0:
            tmc.setRampMode(0); //Set to position mode
            tmc.targetPosition(0); //Set target position to zero
            pendulum.updateAcceleration(tmc); //Set acceleration to 2 m/s^2
            if (pendulum.underLowLimit()) {phaseCounter = 1;}
            break;
        case 1:
            pendulum.dumping(tmc);
            if (pendulum.underLowLimit2()) {setMoveMode(BALANCE);}
            break;
    }
}
void resetTMC(){
    tmc.init(0.05, 0.25, MSTEPS); //Initialize TMC5160 with hold current 0.05A and run current 0.2A
    tmc.setGlobalScaler(180); //Set global scaler to 180/256
    standby();
}
void resetEncoder(){
    encoder.resetPosition(); //Reset encoder position to zero
}

void transmitFloat(float value, byte address) {
  byte data[6];
  data[0] = 0xAA; //Start byte
  data[1] = address; //Set address
  for (int i = 0; i < 4; i++) {
    data[i + 2] = ((uint32_t&)value >> (i * 8)) & 0xFF; //Set value in little-endian format
  }
  Serial.write(data, sizeof(data)); //Send data over serial
  Serial.flush(); //Wait for transmission to complete
}
float getFloatValue(byte *input) {
  FloatBytes fb;
  for (int i = 0; i < 4; i++) {
    fb.bytes[i] = input[i+1]; //Copy bytes from input to union
  }
  float value = fb.value; //Get float value from union
  return value;
}
void setMoveMode(MoveMode mode) {
    phaseCounter = 0;
    moveMode = mode;
}