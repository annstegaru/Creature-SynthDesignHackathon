#include "DaisyDuino.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

DaisyHardware hw;
AnalogBassDrum bd;
AnalogSnareDrum sd;
size_t num_channels;
//Metro tick;

float saw, freq_flt, freq_osc, output;
static Oscillator osc, lfo;
static ATone flt;
float val;
static Chorus cho;


bool trigger_kick = false;
bool trigger_snare = false;
bool prev1 = false;
bool current1 = false;
bool prev2 = false;
bool current2 = false;

//float pitchknob;
//LDRs
int photocellPin1 = A0;
int photocellPin2 = A1;
int photocellReading1_old;
int photocellReading2_old;
////int photocellReading1_new;
////int photocellReading2_new;
//int a = 0.1;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

void MyCallback(float **in, float **out, size_t size) {
  // Convert Pitchknob MIDI Note Number to frequency

  float output_left, output_right;
  float b1, s1, bs;
  float filtered_saw;

  lfo.SetFreq(map(val, -600, 600, 0.5, 4));
  for (size_t i = 0; i < size; i++) {

    freq_flt = map(val, -600, 600, 0, 15000);
    freq_osc = 1000 + (lfo.Process() * 20);

    osc.SetFreq(freq_osc);
    saw = osc.Process();
    flt.SetFreq(freq_flt);

    filtered_saw = flt.Process(saw);
    cho.Process(filtered_saw);
    output_left = cho.GetLeft();
    output_right = cho.GetRight();

    //    out[1][i] = output_left;
    //    out[0][i] = output_right;


    if (trigger_kick) {
      bd.SetTone(.7f * random()  / (float)RAND_MAX);
      bd.SetDecay(random()  / (float)RAND_MAX);
      bd.SetSelfFmAmount(random()  / (float)RAND_MAX);
    }

    if (trigger_snare) {
      sd.SetDecay(random()  / (float)RAND_MAX);
      sd.SetSnappy(random()  / (float)RAND_MAX);
      sd.SetTone(.8f * random()  / (float)RAND_MAX);
    }

    b1 = bd.Process(trigger_kick);
    s1 = sd.Process(trigger_snare);
    bs = b1 + s1;
    out[0][i] = output_left + bs;
    out[1][i] = output_right + bs;

  }

}


void setup() {
  Serial.begin(9600);
  //MOTOR
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(30);
  myMotor->run(FORWARD);
  //  // turn on motor
  myMotor->run(RELEASE);

  //GYRO
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case ICM20948_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20948_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20948_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
    case AK09916_MAG_DATARATE_SHUTDOWN:
      Serial.println("Shutdown");
      break;
    case AK09916_MAG_DATARATE_SINGLE:
      Serial.println("Single/One shot");
      break;
    case AK09916_MAG_DATARATE_10_HZ:
      Serial.println("10 Hz");
      break;
    case AK09916_MAG_DATARATE_20_HZ:
      Serial.println("20 Hz");
      break;
    case AK09916_MAG_DATARATE_50_HZ:
      Serial.println("50 Hz");
      break;
    case AK09916_MAG_DATARATE_100_HZ:
      Serial.println("100 Hz");
      break;
  }
  Serial.println();

  // Initialize for Daisy pod at 48kHz
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = hw.num_channels;

  float sample_rate;
  sample_rate = DAISY.get_samplerate();
  flt.Init(sample_rate);

  lfo.Init(sample_rate);
  lfo.SetWaveform(Oscillator::WAVE_TRI);
  lfo.SetFreq(1);
  lfo.SetAmp(0.5);

  osc.Init(sample_rate);
  osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
  osc.SetFreq(1000);
  osc.SetAmp(0.25);

  cho.Init(sample_rate);

  bd.Init(sample_rate);
  bd.SetFreq(50.f);

  sd.Init(sample_rate);

  DAISY.begin(MyCallback);
}

void loop() {

  // pitchknob = 24.0 + ((analogRead(A0) / 1023.0) * 60.0);
  //GYRO

  icm.getEvent(&accel, &gyro, &temp, &mag);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  val = mag.magnetic.x;

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  //delay(100);


  //LDRs
    photocellReading1_old = analogRead(photocellPin1);
    photocellReading2_old = analogRead(photocellPin2);
    Serial.print("Analog reading1 = ");
    Serial.println(photocellReading1_old);     // the raw analog reading
    Serial.print("Analog reading2 = ");
    Serial.println(photocellReading2_old);
    
  if ((photocellReading1_old > 20)|| (photocellReading2_old > 20))
      current1 = true;
  else
      current1 = false;

  //if (photocellReading2_old > 20)
  //    current2 = true;
  //else
  //    current2 = false;
  //
  if (!prev1 && current1)
    trigger_kick = true;
  else
    trigger_kick = false;

  //if (!prev2 && current2)
  //    trigger_snare = true;
  //else
  //    trigger_snare = false;
  //
  //prev1 = current1;
  //prev2 = current2;

  //  if(!trigger_kick && ((photocellReading1_old > 20) || (photocellReading2_old > 20)))
  //    trigger_kick = true;
  //    else
  //      trigger_kick = false;


  //  if(!trigger_kick && photocellReading2_old > 20){
  //    trigger_kick = true;
  //    //Serial.println("snare");
  //  }
  //    else
  //      trigger_kick = false;

  //  photocellReading1_new = photocellReading1_old * a + analogRead(1 - a);
  //  photocellReading2_new = photocellReading2_old * a + analogRead(1 - a);
  //
  //  Serial.print("Analog reading1 new = ");
  //  Serial.println(photocellReading1_new);
  //  Serial.print("Analog reading2 new = ");
  //  Serial.println(photocellReading2_new);

  //  photocellReading1_old =  photocellReading1_new;
  //  photocellReading1_old =  photocellReading2_new;


  //MOTOR
  uint8_t i;
  //Serial.print("tick");

  myMotor->run(FORWARD);
  //  for (i = 0; i < 255; i++) {
  myMotor->setSpeed(50); // photocellReading
  delay(10);
  //  }
  //  for (i = 255; i != 0; i--) {
  //    myMotor->setSpeed(i);
  //    delay(10);
  //  }

  //Serial.print("tock");
  //
  //  myMotor->run(BACKWARD);
  //  for (i = 0; i < 255; i++) {
  //    myMotor->setSpeed(i);
  //    delay(10);
  //  }
  //  for (i = 255; i != 0; i--) {
  //    myMotor->setSpeed(i);
  //    delay(10);
  //  }

  //Serial.print("tech");
  myMotor->run(RELEASE);
  //delay(100);

}
