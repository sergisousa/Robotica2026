/* Copyright (c) 2025  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

int version = 1103;

#include <Arduino.h>

#include <WiFi.h>

//#include <RPi_Pico_TimerInterrupt.h>

#define ELEGANT_OTA 1
#ifdef ELEGANT_OTA
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WebServer.h>
#include <ElegantOTA.h>

WebServer server(80);

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
  Serial.flush();
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    Serial.flush();
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
  Serial.flush();
}
#endif

#include <WiFiUdp.h>
#include <LittleFS.h>

#define max_wifi_str 32

char ssid[max_wifi_str];
char password[max_wifi_str];

int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

// UDP socket to communicate with the referee box
WiFiUDP Ref_Udp;
unsigned int Ref_localUdpPort = 1234;  // local port to listen on


// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer
//RPI_PICO_Timer ITimer1(1);

#include "aruco.h"

#include "pico4drive.h"
pico4drive_t pico4drive;

#include "PicoEncoder.h"

#define ENC1_PIN_A 2
#define ENC1_PIN_B 3

#define ENC2_PIN_A 6
#define ENC2_PIN_B 7

#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENC1_PIN_A, ENC2_PIN_A};

bool calibration_requested;

#define TEST_PIN 27

//#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
//#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)

//#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)

//bool timer_handler(struct repeating_timer *t)
//{
//
//}

// PWM stuff

#define MOTOR1A_PIN 16
#define MOTOR1B_PIN 17

#define MOTOR2A_PIN 14
#define MOTOR2B_PIN 15

#define SOLENOID_PIN_A 12
#define SOLENOID_PIN_B 13

#define LIDAR_MOT_PIN_A 10
#define LIDAR_MOT_PIN_B 11

//int debug;

#include <Wire.h>

//#define HAS_VL53L0X 1
#ifdef HAS_VL53L0X
#include <VL53L0X.h>
VL53L0X tof;
#endif

//#define HAS_VL53L0X_X3 1
#ifdef HAS_VL53L0X_X3
#include <VL53L0X.h>
VL53L0X tof, tof_left, tof_right;
#endif

//#define HAS_INA266 1
#ifdef HAS_INA266
#include <INA226_WE.h>

// There are several ways to create your INA226 object:
// INA226_WE ina226 = INA226_WE(); -> uses I2C Address = 0x40 / Wire
// INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
// INA226_WE ina226 = INA226_WE(&Wire); -> uses I2C_ADDRESS = 0x40, pass any Wire Object
// INA226_WE ina226 = INA226_WE(&Wire, I2C_ADDRESS);

INA226_WE ina226 = INA226_WE(0x40);
#endif


//#define HAS_SERVO 1
#if defined(HAS_SERVO)
#define SERVO_PIN 22
#include <Servo.h>
Servo pen_servo;
#endif


#include "robot.h"
#include "display.h"

void init_control(robot_t& robot);
void control(robot_t& robot);

#include "actions.h"

PID_pars_t wheel_PID_pars;

#include "control.h"

//#define IRLINE_SENSOR 5

void readIRSensors(IRLine_t& IRLine)
{
  byte c;  // Read the five IR sensors using the AD converter
  int offset = 8 - IRLine.sensor_count;
  for (c = 0; c < IRLine.sensor_count; c++) {
    if (IRLine.invert_signal) {
      IRLine.IR_values[(IRLine.sensor_count - 1) - c] = 1023 - pico4drive.read_adc(offset + c);
    } else {
      IRLine.IR_values[(IRLine.sensor_count - 1) - c] = pico4drive.read_adc(offset + c);
    }
  }
}

uint32_t encodeIRSensors(void)
{
  byte c;  // Encode five IR sensors with 6 bits for each sensor
  uint32_t result = robot.IRLine.IR_values[0] >> 4; // From 10 bits to 6 bits
  for (c = 1; c < robot.IRLine.sensor_count; c++) {
    result = (result << 6) | (robot.IRLine.IR_values[c] >> 4);
  }
  return result;
}

int debug_level;

uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t cycle_count;

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
  robot.dt = new_interval;   // In seconds
  wheel_PID_pars.dt = robot.dt;
}

// Remote commands


#include "gchannels.h"
#include "file_gchannels.h"

gchannels_t udp_commands;
gchannels_t serial_commands;
commands_list_t pars_list;

const char* pars_fname = "pars.cfg";
bool load_pars_requested = false;

void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("mo")) { // The 'mo'de command ...
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("se1")) {
    robot.Senc1 = frame.value;

  } else if (frame.command_is("se2")) {
    robot.Senc2 = frame.value;

  } else if (frame.command_is("u1")) { // The 'u1' command sets the voltage for motor 1
    robot.u1_req = frame.value;

  } else if (frame.command_is("u2")) { // The 'u2' command sets the voltage for motor 2
    robot.u2_req = frame.value;

  } else if (frame.command_is("w1")) {
    robot.w1_req = frame.value;

  } else if (frame.command_is("w2")) {
    robot.w2_req = frame.value;

  } else if (frame.command_is("st")) {
     robot.pfsm->force_state(frame.value);

  } else if (frame.command_is("dt")) {
     set_interval(frame.value);

  } else if (frame.command_is("v")) {
    robot.v_req = frame.value;
    robot.vw_timer.start();

  } else if (frame.command_is("w")) {
    robot.w_req = frame.value;
    robot.vw_timer.start();

  } else if (frame.command_is("lv")) {
    float u = frame.value;
    u = min(u, 3.8);
    u = max(u, -3.8);
    robot.lidar_mot_u = u;

  } else if (frame.command_is("pen")) {
    if (frame.value == 0)  robot.servo_us = robot.servo_down;
    else if (frame.value == 1)  robot.servo_us = robot.servo_up;
    else robot.servo_us = frame.value;

  } else if (frame.command_is("sl")) {
    robot.solenoid_u = frame.value;

  } else if (frame.command_is("xr")) {
    robot.xe = frame.value;

  } else if (frame.command_is("yr")) {
    robot.ye = frame.value;

  } else if (frame.command_is("tr")) {
    robot.thetae = frame.value;

  } else if (frame.command_is("pl")) {
    load_pars_requested = true;

  } else if (frame.command_is("ps")) {
    save_commands(pars_fname, pars_list, serial_commands);

  } else if (frame.command_is("ssid")) {
    strncpy(ssid, frame.text, max_wifi_str - 1);
    ssid[max_wifi_str - 1] = 0;

  } else if (frame.command_is("pass")) {
    if (strlen(frame.text) < 8) return;
    strncpy(password, frame.text, max_wifi_str - 1);
    password[max_wifi_str - 1] = 0;

  } else if (frame.command_is("wifi")) {
    if (frame.value == 1) {
      if (WiFi.connected()) WiFi.end();
      WiFi.begin(ssid, password);
    } else if (frame.value == 0) {
      WiFi.end();
    }

  } else if (frame.command_is("msg")) {  //
    serial_commands.send_command("msg", frame.text);

  } else if (frame.command_is("cat")) {
    send_file(frame.text, serial_commands, true);

  } else if (frame.command_is("udp")) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(UdpInPacket, frame.value);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();

  } else if (frame.command_is("dl")) {
    debug_level = frame.value;

  } // Put here more commands...
}


void read_PIO_encoders(void)
{
  encoders[0].update();
  encoders[1].update();
  robot.enc1 = encoders[0].speed / 64.0;
  robot.enc2 = encoders[1].speed / 64.0;
  robot.Senc1 += robot.enc1;
  robot.Senc2 += robot.enc2;
}


void serial_write(const char *buffer, size_t size)
{
  Serial.write(buffer, size);
  if (udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer, size);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();
  }
}

const char *encToString(uint8_t enc) {
  switch (enc) {
    case ENC_TYPE_NONE: return "NONE";
    case ENC_TYPE_TKIP: return "WPA";
    case ENC_TYPE_CCMP: return "WPA2";
    case ENC_TYPE_AUTO: return "AUTO";
  }
  return "UNKN";
}


void wifi_list(void)
{
  Serial.printf("Beginning scan at %d\n", millis());
  int cnt = WiFi.scanNetworks();
  if (!cnt) {
    Serial.printf("No networks found\n");
  } else {
    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %2s %4s\n", "SSID", "ENC", "CH", "RSSI");
    for (int i = 0; i < cnt; i++) {
      uint8_t bssid[6];
      WiFi.BSSID(i, bssid);
      Serial.printf("%32s %5s %2d %4d\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
}


#ifdef HAS_INA266
int setup_ina226(void)
{
  // Set Number of measurements for shunt and bus voltage which shall be averaged
  //* Mode *     * Number of samples *
  //AVERAGE_1            1 (default)
  //AVERAGE_4            4
  //AVERAGE_16          16
  //AVERAGE_64          64
  //AVERAGE_128        128
  //AVERAGE_256        256
  //AVERAGE_512        512
  //AVERAGE_1024      1024

  //ina226.setAverage(AVERAGE_64); // choose mode and uncomment for change of default
  ina226.setAverage(AVERAGE_4); // choose mode and uncomment for change of default

  // Set conversion time in microseconds
  // One set of shunt and bus voltage conversion will take:
  // number of samples to be averaged x conversion time x 2
  //
  // * Mode *         * conversion time *
  // CONV_TIME_140          140 µs
  // CONV_TIME_204          204 µs
  // CONV_TIME_332          332 µs
  // CONV_TIME_588          588 µs
  // CONV_TIME_1100         1.1 ms (default)
  // CONV_TIME_2116       2.116 ms
  // CONV_TIME_4156       4.156 ms
  // CONV_TIME_8244       8.244 ms

  //ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default
  ina226.setConversionTime(CONV_TIME_4156);
  //ina226.setConversionTime(CONV_TIME_204);

  // Set measure mode
  //POWER_DOWN - INA226 switched off
  //TRIGGERED  - measurement on demand
  //CONTINUOUS  - continuous measurements (default)

  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  // Set Current Range
  //* Mode *   * Max Current *
  // MA_400          400 mA
  // MA_800          800 mA (default)

  //ina226.setCurrentRange(MA_800); // choose gain and uncomment for change of default

  // If the current values delivered by the INA226 differ by a constant factor
  // from values obtained with calibrated equipment you can define a correction factor.
  // Correction factor = current delivered from calibrated equipment / current delivered by INA226

  // ina226.setCorrectionFactor(0.95);

  Serial.println("INA226 Current Sensor - Continuous");

  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  return 1;
}
#endif

void update_button_state(uint32_t now)
{
  static int last_button_state;
  static uint32_t press_start;

  if (pico4drive.button_state && !last_button_state)
    press_start = now;
  else if (pico4drive.button_state && last_button_state && press_start != 0) {
    uint32_t press_time = now - press_start;
    if (current_screen == SCREEN_MAIN) {
      if (press_time > 5000000) {
        press_start = 0;
        calib_camera();
      }
    } else if (current_screen == SCREEN_MAP) {
      if (press_time > 1000000) {
        press_start = 0;
        if (robot.pfsm->state != 301)
          robot.pfsm->force_state(300);
        else
          robot.pfsm->force_state(200);
      }
    }
  } else if (!pico4drive.button_state && last_button_state) {
    if (press_start != 0)
      current_screen = (current_screen + 1) % SCREEN_COUNT;
  }
  last_button_state = pico4drive.button_state;
}

void setup()
{
  // Set the pins as input or output as needed
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENC1_PIN_A, INPUT_PULLUP);
  pinMode(ENC1_PIN_B, INPUT_PULLUP);
  pinMode(ENC2_PIN_A, INPUT_PULLUP);
  pinMode(ENC2_PIN_B, INPUT_PULLUP);

  pinMode(TEST_PIN, OUTPUT);

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);

  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  pinMode(SOLENOID_PIN_A, OUTPUT);
  pinMode(SOLENOID_PIN_B, OUTPUT);

  pinMode(LIDAR_MOT_PIN_A, OUTPUT);
  pinMode(LIDAR_MOT_PIN_B, OUTPUT);

  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);

  //robot.IRLine.sensor_count = 8;
  //robot.IRLine.invert_signal = 0;
  //robot.IRLine.sensor_dist = 9.5; // mm

  pico4drive.init();

  analogReadResolution(10);

  pars_list.max_sparce_send = 4;
  pars_list.register_command("ver", &version);
  pars_list.register_command("to", &(robot.timeout));
  pars_list.register_command("nli", &(robot.no_line_interval));
  pars_list.register_command("rid", &robot.id_number);

  pars_list.register_command("whd", &(robot.wheel_dist));
  pars_list.register_command("whr", &(robot.wheel_radius));
  pars_list.register_command("gr", &(robot.gear_ratio));
  pars_list.register_command("ep", &(robot.encoder_poles));

  pars_list.register_command("su", &(robot.servo_up));
  pars_list.register_command("sd", &(robot.servo_down));

  pars_list.register_command("kf", &(wheel_PID_pars.Kf));
  pars_list.register_command("kc", &(wheel_PID_pars.Kc));
  pars_list.register_command("ki", &(wheel_PID_pars.Ki));
  pars_list.register_command("kd", &(wheel_PID_pars.Kd));
  pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
  pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));

  pars_list.register_command("af", &(action.thetaf));
  pars_list.register_command("xf", &(action.Pf.x));
  pars_list.register_command("yf", &(action.Pf.y));

  pars_list.register_command("xi", &(action.Pi.x));
  pars_list.register_command("yi", &(action.Pi.y));

  pars_list.register_command("cx", &(action.C.x));
  pars_list.register_command("cy", &(action.C.y));

  //pars_list.register_command("fk", &(robot.i_lambda));
  pars_list.register_command("kt", &(action.ktheta));
  pars_list.register_command("kst", &(action.kset_theta));
  pars_list.register_command("kn", &(action.kn));

  pars_list.register_command("ktr", &(action.ktrack));
  pars_list.register_command("wz", &(action.wz));

  pars_list.register_command("vnom", &(action.v_nom));
  pars_list.register_command("alpha", &(action.alpha));

  pars_list.register_command("w0", &(action.w0));

  pars_list.register_command("eat", &(action.e_theta_tresh));
  pars_list.register_command("exyt", &(action.e_xy_tresh));

  pars_list.register_command("fv", &(robot.follow_v));
  pars_list.register_command("fk", &(robot.follow_k));

  pars_list.register_command("irt", &(robot.IRLine.IR_tresh));
  pars_list.register_command("iwl", &(robot.IRLine.IR_WaterLevel));

  pars_list.register_command("Vcenter", &(robot.Vcenter));
  pars_list.register_command("Vside", &(robot.Vside));
  pars_list.register_command("Vedge", &(robot.Vedge));

  pars_list.register_command("iwg0", &(robot.IR_W_gain[0]));
  pars_list.register_command("iwg1", &(robot.IR_W_gain[1]));
  pars_list.register_command("iwg2", &(robot.IR_W_gain[2]));
  pars_list.register_command("iwg3", &(robot.IR_W_gain[3]));
  pars_list.register_command("iwg4", &(robot.IR_W_gain[4]));

  pars_list.register_command("mawt", &(robot.mean_abs_w_tresh));

  pars_list.register_command("ssid", ssid, 32);
  pars_list.register_command("pass", password, 32)->sparse_send = false;

  pars_list.register_command("cam_rx", &(camera_pars.rot_x));
  pars_list.register_command("cam_ry", &(camera_pars.rot_y));
  pars_list.register_command("cam_z", &(camera_pars.pos_z));
  pars_list.register_command("cam_off", &(camera_pars.offset_from_axis));
  pars_list.register_command("cam_crop", &(camera_pars.crop_offset));
  pars_list.register_command("cam_rz", &(camera_pars.rot_z));
  pars_list.register_command("goal_node",&(action.goal_node));

  udp_commands.init(process_command, serial_write);

  serial_commands.init(process_command, serial_write);

  robot.pchannels = &serial_commands;

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  // start the serial port to talk to the aruco sensor
  Serial2.setPinout(4, 5);
  Serial2.setFIFOSize(1024);
  Serial2.begin(1000000);

  // start the serial port to talk to referee gateway
  //Serial1.setPinout(0, 1);
  //Serial1.setFIFOSize(1024);
  //Serial1.begin(100000);


  LittleFS.begin();

  float control_interval = 0.02;  // In seconds

  // All wheeel PID controllers share the same parameters
  wheel_PID_pars.Kf = 0.0;
  wheel_PID_pars.Kc = 0.24;
  wheel_PID_pars.Ki = 2.4;
  wheel_PID_pars.Kd = 0;
  wheel_PID_pars.Kfd = 0;
  wheel_PID_pars.dt = control_interval;
  wheel_PID_pars.dead_zone = 0;
  int i;
  for (i = 0; i < NUM_WHEELS; i++) {
    robot.PID[i].init_pars(&wheel_PID_pars);
  }

  load_commands(pars_fname, serial_commands);

  camera_pars.update_matrix();
  obtain_adj_matrix(node_adj_matrix);

  // WiFi Credentials
  #include "credentials.h"
  
  strcpy(ssid, WIFI_SSID);
  strcpy(password, WIFI_PASSWORD);

  //strcpy(ssid, "TP-Link_29CD");
  //strcpy(password, "49871005");

  //strcpy(ssid, "robot_hotspot");
  //strcpy(password, "feup5dpo");


  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);

  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);


  //if (ITimer1.attachInterrupt(40000, timer_handler))
  //  Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  //else
  //  Serial.println("Can't set ITimer. Select another freq. or timer");


  #ifdef HAS_VL53L0X_X3

  const int tof_left_pin = 6;
  const int tof_pin = 5;
  const int tof_right_pin = 4;
  pinMode(tof_left_pin, OUTPUT);
  digitalWrite(tof_left_pin, 0);
  pinMode(tof_pin, OUTPUT);
  digitalWrite(tof_pin, 0);
  pinMode(tof_right_pin, OUTPUT);
  digitalWrite(tof_right_pin, 0);
  delay(50);
  #endif

  Wire.setSDA(8);
  Wire.setSCL(9);

  Wire.begin();

  #ifdef HAS_INA266

  while (!ina226.init()) {
    Serial.println("could not connect ina226!");
    delay(100);
  }

  setup_ina226();
  #endif


  #ifdef HAS_VL53L0X

  //tof.setAddress(0x22);

  tof.setTimeout(100);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startContinuous(0);

  #endif


  #ifdef HAS_VL53L0X_X3
  digitalWrite(tof_left_pin, 1);
  delay(50);
  tof_left.setAddress(0x22);
  tof_left.setTimeout(100);
  while (!tof_left.init()) {
    Serial.println(F("Failed to detect and initialize the left VL53L0X"));
    delay(100);
  }

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof_left.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof_left.startContinuous(0);

  digitalWrite(tof_pin, 1);
  delay(50);
  tof.setAddress(0x22 + 2);
  tof.setTimeout(100);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize the center VL53L0X"));
    delay(100);
  }

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startContinuous(0);

  digitalWrite(tof_right_pin, 1);
  delay(50);
  tof_right.setAddress(0x22 + 4);
  tof_right.setTimeout(100);
  while (!tof_right.init()) {
    Serial.println(F("Failed to detect and initialize the right VL53L0X"));
    delay(100);
  }

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof_right.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof_right.startContinuous(0);

  #endif


  #ifdef HAS_SERVO
  pen_servo.attach(SERVO_PIN);
  robot.servo_up = 1000;
  robot.servo_down = 1800;
  robot.servo_us = robot.servo_up;
  #endif


  debug_level = 1;
  set_interval(control_interval);    // In seconds
  init_control(robot);

}

static int compare_int(const void *a, const void *b)
{
  return *((int *)b) - *((int *)a);
}

void loop()
{
  if (WiFi.connected() && !ip_on) {
    // Connection established
    serial_commands.send_command("msg", (String("Pico W is connected to WiFi network with SSID ") + WiFi.SSID()).c_str());

    // Print IP Address
    ip_on = Udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
    //serial_commands.send_command("msg", (String("Now listening at IP ") + WiFi.localIP().toString()).c_str());
    //serial_commands.send_command("msg", (String(" UDP port ") + String(localUdpPort)).c_str());

    Ref_Udp.begin(Ref_localUdpPort);

    #ifdef ELEGANT_OTA
    server.on("/", []() {
      server.send(200, "text/plain", "Use: xx.xx.xx.xx/update");
    });

    ElegantOTA.begin(&server);    // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    //Serial.println("HTTP server started");
    serial_commands.send_command("msg", "HTTP server started");
    #endif
  }

  encoders[0].autoCalibratePhases();
  encoders[1].autoCalibratePhases();

  if (ip_on) {

    #ifdef ELEGANT_OTA
    server.handleClient();
    ElegantOTA.loop();
    #endif

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_commands.process_char(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }
  }

  int packetSize = Udp.parsePacket();


  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read

    b = Serial.read();
    serial_commands.process_char(b);
    //Serial.write(b);
  }

  if (Serial2.available()) {
    b = Serial2.read();
    process_aruco_data(b);
    if (aruco_robot_state.updated) {
      aruco_robot_state.updated = 0;
      robot.update_aruco(aruco_robot_state.position.x, aruco_robot_state.position.y, aruco_robot_state.theta, aruco_robot_state.point_count);
    }
  }

  pico4drive.update();

  if (load_pars_requested) {
     load_commands(pars_fname, serial_commands);
     load_pars_requested = false;
  }

  // Do this only every "interval" microseconds
  uint32_t now = micros();
  uint32_t delta = now - last_cycle;
  if (delta >= interval) {
    loop_micros = now;
    last_cycle = now;
    //last_cycle += interval;
    cycle_count++;

    // Read and process sensors
    read_PIO_encoders();

    robot.odometry();
    //robot.battery_voltage = 7.4; // if it could not be measured...

    #ifdef IRLINE_SENSOR
    readIRSensors(robot.IRLine);

    robot.IRLine.calcLastBlackTime();

    robot.IRLine.calcIRLineEdgeLeft();
    robot.IRLine.calcIRLineEdgeRight();
    robot.IRLine.calcCrosses();

    robot.IRLine.calcIRLineCenter();
    robot.IRLine.calcIRLineCenterDigital();
    #endif

    #ifdef HAS_VL53L0X
    if (tof.readRangeAvailable()) {
      robot.prev_tof_dist = robot.tof_dist;
      robot.tof_dist = tof.readRangeMillimeters() * 1e-3;
    }
    #endif

    #ifdef HAS_VL53L0X_X3
    if (tof.readRangeAvailable()) {
      robot.prev_tof_dist = robot.tof_dist;
      robot.tof_dist = tof.readRangeMillimeters() * 1e-3;
    }

    if (tof_left.readRangeAvailable()) {
      robot.prev_tof_dist_left = robot.tof_dist_left;
      robot.tof_dist_left = tof_left.readRangeMillimeters() * 1e-3;
    }

    if (tof_right.readRangeAvailable()) {
      robot.prev_tof_dist_right = robot.tof_dist_right;
      robot.tof_dist_right = tof_right.readRangeMillimeters() * 1e-3;
    }
    #endif

    #ifdef HAS_INA266
    ina226.readAndClearFlags();
    float shuntVoltage_mV = ina226.getShuntVoltage_mV();
    float busVoltage_V = ina226.getBusVoltage_V();
    float current_mA = -ina226.getCurrent_mA();
    //float power_mW = ina226.getBusPower();
    //float loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);

    robot.i_sense = robot.i_lambda * robot.i_sense + (1 - robot.i_lambda) * current_mA * 1e-3;
    robot.u_sense = busVoltage_V * 1000;
    #endif

    update_button_state(now);

    // Control the robot here by choosing:
    //   PWM_1_req and PWM_1_req  when robot.control_mode = cm_pwm
    //   v1_req and v2_req        when robot.control_mode = cm_pid
    //   v_req and w_req          when robot.control_mode = cm_kinematics
    control(robot);

    // Calc outputs
    //robot.accelerationLimit();
    robot.vref = robot.v_req;
    robot.wref = robot.w_req;

    robot.calcMotorsVoltage();

    robot.PWM_1 = pico4drive.voltage_to_PWM(robot.u1);
    robot.PWM_2 = pico4drive.voltage_to_PWM(robot.u2);

    if (robot.stoped) {
      robot.PWM_1 = 0;
      robot.PWM_2 = 0;
      robot.solenoid_u = 0;
    }

    pico4drive.set_driver_PWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    pico4drive.set_driver_PWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

    pico4drive.set_driver_PWM(pico4drive.voltage_to_PWM(robot.solenoid_u), SOLENOID_PIN_A, SOLENOID_PIN_B);

    robot.lidar_mot_PWM = pico4drive.voltage_to_PWM(robot.lidar_mot_u);
    pico4drive.set_driver_PWM(robot.lidar_mot_PWM, LIDAR_MOT_PIN_A, LIDAR_MOT_PIN_B);

    #ifdef HAS_SERVO
      pen_servo.writeMicroseconds(robot.servo_us);
    #endif

    digitalWrite(LED_BUILTIN, robot.led);

    if (debug_level > 0) {
      // Debug information
      serial_commands.send_command("dte", delta);

      serial_commands.send_command("u1", robot.u1);
      serial_commands.send_command("u2", robot.u2);

      serial_commands.send_command("e1", robot.enc1);
      serial_commands.send_command("e2", robot.enc2);

      //serial_commands.send_command("se1", robot.Senc1);
      //serial_commands.send_command("se2", robot.Senc2);

      serial_commands.send_command("w1", robot.w1e);
      serial_commands.send_command("w2", robot.w2e);

      serial_commands.send_command("Vbat", pico4drive.battery_voltage);

      serial_commands.send_command("ve", robot.ve);
      serial_commands.send_command("we", robot.we);

      //serial_commands.send_command("sl", robot.solenoid_PWM);

      #ifdef HAS_INA266
      serial_commands.send_command("is", robot.i_sense);
      serial_commands.send_command("us", robot.u_sense);
      #endif

      serial_commands.send_command("mode", robot.control_mode);

      serial_commands.send_command("st", robot.pfsm->state);

      // send the first 5 arucos being seen, sort to avoid having the list
      // shuffle around from frame to frame
      int aruco_set[MAX_ARUCOS_PER_FRAME];
      memset(aruco_set, 0xFF, sizeof(aruco_set)); // set to -1
      for (int i = 0; i < image_aruco_count; i++)
        aruco_set[i] = image_arucos[i].idx;

      qsort(aruco_set, image_aruco_count, sizeof(aruco_set[0]), compare_int);

      char aname[3] = "a1";
      for (int i = 0; i < 5; i++) {
        aname[1] = i + '1';
        serial_commands.send_command(aname, aruco_set[i]);
      }

      if (cycle_count % 2 == 0) {
        serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
      } else {
        if (udp_on) serial_commands.send_command("IPR", Udp.remoteIP().toString().c_str());
      }

      #undef IRLINE_SENSOR
      #ifdef IRLINE_SENSOR
      serial_commands.send_command("IR0", robot.IRLine.IR_values[0]);
      serial_commands.send_command("IR1", robot.IRLine.IR_values[1]);
      serial_commands.send_command("IR2", robot.IRLine.IR_values[2]);
      serial_commands.send_command("IR3", robot.IRLine.IR_values[3]);
      serial_commands.send_command("IR4", robot.IRLine.IR_values[4]);
      if (robot.IRLine.sensor_count == 8) {
        serial_commands.send_command("IR5", robot.IRLine.IR_values[5]);
        serial_commands.send_command("IR6", robot.IRLine.IR_values[6]);
        serial_commands.send_command("IR7", robot.IRLine.IR_values[7]);
      }

      serial_commands.send_command("pr", robot.IRLine.pos_right);
      serial_commands.send_command("pc", robot.IRLine.pos_center);
      serial_commands.send_command("pcd", robot.IRLine.pos_center_digital);
      serial_commands.send_command("pl", robot.IRLine.pos_left);
      #endif

      //serial_commands.send_command("blk", robot.IRLine.blacks);

      #ifdef HAS_VL53L0X
      serial_commands.send_command("d0", robot.tof_dist);
      #endif

      #ifdef HAS_VL53L0X_X3
      serial_commands.send_command("d0", robot.tof_dist);
      serial_commands.send_command("dl", robot.tof_dist_left);
      serial_commands.send_command("dr", robot.tof_dist_right);
      #endif


      //serial_commands.send_command("m1", robot.PWM_1);
      //serial_commands.send_command("m2", robot.PWM_2);

      serial_commands.send_command("xe", robot.xe);
      serial_commands.send_command("ye", robot.ye);
      serial_commands.send_command("te", robot.thetae);
      //serial_commands.send_command("maw", robot.mean_abs_w);
      serial_commands.send_command("rvr", robot.v_req);
      serial_commands.send_command("rwr", robot.w_req);

      serial_commands.send_command("xa", aruco_robot_state.position.x);
      serial_commands.send_command("ya", aruco_robot_state.position.y);
      serial_commands.send_command("ta", aruco_robot_state.theta);

      pars_list.send_sparse_commands(serial_commands);

      Serial.print(" cmd: ");
      Serial.print(serial_commands.frame.command);
      Serial.print("; ");

      //robot.debug = serial_commands.out_count;
      //robot.debug = robot.align_index;

      serial_commands.send_command("dbg", image_aruco_count);
      //if (raw_aruco_count) {
      //  raw_aruco_count = 0;
      //}
      serial_commands.send_command("loop", micros() - loop_micros);

      serial_commands.flush();
      Serial.println();
    }
  }
}


// use the second core just to update the display

void setup1(void)
{
  display_setup();
}

void loop1(void)
{
  display_loop();
}
