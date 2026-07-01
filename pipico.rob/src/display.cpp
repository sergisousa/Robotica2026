#include <WiFi.h>
#include <Arduino.h>
#include "display.h"
#include "aruco.h"
#include "robot.h"

#include "pico4drive.h"
extern pico4drive_t pico4drive;

#define SSD1306_NO_SPLASH
#define I2C_BUFFER_LENGTH   256
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, 2500000, 400000);



volatile int current_screen;


void display_setup(void)
{
  // initialize the I2C bus to talk to the display
  const int I2C1_SDA = 26;
  const int I2C1_SCL = 27;
  pinMode(I2C1_SDA, INPUT_PULLUP);
  pinMode(I2C1_SCL, INPUT_PULLUP);

  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();

  delay(20);

  // initialize the display
  while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    delay(500);  // Wait to try again
  }

  // clear the display
  display.clearDisplay();
  display.display();
}

static int have_ip;
static char ip_address_str[32];

static uint32_t update_time;

// local copy of the arucos to reduce the possibility of they being updated
// while we are displaying them
static aruco_image_t local_arucos[MAX_ARUCOS_PER_FRAME];
static int local_aruco_count;

static void display_main(void)
{
  // display robot coordinates
  display.printf("X %7.3f\n", robot.xe);
  display.printf("Y %7.3f\n", robot.ye);
  display.printf("T %7.1f\n", degrees(robot.thetae));
  //display.printf("V %7.2f\n\n", task_speed);

  // display robot number, on a white background if the wifi is connected
  if (ip_address_str[0] == '\0') {
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(19, 42, 20, 20, 1);
  } else {
    display.setTextColor(SSD1306_BLACK);
    display.fillRect(19, 42, 20, 20, 1);
  }
  display.setTextSize(2);
  display.setCursor(24, 45);
  display.printf("%d", robot.id_number);

  // draw the aruco edges
  for (int i = 0; i < local_aruco_count; i++) {
    int pt[4][2];
    for (int j = 0; j < 4; j++) {
      pt[j][0] = round(local_arucos[i].pix[j].x * (-32.0 / 200.0) + 96.0);
      pt[j][1] = round(local_arucos[i].pix[j].y * (-32.0 / 200.0) + 32.0);
    }
    for (int j = 0; j < 4; j++)
      display.drawLine(pt[j][0], pt[j][1], pt[(j+1)&3][0], pt[(j+1)&3][1], 1);
    display.fillRect(pt[0][0] - 1, pt[0][1] - 1, 3, 3, 1);
  }
  display.drawLine(63, 0, 63, 63, 1);
}


static void world_to_map(Vec2f w, int m[2])
{
  const float scale = 64 / 1.2; // screen height / field height
  m[0] = (w.x * scale + 64.5);
  m[1] = (-w.y * scale + 32.5);
}

static void draw_aruco(int id, int seen)
{
  aruco_type_t type = get_aruco_type_from_id(id);
  if (type == ARUCO_INVALID || type == ARUCO_BOX)
    return;

  Vec2f pt;
  if (type == ARUCO_FLOOR)
    get_floor_coordinates(id, pt);
  else
    get_door_coordinates(id, 0.0, pt);

  int p[2];
  world_to_map(pt, p);

  if (seen)
    display.fillRect(p[0] - 1, p[1] - 1, 3, 3, 1);
  else
    display.drawPixel(p[0], p[1], 1);
}

static void display_map()
{
  // display all arucos as "not seen"
  for (int i = MIN_ARUCO_IDX_FLOOR; i <= MAX_ARUCO_IDX_WALL; i++)
    draw_aruco(i, 0);

  // display the arucos being seen as "seen"
  for (int i = 0; i < local_aruco_count; i++)
    draw_aruco(local_arucos[i].idx, 1);

  // draw the robot as a small arrow
  int pt[3][2];
  Vec2f r(robot.xe, robot.ye);
  float t = robot.thetae;
  Vec2f fwd(cos(t), sin(t));
  Vec2f perp(-fwd.y, fwd.x);

  world_to_map(r + fwd * 0.15, pt[0]);
  world_to_map(r + perp * 0.05, pt[1]);
  world_to_map(r - perp * 0.05, pt[2]);

  for (int j = 0; j < 3; j++) {
    if (pt[j][0] < -10 || pt[j][0] >= 138)
      return;
    if (pt[j][1] < -10 || pt[j][1] >= 74)
      return;
  }

  for (int j = 0; j < 3; j++)
    display.drawLine(pt[j][0], pt[j][1], pt[(j+1)%3][0], pt[(j+1)%3][1], 1);

  // display robot number
  display.setCursor(0, 28);
  display.printf("%d", robot.id_number);

  // display a small rect when we are waiting for the referee
  if (robot.pfsm->state == 301)
    display.fillRect(0, 0, 4, 16, 1);

  // display a small rect when we are connected to WiFi
  if (ip_address_str[0] != '\0')
    display.fillRect(0, 56, 4, 8, 1);

}

static void display_data(void)
{
    display.printf("IP: %s\n", ip_address_str);
    display.printf("Vbat: %.2f V\n", pico4drive.battery_voltage);

    display.printf("\nRX: %.1f\n", degrees(camera_pars.rot_x));
    display.printf("RY: %.1f\n", degrees(camera_pars.rot_y));
    display.printf("RZ: %.2f\n", degrees(camera_pars.rot_z));
    display.printf("PZ: %.3f\n", camera_pars.pos_z);
    display.printf("OX: %.3f\n", camera_pars.offset_from_axis);
}


void display_loop(void)
{
  static uint32_t last_cycle;

  // get th IP address string only once after connecting
  if (!have_ip && WiFi.isConnected()) {
    have_ip = 1;
    strcpy(ip_address_str, WiFi.localIP().toString().c_str());
  }

  // do this only every "interval" microseconds
  uint32_t now = micros();
  uint32_t delta = now - last_cycle;
  if (delta >= 20000) {
    last_cycle = now;

    // OLED output
    display.clearDisplay();

    // copy the image arucos to a local structure and repeat the copy if the
    // number of arucos changed in between. Different screens may want to show
    // aruco information, so just do it here once
    do {
      local_aruco_count = image_aruco_count;
      memcpy(local_arucos, image_arucos, sizeof(image_arucos[0]) * local_aruco_count);
    } while (image_aruco_count != local_aruco_count);

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner

    switch (current_screen) {
      case SCREEN_MAIN: display_main(); break;
      case SCREEN_MAP: display_map(); break;
      case SCREEN_DATA: display_data(); break;
    }
    display.display();
  }
}
