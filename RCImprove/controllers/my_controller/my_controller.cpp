// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/vehicle/driver.h>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>

#include <math.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool PID_need_reset = false;

void set_speed(Driver *driver, double kmh) {
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  driver->setCruisingSpeed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(Driver *driver, double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  driver->setSteeringAngle(wheel_angle);
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double *obstacle_dist) {
  const int HALF_AREA = 20;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 20.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display(Driver *driver) {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed_ms = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Driver *driver = new Driver();

  // get the time step of the current world.
  // int timeStep = (int)driver->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (driver->step() != -1) {
    set_speed(driver, 50.0);
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  // delete driver;
  return 0;
}
