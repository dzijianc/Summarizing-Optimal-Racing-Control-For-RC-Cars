// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/vehicle/driver.hpp>
#include <webots/camera.hpp>
#include <webots/device.hpp>
#include <webots/display.hpp>
#include <webots/gps.hpp>
#include <webots/keyboard.hpp>
#include <webots/lidar.hpp>
#include <webots/robot.hpp>
#include "Eigen/Eigen"

#include <math.h>
#include <stdio.h>
#include <string.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Line following PID
#define KP 1
#define KI 0.01
#define KD 2

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

// camera
Camera *camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
Lidar *sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
Display *display;
int display_width = 0;
int display_height = 0;
ImageRef *speedometer_image;

// GPS
GPS *gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool PID_need_reset = false;

void setupVariables() {
  /*
  Bicycle model:
  beta = steering angle
  r
  e
  delta_psi



  a = 1.5
  b = 1.5
  k_p - .02
  I_z = 4346 (from Inertia matrix)
  x_LA = camera.far
  C_R = 160 
  C_F = 180
  U_x = gps.velocity[0]
  m = 2000kg
  */
}

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

void update_display(Driver *driver) {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  display->imagePaste(speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = driver->getCurrentSpeed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  display->drawLine(100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  display->drawText(txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  display->drawText(txt, 10, 140);
}

void compute_gps_speed() {
  const double *coords = gps->getValues();
  const double speed_ms = gps->getSpeed();
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

  // check if there is a SICK and a display
  int j = 0;
  for (j = 0; j < driver->getNumberOfDevices(); ++j) {
    Device *device = driver->getDeviceByIndex(j);
    const std::string name = device->getName();
    if (name.compare("display") == 0)
      enable_display = true;
    else if (name.compare("gps") == 0)
      has_gps = true;
    else if (name.compare("camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = (Camera *)driver->getDevice("camera");
    camera->enable(TIME_STEP);
    camera_width = camera->getWidth();
    camera_height = camera->getHeight();
    camera_fov = camera->getFov();
  }

  // initialize gps
  if (has_gps) {
    gps = (GPS *) driver->getDevice("gps");
    gps->enable(TIME_STEP);
  }

  // initialize display (speedometer)
  if (enable_display) {
    display = (Display *)driver->getDevice("display");
    speedometer_image = display->imageLoad("speedometer.png");
  }

  // start engine
  if (has_camera)
    set_speed(driver, 50.0);  // km/h

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (driver->step() != -1) {
    static int i = 0;

    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / driver->getBasicTimeStep()) == 0) {
      // read sensors
      const unsigned char *camera_image = NULL;
      if (has_camera)
        camera_image = camera->getImage();

      if (has_camera) {
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));

        // avoid obstacles and follow yellow line
        if (yellow_line_angle != UNKNOWN) {
          // no obstacle has been detected, simply follow the line
          driver->setBrakeIntensity(0.0);
          set_steering_angle(driver, applyPID(yellow_line_angle));
        } else {
          // no obstacle has been detected but we lost the line => we brake and hope to find the line again
          driver->setBrakeIntensity(0.4);
          PID_need_reset = true;
        }
      }

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display(driver);
    }
    ++i;
  };

  // Enter here exit cleanup code.

  delete driver;
  return 0;
}
