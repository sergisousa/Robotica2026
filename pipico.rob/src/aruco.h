#ifndef ARUCO_H
#define ARUCO_H

#include <stdint.h>
#include <VectorXf.h>
#include "htransf_3d.h"

enum {
  MIN_ARUCO_IDX_FLOOR	=  0,
  MAX_ARUCO_IDX_FLOOR	= 35,

  MIN_ARUCO_IDX_WALL	= 50,
  MAX_ARUCO_IDX_WALL	= 65,

  BOX_ARUCO_BLUE      = 70,
  BOX_ARUCO_GREEN     = 71,
  BOX_ARUCO_RED       = 72,

  ARUCO_ROBOT0_SET    = 80,

  ARUCO_ROBOT1_BACK   = 75,
  ARUCO_ROBOT1_SIDE   = 76,
  ARUCO_ROBOT1_SET    = 81,

  ARUCO_ROBOT2_BACK   = 77,
  ARUCO_ROBOT2_SIDE   = 78,
  ARUCO_ROBOT2_SET    = 82,

  ARUCO_SET_SPEED_MIN = 83,
  ARUCO_SET_SPEED_MAX = 88,

  ARUCO_START_PLAY    = 89,
};

// maximum number of arucos we can receive per frame
#define MAX_ARUCOS_PER_FRAME  32

// height of a box in meters
#define BOX_HEIGHT  0.064

// height of the bottom of a robot aruco marker in meters
#define ROBOT_MARKER_HEIGHT  0.065


// configuration parameters
struct camera_pars_t {
  float rot_y;
  float rot_x;
  float pos_z;
  float offset_from_axis;
  float crop_offset;
  float rot_z;

  // this matrix is used to speed up location operations. When any parameter is
  // changed, "update_matrix" must be called to update it
  transform3d_t camera_rot;

  void update_matrix(void);
};
extern camera_pars_t camera_pars;

// times in us received from the aruco sensor
extern int aruco_process_time;
extern int frame_time;

// aruco classification (derived from aruco id)
enum aruco_type_t {
  ARUCO_FLOOR,
  ARUCO_WALL,
  ARUCO_BOX,
  ARUCO_INVALID,
};
aruco_type_t get_aruco_type_from_id(int idx);


// coordinates in image space received from the aruco sensor
struct aruco_image_t {
  // received data
  int idx;
  Vec2f pix[4];

  // derived data
  aruco_type_t type;
};
extern aruco_image_t image_arucos[MAX_ARUCOS_PER_FRAME];
extern int image_aruco_count;

// robot location computed by aruco_compute_location
struct aruco_robot_state_t {
  Vec2f position;
  float theta;
  int updated; // set to 1 when updated
  int point_count; // number of aruco vertices used to estimate state
};
extern aruco_robot_state_t aruco_robot_state;

// process one byte of incoming data form the aruco sensor serial port
void process_aruco_data(uint8_t b);

// take the aruco data from the aruco sensor and compute the robot coordinates
void aruco_compute_location(void);

// use aruco data to calibrate the camera parameters
void calib_camera(void);

// use the aruco world coordinates to compute a coordinate in front of a
// warehouse or machine. The door number is the aruco id
void get_door_coordinates(int door, float distance_from_wall, Vec2f &pt);

// get the center coordinates of a floor aruco
void get_floor_coordinates(int aruco, Vec2f &pt);

// this function converts a image coordinate into the robot coordinate system,
// if we know the Z coordinate of the point in the world. This can be useful to
// track boxes relative to the robot
Vec3f image_vertex_to_robot_coord(const Vec2f &pt, float z);

#endif
