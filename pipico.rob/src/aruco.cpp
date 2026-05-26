#include <stdint.h>
#include <Arduino.h>

#include "aruco.h"
#include "htransf_3d.h"
#include "actions.h"


// size in meters of something at 1m distance that matches one pixel in the image
static const float camera_intrinsic_pix_size = 0.001627f;

// robot position, result of calling aruco_compute_location
aruco_robot_state_t aruco_robot_state;


aruco_type_t get_aruco_type_from_id(int idx)
{
  switch (idx) {
    case MIN_ARUCO_IDX_FLOOR ... MAX_ARUCO_IDX_FLOOR:
      // these arucos don't actually exist on the floor, although they are in
      // the middle of the id range
      if (idx == 27 || idx == 28 || idx == 29)
        return ARUCO_INVALID;
      return ARUCO_FLOOR;

    case MIN_ARUCO_IDX_WALL ... MAX_ARUCO_IDX_WALL:
      return ARUCO_WALL;

    case BOX_ARUCO_BLUE:
    case BOX_ARUCO_GREEN:
    case BOX_ARUCO_RED:
      return ARUCO_BOX;

    default: return ARUCO_INVALID;
  }
}

// -----------------------------------------------------------------------------
//     serial data processing and structures that hold the processed data

// times received from the aruco sensor
int aruco_process_time;
int frame_time;

// arucos that have been received from the aruco sensor
aruco_image_t image_arucos[MAX_ARUCOS_PER_FRAME];
int image_aruco_count;


// code to perform special actions when certain arucos are seen
static void check_aruco_actions(int id)
{
  switch (id) {
    case ARUCO_ROBOT0_SET: robot.id_number = 0; break;
    case ARUCO_ROBOT1_SET: robot.id_number = 1; break;
    case ARUCO_ROBOT2_SET: robot.id_number = 2; break;

    case ARUCO_SET_SPEED_MIN ... ARUCO_SET_SPEED_MAX:
      //task_speed = (id - ARUCO_SET_SPEED_MIN) * ((0.85 - 0.1) / (ARUCO_SET_SPEED_MAX - ARUCO_SET_SPEED_MIN)) + 0.1;
      break;

    case ARUCO_START_PLAY:
      //force_referee("GRGB");
      break;
  }
}


static int read_14_bit(uint8_t *ptr)
{
  int16_t s = ((int)ptr[0] << 2) | ((int)ptr[1] << 9);
  return s >> 2;
}

//#include "gchannels.h"

static int send_debug;

//extern gchannels_t serial_commands;

void process_aruco_data(uint8_t b)
{
  const int max_packet_size = 1 + 2 * 2 + MAX_ARUCOS_PER_FRAME * 9 * 2;
  static uint8_t buffer[max_packet_size];
  static int pos = 0;

  // if we get a sync byte, start a packet
  if (b & 0x80) {
    buffer[0] = b;
    pos = 1;
    return;
  }

  // if we are getting data bytes, but we haven't started receiving the sync
  // byte, just ignore it
  if (pos == 0)
    return;

  if (pos >= max_packet_size) {
    pos = 0;
    return;
  }

  // add the byte to the buffer
  buffer[pos] = b;
  pos++;

  // check if the packet is complete
  int expected_size = 1 + 2 * 2 + (buffer[0] & 31) * (9 * 2);
  if (pos < expected_size)
    return;

  pos = 0;

  //char debug_msg[512], part[32];
  //debug_msg[0] = '\0';

  image_aruco_count = buffer[0] & 31;

  aruco_process_time = read_14_bit(buffer + 1) * 16;
  frame_time = read_14_bit(buffer + 3) * 16;

  for (int i = 0; i < image_aruco_count; i++) {
    uint8_t *ptr = buffer + 5 + i * 9 * 2;
    aruco_image_t &aruco = image_arucos[i];

    aruco.idx = read_14_bit(ptr);
    aruco.type = get_aruco_type_from_id(aruco.idx);

    //sprintf(part, "%d,", aruco.idx);
    //if (strlen(debug_msg) < 500)
    //  strcat(debug_msg, part);
    for (int j = 0; j < 4; j++) {
      aruco.pix[j].x = read_14_bit(ptr + 2 + j * 4) * (400.0 / 8191.0);
      aruco.pix[j].y = read_14_bit(ptr + 2 + j * 4 + 2) * (400.0 / 8191.0);

      //sprintf(part, "%d,%d,", read_14_bit(ptr + 2 + j * 4), read_14_bit(ptr + 2 + j * 4 + 2));
      //if (strlen(debug_msg) < 500)
      //  strcat(debug_msg, part);
    }

    check_aruco_actions(aruco.idx);
  }
  aruco_compute_location();

  /*if (send_debug) {
    send_debug--;
    serial_commands.send_command("msg", debug_msg);
  }*/
}



// ---------------------------------------------------------------
//     world configuration and helpers

struct floor_aruco_t {
  int idx;
  Vec2f center;
};

static const floor_aruco_t floor_arucos[] = {
  { 0,	{ -0.695,	0.355	} },
  { 1,	{ -0.545,	0.355	} },
  { 2,	{ -0.395,	0.355	} },
  { 3,	{ -0.245,	0.355	} },
  { 4,	{ 0.0,		0.355	} },
  { 5,	{ 0.695,	0.355	} },
  { 6,	{ 0.0,		0.150	} },
  { 7,	{ 0.227,	0.150	} },
  { 8,	{ 0.468,	0.150	} },
  { 9,	{ 0.695,	0.150	} },
  { 10,	{ -0.695,	0.0	} },
  { 11,	{ -0.468,	0.0	} },
  { 12,	{ -0.227,	0.0	} },
  { 13,	{ 0.0,		0.0	} },
  { 14,	{ 0.227,	0.0	} },
  { 15,	{ 0.468,	0.0	} },
  { 16,	{ 0.695,	0.0	} },
  { 17,	{ -0.695,	-0.150  } },
  { 18,	{ -0.468,	-0.150  } },
  { 19,	{ -0.227,	-0.150  } },
  { 20,	{ 0.0,		-0.150  } },
  { 21,	{ -0.695,	-0.355  } },
  { 22,	{ 0.0,		-0.355  } },
  { 23,	{ 0.245,	-0.355  } },
  { 24,	{ 0.395,	-0.355  } },
  { 25,	{ 0.545,	-0.355  } },
  { 26,	{ 0.695,	-0.355  } },
  { 27,	{ NAN,		NAN     } },
  { 28,	{ NAN,		NAN     } },
  { 29,	{ NAN,		NAN     } },
  { 30,	{ 0.227,	0.355	} },
  { 31,	{ 0.468,	0.355	} },
  { 32,	{ -0.468,	-0.355  } },
  { 33,	{ -0.227,	-0.355  } },
  { 34,	{ -0.695,	0.150	} },
  { 35,	{ 0.695,	-0.150  } },
};

static Vec3f get_floor_aruco_coord(int idx, int vertex)
{
  Vec2f ret = floor_arucos[idx].center;

  switch (vertex) {
    case 0: ret += Vec2f(-0.030,  0.030); break;
    case 1: ret += Vec2f( 0.030,  0.030); break;
    case 2: ret += Vec2f( 0.030, -0.030); break;
    case 3: ret += Vec2f(-0.030, -0.030); break;
  }
  return Vec3f(ret.x, ret.y, 0.0);
}

void get_floor_coordinates(int aruco, Vec2f &pt)
{
  pt = floor_arucos[aruco].center;
}
struct wall_aruco_t {
  int idx;
  Vec3f translate;
  Vec3f rotate;

  transform3d_t matrix;
  bool matrix_done;
};

static wall_aruco_t wall_arucos[] = {
  { 50, { -0.695,    0.565, 0.100 }, { radians( 90), radians(  0), radians(  0) } },
  { 51, { -0.545,    0.565, 0.100 }, { radians( 90), radians(  0), radians(  0) } },
  { 52, { -0.395,    0.565, 0.100 }, { radians( 90), radians(  0), radians(  0) } },
  { 53, { -0.245,    0.565, 0.100 }, { radians( 90), radians(  0), radians(  0) } },
  { 54, {  0.3375,   0.150, 0.100 }, { radians(  0), radians(-90), radians(-90) } },
  { 55, {  0.3575,   0.150, 0.100 }, { radians(  0), radians( 90), radians( 90) } },
  { 56, { -0.3575,     0.0, 0.100 }, { radians(  0), radians(-90), radians(-90) } },
  { 57, { -0.3375,     0.0, 0.100 }, { radians(  0), radians( 90), radians( 90) } },
  { 58, {  0.3375,     0.0, 0.100 }, { radians(  0), radians(-90), radians(-90) } },
  { 59, {  0.3575,     0.0, 0.100 }, { radians(  0), radians( 90), radians( 90) } },
  { 60, { -0.3575,  -0.150, 0.100 }, { radians(  0), radians(-90), radians(-90) } },
  { 61, { -0.3375,  -0.150, 0.100 }, { radians(  0), radians( 90), radians( 90) } },
  { 62, {  0.245,   -0.565, 0.100 }, { radians(-90), radians(  0), radians(180) } },
  { 63, {  0.395,   -0.565, 0.100 }, { radians(-90), radians(  0), radians(180) } },
  { 64, {  0.545,   -0.565, 0.100 }, { radians(-90), radians(  0), radians(180) } },
  { 65, {  0.695,   -0.565, 0.100 }, { radians(-90), radians(  0), radians(180) } },
};

static Vec3f transform_wall_aruco_coord(int door, const Vec3f &src)
{
  if (door < MIN_ARUCO_IDX_WALL	|| door > MAX_ARUCO_IDX_WALL)
    return Vec3f(0, 0, 0);

  wall_aruco_t &def = wall_arucos[door - MIN_ARUCO_IDX_WALL];

  if (!def.matrix_done) {
    def.matrix = translate3d(def.translate) * rot3d_x(def.rotate.x) * rot3d_y(def.rotate.y) * rot3d_z(def.rotate.z);
    def.matrix_done = true;
  }
  return def.matrix * src;
}

void get_door_coordinates(int door, float distance_from_wall, Vec2f &pt)
{
  Vec3f pt3d = transform_wall_aruco_coord(door, Vec3f(0, 0, distance_from_wall));
  pt = Vec2f(pt3d.x, pt3d.y);
}

static Vec3f get_wall_aruco_coord(int idx, int vertex)
{
  Vec3f src;
  switch (vertex) {
    case 0: src = Vec3f(-0.020,  0.020, 0.0); break;
    case 1: src = Vec3f( 0.020,  0.020, 0.0); break;
    case 2: src = Vec3f( 0.020, -0.020, 0.0); break;
    case 3: src = Vec3f(-0.020, -0.020, 0.0); break;
  }
  return transform_wall_aruco_coord(idx, src);
}

static Vec3f get_world_aruco_coord(int idx, int vertex)
{
  switch (get_aruco_type_from_id(idx)) {
    case ARUCO_FLOOR: return get_floor_aruco_coord(idx, vertex);
    case ARUCO_WALL: return get_wall_aruco_coord(idx, vertex);
    default: return Vec3f(0,0,0);
  }
}


// -----------------------------------------------------------------------------
//     robot position estimation

static void compute_rigid_transform_2d(const Vec2f *A, const Vec2f *B, int N,
				float &theta, float &ct, float &st, Vec2f &out_t)
{
	// 1. Compute centroids
	Vec2f a(0, 0), b(0, 0);

	for (int i = 0; i < N; i++) {
		a += A[i];
		b += B[i];
	}
	a /= N;
	b /= N;

	// 2. Compute Sxx and Sxy
	float Sxx = 0.0f;
	float Sxy = 0.0f;

	for (int i = 0; i < N; i++) {
		Vec2f ac = A[i] - a;
		Vec2f bc = B[i] - b;
		Sxx += ac.x * bc.x + ac.y * bc.y;
		Sxy += ac.x * bc.y - ac.y * bc.x;
	}

	// 3. Compute rotation angle
	theta = atan2(Sxy, Sxx);

	ct = cos(theta);
	st = sin(theta);

	// 4. Compute translation
	// t = centroid_B - R * centroid_A
	out_t.x = b.x - (ct * a.x - st * a.y);
	out_t.y = b.y - (st * a.x + ct * a.y);
}

Vec2f transform_point(const Vec2f& p, float cr, float sr, const Vec2f& t)
{
	Vec2f r;
	r.x = cr * p.x - sr * p.y + t.x;
	r.y = sr * p.x + cr * p.y + t.y;
	return r;
}


void camera_pars_t::update_matrix(void)
{
  camera_rot = rot3d_y(rot_y) * rot3d_x(rot_x);
}

camera_pars_t camera_pars;


// structure used to describe from in the image arucos where each world vertex
// comes from
struct vertex_id_t {
  float z;
  int image_aruco_vec_idx;  // index
  int vertex;
};

// structures to hold the vertices to register
static Vec2f world_vertices[MAX_ARUCOS_PER_FRAME * 4];
static vertex_id_t world_vertices_src[MAX_ARUCOS_PER_FRAME * 4];
static Vec2f image_vertices[MAX_ARUCOS_PER_FRAME * 4];
static int valid_vertices;
static bool image_vertices_valid;

// take aruco ids from image and build array of world vertices
static void build_world_vertices(void)
{
  int i, j, used;

  valid_vertices = 0;
  for (i = 0; i < image_aruco_count; i++) {

    switch (image_arucos[i].type) {
      case ARUCO_FLOOR: used = 0b1111; break;
      case ARUCO_WALL: used = 0b1100; break;
      default: used = 0; break;
    }

    for (j = 0; j < 4; j++) {
      if ((used & (1 << j)) == 0)
        continue;
      Vec3f pt = get_world_aruco_coord(image_arucos[i].idx, j);
      world_vertices[valid_vertices] = Vec2f(pt.x, pt.y);
      vertex_id_t &vid = world_vertices_src[valid_vertices];
      vid.z = pt.z;
      vid.image_aruco_vec_idx = i;
      vid.vertex = j;
      valid_vertices++;
    }
  }
}

// this function converts a image coordinate into the robot coordinate system,
// if we know the Z coordinate of the point in the world. This can be useful to
// track boxes relative to the robot
Vec3f image_vertex_to_robot_coord(const Vec2f &pt, float z)
{
  //Vec3f p = { 1.0, (pt.x - camera_pars.crop_offset) * 0.001627f, pt.y * 0.001627f };  // original camera
  Vec3f p = { 1.0, (pt.x - camera_pars.crop_offset) * 0.00251f, pt.y * 0.00251f };  // 100 degree camera

  p = camera_pars.camera_rot * p;
  // if even one of the vertices can not be mapped to the
  // floor, then this transformation is not good
  if (p.z >= 0)
    return Vec3f(NAN, NAN, NAN);

  p *= -(camera_pars.pos_z - z) / p.z;
  p.x += camera_pars.offset_from_axis;

  return p;
}

// take aruco info from image and build array of vertices in the camera world,
// using the current camera parameters
static void build_image_vertices(void)
{
  int vertex_idx = 0;

  image_vertices_valid = false;

  for (vertex_idx = 0; vertex_idx < valid_vertices; vertex_idx++) {
    const vertex_id_t &vid = world_vertices_src[vertex_idx];
    const aruco_image_t &cam = image_arucos[vid.image_aruco_vec_idx];

    Vec3f p = image_vertex_to_robot_coord(cam.pix[vid.vertex], vid.z);

    // if even one of the vertices can not be mapped to the
    // floor, then this transformation is not good
    if (isnan(p.x))
      return;

    image_vertices[vertex_idx] = { p.x, p.y };
  }
  image_vertices_valid = true;
}


// take the aruco data from the aruco sensor and compute the robot coordinates
void aruco_compute_location(void)
{
  float ct, st;
  Vec2f out_t;

  build_world_vertices();
  build_image_vertices();

  if (valid_vertices < 2 || image_vertices_valid == false)
    return;

  compute_rigid_transform_2d(image_vertices, world_vertices, valid_vertices, aruco_robot_state.theta, ct, st, out_t);

  aruco_robot_state.theta = normalize_angle(aruco_robot_state.theta + camera_pars.rot_z);

  Vec2f previous = aruco_robot_state.position;
  aruco_robot_state.position = transform_point(Vec2f(0,0), ct, st, out_t);
  aruco_robot_state.updated = 1;
  aruco_robot_state.point_count = valid_vertices;

  if (previous.distance(aruco_robot_state.position) > 0.1)
    send_debug = 5;
}


// -----------------------------------------------------------------------------
//     camera position calibration (registration methods)

// optimizer search space definition
static float grid_range[3][2] = {
  { radians(0), radians(50) },		// up-down angle
  { radians(-25), radians(25) },	// horizontal tilt
  { 0.05, 0.35 }, 		// camera Z
};


// evaluate one point of the search space for the optimizer
static float evaluate(float rot_y, float rot_x, float z)
{
  // use the parameters as camera parameters
  camera_pars.rot_y = rot_y;
  camera_pars.rot_x = rot_x;
  camera_pars.pos_z = z;
  camera_pars.offset_from_axis = 0;
  camera_pars.crop_offset = 0;

  camera_pars.update_matrix();

  // build the image vertices with these camera parameters
  build_image_vertices();

  // if this produces invalid vertices, the match is super bad
  if (image_vertices_valid == false)
    return 1e9;

  // compute best rotation / translation to camera points
  float rotation, cr, sr;
  Vec2f translation;
  compute_rigid_transform_2d(image_vertices, world_vertices, valid_vertices, rotation, cr, sr, translation);

  // apply it to the camera points
  for (int i = 0; i < valid_vertices; i++)
    image_vertices[i] = transform_point(image_vertices[i], cr, sr, translation);

  // compute average error
  float error = 0.0;
  for (int i = 0; i < valid_vertices; i++)
    error += image_vertices[i].squareDistance(world_vertices[i]);

  return error;
}

static float grid_to_coord(float x, int idx)
{
  return grid_range[idx][0] + (grid_range[idx][1] - grid_range[idx][0]) * x;
}

static void evaluate_grid(float center[3], float grid_size)
{
  //const int grid = 3;
  //const float big_step = 0.666666;
  const int grid = 5;
  const float big_step = 0.4;

  const float step = grid_size / grid;
  float c[3], error, best_error;
  int best[3];

  const float start[3] = {
    center[0] - grid_size * 0.5f,
    center[1] - grid_size * 0.5f,
    center[2] - grid_size * 0.5f,
  };

  best_error = 1e9;
  for (int i0 = 0; i0 <= grid; i0++) {
    c[0] = grid_to_coord(start[0] + i0 * step, 0);

    for (int i1 = 0; i1 <= grid; i1++) {
      c[1] = grid_to_coord(start[1] + i1 * step, 1);

      for (int i2 = 0; i2 <= grid; i2++) {
        c[2] = grid_to_coord(start[2] + i2 * step, 2);

        error = evaluate(c[0], c[1], c[2]);
        if (error < best_error) {
          best_error = error;
          best[0] = i0;
          best[1] = i1;
          best[2] = i2;
        }
      }
    }
  }

  float new_center[3];
  for (int i = 0; i < 3; i++)
    new_center[i] = start[i] + best[i] * step;

  if (grid_size < 0.001) {
    camera_pars.rot_y = grid_to_coord(new_center[0], 0);
    camera_pars.rot_x = grid_to_coord(new_center[1], 1);
    camera_pars.pos_z = grid_to_coord(new_center[2], 2);
  } else {
    grid_size *= big_step;
    evaluate_grid(new_center, grid_size);
  }
}


void calib_camera(void)
{
  if (image_aruco_count < 3)
    return;

  // run the optimizer to find camera parameters, start with the whole cube
  float center[3] = { 0.5, 0.5, 0.5 };
  evaluate_grid(center, 1.0);

  //TODO: use a procedure where we place the robot in a known world location to
  // calibrate the camera offset
  // compute the robot location and use that to set the camera offset
  //aruco_compute_location();

  // for now set a constant
  //camera_pars.offset_from_axis = 0.05;
}

