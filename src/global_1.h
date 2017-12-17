// Mars lander simulator
// Version 1.8
// Global variables 1 header
// Gabor Csanyi and Andrew Gee, October 2014

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef __GLOBAL_1_INCLUDED__
#define __GLOBAL_1_INCLUDED__

#if defined (__MINGW32__) && !defined (WIN32)
#define WIN32
#endif

#ifdef WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#ifndef WIN32
#ifndef __APPLE__
#include <irrklang/irrKlang.h>
#endif
#endif
#include <SOIL.h>

#include "define_constants.h"
#include "vector3d.h"
#include "kepler_solver.h"
#include "orbiting_object.h"
#include "model_obj.h"
#include "other_data_types.h"

using namespace std;

extern bool stabilized_attitude, stabilized_attitude_in_plane_wrt_mars, autopilot_enabled;
extern bool rotation_on, steady_wind_on, gust_wind_on; // for modelling planet rotation and wind
extern bool moon_effect_on; // gravitational effect of Phobos & Deimos on lander
extern bool lander_unheld; // for launching lander
extern double throttle, fuel;
extern double gust_speed; // for modelling planet rotation and wind
extern unsigned short scenario;
extern string scenario_description[];
extern vector3d position, orientation, velocity;
extern vector3d previous_out, previous_left, previous_up; // for manual attitude control
extern vector3d out_axis, left_axis, up_axis; // for attitude control
extern Orbiting_object Phobos, Deimos;
extern Kepler_solver lander_Kepler, Phobos_Kepler, Deimos_Kepler;
extern parachute_status_t parachute_status;
extern lander_phases current_lander_phase;
extern autopilot_modes current_autopilot_mode;
extern manual_attitude_command input_attitude_command;
extern closeup_coords_t closeup_coords; // for manual attitude control
extern double stabilized_attitude_angle;
extern double input_attitude_angle; // for manual attitude control
extern bool accept_input_altitude; // for user input altitude
extern int input_altitude; // for user input altitude

// Function prototypes
void invert (double m[], double mout[]);
void xyz_euler_to_matrix (vector3d ang, double m[]);
vector3d matrix_to_xyz_euler (double m[]);
void normalize_quat (quat_t &q);
quat_t axis_to_quat (vector3d a, const double phi);
double project_to_sphere (const double r, const double x, const double y);
quat_t add_quats (quat_t q1, quat_t q2);
void quat_to_matrix (double m[], const quat_t q);
quat_t track_quats (const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time (unsigned long long &t);
void fghCircleTable (double **sint, double **cost, const int n);
void glutOpenHemisphere (GLdouble radius, GLint slices, GLint stacks);
void glutMottledSphere (GLdouble radius, GLint slices, GLint stacks);
void glutCone (GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void enable_lights (void);
void setup_lights (void);
void glut_print (float x, float y, string s);
double atmospheric_density (vector3d pos);
void draw_dial (double cx, double cy, double val, string title, string units);
void draw_control_bar (double tlx, double tly, double val, double red, double green, double blue, string title);
void draw_indicator_lamp (double tcx, double tcy, string off_text, string on_text, bool on);
void draw_instrument_window (void);
void display_help_arrows (void);
void display_help_prompt (void);
void display_help_text (void);
void draw_orbital_window (void);
void draw_parachute_quad (double d);
void draw_parachute (double d);
bool generate_terrain_texture (void);
void update_closeup_coords (void);
void draw_closeup_window (void);
void draw_main_window (void);
void refresh_all_subwindows (void);
bool safe_to_deploy_parachute (void);
void update_visualization (void);
void attitude_stabilization (void);
vector3d thrust_wrt_world (void);
void autopilot (void);
void numerical_dynamics (void);
void initialize_simulation (void);
void update_lander_state (void);
void reset_simulation (void);
void set_orbital_projection_matrix (void);
void reshape_main_window (int width, int height);
void orbital_mouse_button (int button, int state, int x, int y);
void orbital_mouse_motion (int x, int y);
void closeup_mouse_button (int button, int state, int x, int y);
void closeup_mouse_motion (int x, int y);
void glut_special (int key, int x, int y);
void glut_key (unsigned char k, int x, int y);

// More function prototypes
double current_lander_mass (void);
vector3d acceleration_drag (void);
vector3d acceleration_gravity (void);
vector3d acceleration (void);
double weibull_random_number (void);
vector3d mars_velocity_wrt_world (double distance_from_centre, bool surface_velocity);
vector3d rodrigues_rotation (vector3d n, vector3d rotation_axis, double rotation_angle);
void draw_future_trajectory (Kepler_solver object_Kepler, float colour_red, float colour_green, float colour_blue, string s);
void draw_future_trajectory_closeup (Kepler_solver object_Kepler, float colour_red, float colour_green, float colour_blue);
vector3d matrix_times_vector (double m[], vector3d n);
void glut_print_3d (float x, float y, float z, string s);
void draw_attitude_indicator (double cx, double cy, double val, double val_2, string title, string title_2, string units);
void draw_smaller_indicator_lamp (double tcx, double tcy, string off_text, string on_text, bool on);
void draw_input_altitude_lamp (double tcx, double tcy, double val, string title, string units, bool on);
void draw_lander_phase_lamp (double tcx, double tcy, string text, string title, bool on);
bool setup_texture (string filename, GLuint &id);

#endif
