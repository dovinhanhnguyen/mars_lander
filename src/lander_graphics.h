// Mars lander simulator
// Version 1.8
// Graphics header
// Gabor Csanyi and Andrew Gee, October 2014

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

// Some functions adapted from freeglut_geometry.c, which is covered by the
// following license:
//
// Copyright (c) 1999-2000 Pawel W. Olszta. All Rights Reserved.
// Written by Pawel W. Olszta, <olszta@sourceforge.net>
// Creation date: Fri Dec 3 1999
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// PAWEL W. OLSZTA BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Some functions adapted from trackball.cpp by Gavin Bell, which is covered by
// the following license:
//
// (c) Copyright 1993, 1994, Silicon Graphics, Inc.
// ALL RIGHTS RESERVED
// Permission to use, copy, modify, and distribute this software for
// any purpose and without fee is hereby granted, provided that the above
// copyright notice appear in all copies and that both the copyright notice
// and this permission notice appear in supporting documentation, and that
// the name of Silicon Graphics, Inc. not be used in advertising
// or publicity pertaining to distribution of the software without specific,
// written prior permission.
//
// THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"
// AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,
// INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR
// FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL SILICON
// GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,
// SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY
// KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,
// LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF
// THIRD PARTIES, WHETHER OR NOT SILICON GRAPHICS, INC.  HAS BEEN
// ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE
// POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.
//
// US Government Users Restricted Rights
// Use, duplication, or disclosure by the Government is subject to
// restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
// (c)(1)(ii) of the Rights in Technical Data and Computer Software
// clause at DFARS 252.227-7013 and/or in similar or successor
// clauses in the FAR or the DOD or NASA FAR Supplement.
// Unpublished-- rights reserved under the copyright laws of the
// United States.  Contractor/manufacturer is Silicon Graphics,
// Inc., 2011 N.  Shoreline Blvd., Mountain View, CA 94039-7311.
//
// OpenGL(TM) is a trademark of Silicon Graphics, Inc.
//
// SOIL (Simple OpenGL Image Library) is in the public domain.
// http://www.lonesock.net/soil.html

#ifndef __LANDER_GRAPHICS_1_INCLUDED__
#define __LANDER_GRAPHICS_1_INCLUDED__

#include "global_1.h"
#include "global_2.h"

// GLUT mouse wheel operations work under Linux only
#if !defined (GLUT_WHEEL_UP)
#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
#endif

using namespace std;

// GL windows and objects
int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
GLUquadricObj *quadObj;
GLuint terrain_texture, closeup_mars_texture, orbital_mars_texture, closeup_background_texture, orbital_background_texture; // texture handles
short throttle_control;
track_t track, track_Phobos, track_Deimos;
bool texture_available;

// obj model for Mars terrain
Model_obj mars_model;

// Simulation parameters
bool help = false;
bool paused = false;
bool landed = false;
bool crashed = false;
int last_click_x = -1;
int last_click_y = -1;
short simulation_speed = 5;
double delta_t, simulation_time;
unsigned short scenario = 0;
string scenario_description[10];
bool static_lighting = false;
closeup_coords_t closeup_coords;
float randtab[N_RAND];
bool do_texture = true;
unsigned long throttle_buffer_length, throttle_buffer_pointer;
double *throttle_buffer = NULL;
unsigned long long time_program_started;

// Lander state - the visualization routines use velocity_from_positions, so not sensitive to 
// any errors in the velocity update in numerical_dynamics
vector3d position, orientation, velocity, velocity_from_positions, last_position;
vector3d out_axis, left_axis, up_axis; // for manual attitude control
vector3d previous_out, previous_left, previous_up; // for manual attitude control
Orbiting_object Phobos, Deimos;
Kepler_solver lander_Kepler, Phobos_Kepler, Deimos_Kepler;
double climb_speed, ground_speed, altitude, throttle, fuel;
bool stabilized_attitude, stabilized_attitude_in_plane_wrt_mars, autopilot_enabled, parachute_lost;
bool rotation_on, steady_wind_on, gust_wind_on; // for modelling planet rotation and wind
bool moon_effect_on; // gravitational effect of Phobos & Deimos on lander
bool display_predicted_trajectory; // lander predicted trajectory on/off
bool second_control_panel_on;// for switching between control panels
bool accept_input_altitude; // for user input altitude
bool lander_unheld; // for launching lander
int input_altitude; // for user input altitude
double gust_speed; // for modelling planet rotation and wind
parachute_status_t parachute_status;
lander_phases current_lander_phase;
autopilot_modes current_autopilot_mode;
manual_attitude_command input_attitude_command;
double stabilized_attitude_angle;
double input_attitude_angle; // for manual attitude control

// Orbital and closeup view parameters
double orbital_zoom, save_orbital_zoom, closeup_offset, closeup_xr, closeup_yr, terrain_angle;
quat_t orbital_quat;

// For GL lights
GLfloat plus_y[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat minus_y[] = { 0.0, -1.0, 0.0, 0.0 };
GLfloat plus_z[] = { 0.0, 0.0, 1.0, 0.0 };
GLfloat top_right[] = { 1.0, 1.0, 1.0, 0.0 };
GLfloat straight_on[] = { 0.0, 0.0, 1.0, 0.0 };

// For Irrklang sound effects
#ifndef WIN32
#ifndef __APPLE__
irrklang::ISoundEngine *SoundEngine;
irrklang::ISound *theme_sound, *landing_theme_sound, *thruster_sound, *space_sound, *wind_sound;
bool sound_on;
#endif
#endif

#endif
