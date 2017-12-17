// Mars lander simulator
// Version 1.8
// Header for other data types
// Gabor Csanyi and Andrew Gee, October 2014

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef __OTHER_DATA_TYPES_INCLUDED__
#define __OTHER_DATA_TYPES_INCLUDED__

#include "vector3d.h"

using namespace std;

// Data type for recording lander's previous positions
struct track_t {
  unsigned short n;
  unsigned short p;
  vector3d pos[N_TRACK];
};

// Quaternions for orbital view transformation
struct quat_t {
  vector3d v;
  double s;
};

// Data structure for the state of the close-up view's coordinate system
struct closeup_coords_t {
  bool initialized;
  bool backwards;
  vector3d right;
};

// Enumerated data types
enum parachute_status_t { NOT_DEPLOYED = 0, DEPLOYED = 1, LOST = 2 };
enum lander_phases {let_it_be, chariots_of_fire, the_sound_of_silence, viva_la_vida, let_it_go}; // current state of lander
enum autopilot_modes {descent_mode, transfer_mode, maintain_mode, launch_mode}; // current autopilot mode
enum manual_attitude_command {roll_command, pitch_command, yaw_command, stabilize_command, reset_command}; // for manual attitude control

#endif
