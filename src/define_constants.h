// Mars lander simulator
// Version 1.8
// Various constants
// Gabor Csanyi and Andrew Gee, October 2014

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#ifndef __DEFINE_CONSTANTS_INCLUDED__
#define __DEFINE_CONSTANTS_INCLUDED__

// Graphics constants
#define GAP 5
#define SMALL_NUM 0.0000001
#define N_RAND 20000
#define PREFERRED_WIDTH 1024
#define PREFERRED_HEIGHT 768
#define MIN_INSTRUMENT_WIDTH 1024
#define INSTRUMENT_HEIGHT 300
#define GROUND_LINE_SPACING 20.0
#define CLOSEUP_VIEW_ANGLE 30.0
#define TRANSITION_ALTITUDE 10000.0
#define TRANSITION_ALTITUDE_NO_TEXTURE 4000.0
#define TERRAIN_TEXTURE_SIZE 1024
#define INNER_DIAL_RADIUS 65.0
#define OUTER_DIAL_RADIUS 75.0
#define MAX_DELAY 160000
#define N_TRACK 1000
#define TRACK_DISTANCE_DELTA 100000.0
#define TRACK_ANGLE_DELTA 0.999
#define HEAT_FLUX_GLOW_THRESHOLD 1000000.0

// Mars constants
#define MARS_RADIUS 3386000.0 // (m)
#define MARS_MASS 6.42E23 // (kg)
#define GRAVITY 6.673E-11 // (m^3/kg/s^2)
#define MARS_DAY 88642.65 // (s)
#define EXOSPHERE 200000.0 // (m)

// Mars moons constants
#define PHOBOS_RADIUS 11100.0 // (m)
#define PHOBOS_MASS 1.07E16 // (kg)
#define DEIMOS_RADIUS 6200.0 // (m)
#define DEIMOS_MASS 1.48E15 // (kg)

// Lander constants
#define LANDER_SIZE 1.0 // (m)
#define UNLOADED_LANDER_MASS 100.0 // (kg)
#define FUEL_CAPACITY 400.0 // (l) used to be 100.0
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
#define FUEL_DENSITY 1.0 // (kg/l)
// MAX_THRUST, as defined below, is 1.5 * weight of fully loaded lander at surface
#define MAX_THRUST (1.5 * (FUEL_DENSITY*FUEL_CAPACITY+UNLOADED_LANDER_MASS) * (GRAVITY*MARS_MASS/(MARS_RADIUS*MARS_RADIUS))) // (N)
#define ENGINE_LAG 0.2 // (s) used to be 0.0
#define ENGINE_DELAY 0.2 // (s) used to be 0.0
#define DRAG_COEF_CHUTE 2.0
#define DRAG_COEF_LANDER 1.0
#define MAX_PARACHUTE_DRAG 80000.0 // (N) used to be 20000.0
#define MAX_PARACHUTE_SPEED 2000.0 // (m/s) used to be 500.0
#define THROTTLE_GRANULARITY 20 // for manual control
#define ATTITUDE_GRANULARITY 5.0 // for manual attitude control
#define MAX_IMPACT_GROUND_SPEED 1.0 // (m/s)
#define MAX_IMPACT_DESCENT_RATE 1.0 // (m/s)
#define LAUNCHPAD_HEIGHT 20.0 // (m) for launching lander

#endif
