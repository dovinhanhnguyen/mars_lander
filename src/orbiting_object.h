// Mars lander simulator
// Version 1.8
// Orbiting_object class header
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

#ifndef __ORBITING_OBJECT_INCLUDED__
#define __ORBITING_OBJECT_INCLUDED__

#include <cmath>

#include "define_constants.h"
#include "global_2.h"
#include "vector3d.h"

using namespace std;

class Orbiting_object
{
  private:
    vector3d object_position;
    vector3d previous_object_position;
    vector3d object_velocity;
    double object_mass;
  
  public:
    Orbiting_object() {object_position=vector3d(0.0,0.0,0.0); previous_object_position=vector3d(0.0,0.0,0.0); object_velocity=vector3d(0.0,0.0,0.0); object_mass=0.0;}
    Orbiting_object(vector3d initial_position, vector3d initial_velocity, double initial_mass); // constructor
    vector3d get_position(void);
    vector3d get_velocity(void);
    vector3d get_acceleration(void);
    double get_mass(void);
    void update_object(void);
};

#endif
