// Mars lander simulator
// Version 1.8
// Orbiting_object class implementation
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

#include "orbiting_object.h"

// Orbiting_object class's member functions

// constructor
Orbiting_object::Orbiting_object(vector3d initial_position, vector3d initial_velocity, double initial_mass)
{
  object_position = initial_position;
  object_velocity = initial_velocity;
  object_mass = initial_mass;
}

// get position
vector3d Orbiting_object::get_position(void)
{
  return object_position;
}

// get velocity
vector3d Orbiting_object::get_velocity(void)
{
  return object_velocity;
}

// get acceleration
vector3d Orbiting_object::get_acceleration(void)
{
  return object_position.norm()*(-GRAVITY*MARS_MASS/object_position.abs2());
}

// get mass
double Orbiting_object::get_mass(void)
{
  return object_mass;
}

// update mechanical dynamics
void Orbiting_object::update_object(void)
{
  vector3d temp_object_position;
  
  if (simulation_time == 0.0) { // first iteration, use Euler
    previous_object_position = object_position;
    object_position = object_position + object_velocity*delta_t + get_acceleration()*(0.5*delta_t*delta_t);
    object_velocity = object_velocity + get_acceleration()*delta_t;
  }
  else { // subsequent iterations, use Verlet
    temp_object_position = object_position;
    object_position = object_position*2.0 - previous_object_position + get_acceleration()*(delta_t*delta_t);
    object_velocity = object_velocity + get_acceleration()*delta_t;
    previous_object_position = temp_object_position;
  }
}

