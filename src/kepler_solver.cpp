// Mars lander simulator
// Version 1.8
// Kepler_solver class implementation
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

// Formulae used here are adapted from http://space.stackexchange.com/questions/1904/how-to-programmatically-calculate-orbital-elements-using-position-velocity-vecto
//                                 and http://www.bogan.ca/orbits/kepler/orbteqtn.html

#include "kepler_solver.h"

// Kepler_solver class's member functions

void Kepler_solver::update_Kepler(vector3d r, vector3d v)
{
  double mu = GRAVITY*MARS_MASS;
  
  // Solve for h, e, energy
  h=r^v;
  e = (((v.abs2()-mu/r.abs())*r)-((r*v)*v))/mu;
  energy = 0.5*v.abs2() - mu/r.abs(); // negative for circular and ellipse orbits, zero for parabolic escape, positive for hyperbolic escape
  
  // Solve for a, p, q, q_complement
  if (abs(e.abs()-1.0) > SMALL_NUM) {
    // either circular, elliptic or hyperbolic BUT NOT parabolic
    if (e.abs() <= SMALL_NUM) {
      // circular orbit
      a = -0.5*mu/energy; // positive
      p = a; // simplified version to improve speed of program
      q = a; // simplified version to improve speed of program
      q_complement = a*(1+e.abs());
    }
    else if (SMALL_NUM < e.abs() && e.abs() < 1) {
      // elliptical orbit
      a = -0.5*mu/energy; // positive
      p = a*(1-e.abs2()); // positive
      q = a*(1-e.abs()); // positive
      q_complement = a*(1+e.abs());
    }
    else {
      // hyperbolic escape
      a = -0.5*mu/energy; // negative
      p = a*(1-e.abs2()); // positive
      q = a*(1-e.abs()); // positive
    }
  }
  else {
    // parabolic escape
    a = 0.0; // actually it's undefined
    p = h.abs2()/mu; // positive
    q = 0.5*p; // positive
  }
}
