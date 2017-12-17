// Mars lander simulator
// Version 1.8
// Kepler_solver class header
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

// FOrmulae used here are adapted from http://space.stackexchange.com/questions/1904/how-to-programmatically-calculate-orbital-elements-using-position-velocity-vecto
//                                 and http://www.bogan.ca/orbits/kepler/orbteqtn.html

#ifndef __KEPLER_SOLVER_INCLUDED__
#define __KEPLER_SOLVER_INCLUDED__

#include <cmath>
#include <cstdlib>

#include "define_constants.h"
#include "vector3d.h"

using namespace std;

class Kepler_solver
{
  public:
    // h = angular momentum vector
    // e = eccentricity vector
    // energy = specific mechanical energy
    // a = semi-major axis
    // p = semi-latus rectum
    // q = periapsis distance
    // q_complement = apoapsis distance
    vector3d h, e;
    double energy, a, p, q, q_complement;
    
    // constructor
    Kepler_solver() {h=vector3d(0.0,0.0,0.0); e=vector3d(0.0,0.0,0.0); energy=0.0; a=0.0; p=0.0; q=0.0; q_complement=0.0;}
    
    // update h, e, energy, a, p, q, q_complement
    void update_Kepler(vector3d r, vector3d v);
};

#endif
