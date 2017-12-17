// Mars lander simulator
// Version 1.8
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, October 2014
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk, gc121@eng.cam.ac.uk, and dvan2@cam.ac.uk.

#include "lander_dynamics.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  double Kh_radial;
  double Kp = 0.3; // obtained by trial and error
  double P_out = 0.0;
  double throttle_offset = 0.0;
  static double target_radial_speed, actual_radial_speed;
  static double target_tangential_speed, actual_tangential_speed;
  static double current_radius, target_radius;
  static bool one_more_ignition_needed;
  double cosine_between_velocity_and_position;
  
  if (!accept_input_altitude) {
    
    switch (current_lander_phase) {
    
    // ASCENT GUIDANCE
    case let_it_go:
      if (lander_unheld) {
        if (position.abs()-MARS_RADIUS <= 1000.0) {
          throttle = 1.0;
          stabilized_attitude = true;
          stabilized_attitude_in_plane_wrt_mars = false;
          stabilized_attitude_angle = 0.0;
        }
        else if (position.abs()-MARS_RADIUS > 1000.0 && position.abs()-MARS_RADIUS <= EXOSPHERE) {
          throttle = 1.0;
          stabilized_attitude = true;
          stabilized_attitude_in_plane_wrt_mars = false;
          stabilized_attitude_angle = 30.0;
          if (lander_Kepler.q_complement-MARS_RADIUS>=300000.0 && position.abs()-MARS_RADIUS > 50000.0) throttle = 0.0;
        }
        else {
          throttle = 0.0;
          stabilized_attitude = true;
          stabilized_attitude_in_plane_wrt_mars = false;
          stabilized_attitude_angle = (acos((velocity*position)/((velocity.abs())*(position.abs()))))*180/M_PI-10.0; // point the nose in the direction of travel for added realism
          current_radius = position.abs();
          target_radius = lander_Kepler.q_complement;
          target_tangential_speed = sqrt((2*GRAVITY*MARS_MASS)*(target_radius/current_radius)/(target_radius+current_radius));
          actual_tangential_speed = (velocity - (position.norm())*(velocity*position.norm())).abs();
          one_more_ignition_needed = true;
          cosine_between_velocity_and_position = (velocity.norm())*(position.norm());
          if (-0.0005 <= cosine_between_velocity_and_position && cosine_between_velocity_and_position <= 0.0005) {
            // ignite engine at perigee/apogee
            current_lander_phase = chariots_of_fire;
          }
        }
      }
      break;
    
    // LANDER IN STABLE ORBIT
    case let_it_be:
      current_radius = position.abs();
      switch (current_autopilot_mode) {
      case descent_mode:
        target_radius = MARS_RADIUS + 15000.0;
        target_tangential_speed = sqrt((2*GRAVITY*MARS_MASS)*(target_radius/current_radius)/(target_radius+current_radius));
        actual_tangential_speed = (velocity - (position.norm())*(velocity*position.norm())).abs();
        current_lander_phase = chariots_of_fire;
        break;
      case transfer_mode:
        target_radius = MARS_RADIUS + input_altitude;
        target_tangential_speed = sqrt((2*GRAVITY*MARS_MASS)*(target_radius/current_radius)/(target_radius+current_radius));
        actual_tangential_speed = (velocity - (position.norm())*(velocity*position.norm())).abs();
        one_more_ignition_needed = true;
        cosine_between_velocity_and_position = (velocity.norm())*(position.norm());
        if (-0.0005 <= cosine_between_velocity_and_position && cosine_between_velocity_and_position <= 0.0005) {
          // ignite engine at perigee/apogee
          current_lander_phase = chariots_of_fire;
        }
        break;
      case maintain_mode:
        stabilized_attitude = true;
        stabilized_attitude_angle = 0;
        break;
      }
      break;
    
    // APPLYING IMPULSE
    case chariots_of_fire:
      actual_tangential_speed = (velocity - (position.norm())*(velocity*position.norm())).abs();
      if (actual_tangential_speed < (target_tangential_speed + 0.5) && actual_tangential_speed > (target_tangential_speed - 0.5))
      {
        throttle = 0.0;
        stabilized_attitude = true;
        stabilized_attitude_angle = 0;
        current_lander_phase = the_sound_of_silence;
        break;
      }
      if (actual_tangential_speed > target_tangential_speed)
      {
        throttle = 1.0;
        stabilized_attitude = true;
        stabilized_attitude_angle = -90;
        if (current_autopilot_mode == launch_mode) stabilized_attitude_angle = -80;
        break;
      }
      if (actual_tangential_speed < target_tangential_speed)
      {
        throttle = 1.0;
        stabilized_attitude = true;
        stabilized_attitude_angle = 90;
        if (current_autopilot_mode == launch_mode) stabilized_attitude_angle = 80;
        break;
      }
      break;
    
    // COASTING PHASE
    case the_sound_of_silence:
      if (current_autopilot_mode == descent_mode) {
        current_lander_phase = viva_la_vida;
        break;
      }
      cosine_between_velocity_and_position = (velocity.norm())*(position.norm());
      if (((target_radius*0.975) <= position.abs()) && (position.abs() <= (target_radius*1.025)) && -0.0005 <= cosine_between_velocity_and_position && cosine_between_velocity_and_position <= 0.0005)
      {
        if (one_more_ignition_needed)
        {
          target_tangential_speed = sqrt(GRAVITY*MARS_MASS/target_radius);
          one_more_ignition_needed = false;
          current_lander_phase = chariots_of_fire;
        }
        else
        {
          current_autopilot_mode = maintain_mode;
          current_lander_phase = let_it_be;
        }
      }
      else {
        stabilized_attitude_angle = (acos((velocity*position)/((velocity.abs())*(position.abs()))))*180/M_PI; // point the nose opposite the direction of travel for added realism
      }
      break;
    
    // ENTRY, DESCENT, AND LANDING
    case viva_la_vida:
    
      // Handle vertical speed
      if ((position.abs()-MARS_RADIUS) >= 12000.0)
      { // slow down to ~ -480m/s at the altitude of 12km
        Kh_radial = 0.003;
        target_radial_speed = -440.0-Kh_radial*(position.abs()-MARS_RADIUS);
        actual_radial_speed = velocity*position.norm();
        P_out = Kp*(target_radial_speed-actual_radial_speed);
      }
      else
      { // deploy parachute and slow down to ~ -0.5m/s at surface
        Kh_radial = 0.05;
        if ((safe_to_deploy_parachute() == true) && (parachute_status == NOT_DEPLOYED) && ((acceleration_drag().abs())*current_lander_mass() < MAX_PARACHUTE_DRAG) && (velocity.abs() < MAX_PARACHUTE_SPEED))
        {
        parachute_status = DEPLOYED;
        }
        if ((position.abs()-MARS_RADIUS) <= 100.0) parachute_status = LOST;
        target_radial_speed = -0.5-Kh_radial*(position.abs()-MARS_RADIUS);
        actual_radial_speed = velocity*position.norm();
        P_out = Kp*(target_radial_speed-actual_radial_speed);
      }
      
      // Tuning throttle offset
      double temp_fuel = fuel; // store fuel at 'x(t)'
      fuel = fuel - throttle*FUEL_RATE_AT_MAX_THRUST*delta_t/FUEL_CAPACITY; // get fuel at 'x(t+dt)'
      throttle_offset = ((acceleration_gravity() + acceleration_drag())*position.norm())*(-current_lander_mass())/MAX_THRUST; // minus sign as thrust is on when acceleration is 'downwards'
      fuel = temp_fuel; // set fuel to its value at 'x(t)' so the graphic routines work properly    
    
      // Set throttle value
      if (P_out <= -throttle_offset) {
        throttle = 0;
      }
      else if (P_out >= 1-throttle_offset) {
        throttle = 1;
      }
      else {
        throttle = throttle_offset + P_out;
      }
      
      // Handle attitude control
      stabilized_attitude = true;
      stabilized_attitude_in_plane_wrt_mars = true;
      stabilized_attitude_angle = 0.0;
      
      // Handle horizontal speed
      if ((position.abs()-MARS_RADIUS) <= 100.0)
      {
        if ((velocity - (velocity*(position.norm()))*position.norm() - mars_velocity_wrt_world(position.abs(),true)).abs() >= 0.5) stabilized_attitude_angle = -5;
        if ((velocity - (velocity*(position.norm()))*position.norm() - mars_velocity_wrt_world(position.abs(),true)).abs() >= 2.0) stabilized_attitude_angle = -45;
      }
      
      break;
    }
  }
  
  
  //~ // Comment this out to read lander data into a csv file //
  //~ //-----------------------LANDER DATA RECORDER-----------------------//
  //~ // Parameters recorded: Kh, Kp, altitude, actual descent rate, target descent rate
  //~ ofstream fileout;
  //~ fileout.open("../octave-assignment/lander_data.csv", ios::binary | ios::app);
  //~ if (!fileout.good()) {
    //~ cout << "Error while opening file\n";
  //~ }
  //~ else {
    //~ fileout << Kh_radial << "," << Kp << "," << position.abs()-MARS_RADIUS << "," << actual_radial_speed << "," << target_radial_speed <<"\n";
  //~ }
  //~ fileout.close();
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  static vector3d previous_position; // static local variable to store previous position
  vector3d temp_position = vector3d(0.0, 0.0, 0.0); // local variable to store 'x(t-dt)' position
  vector3d z_axis = vector3d(0.0, 0.0, 1.0); // for moving the lander around on launchpad
  
  gust_speed = weibull_random_number(); // random gust speed
  
  // UPDATE LANDER'S POSE
  if (simulation_time == 0.0) { // first iteration
    previous_position = position;
    if (lander_unheld) {
      position = position + velocity*delta_t +acceleration()*(0.5*delta_t*delta_t);
      velocity = (position - previous_position)/delta_t;
    }
    else {
      position = rodrigues_rotation(position, z_axis, rotation_on*delta_t*2*M_PI/MARS_DAY);
      velocity = rodrigues_rotation(velocity, z_axis, rotation_on*delta_t*2*M_PI/MARS_DAY);
    }
  }
  else { // subsequent iterations
    if (lander_unheld) {
      temp_position = position;
      position = position*2.0 - previous_position + acceleration()*(delta_t*delta_t);
      previous_position = temp_position;
      velocity = (position - previous_position)/delta_t;
    }
    else {
      temp_position = position;
      position = rodrigues_rotation(position, z_axis, rotation_on*delta_t*2*M_PI/MARS_DAY);
      previous_position = temp_position;
      velocity = rodrigues_rotation(velocity, z_axis, rotation_on*delta_t*2*M_PI/MARS_DAY);
    }
  }
  
  // UPDATE LANDER'S KEPLERIAN ELEMENTS
  lander_Kepler.update_Kepler(position, velocity);
  
  // UPDATE PHOBOS & DEIMOS STATES
  Phobos.update_object();
  Phobos_Kepler.update_Kepler(Phobos.get_position(), Phobos.get_velocity());
  Deimos.update_object();
  Deimos_Kepler.update_Kepler(Deimos.get_position(), Deimos.get_velocity());
  
  // AUTOPILOT AND ATTITUDE STABILIZATION ROUTINES
  if (autopilot_enabled) autopilot(); // autopilot to adjust the thrust, parachute and attitude
  if (stabilized_attitude) attitude_stabilization(); // 3D stabilization
  
  // Update closeup view axes to be used in manual attitude control
  update_closeup_coords();
  previous_out = (closeup_coords.right).norm();
  previous_up = position.norm();
  previous_left = (previous_up^previous_out).norm();
  
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  //~ // Comment this out to read lander data into a csv file //
  //~ // Truncate old data in .csv file before each simulation
  //~ ofstream fileout;
  //~ fileout.open("../octave-assignment/lander_data.csv", ios::binary | ios::trunc);
  //~ if (!fileout.good()) {
    //~ cout << "Error while initializing data file\n";
  //~ }
  //~ fileout.close();
  
  // Initialise Phobos & Deimos states
  vector3d moon_initial_position, moon_initial_velocity;
  
  moon_initial_position = vector3d(9234420*cos(1.093*M_PI/180.0), 0.0, 9234420*sin(1.093*M_PI/180.0)); // approx. circular motion
  moon_initial_velocity = vector3d(0.0, sqrt(GRAVITY*MARS_MASS/9234420), 0.0);
  Phobos = Orbiting_object(moon_initial_position, moon_initial_velocity, PHOBOS_MASS);
  
  moon_initial_position = vector3d(-23455500*cos(0.93*M_PI/180.0), 0.0, -23455500*sin(0.93*M_PI/180.0)); // approx. circular motion
  moon_initial_velocity = vector3d(0.0, -sqrt(GRAVITY*MARS_MASS/23455500), 0.0);
  Deimos = Orbiting_object(moon_initial_position, moon_initial_velocity, DEIMOS_MASS);
  
  // Set some parameters

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "polar elliptical orbit";
  scenario_description[3] = "drag prevents polar escape";
  scenario_description[4] = "elliptical orbit that clips the atmosphere";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "areostationary orbit";
  scenario_description[7] = "polar launch";
  scenario_description[8] = "equatorial launch";
  scenario_description[9] = "launch from Northern hemisphere";
  
  lander_unheld = true;
  
  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    current_lander_phase = let_it_be;
    current_autopilot_mode = maintain_mode;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = viva_la_vida;
    current_autopilot_mode = descent_mode;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    current_lander_phase = let_it_be;
    current_autopilot_mode = maintain_mode;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    current_lander_phase = viva_la_vida;
    current_autopilot_mode = descent_mode;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    current_lander_phase = viva_la_vida;
    current_autopilot_mode = descent_mode;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = viva_la_vida;
    current_autopilot_mode = descent_mode;
    break;

  case 6:
    // an areostationary orbit
    position = vector3d(cbrt((GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY)/(4.0*M_PI*M_PI)), 0.0, 0.0);
    velocity = vector3d(0.0, 2.0*M_PI*cbrt((GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY)/(4.0*M_PI*M_PI))/MARS_DAY, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = let_it_be;
    current_autopilot_mode = maintain_mode;
    break;
  
  case 7:
    // polar launch
    position = vector3d(0.0, 0.0, MARS_RADIUS+LAUNCHPAD_HEIGHT);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = let_it_go;
    current_autopilot_mode = launch_mode;
    lander_unheld = false;
    break;

  case 8:
    // equatorial launch
    position = vector3d(MARS_RADIUS+LAUNCHPAD_HEIGHT, 0.0, 0.0);
    velocity = mars_velocity_wrt_world(MARS_RADIUS+LAUNCHPAD_HEIGHT, true);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = let_it_go;
    current_autopilot_mode = launch_mode;
    lander_unheld = false;
    break;

  case 9:
    // random launch
    position = vector3d((MARS_RADIUS+LAUNCHPAD_HEIGHT)*cos(M_PI/4), 0.0, (MARS_RADIUS+LAUNCHPAD_HEIGHT)*cos(M_PI/4));
    velocity = mars_velocity_wrt_world(MARS_RADIUS+LAUNCHPAD_HEIGHT, true);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    current_lander_phase = let_it_go;
    current_autopilot_mode = launch_mode;
    lander_unheld = false;
    break;

  }
}

