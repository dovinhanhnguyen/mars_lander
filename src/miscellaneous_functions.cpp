// Mars lander simulator
// Version 1.8
// Miscellaneous functions
// Anh Nguyen, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// dvan2@cam.ac.uk.

#include "global_1.h"

//------dvan2's function definitions------//

double current_lander_mass (void)
  // This function calculates current mass of lander
{
  return UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
}

vector3d acceleration_drag (void)
  // This function calculates the acceleration due to drag
{
  vector3d force_lander_drag; // drag force due to lander
  vector3d force_chute_drag; // drag force due to parachute
  
  // Drag force due to lander
  force_lander_drag = (velocity-mars_velocity_wrt_world(position.abs(),false)).norm()*(-0.5*atmospheric_density(position)*DRAG_COEF_LANDER*M_PI*LANDER_SIZE*LANDER_SIZE*(velocity-mars_velocity_wrt_world(position.abs(),false)).abs2());
  
  // Drag force due to parachute
  if (parachute_status == DEPLOYED) {
    force_chute_drag = (velocity-mars_velocity_wrt_world(position.abs(),false)).norm()*(-0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity-mars_velocity_wrt_world(position.abs(),false)).abs2());
  }
  else {
    force_chute_drag = vector3d(0.0, 0.0, 0.0);
  }
  
  // Return acceleration due to total drag
  return (force_lander_drag + force_chute_drag)/current_lander_mass();
}

vector3d acceleration_gravity (void)
  // This function calculates the acceleration due to gravity
{
  vector3d lander_position_wrt_Phobos, lander_position_wrt_Deimos;
  lander_position_wrt_Phobos = position - Phobos.get_position();
  lander_position_wrt_Deimos = position - Deimos.get_position();
  
  if (moon_effect_on) { // gravitation force due to Mars, Phobos, Deimos
    return (position.norm()*(-GRAVITY*MARS_MASS/position.abs2())) + (lander_position_wrt_Phobos.norm()*(-GRAVITY*PHOBOS_MASS/lander_position_wrt_Phobos.abs2())) + (lander_position_wrt_Deimos.norm()*(-GRAVITY*DEIMOS_MASS/lander_position_wrt_Deimos.abs2()));
  }
  else { // gravitational force due to Mars alone
    return position.norm()*(-GRAVITY*MARS_MASS/position.abs2());
  }
}

vector3d acceleration (void)
  // This function calculates the total instantaneous acceleration
{
  return acceleration_drag() + acceleration_gravity() + thrust_wrt_world()/current_lander_mass();
}

vector3d mars_velocity_wrt_world (double distance_from_centre, bool surface_velocity)
  // Calculates either Mars atmosphere velocity or steady wind velocity at a point directly below the lander
{
  vector3d mars_angular_velocity = vector3d(0.0, 0.0, 2*M_PI/MARS_DAY);
  if (surface_velocity) return (mars_angular_velocity^((position.norm())*distance_from_centre))*rotation_on;
  else return (mars_angular_velocity^((position.norm())*distance_from_centre))*rotation_on + ((mars_angular_velocity^((position.norm())*distance_from_centre)).norm())*10.0*steady_wind_on + ((mars_angular_velocity^((position.norm())*distance_from_centre)).norm())*gust_speed*gust_wind_on;
}

double weibull_random_number (void)
 // Generate random number that follows Weibull distribution - to model gust speed
{
  double random_gust_direction = (((double)rand()/(double)RAND_MAX) >= 0.5) ? 1 : -1; // random direction
  
  double uniform_distributed_between_0_and_1 = ((double)rand()/(double)RAND_MAX); // a uniformly distributed number between 0 and 1
  
  // random Weibull-distributed number, https://www.taygeta.com/random/weibull.html
  if (uniform_distributed_between_0_and_1 == 1.0) (weibull_random_number()); // if number is 1, regenerate
  else return 2.0*random_gust_direction*pow((-100.0*log(1-uniform_distributed_between_0_and_1)),0.5); // generate a random Weibull-distributed number with direction (factor of 2 to make the gust stronger)
}

void glut_print_3d (float x, float y, float z, string s)
  // Prints string at location (x,y,z) in a bitmap font
{
  unsigned short i;

  glRasterPos3f(x, y, z);
  for (i = 0; i < s.length(); i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, s[i]);
}

vector3d matrix_times_vector (double m[], vector3d n)
{ // Pre-multiply a vector by a matrix
  vector3d result_vector;
  
  result_vector.x = m[0]*n.x + m[4]*n.y + m[8]*n.z;
  result_vector.y = m[1]*n.x + m[5]*n.y + m[9]*n.z;
  result_vector.z = m[2]*n.x + m[6]*n.y + m[10]*n.z;
  
  return result_vector;
}

vector3d rodrigues_rotation (vector3d n, vector3d rotation_axis, double rotation_angle)
{ // Rotate a vector using Rodrigues's rotation formula
  n = n*cos(rotation_angle) + (rotation_axis^n)*sin(rotation_angle) + rotation_axis*((rotation_axis*n)*(1-cos(rotation_angle)));
  return n;
}

void draw_future_trajectory (Kepler_solver object_Kepler, float colour_red, float colour_green, float colour_blue, string s)
  // Draw future trajectory of an orbiting object after solving for Kepler elements
{
  // colour_red/green/blue = RGB colour that will be used to draw the trajectory
  // s = string that will be used to label the trajectory drawn
  
  
  // VARIABLES EXPLAINED
  // h = angular momentum vector
  // h_hat = normalized h
  // e = eccentricity vector
  // e_hat = normalized e
  // h_e = normalized vector that is perpendicular to both h and e
  // mu = standard grabitational parameter of Mars = GRAVITY*MARS_MASS
  // energy = specific mechanical energy
  // a = semi-major axis
  // p = semi-latus rectum
  // q = periapsis distance
  // q_complement = apoapsis distance
  // m = matrix that transforms a vector from reference plane to orbit plane
  // polar_r = first polar coordinates parameter
  // theta = second polar coordinates parameter
  // point_in_ref_plane = position of a point on the orbit in the reference plane
  // point_in_orbit_plane = position of a point on the orbit in the orbit plane
  // i = counting variable
  // points = number of points to be drawn
  // display_periapsis = string that says "Periapsis"
  // display_apoapsis = string that says "Apoapsis"
  // covert = stream used for coverting from double to string
  vector3d h_hat, e_hat, h_e;
  double m[16], theta = 0, polar_r = 0;
  vector3d point_in_ref_plane, point_in_orbit_plane;
  int i = 0, points = 50;
  string display_periapsis = "Periapsis ";
  string display_apoapsis = "Apoapsis ";
  ostringstream convert;
  
  
  // CONSTRUCT TRANSFORMATION MATRIX
  h_hat = object_Kepler.h.norm();
  e_hat = object_Kepler.e.norm();
  h_e = (h_hat^e_hat).norm();
  m[0]=e_hat.x, m[4]=h_e.x, m[8]=h_hat.x, m[12]=0.0;
  m[1]=e_hat.y, m[5]=h_e.y, m[9]=h_hat.y, m[13]=0.0;
  m[2]=e_hat.z, m[6]=h_e.z, m[10]=h_hat.z, m[14]=0.0;
  m[3]=0.0, m[7]=0.0, m[11]=0.0, m[15]=0.0;
  
  
  // INITIALISE SOME GRAPHICS VARIABLES
  glDisable(GL_LIGHTING); // disable lighting for visibility
  glColor3f(colour_red, colour_green, colour_blue);
  glPointSize(2.0);
  
  
  // DRAW ORBIT THAT IS PREDICTED BASED ON CURRENT POSITION AND VELOCITY
  if (abs(object_Kepler.e.abs()-1.0) > SMALL_NUM) {
    // either circular, elliptic or hyperbolic BUT NOT parabolic
    
    if (object_Kepler.e.abs() <= SMALL_NUM) {
      // circular orbit
      
      // if it's a collision trajectory, draw in red and display warning
      // else if the trajectory clips the atmosphere, draw in yellow and display warning
      if (object_Kepler.q <= MARS_RADIUS) {
        glColor3f(1.0, 0.0, 0.0);
        s = "ON COLLISION COURSE";
      }
      else if (MARS_RADIUS < object_Kepler.q && object_Kepler.q <= MARS_RADIUS+EXOSPHERE) {
        glColor3f(1.0, 1.0, 0.0);
        s = "CLIPS ATMOSPHERE";
      }
      else {
      }
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(2*M_PI*i)/points;
        polar_r = object_Kepler.p; // simplified version of conic polar equation to improve speed of program
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
    else if (SMALL_NUM < object_Kepler.e.abs() && object_Kepler.e.abs() < 1) {
      // elliptical orbit
      
      // if it's a collision trajectory, draw in red and display warning
      // else if the trajectory clips the atmosphere, draw in yellow and display warning
      if (object_Kepler.q <= MARS_RADIUS) {
        glColor3f(1.0, 0.0, 0.0);
        s = "ON COLLISION COURSE";
      }
      else if (MARS_RADIUS < object_Kepler.q && object_Kepler.q <= MARS_RADIUS+EXOSPHERE) {
        glColor3f(1.0, 1.0, 0.0);
        s = "CLIPS ATMOSPHERE";
      }
      else {
      }
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(2*M_PI*i)/points;
        polar_r = object_Kepler.p/(1+cos(theta)*object_Kepler.e.abs());
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
    else {
      // hyperbolic escape
      
      // if it's a collision trajectory, draw in red and display warning
      // else if the trajectory clips the atmosphere, draw in yellow and display warning
      if (object_Kepler.q <= MARS_RADIUS) {
        glColor3f(1.0, 0.0, 0.0);
        s = "ON COLLISION COURSE";
      }
      else if (MARS_RADIUS < object_Kepler.q && object_Kepler.q <= MARS_RADIUS+EXOSPHERE) {
        glColor3f(1.0, 1.0, 0.0);
        s = "CLIPS ATMOSPHERE";
      }
      else {
      }
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(1.5*M_PI*i)/points-0.75*M_PI; // draw from -135 to 135 degree only
        polar_r = object_Kepler.p/(1+cos(theta)*object_Kepler.e.abs());
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
  }
  else {
    // parabolic escape
    
    // if it's a collision trajectory, draw in red and display warning
    // else if the trajectory clips the atmosphere, draw in yellow and display warning
    if (object_Kepler.q <= MARS_RADIUS) {
      glColor3f(1.0, 0.0, 0.0);
      s = "ON COLLISION COURSE";
    }
    else if (MARS_RADIUS < object_Kepler.q && object_Kepler.q <= MARS_RADIUS+EXOSPHERE) {
      glColor3f(1.0, 1.0, 0.0);
      s = "CLIPS ATMOSPHERE";
    }
    else {
    }
    
    glBegin(GL_POINTS);
    for(i=0.0; i<points; i++ ) {
      theta=(1.5*M_PI*i)/points-0.75*M_PI; // draw from -135 to 135 degree only
      polar_r = object_Kepler.p/(1+cos(theta)); // simplified version of conic polar equation to improve speed of program
      point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
      point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
      glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
    }
    glEnd();
  }
  
  
  // LABEL THE TRAJECTORY DRAWN
  glDisable(GL_DEPTH_TEST);
  
  polar_r = object_Kepler.p/(1+cos(0.5*M_PI)*object_Kepler.e.abs()); // print label at 90 degree to the periapsis
  point_in_ref_plane = vector3d(polar_r*cos(0.5*M_PI), polar_r*sin(0.5*M_PI), 0.0);
  point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
  glut_print_3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z, s);
  
  polar_r = object_Kepler.p/(1+object_Kepler.e.abs()); // print periapsis distance at periapsis
  point_in_ref_plane = vector3d(polar_r, 0.0, 0.0);
  point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
  convert.precision(1);
  convert << fixed << object_Kepler.q; // insert the textual representation of double q in the characters in the stream
  s = convert.str(); // set s to the contents of the stream
  glut_print_3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z, display_periapsis+s); // concatenate "Periapsis " and periapsis distance
  convert.str(""); // clear ostringstream content
  
  if (abs(object_Kepler.e.abs()-1.0) > SMALL_NUM) { // if not parabolic escape
    if (object_Kepler.e.abs() > 1.0) { // if hyperbolic escape, don't print apoapsis distance
    }
    else { // if circular or elliptic orbit, print apoapsis distance at apoapsis
      polar_r = object_Kepler.p/(1-object_Kepler.e.abs()); // print periapsis distance at periapsis
      point_in_ref_plane = vector3d(-polar_r, 0.0, 0.0);
      point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
      convert.precision(1);
      convert << fixed << object_Kepler.q_complement; // insert the textual representation of double q_complement in the characters in the stream
      s = convert.str(); // set s to the contents of the stream
      glut_print_3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z, display_apoapsis+s); // concatenate "Apoapsis " and apoapsis distance
    }
  }
  glEnable(GL_DEPTH_TEST);
  
  glEnable(GL_LIGHTING); // enable lighting
}

void draw_future_trajectory_closeup (Kepler_solver object_Kepler, float colour_red, float colour_green, float colour_blue)
  // Adapt from draw_moon_future_trajectory(...) above
{
  // colour_red/green/blue = RGB colour that will be used to draw the trajectory
  
  
  // VARIABLES EXPLAINED
  // h = angular momentum vector
  // h_hat = normalized h
  // e = eccentricity vector
  // e_hat = normalized e
  // h_e = normalized vector that is perpendicular to both h and e
  // mu = standard grabitational parameter of Mars = GRAVITY*MARS_MASS
  // energy = specific mechanical energy
  // a = semi-major axis
  // p = semi-latus rectum
  // m = matrix that transforms a vector from reference plane to orbit plane
  // polar_r = first polar coordinates parameter
  // theta = second polar coordinates parameter
  // point_in_ref_plane = position of a point on the orbit in the reference plane
  // point_in_orbit_plane = position of a point on the orbit in the orbit plane
  // i = counting variable
  // points = number of points to be drawn
  // relative_point_position = position of a point relative to lander, expressed IN PLANET FRAME
  vector3d h_hat, e_hat, h_e;
  double m[16], theta = 0, polar_r = 0;
  vector3d point_in_ref_plane, point_in_orbit_plane;
  int i = 0, points = 50;
  
  
  // CONSTRUCT TRANSFORMATION MATRIX
  h_hat = object_Kepler.h.norm();
  e_hat = object_Kepler.e.norm();
  h_e = (h_hat^e_hat).norm();
  m[0]=e_hat.x, m[4]=h_e.x, m[8]=h_hat.x, m[12]=0.0;
  m[1]=e_hat.y, m[5]=h_e.y, m[9]=h_hat.y, m[13]=0.0;
  m[2]=e_hat.z, m[6]=h_e.z, m[10]=h_hat.z, m[14]=0.0;
  m[3]=0.0, m[7]=0.0, m[11]=0.0, m[15]=0.0;
  
  
  // INITIALISE SOME GRAPHICS VARIABLES
  glDisable(GL_LIGHTING); // disable lighting for visibility
  glColor3f(colour_red, colour_green, colour_blue);
  glPointSize(3.0);
  
  
  // DRAW ORBIT THAT IS PREDICTED BASED ON CURRENT POSITION AND VELOCITY
  if (abs(object_Kepler.e.abs()-1.0) > SMALL_NUM) {
    // either circular, elliptic or hyperbolic BUT NOT parabolic
    
    if (object_Kepler.e.abs() <= SMALL_NUM) {
      // circular orbit
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(2*M_PI*i)/points;
        polar_r = object_Kepler.p; // simplified version of conic polar equation to improve speed of program
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
    else if (SMALL_NUM < object_Kepler.e.abs() && object_Kepler.e.abs() < 1) {
      // elliptical orbit
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(2*M_PI*i)/points;
        polar_r = object_Kepler.p/(1+cos(theta)*object_Kepler.e.abs());
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
    else {
      // hyperbolic escape
      
      glBegin(GL_POINTS);
      for(i=0.0; i<points; i++ ) {
        theta=(1.5*M_PI*i)/points-0.75*M_PI; // draw from -135 to 135 degree only
        polar_r = object_Kepler.p/(1+cos(theta)*object_Kepler.e.abs());
        point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
        point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
        glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
      }
      glEnd();
    }
  }
  else {
    // parabolic escape
    
    glBegin(GL_POINTS);
    for(i=0.0; i<points; i++ ) {
      theta=(1.5*M_PI*i)/points-0.75*M_PI; // draw from -135 to 135 degree only
      polar_r = object_Kepler.p/(1+cos(theta)); // simplified version of conic polar equation to improve speed of program
      point_in_ref_plane = vector3d(polar_r*cos(theta), polar_r*sin(theta), 0.0);
      point_in_orbit_plane = matrix_times_vector(m, point_in_ref_plane);
      glVertex3d(point_in_orbit_plane.x, point_in_orbit_plane.y, point_in_orbit_plane.z);
    }
    glEnd();
  }
  
  glEnable(GL_LIGHTING); // enable lighting
}

void draw_pitch_indicator (double cx, double cy, double val, string title, string units)
  // Draws a single instrument dial, position (cx, cy), value val, title
  // Adapt from draw_dial(...) function
{
  int i;
  double tick_height;
  ostringstream s;
  
  // Draw four edges of the indicator
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(cx-OUTER_DIAL_RADIUS*0.75, cy-OUTER_DIAL_RADIUS*1.3); // lower left
  glVertex2d(cx-OUTER_DIAL_RADIUS*0.75, cy+OUTER_DIAL_RADIUS*1.3); // upper left
  glVertex2d(cx+OUTER_DIAL_RADIUS*0.75, cy+OUTER_DIAL_RADIUS*1.3); // upper right
  glVertex2d(cx+OUTER_DIAL_RADIUS*0.75, cy-OUTER_DIAL_RADIUS*1.3); // lower right
  glEnd();
  
  // Draw pitch angle line in green
  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_LINES);
  glVertex2d(cx-OUTER_DIAL_RADIUS*0.25, cy); // left end
  glVertex2d(cx+OUTER_DIAL_RADIUS*0.5, cy); // right end
  glEnd();
  glColor3f(1.0, 1.0, 1.0);
  s.precision(1);
  s.str(""); s << fixed << val;
  glut_print(cx-OUTER_DIAL_RADIUS*0.6, cy-3, s.str()); // value of pitch angle in white
  
  // Draw reference lines
  for(i=180;i>-180;i-=5) {
    if (i==0) glColor3f(0.0, 1.0, 1.0);
    else glColor3f(1.0, 1.0, 1.0);
    tick_height=i-val; // shift from 0 degree
    if(tick_height+345.0<=SMALL_NUM) {
      tick_height+=360.0; // if too low, push to top
    }
    if(tick_height-345.0>=SMALL_NUM) {
      tick_height-=360; // if too high, push to bottom
    }
    tick_height=tick_height*4.0; // convert to pixel height
    if(tick_height>=OUTER_DIAL_RADIUS+8.0 || tick_height<=-OUTER_DIAL_RADIUS-8.0) {
      // if tick height is out of display, don't draw it
    }
    else {
      glBegin(GL_LINES);
      glVertex2d(cx-OUTER_DIAL_RADIUS*0.25, cy+tick_height); // left end
      glVertex2d(cx+OUTER_DIAL_RADIUS*0.25, cy+tick_height); // right end
      glEnd();
      s.str(""); s << fixed << i;
      glColor3f(1.0, 1.0, 1.0);
      glut_print(cx+OUTER_DIAL_RADIUS*0.3, cy+tick_height-3, s.str()); // value of angle
    }
  }
  
  // Draw value and title
  glColor3f(1.0, 1.0, 1.0);
  glut_print(cx+10-3.2*title.length(), cy-OUTER_DIAL_RADIUS-40, title);
  s.str(""); s << fixed << val << " " << units;
  glut_print(cx+10-3.2*s.str().length(), cy-OUTER_DIAL_RADIUS-55, s.str());
}

void draw_attitude_indicator (double cx, double cy, double val, double val_2, string title, string title_2, string units)
  // Draws a single instrument dial, position (cx, cy), value val_roll and val_pitch, title
  // Adapt from draw_dial(...) function
{
  int i;
  double tick_height;
  ostringstream s;
  s.precision(1);
  
  // Draw circumference of the dial
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  for(i=0;i<360;i=i+8) {
    glVertex2d(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)); // lower left
  }
  glEnd();
  
  glBegin(GL_LINE_LOOP);
  for(i=0;i<360;i=i+8) {
    glVertex2d(cx-1.6*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.6*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)); // lower left
  }
  glEnd();
  
  // Draw dial ticks
  glBegin(GL_LINES);
  for(i=0;i<360;i=i+30) {
    if (i==90) glColor3f(0.0, 1.0, 1.0); // if line zero, draw in blue
    else glColor3f(1.0, 1.0, 1.0);
    glVertex2d(cx-1.25*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.25*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0));
    glVertex2d(cx-1.25*INNER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.25*INNER_DIAL_RADIUS*sin(i*M_PI/180.0));
  }
  glEnd();
  
  // Draw dial labels
  for(i=0;i<360;i=i+30) {
    glColor3f(1.0, 1.0, 1.0);
    switch(i) {
    case 0:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-20, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-3, "-90");
      break;
    case 30:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-18, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0), "-60");
      break;
    case 60:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-18, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0), "-30");
      break;
    case 90:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-3, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)+3, "0");
      break;
    case 120:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0), "30");
      break;
    case 150:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0), cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0), "60");
      break;
    case 180:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)+3, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-3, "90");
      break;
    case 210:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)+3, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-5, "120");
      break;
    case 240:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)+3, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-10, "150");
      break;
    case 270:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-10, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-10, "180");
      break;
    case 300:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-20, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-10, "-150");
      break;
    case 330:
      glut_print(cx-1.3*OUTER_DIAL_RADIUS*cos(i*M_PI/180.0)-20, cy+1.3*OUTER_DIAL_RADIUS*sin(i*M_PI/180.0)-10, "-120");
      break;
    }
  }
  
  // Draw lander roll line in green
  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_LINES); // long one
  glVertex2d(cx-INNER_DIAL_RADIUS*cos(val*M_PI/180.0), cy+INNER_DIAL_RADIUS*sin(val*M_PI/180.0)); // left end
  glVertex2d(cx+0.75*INNER_DIAL_RADIUS*cos(val*M_PI/180.0), cy-0.75*INNER_DIAL_RADIUS*sin(val*M_PI/180.0)); // right end
  glEnd();
  glBegin(GL_LINES); // short one
  glVertex2d(cx, cy);
  glVertex2d(cx+0.25*INNER_DIAL_RADIUS*sin(val*M_PI/180.0), cy+0.25*INNER_DIAL_RADIUS*cos(val*M_PI/180.0));
  glEnd();
  s.str(""); s << fixed << val_2; // label value of pitch angle
  glColor3f(1.0, 1.0, 1.0);
  glut_print(cx+0.8*INNER_DIAL_RADIUS*cos(val*M_PI/180.0), cy-0.75*INNER_DIAL_RADIUS*sin(val*M_PI/180.0)-3, s.str());
  
  // Draw reference pitch lines in white
  for(i=180;i>-180;i-=5) {
    if (i==0) glColor3f(0.0, 1.0, 1.0); // if line zero, draw in blue
    else glColor3f(1.0, 1.0, 1.0);
    tick_height=i-val_2; // shift from 0 degree
    if(tick_height+340.0<=SMALL_NUM) {
      tick_height+=360.0; // if too low, push to top
    }
    if(tick_height-345.0>=SMALL_NUM) {
      tick_height-=360.0; // if too high, push to bottom
    }
    tick_height=tick_height*4.0; // convert to pixel height
    if(tick_height>=OUTER_DIAL_RADIUS+8.0 || tick_height<=-OUTER_DIAL_RADIUS-8.0) {
      // if tick height is out of display, don't draw it
    }
    else {
      glBegin(GL_LINES);
      glVertex2d(cx-0.25*INNER_DIAL_RADIUS*cos(val*M_PI/180.0)+tick_height*sin(val*M_PI/180.0), cy+0.25*INNER_DIAL_RADIUS*sin(val*M_PI/180.0)+tick_height*cos(val*M_PI/180.0));
      glVertex2d(cx+0.25*INNER_DIAL_RADIUS*cos(val*M_PI/180.0)+tick_height*sin(val*M_PI/180.0), cy-0.25*INNER_DIAL_RADIUS*sin(val*M_PI/180.0)+tick_height*cos(val*M_PI/180.0));
      glEnd();
      s.str(""); s << fixed << i;
      glColor3f(1.0, 1.0, 1.0);
      glut_print(cx+0.25*INNER_DIAL_RADIUS*cos(val*M_PI/180.0)+tick_height*sin(val*M_PI/180.0), cy-0.25*INNER_DIAL_RADIUS*sin(val*M_PI/180.0)+tick_height*cos(val*M_PI/180.0)-3, s.str()); // value of angle
    }
  }
  
  // Draw value and title
  glColor3f(1.0, 1.0, 1.0);
  glut_print(cx-30, cy-OUTER_DIAL_RADIUS-60, title);
  s.str(""); s << fixed << val << " " << units;
  glut_print(cx+5, cy-OUTER_DIAL_RADIUS-60, s.str());
  glut_print(cx-30, cy-OUTER_DIAL_RADIUS-75, title_2);
  s.str(""); s << fixed << val_2 << " " << units;
  glut_print(cx+5, cy-OUTER_DIAL_RADIUS-75, s.str());
}

void draw_smaller_indicator_lamp (double tcx, double tcy, string off_text, string on_text, bool on)
  // Draws smaller indicator lamp, top centre (tcx, tcy), appropriate text and background colour depending on on/off
{
  if (on) glColor3f(0.5, 0.0, 0.0);
  else glColor3f(0.0, 0.5, 0.0);
  glBegin(GL_QUADS);
  glVertex2d(tcx-59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-0.5);
  glVertex2d(tcx-59.5, tcy-0.5);
  glEnd();
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(tcx-60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy);
  glVertex2d(tcx-60.0, tcy);
  glEnd();
  if (on) glut_print(tcx-55.0, tcy-14.0, on_text);
  else glut_print(tcx-55.0, tcy-14.0, off_text);
}

void draw_input_altitude_lamp (double tcx, double tcy, double val, string title, string units, bool on)
  // Draws smaller indicator lamp, top centre (tcx, tcy), appropriate text and background colour (blue, grey) depending on on/off
{
  ostringstream s;
  s.precision(0);
  
  if (on) glColor3f(0.0, 0.0, 0.5);
  else glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_QUADS);
  glVertex2d(tcx-59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-0.5);
  glVertex2d(tcx-59.5, tcy-0.5);
  glEnd();
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(tcx-60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy);
  glVertex2d(tcx-60.0, tcy);
  glEnd();
  
  glut_print(tcx-60.0, tcy+10.0, title);
  s.str(""); s << fixed << val << " " << units;
  glut_print(tcx+59.5-6.2*s.str().length(), tcy-14.0, s.str());
}

void draw_lander_phase_lamp (double tcx, double tcy, string text, string title, bool on)
  // Draws smaller indicator lamp, top centre (tcx, tcy), appropriate text and background colour depending on on/off
{
  if (on) glColor3f(0.0, 0.0, 0.5);
  else glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_QUADS);
  glVertex2d(tcx-59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-19.5);
  glVertex2d(tcx+59.5, tcy-0.5);
  glVertex2d(tcx-59.5, tcy-0.5);
  glEnd();
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2d(tcx-60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy-20.0);
  glVertex2d(tcx+60.0, tcy);
  glVertex2d(tcx-60.0, tcy);
  glEnd();
  
  glut_print(tcx-60.0, tcy+10.0, title);
  glut_print(tcx+59.5-5.6*text.length(), tcy-14.0, text);
}

