#ifndef ANGLE_MOD_H
#define ANGLE_MOD_H

/*
 * fmodp: computes fmod(x, y), but always gives a
 * positive value
 */
double fmodp(double x, double y);

/*
 * angle_mod: computes fmodp(theta, 2*M_PI)
 */
double angle_mod(double theta);

/*
 * angdiff: returns the shortest distance from th1 to th2
 * along the unit circle (always less than or equal to PI)
 */
double angdiff(double th1, double th2);

#endif
