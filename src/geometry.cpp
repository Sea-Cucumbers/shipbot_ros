#include "geometry.h"
#include <math.h>
#include <array>

using namespace std;

bool rectIntersectsLineSegment(double length, double width, double orientation,
                               double x, double y, double xi, double xf, double yi, double yf) {
  array<double, 4> xis;
  array<double, 4> yis;
  array<double, 4> xfs;
  array<double, 4> yfs;

  xis[0] = x + length/2;
  yis[0] = y - width/2;
  xfs[0] = x + length/2;
  yfs[0] = y + width/2;

  xis[1] = x + length/2;
  yis[1] = y + width/2;
  xfs[1] = x - length/2;
  yfs[1] = y + width/2;

  xis[2] = x - length/2;
  yis[2] = y + width/2;
  xfs[2] = x - length/2;
  yfs[2] = y - width/2;

  xis[3] = x - length/2;
  yis[3] = y - width/2;
  xfs[3] = x + length/2;
  yfs[3] = y - width/2;

  double c = cos(orientation);
  double s = sin(orientation);

  for (int i = 0; i < 4; ++i) {
    double rxi = c*xis[i] - s*yis[i];
    double ryi = s*xis[i] + c*yis[i];
    double rxf = c*xfs[i] - s*yfs[i];
    double ryf = s*xfs[i] + c*yfs[i];

    if (lineSegmentsIntersect(rxi, ryi, rxf, ryf, xi, yi, xf, yf)) {
      return true;
    }
  }

  return false;
}

bool lineSegmentsIntersect(double x1i, double y1i, double x1f, double y1f,
                           double x2i, double y2i, double x2f, double y2f) {
  double a = x1f - x1i;
  double b = x2i - x2f;
  double c = y1f - y1i;
  double d = y2i - y2f;

  double det = a*d - b*c;

  if (det == 0) {
    // Segments are parallel. Even if they're collinear,
    // this is practically not an intersection anyway, so just
    // return false
    return false;
  }

  double lx = x2i - x1i;
  double ly = y2i - y1i;

  double tx = (d*lx - b*ly)/det;
  double ty = (-c*lx + a*ly)/det;
  return 0 < tx && tx < 1 && 0 < ty && ty < 1;
}
