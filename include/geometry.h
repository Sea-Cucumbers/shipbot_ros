#ifndef GEOMETRY_H
#define GEOMETRY_H

/*
 * rectIntersectsLineSegment: determines whether a rectangle intersects a line segment. Note
 * that this function does NOT check if the line segment is fully contained in the rectangle
 * ARGUMENTS
 * length: when the rectangle has an orientation of zero, this is the
 * dimension along the x axis
 * width: when the rectangle has an orientation of zero, this is the
 * dimension along the y axis
 * orientation: rectangle orientation
 * x: x position of rectangle center
 * y: y position of rectangle center
 * xi, xf, yi, yf: line segment to check
 * RETURN: true if intersect, false if not
 */
bool rectIntersectsLineSegment(double length, double width, double orientation,
                               double x, double y, double xi, double xf, double yi, double yf);

/*
 * lineSegmentsIntersect: determines whether two line segments intersect
 * ARGUMENTS
 * x1i, y1i, x1f, y1f: initial and final coordinates of segment 1
 * x2i, y2i, x2f, y2f: initial and final coordinates of segment 2
 * RETURN: true if the segments intersect, false if not
 */
bool lineSegmentsIntersect(double x1i, double y1i, double x1f, double y1f,
                           double x2i, double y2i, double x2f, double y2f);

#endif
