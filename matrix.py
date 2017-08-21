#!/usr/bin/env python
# coding: UTF-8
#
## @package matrix
#
# OpenGL is column major and numpy row major.
# We use row major here, but could add order='F' in reshape,
# to transform to column major, I guess...
#
# @author Paulo Cavalcanti
# @since 13/02/2017
# @see http://3dengine.org/Rotate_arb
# @see http://www.python-course.eu/matrix_arithmetic.php
#
import sys, math
import numpy as np 
from math import cos,sin

## Returns a translation matrix.
#
#  @param dx x translation.
#  @param dy y translation.
#  @param dz z translation.
#  @return translation matrix.
#
def translate(dx,dy,dz):
	M = identity()

	M[0,3] = dx
	M[1,3] = dy
	M[2,3] = dz

	return M

## Returns a scale matrix.
#
#  @param sx x scale.
#  @param sy y scale.
#  @param sz z scale.
#  @return scale matrix.
#
def scale(sx,sy,sz):
	M = identity()

	M[0,0] = sx
	M[1,1] = sy
	M[2,2] = sz

	return M

## Returns a rotation matrix.
#
#  @param ang rotation angle in degrees.
#  @param x rotation axis (x vector component).
#  @param y rotation axis (y vector component).
#  @param z rotation axis (z vector component).
#  @return rotation matrix.
#
def rotate(ang, x,y,z):
    ang = np.deg2rad(ang)
    c=cos(ang)
    s=sin(ang)
    t=1-c

    len = math.sqrt(x * x + y * y + z * z)
    len = 1 / len
    x *= len
    y *= len
    z *= len

    M = np.matrix([
        [t*x*x+c,    t*x*y-s*z,  t*x*z+s*y,  0],
        [t*x*y+s*z,  t*y*y+c,    t*y*z-s*x,  0],
        [t*x*z-s*y,  t*y*z+s*x,  t*z*z+c,    0],
        [0,  0,  0,  1],
    ])

    return M

## Returns an identity matrix.
#
#  Same as:
#    glPushMatrix()
#    glLoadIdentity()
#    c = glGetDoublev ( GL_MODELVIEW_MATRIX )
#    glPopMatrix()
#    return c
#
#  @return identity matrix.
#
def identity():
    M = np.matrix(np.identity(4))

    return M

## Matrix multiplication.
#  The matrix objects are a subclass of the numpy arrays (ndarray). 
#  The matrix objects inherit all the attributes and methods of ndarray. 
#  Another difference is that numpy matrices are strictly 2-dimensional, 
#  while numpy arrays can be of any dimension, i.e. they are n-dimensional. 
#
#  The most important advantage of matrices is that they provide convenient 
#  notations for the matrix multiplication. If X and Y are two Matrices then 
#  X * Y defines the matrix multiplication. While on the other hand, 
#  if X and Y are ndarrays, X * Y define an element by element multiplication.
#
#  If we want to perform matrix multiplication with two numpy arrays (ndarray), 
#  we have to use the dot product.
#
#  Same as:
#    glPushMatrix()
#    glLoadMatrixf(a)
#    glMultMatrixf(b)
#    c = glGetDoublev ( GL_MODELVIEW_MATRIX )
#    glPopMatrix()
#    return c
#
#  @see http://www.python-course.eu/matrix_arithmetic.php
#  @param a first matrix.
#  @param b second matrix.
#  @return a x b.
#
def dot(a,b):
	return np.dot(a,b)

## Rotate around an axis, passing through a given point.
#
#   Same as:
#     glPushMatrix()
#     glLoadIdentity()
#     glTranslate(p.x,p.y,p.z)
#     glRotate(ang, axix.x, axis.y, axis.z)
#     glTranslate(-p.x,-p.y,-p.z)
#     T = glGetDoublev ( GL_MODELVIEW_MATRIX )
#     glPopMatrix()
#     return T 
#
#   @param ang rotation angle.
#   @param p point the axix passes through.
#   @param axis rotation axis.
#   @return rotation matrix: T.R.-T
#
def translateAndRotate(ang, p, axis):
	T = translate(p.x,p.y,p.z) * \
        rotate(ang, axis.x, axis.y, axis.z) * \
        translate(-p.x,-p.y,-p.z)
	return T

## Apply a given transformation t, using p as the fixed point.
def translateAndTransform(t, p):
	T = translate(p.x,p.y,p.z) * t * translate(-p.x,-p.y,-p.z)
	return T

## Return a rotation matrix, given three angles in the order: ZYX (apply Z first, then Y, then X).
#  When the rotation is specified as rotations about three distinct 
#  axes (e.g. X-Y-Z), they should be called Tait–Bryan angles, 
#  but the popular term is still Euler angles. Therefore, we are going 
#  to call them Euler angles as well.
#
#  The industry standard is Z-Y-X because that corresponds to yaw, pitch and roll.
#
#  Note that: @f$(XYZ)^T = Z^T(a_z)\ Y^T(a_y)\ X^T(a_x) = Z(-a_z)\ Y(-a_y)\ X(-a_x)@f$
#
#  @see http://www.chrobotics.com/library/understanding-euler-angles
#  @see https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#  @param angles a list with angle x, y and z.
#  @return a matrix X * Y * Z to be applied on column vectors (vector is on the right).
#
def rotateZYX(angles):
	return rotateXYZ(angles).T

## Return a rotation matrix, given three angles in the order: XYZ (apply X first, then Y, then Z).
#
#  @see http://www.chrobotics.com/library/understanding-euler-angles
#  @param angles a list with angle x, y and z.
#  @return a matrix Z * Y * X to be applied on column vectors (vector is on the right).
#
def rotateXYZ(angles):
	rx = rotate(angles[0], 1.0, 0.0, 0.0)
	ry = rotate(angles[1], 0.0, 1.0, 0.0)
	rz = rotate(angles[2], 0.0, 0.0, 1.0)
	# rz * ry * rx
	m = dot(rz,dot(ry,rx))
	return np.asarray(m)

## Returns a rotation matrix about a given axis.
#
#  @param angle rotation angle in degrees.
#  @param axis: 0 - x, 1 - y, 2 - z
#  @return rotation matrix.
#
def getRotationMatrix(angle, axis):
	if axis == 0:
		return rotate(angle, 1.0, 0.0, 0.0)
	elif axis == 1:
		return rotate(angle, 0.0, 1.0, 0.0)
	else:
		return rotate(angle, 0.0, 0.0, 1.0)

## Main program for testing.
def main():
    t = translate (3,4,5)

    print ("t = translation matrix =\n%s (type = %s)" % (t,type(t)) )

    r = rotate(90, 1,1,1)
    print ("r = rotation matrix =\n%s (type = %s)" % (r,type(r)) )

    m = t*r
    print ("t*r =\n%s (type = %s)" % (m,type(m)) )

if __name__=="__main__":
    sys.exit(main())
