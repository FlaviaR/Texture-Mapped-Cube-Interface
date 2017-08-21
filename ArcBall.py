#!/usr/bin/env python
# coding: UTF-8
#
## @package ArcBall
#
#  Math utilities, vector, matrix types and ArcBall quaternion rotation class.
#
#  @author Paulo Cavalcanti
#  @since 02/02/2017
#  @see http://pydoc.net/Python/PyOpenGL-Demo/3.0.1b1/PyOpenGL-Demo.NeHe.lesson48.ArcBall/
#  @see Alan Watt's Adavanced Animation and Rendering Techniques, page 360.
#  @see http://quaternions.online
#  @see http://www.energid.com/resources/orientation-calculator/
#  @see https://books.google.com.br/books?id=JnbNCgAAQBAJ&pg=PA187&lpg=PA187&dq=draw+arcball+rotation+axis&source=bl&ots=cbkSpGAgEW&sig=aFAw96HgKmrSQiPh4S5abAFoVIE&hl=en&sa=X&ved=0ahUKEwi-s8ya_tDTAhXGkJAKHaXyD8sQ6AEIZjAJ#v=onepage&q=draw%20arcball%20rotation%20axis&f=false
#

"""
>>> unit_test_ArcBall_module ()
unit testing ArcBall
Quat for first drag
[ 0.08438914 -0.08534208 -0.06240179  0.99080837]
First transform
[[ 0.97764552 -0.13806032  0.15858324  0.        ]
 [ 0.10925253  0.97796899  0.17787793  0.        ]
 [-0.17964737 -0.15657593  0.97119039  0.        ]
 [ 0.          0.          0.          1.        ]]
LastRot at end of first drag
[[ 0.97764552 -0.13806032  0.15858324]
 [ 0.10925254  0.97796899  0.17787793]
 [-0.17964737 -0.15657593  0.97119039]]
Quat for second drag
[ 0.00710336  0.3183279  0.02679029  0.94757545]
Second transform
[[ 0.88022292 -0.08322024 -0.46720675  0.        ]
 [ 0.14910148  0.98314679  0.10578786  0.        ]
 [ 0.4505291  -0.16277809  0.87779653  0.        ]
 [ 0.          0.          0.          1.        ]]
"""

try:
	import numpy as Numeric
	def sumDot( a,b ):
		return Numeric.dot (a, b)
except (ImportError, err):
	try: 
		import Numeric
		def sumDot( a,b ):
			return sum (Numeric.dot (a, b) )
	except (ImportError, err):
		print ("This demo requires the numpy or Numeric extension, sorry")
		import sys
		sys.exit()
import copy
from math import sqrt, sin, acos, atan2, pi
import matrix

## Assuming IEEE-754(GLfloat), which I believe has max precision of 7 bits.
EPSILON = 1.0e-5

## An arcball class. It uses the openGL idle function to rotate the model.
#
class ArcBallT(object):
	def __init__ (self, NewWidth, NewHeight):
		self.m_StVec = Vector3fT ()
		self.m_EnVec = Vector3fT ()
		self.m_AdjustWidth = 1.0
		self.m_AdjustHeight = 1.0
		self.setBounds (NewWidth, NewHeight)

	def __str__ (self):
		str_rep = ""
		str_rep += "StVec = " + str (self.m_StVec)
		str_rep += "\nEnVec = " + str (self.m_EnVec)
		str_rep += "\n scale coords %f %f" % (self.m_AdjustWidth, self.m_AdjustHeight)
		return str_rep

	def setBounds (self, NewWidth, NewHeight):
		# Set new bounds
		assert (NewWidth > 1.0 and NewHeight > 1.0), "Invalid width or height for bounds."
		# Set adjustment factor for width/height
		self.m_AdjustWidth = 1.0 / ((NewWidth - 1.0) * 0.5)
		self.m_AdjustHeight = 1.0 / ((NewHeight - 1.0) * 0.5)

	def _mapToSphere (self, NewPt):
		"""Given a new window coordinate, it will modify NewVec in place."""
		X = 0
		Y = 1
		Z = 2

		NewVec = Vector3fT ()
		# Copy parameter into temp point
		TempPt = copy.copy (NewPt)
		### print 'NewPt', NewPt, TempPt
		# Adjust point coords and scale down to range of [-1 ... 1]
		TempPt [X] = (NewPt [X] * self.m_AdjustWidth) - 1.0
		TempPt [Y] = 1.0 - (NewPt [Y] * self.m_AdjustHeight)
		# Compute the square of the length of the vector to the point from the center
		length = sumDot( TempPt, TempPt)
		# If the point is mapped outside of the sphere... (length > radius squared)
		if (length > 1.0):
			# Compute a normalizing factor (radius / sqrt(length))
			norm    = 1.0 / sqrt (length)

			# Return the "normalized" vector, a point on the sphere
			NewVec [X] = TempPt [X] * norm
			NewVec [Y] = TempPt [Y] * norm
			NewVec [Z] = 0.0
		else:			# Else it's on the inside
			# Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
			NewVec [X] = TempPt [X]
			NewVec [Y] = TempPt [Y]
			NewVec [Z] = sqrt (1.0 - length)

		return NewVec

	def click (self, NewPt):
		""" Mouse down (Point2fT)."""
		self.m_StVec = self._mapToSphere (NewPt)
		return

	def drag (self, NewPt):
		""" Mouse drag, calculate rotation (Point2fT Quat4fT).
            drag (Point2fT mouse_coord) -> new_quaternion_rotation_vec
		"""
		X = 0
		Y = 1
		Z = 2
		W = 3

		self.m_EnVec = self._mapToSphere (NewPt)

		# Compute the vector perpendicular to the begin and end vectors
		# Perp = Vector3fT ()
		# The length of Perp is sine (theta / 2): |a⃗ × b⃗| = |a⃗| |b⃗| sin(θ)
		# Perp will be a unit vector if and only if θ = π/2.
		Perp = Vector3fCross(self.m_StVec, self.m_EnVec)

		NewRot = Quat4fT ()
		# Compute the length of the perpendicular vector
		if (Vector3fLength(Perp) > EPSILON):		# if its non-zero
			# We're ok, so return the perpendicular vector as the transform after all
			NewRot[X] = Perp[X]
			NewRot[Y] = Perp[Y]
			NewRot[Z] = Perp[Z]
			# In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
			NewRot[W] = Vector3fDot(self.m_StVec, self.m_EnVec)
		else:										# if its zero
			# The begin and end vectors coincide, so return a quaternion of zero matrix (no rotation)
			NewRot[X] = NewRot[Y] = NewRot[Z] = NewRot[W] = 0.0
			
		return NewRot


# ##################### Math utility ##########################################

def radToDeg(ang):
	"""Convert radians to degrees."""
	return ang*180/pi

def degToRad(ang):
	"""Convert degrees to radians."""
	return ang*pi/180

def Matrix4fT ():
	"""Return an identity 4x4 matrix."""
	return Numeric.identity (4, 'f')

def Matrix3fT ():
	"""Return an identity 3x3 matrix."""
	return Numeric.identity (3, 'f')

def Quat4fT ():
	"""Return a null quaternion."""
	return Numeric.zeros (4, 'f')

def Quat4fTConjugate(q):
	"""Return the conjugate of quaternion q."""
	qc = copy.copy(q)
	qc[0:3] *= -1
	return qc

def Vector3fT ():
	return Numeric.zeros (3, 'f')

def Point2fT (x = 0.0, y = 0.0):
	pt = Numeric.zeros (2, 'f')
	pt [0] = x
	pt [1] = y
	return pt

def Vector3fDot(u, v):
	"""Dot product of two 3f vectors."""
	dotprod = Numeric.dot (u,v)
	return dotprod

def Vector3fCross(u, v):
	"""Cross product of two 3f vectors."""
	X = 0
	Y = 1
	Z = 2
	cross = Numeric.zeros (3, 'f')
	cross [X] = (u[Y] * v[Z]) - (u[Z] * v[Y])
	cross [Y] = (u[Z] * v[X]) - (u[X] * v[Z])
	cross [Z] = (u[X] * v[Y]) - (u[Y] * v[X])
	return cross

def Vector3fLength (u):
	"""Return the vector length: sqrt(u.u)."""
	mag_squared = sumDot(u,u)
	mag = sqrt (mag_squared)
	return mag
	
def Matrix3fSetIdentity ():
	"""Returns a 3x3 identity matrix."""
	return Numeric.identity (3, 'f')

def Matrix3fMulMatrix3f (matrix_a, matrix_b):
	"""Returns matrix_a * matrix_b."""
	return sumDot( matrix_a, matrix_b )

def Matrix4fSVD (NewObj):
	X = 0
	Y = 1
	Z = 2
	s = sqrt ( (
	(NewObj [X][X] * NewObj [X][X]) + (NewObj [X][Y] * NewObj [X][Y]) + (NewObj [X][Z] * NewObj [X][Z]) +
	(NewObj [Y][X] * NewObj [Y][X]) + (NewObj [Y][Y] * NewObj [Y][Y]) + (NewObj [Y][Z] * NewObj [Y][Z]) +
	(NewObj [Z][X] * NewObj [Z][X]) + (NewObj [Z][Y] * NewObj [Z][Y]) + (NewObj [Z][Z] * NewObj [Z][Z]) ) / 3.0 )
	return s

def Matrix4fSetRotationScaleFromMatrix3f(NewObj, three_by_three_matrix):
	""" Modifies NewObj in-place by replacing its upper 3x3 portion from the 
        passed in 3x3 matrix.
        NewObj = Matrix4fT ()
	"""
	NewObj [0:3,0:3] = three_by_three_matrix
	return NewObj

##
#  Sets the rotational component (upper 3x3) of this matrix to the matrix
#  values in the T precision Matrix3d argument. 
#  The other elements of this matrix are unchanged. 
#  A singular value decomposition is performed on this object's upper
#  3x3 matrix to factor out the scale. 
#  Then this object's upper 3x3 matrix components are replaced 
#  by the passed rotation components, and then the scale is reapplied
#  to the rotational components.
#
#  @param NewObj Matrix4 created.
#  @param three_by_three_matrix T precision 3x3 matrix
# 
def Matrix4fSetRotationFromMatrix3f (NewObj, three_by_three_matrix):
	#scale = Matrix4fSVD (NewObj)

	NewObj = Matrix4fSetRotationScaleFromMatrix3f(NewObj, three_by_three_matrix)
	#scaled_NewObj = NewObj * scale			 # Matrix4fMulRotationScale(NewObj, scale)
	#return scaled_NewObj
	return NewObj

def Matrix3fSetRotationFromQuat4f (q1, dim=3):
	"""Converts the H quaternion q1 into a new equivalent 3x3 rotation matrix."""
	X = 0
	Y = 1
	Z = 2
	W = 3

	if dim == 4:
		NewObj = Matrix4fT ()
	else:
		NewObj = Matrix3fT ()
	n = sumDot(q1, q1)
	s = 0.0
	if (n > 0.0):
		s = 2.0 / n
	xs = q1 [X] * s;  ys = q1 [Y] * s;  zs = q1 [Z] * s
	wx = q1 [W] * xs; wy = q1 [W] * ys; wz = q1 [W] * zs
	xx = q1 [X] * xs; xy = q1 [X] * ys; xz = q1 [X] * zs
	yy = q1 [Y] * ys; yz = q1 [Y] * zs; zz = q1 [Z] * zs
	# This math all comes about by way of algebra, complex math, and trig identities.
	# See Lengyel pages 88-92
	NewObj [0][0] = 1.0 - (yy + zz);    NewObj [1][0] = xy - wz;            NewObj [2][0] = xz + wy
	NewObj [0][1] =        xy + wz;     NewObj [1][1] = 1.0 - (xx + zz);    NewObj [2][1] = yz - wx
	NewObj [0][2] =        xz - wy;     NewObj [1][2] = yz + wx;          	NewObj [2][2] = 1.0 - (xx + yy)

	if dim == 4:
		NewObj[0][3] = 0; NewObj[1][3] = 0; NewObj[2][3] = 0; NewObj[3][3] = 1.0;
		NewObj[3][0] = 0; NewObj[3][1] = 0; NewObj[3][2] = 0;

	return NewObj

def quatToMat (qt):
	"""Converts the H quaternion q1 into a new equivalent 4x4 rotation matrix."""

	return Matrix3fSetRotationFromQuat4f(qt, 4)

## The set of quaternions, known by mathematicians as the ring of Hamiltonian quaternions, and 
#  denoted by H, can be thought of as a four-dimensional vector space for which an element q has 
#  the form: q = <w,x,y,z> = w + xi + yj + zk. 
#
#  It represents the angle and axis of a rotation as a unit quaternion:
#  q = cos(θ/2) + A sin (θ/2), where A is a unit vector, representing the axis os rotation.
#
#  The homomorphism P' = q [0,P] q^-1 is then the rotation of point P, about axis A, by an angle θ. 
#  Using q = [s,(x,y,z)] and q^-1 = [s,(-x,-y-x)], we have the implementation below, 
#  where the quaternion [s,(x,y,z)] is written as the column vector [x,y,z,w].
#
#  @see http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
#  @see https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
#  @param mat given matrix.
#  @return corresponding quaternion (in fact, its conjugate, so it seems).
#
def Quat4fFromMatrix (mat):
	"""Converts a 3x3 or 4x4 rotation matrix to a quaternion q."""

	X = 0
	Y = 1
	Z = 2
	W = 3
	q = Quat4fT()
	nxt = (Y,Z,X)

	# s = +-1/2 sqrt(mat_11 + mat_22 + mat_33 + mat_44)
	# x = (mat_21 - mat_12) / 4s 
	# y = (mat_02 - mat_20) / 4s
	# z = (mat_10 - mat_01) / 4s

	# The sign of s cannot be determined. Depending on the choice of the sign for s, 
	# the signs for x, y and z change as well. This means choosing between a quaternion
	# and the corresponding negative quaternion. These quaternions yield the same rotation,
	# but the interpolation curve can be influenced by this choice.
	tr = mat[X][X] + mat[Y][Y] + mat[Z][Z]
	if tr > 0.0:
		s = sqrt(tr + 1.0)
		q[W] = s * 0.5
		s = 0.5/s
		
		q[X] = (mat[1][2] - mat[2][1])*s # reversed 
		q[Y] = (mat[2][0] - mat[0][2])*s # reversed
		q[Z] = (mat[0][1] - mat[1][0])*s # reversed

		# This is numerically stable so long as the trace, tr, is not negative; 
		# otherwise, we risk dividing by (nearly) zero. In that case, suppose 
		# Qii is the largest diagonal entry, so i will have the largest magnitude 
		# (the other cases are similar); then the following is safe:
	else:
		# get the largest diagonal entry
		i = X
		if mat[Y][Y] > mat[X][X]: i = Y
		if mat[Z][Z] > mat[i][i]: i = Z
		j = nxt[i]  # avoids using % operator: (i+1)%3
		k = nxt[j]

		s = sqrt( (mat[i][i] - (mat[j][j]+mat[k][k])) + 1.0 )

		q[i] = s * 0.5
		s = 0.5/s
		q[W] = (mat[j][k] - mat[k][j])*s # reversed
		q[j] = (mat[i][j] + mat[j][i])*s
		q[k] = (mat[i][k] + mat[k][i])*s
	return q

## Exactly the same implementation of Quat4fFromMatrix (mat),
#  but returns the conjugated quaternion instead.
#
#  @param mat given matrix.
#  @return corresponding quaternion.
#
def matToQuat(mat):
	"""Converts a 3x3 or 4x4 rotation matrix to a quaternion q."""

	X = 0
	Y = 1
	Z = 2
	W = 3
	q = Quat4fT()
	trace = mat[0][0] + mat[1][1] + mat[2][2]	# I removed + 1.0 see discussion with Ethan
	if trace > 0:								# I changed M_EPSILON to 0
		s = 0.5 / sqrt(trace + 1.0)
		q[W] = 0.25 / s
		q[X] = ( mat[2][1] - mat[1][2] ) * s
		q[Y] = ( mat[0][2] - mat[2][0] ) * s
		q[Z] = ( mat[1][0] - mat[0][1] ) * s
	else:
		if ( mat[0][0] > mat[1][1] and mat[0][0] > mat[2][2] ):
			s = 2.0 * sqrt( 1.0 + mat[0][0] - mat[1][1] - mat[2][2])
			q[W] = (mat[2][1] - mat[1][2] ) / s
			q[X] = 0.25 * s
			q[Y] = (mat[0][1] + mat[1][0] ) / s
			q[Z] = (mat[0][2] + mat[2][0] ) / s
		elif (mat[1][1] > mat[2][2]):
			s = 2.0 * sqrt( 1.0 + mat[1][1] - mat[0][0] - mat[2][2])
			q[W] = (mat[0][2] - mat[2][0] ) / s
			q[X] = (mat[0][1] + mat[1][0] ) / s
			q[Y] = 0.25 * s
			q[Z] = (mat[1][2] + mat[2][1] ) / s
		else:
			s = 2.0 * sqrt( 1.0 + mat[2][2] - mat[0][0] - mat[1][1] )
			q[W] = (mat[1][0] - mat[0][1] ) / s
			q[X] = (mat[0][2] + mat[2][0] ) / s
			q[Y] = (mat[1][2] + mat[2][1] ) / s
			q[Z] = 0.25 * s

	# the conjugate quaternion (negate the imaginary part)
	# q = Quat4fTConjugate(q)
	return q

def matrixToEulerAxisAngle(m):
	"""If the Euler angle is not a multiple of pi,
	   the Euler axis e=[x,y,z] and angle theta can
	   be computed from the elements of the rotation matrix."""

	q = matToQuat(m)
	#q = Quat4fFromMatrix(m)
	return quatToEulerAxisAngle(q)

## Returns the axis and angle of a given quaternion.
#  q = cos(a/2) + i ( x * sin(a/2)) + j (y * sin(a/2)) + k ( z * sin(a/2))
#
#  @see http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
#  @param q given quaternion.
#  @return a tuple (theta, x, y, z). Theta is in degrees.
#
def quatToEulerAxisAngle(q):
	"""Returns the axis and angle of a given quaternion."""

	X = 0
	Y = 1
	Z = 2
	W = 3
	# q[W] = cos(theta/2)
	theta = 2.0*acos(q[W])
	# s = sin(theta/2)
	s = sqrt(1-q[W]*q[W])
	if abs(s) < 0.001:
		s = 1.0
	# q = sin(theta/2)*e
	x = q[X]/s
	y = q[Y]/s
	z = q[Z]/s

	return (radToDeg(theta),x,y,z)

## Return the Euler rotation axis, of a given orthonormal (rotation) matrix.
#  The axis is independent on how the matrix was calculated.
#  It uses the fact that there is only one direction (vector),
#  that is unaffected by the rotation.
#
#  Ru = Iu -> (R-I)u = 0
#  0 = RˆT0 + 0 = RˆT(R-I)u + (R-I)u = (RˆTR - RˆT + R - I)u = 0
#  (I - RˆT + R -I)u = (R - RˆT)u = 0
#
#  @see https://en.wikipedia.org/wiki/Rotation_matrix
#  @param m a rotation matrix.
#  @return rotation axis.
#
def matrixToEulerAxis(m):
	"""If the Euler angle is not a multiple of pi,
	   the Euler axis e=[x,y,z] and angle theta can
	   be computed from the elements of the rotation matrix."""

	return (m[2][1]-m[1][2], m[0][2]-m[2][0], m[1][0]-m[0][1])

def axisAngleToQuat(angle, x, y, z):
	"""Returns the quaternion corresponding to the given axis and angle.
	   Assumes axis is already normalized."""

	X = 0
	Y = 1
	Z = 2
	W = 3
	a = degToRad(angle)
	q = Quat4fT()
	s = sin(a*0.5)
	q[X] = x * s
	q[Y] = y * s
	q[Z] = z * s
	q[W] = cos(a*0.5)

	return q

def lerp(q1, q2, t):
	"""linear quaternion interpolation."""

	qt = q1*(1-t) + q2*t
	qt /= Numeric.linalg.norm(qt)
	return qt

## slerp, spherical linear interpolation, introduced by Ken Shoemake,
#  in the context of quaternion interpolation, for the purpose of animating 3D rotation. 
#
#  It refers to constant-speed motion along a unit-radius great circle arc, 
#  given the ends and an interpolation parameter between 0 and 1.
#
#  Two quaternions q1 and q2 are spherically interpolated using the formula:
#  q(t) = (sin(θ)(1−t)/sin(θ)) q1 + (sin(θ)t/sin(θ)) q2.
#
#  @see https://en.wikipedia.org/wiki/Slerp
#  @param p initial unit quaternion.
#  @param q final unit quaternion.
#  @param t interpolation parameter, between 0 and 1.
#  @param noInvert whether not to choose the shortest path.
#  @return the interpolated quaternion qt.
#
def slerp(p,q,t, noInvert=False):
	"""Returns the interpolated quaternion qt, between p and q, given a parameter t in [0,1]."""

	qt = Quat4fT()
	q1 = Quat4fT()

	# only unit quaternions are valid rotations.
	# calculate cosine -1 < ( p.q ) < 1
	cosom = sumDot(p,q)

	# Robustness: Stay within domain of acos()
	Numeric.clip(cosom, -1.0, 1.0)

	# Since the quaternions q and −q represent the same rotation, 
	# the signs of the quaternions q and p are usually chosen 
	# such that q.p ≥ 0. This also ensures that the interpolation 
	# takes place over the shortest path.
	q1 = Numeric.array(q)
	if not noInvert and cosom < 0.0:
		cosom = -cosom
		q1 = -q1

	if (1.0 - cosom) > EPSILON: # cosom < 1
		# omega = angle between input vectors
		omega = acos(cosom)
		sinom = sin(omega)
		# scale p
		sclp = sin ( (1.0 - t)*omega ) / sinom
		# scale q
		sclq = sin ( t*omega ) / sinom
	else: # cosom == 1
		# "p" and "q" quaternions are very close,
		# so we can do a linear interpolation
		return lerp(p,q,t)

	qt = sclp*p + sclq*q1

	return qt

def squad(q1,q2,a,b,t):
	""" spherical cubic interpolation."""

	c = slerp(q1,q2,t,True),
	d = slerp(a,b,t,True);
	return slerp(c,d,2*t*(1-t),True);

## Return the Euler angles corresponding to a given ZYX rotation matrix.
#  The result is the same as MATLAB except the order
#  of the Euler angles ( x and z are swapped ).
#
#  @see https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#  @see http://gamedev.stackexchange.com/questions/50963/how-to-extract-euler-angles-from-transformation-matrix
#  @param R rotation matrix.
#  @return Euler angles.
#
def rotationMatrixToEulerAngles(R):
	"""Returns the corresponding Euler angles from a given 4x4 rotation matrix."""

	sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
	# Whether the matrix is singular or not (determinant is 0). 
	# Singular matrices are NOT invertible. 
	singular = sy < 1e-6

	# Pay attention to the order in which the angles are stored
	if  not singular :
		x = atan2(R[2,1] , R[2,2])
		y = atan2(-R[2,0], sy)
		z = atan2(R[1,0], R[0,0])
	else :
		x = atan2(-R[1,2], R[1,1])
		y = atan2(-R[2,0], sy)
		z = 0
 
	return (radToDeg(x), radToDeg(y), radToDeg(z))

# ##################### Unit testing ##########################################

def unit_test_ArcBall_module ():
	""" Unit testing of the ArcBall class and the real math behind it.
        Simulates a click and drag followed by another click and drag.
	"""
	print ("unit testing ArcBall")
	Transform = Matrix4fT ()
	LastRot = Matrix3fT ()
	ThisRot = Matrix3fT ()

	ArcBall = ArcBallT (640, 480)
	# print "The ArcBall with NO click"
	# print ArcBall
	# First click
	LastRot = copy.copy (ThisRot)
	mouse_pt = Point2fT (500,250)
	ArcBall.click (mouse_pt)
	# print "The ArcBall with first click"
	# print ArcBall
	# First drag
	mouse_pt = Point2fT (475, 275)
	ThisQuat = ArcBall.drag (mouse_pt)
	# print "The ArcBall after first drag"
	# print ArcBall
	# print
	# print
	print ("Quat for first drag")
	print (ThisQuat)
	ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)
	print ("Quat4fFromMatrix3")
	ThisQuat = Quat4fFromMatrix(ThisRot)
	print (ThisQuat)
	# Linear Algebra matrix multiplication A = old, B = New : C = A * B
	ThisRot = Matrix3fMulMatrix3f (LastRot, ThisRot)
	Transform = Matrix4fSetRotationFromMatrix3f (Transform, ThisRot)
	print ("First transform")
	print (Transform)
	# Done with first drag

	print("slerp qt, t = 0.5")
	qt=slerp(ThisQuat,Quat4fT(),0.5)
	print(qt)


	# second click
	LastRot = copy.copy (ThisRot)
	print ("LastRot at end of first drag")
	print (LastRot)
	mouse_pt = Point2fT (350,260)
	ArcBall.click (mouse_pt)
	# second drag
	mouse_pt = Point2fT (450, 260)
	ThisQuat = ArcBall.drag (mouse_pt)
	# print "The ArcBall"
	# print ArcBall
	print ("Quat for second drag")
	print (ThisQuat)
	ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)
	ThisRot = Matrix3fMulMatrix3f (LastRot, ThisRot)
	# print ThisRot
	Transform = Matrix4fSetRotationFromMatrix3f (Transform, ThisRot)
	print ("Second transform")
	print (Transform)
	# Done with second drag
	LastRot = copy.copy (ThisRot)

	ang = 32
	e0 = 1
	e1 = -2
	e2 = 33
	m = matrix.rotate(ang,e0,e1,e2)
	len = sqrt(e0*e0+e1*e1+e2*e2)
	print ("\nang = %f, x = %f, y =%f, z= %s" % (ang,e0,e1,e2))
	print ("m = %s" % m)
	theta,x,y,z = matrixToEulerAxisAngle(Numeric.asarray(m))
	print ("t = %f, x = %f, y = %f, z = %f\n" % (theta,x*len,y*len,z*len))

	ind = 3
	aList = [[-34, 56, 120], [45, 90,45], [10, 20, 30], [-33, -84, 57]]
	m = Numeric.asarray(matrix.rotateZYX(aList[ind]))
	x,y,z = rotationMatrixToEulerAngles(m)
	print (aList[ind])
	print ("x = %f, y = %f, z = %f" % (x,y,z))
	print ("m = %s" % m)
	theta,x,y,z = matrixToEulerAxisAngle(m)
	print ("q = %s" % matToQuat(m))
	print ("W = %s" % Quat4fFromMatrix(m))
	print ("x = %f, y = %f, z = %f, ang = %f" % (x,y,z, theta))

def _test ():
	""" This will run doctest's unit testing capability.

        doctest introspects the ArcBall module for all docstrings
        that look like interactive python sessions and invokes
        the same commands then and there as unit tests to compare
        the output generated. Very nice for unit testing and
        documentation.

        @see http://www.python.org/doc/current/lib/module-doctest.html
	"""
	import doctest, ArcBall
	return doctest.testmod (ArcBall)

if __name__ == "__main__":
	# Invoke our function that runs python's doctest unit testing tool.
	#_test ()
	unit_test_ArcBall_module ()
