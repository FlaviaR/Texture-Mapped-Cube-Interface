#!/usr/bin/env python
# coding: UTF-8
#
## @package TextureMapInterface
#
# Exercises texture mapping and interface usage.
# Implements a textured cube, which can spin around the X, Y, or Z axis, and can be rotated using Arcball.
#
# @author Flavia Cavalcanti
# @since 27/02/2017
#
# @see http://pyopengl.sourceforge.net/context/tutorials/nehe6.html
#
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time, sys, math

from PyQt4 import QtGui
from PyQt4.QtOpenGL import *
from PyQt4 import QtCore

from ArcBall import * 			

import numpy
import matrix

try:
    from PIL.Image import open
except ImportError as err:
    from Image import open

WIDTH = 700
HEIGHT = 480
ESCAPE = b'\033'

class GLWidget( QGLWidget ):

	## Initializes the class - first function to be called after initialization.
	def __init__(self, parent = None):
		super(GLWidget, self).__init__(parent)
		## Whether the cube is rotating about an axis.
		self.rotating = False
		## The current model view transformation.
		self.modelView = None
		## Handle for the image used for texturing.
		self.imageID = None
		## Current active rotation axis - 0 -> x-axis, 1 -> y-axis, 2 -> z-axis
		self.index = 0
		## Whether the index (axis of rotation) has changed. 
		self.indexChanged = True
		## List to store current Euler rotation angles for each axis 
		self.angleList = [0, 0, 0]
		## Whether rotating through the arcball.
		self.arcBall = True
		## Image used for texturing.
		self.imgName = "images/transistor512.jpg"

		## Angle increment
		self.angInc = 1.0
		## Euler axis coordinate in the x direction.
		self.ex = None
		## Euler axis coordinate in the y direction.
		self.ey = None
		## Euler axis coordinate in the z direction.
		self.ez = None
		## Current arcball transformation: g_LastRot * g_ThisRot.
		self.g_Transform = Matrix4fT ()
		## Last arcball transformation applied.
		self.g_LastRot = Matrix3fT ()
		## Present arcball transformation.
		self.g_ThisRot = Matrix3fT ()

		## Whether the mouse is being dragged.
		self.g_isDragging = False

		## Arcball object for model rotation.
		self.g_ArcBall = ArcBallT (WIDTH, HEIGHT)

		## Rotation axis type
		self.rotationDirection = "Intrinsic"

	## Called whenever the rotation axis has changed.
	#  
	#  @param index new rotation axis index.
	#
	def setIndex(self,index):
		"""Set the rotation axis."""

		self.indexChanged = True
		self.index = index

	## Set the rotation type.
	#
	#  @param val 0: intrinsic, 1: extrinsic, 2: ZYX.
	#
	def setRotationDirection(self,val):
		"""Set the rotation type to Intrinsic, Extrinsic or ZYX."""

		if val not in range (0,3):
			print ("Nonexistent rotation option. Check your input.")
			return 

		direction = ("Intrinsic", "Extrinsic", "ZYX")

		# nothing has changed
		if self.rotationDirection == direction[val]: return

		if self.modelView is not None:
			if self.rotationDirection != "ZYX":
				if val == 2: # extrinsic or intrinsic --> none
					if self.rotationDirection == "Intrinsic":
						self.angleList[0], self.angleList[1], self.angleList[2] = rotationMatrixToEulerAngles(self.modelView)
					else:
						self.angleList[0], self.angleList[1], self.angleList[2] = rotationMatrixToEulerAngles(self.modelView.T)
					self.modelView = matrix.rotateXYZ(self.angleList)
				else: # extrinsic --> intrinsic or intrinsic --> extrinsic
					# intrinsic matrix is the inverse of extrinsic, and vice-versa.
					self.modelView = self.modelView.T
			else:
				if val == 0: # none --> intrinsic
					# intrinsic rotation (internal or local axes)
					self.modelView = self.modelView.T

		self.rotationDirection = direction[val]

	## Load an image file as a 2D texture using PIL.
	def loadImage( self, imageName = None ):
		"""Load an image file as a 2D texture using PIL"""

		if imageName is None:
			imageName = self.imgName

		# PIL defines an "open" method which is Image specific!
		try:
			im = open(imageName)
			ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBA", 0, -1)
		except (SystemError, ValueError):
			ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBX", 0, -1)
		except AttributeError:
			ix, iy, image = im.size[0], im.size[1], im.tostring("raw", "RGBX", 0, -1)
		except IOError:
			sys.exit('Cannot open file %s for reading' % imageName)

		# Generate a texture ID
		ID = glGenTextures(1)

		# Make our new texture ID the current 2D texture
		glBindTexture(GL_TEXTURE_2D, ID)
		glPixelStorei(GL_UNPACK_ALIGNMENT,1)

		# Copy the texture data into the current texture ID
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)

		# Note that only the ID is returned, no reference to the image object or the 
		# string data is stored in user space. 
		# The data is only present within the GL after this call exits.
		return ID

	## Set camera parameters.
	def setCamera(self):
		"""Set camera parameters."""

		glMatrixMode(GL_PROJECTION)
		# Reset the Projection Matrix
		glLoadIdentity()

		# field of view, aspect ratio, near and far
		r = 1.0*math.sqrt(2.0) # ray of a sphere enclosing the object
		d = 6.0 # distance from camera to centre of the sphere (plane z = 0)
		fovy = radToDeg((2.0*math.asin(r/d)))
		gluPerspective(fovy, float(WIDTH)/float(HEIGHT), 1.0, 10000.0)

		glMatrixMode (GL_MODELVIEW)
		glLoadIdentity()
		gluLookAt(3.5, 3.5, d, 0, 0, 0, 0, 1, 0)

	## Return the camera position in world coordinates.
	#
	def getCameraPosition(self):
		"""Returns the camera position in world coordinates.
		    @see https://www.opengl.org/discussion_boards/showthread.php/178484-Extracting-camera-position-from-a-ModelView-Matrix
		"""

		# get matrix and viewport.
		matModelView = glGetDoublev( GL_MODELVIEW_MATRIX ) 
		matProjection = glGetDoublev( GL_PROJECTION_MATRIX ) 
		viewport = glGetIntegerv( GL_VIEWPORT ) 
		camera_pos_x, camera_pos_y, camera_pos_z = gluUnProject( (viewport[2]-viewport[0])/2 , (viewport[3]-viewport[1])/2, 
		0.0, matModelView, matProjection, viewport)
		return (camera_pos_x, camera_pos_y, camera_pos_z)

	## Return the 3x3 submatrix, corresponding to the linear transformation part of a 4x4 projective matrix.
	#
	#  @param mat projective matrix.
	#  @return a 3x3 linear transformation matrix.
	#
	def matrix4To3(self, mat):
		"""Returns the 3x3 part of a given 4x4 matrix."""

		newMat = Matrix3fT()
		for i in range (3):
			for j in range (3):
				newMat[i][j] = mat[i,j]

		return newMat

	## Return a 4x4 projective matrix, corresponding to the given 3x3 linear transformation matrix.
	#
	#  @param mat linear transformation matrix.
	#  @return a 4x4 projective matrix.
	#
	def matrix3To4(self, mat):
		"""Returns a 4x4 matrix filled with a given 3x3 matrix."""

		newMat = Matrix4fT()
		for i in range (len(mat)):
			for j in range (len (mat[0])):
				newMat[i][j] = mat[i][j]

		return newMat

	## Return whether there is a valid Euler axis, associated to the instance vector (ex,ey,ez).
	#
	def isEulerAxisDefined(self):
		"""Return whether there is a valid Euler axis."""
		return self.ex or self.ey or self.ez

	## Renders scene geometry and setups the camera and texture.
	#  There are three transformations involved: 
	#  	1) Camera transformation (setCamera).
	#  	2) Arcball transformation (g_Transform).
	#  	3) Spin, or cube rotation: (m).
	#  Multiplication order: camera * arcball * spin.
	#
	def Render( self ):
		"""Render scene geometry"""

		# Clear Screen And Depth Buffer
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

		self.setCamera() 
		self.drawCoordAxes(2.5)

		if not self.arcBall:
			glMatrixMode (GL_MODELVIEW)

			# If the last control scheme to be used was ArcBall, 
			# then extract the euler angles from the last rotation matrix,
			# and initialize angleList to use such angles to allow for a smooth transition.
			if not self.rotating:
				self.angleList[0], self.angleList[1], self.angleList[2] = rotationMatrixToEulerAngles(self.g_Transform.T)
			else:
				self.angleList[self.index] = (self.angleList[self.index] + self.angInc) % 360

			if self.rotationDirection == "ZYX":
				m = numpy.asarray(matrix.getRotationMatrix(self.angInc, self.index))
				theta, self.ex, self.ey, self.ez = matrixToEulerAxisAngle(m)
				# rotation about index 0 (x) is intrinsic.
				# rotations about indexes 1 (y) and 2 (z) are extrinsic, so it seems...
				if self.index > 0:
					# use just the camera transformation
					self.drawEulerAxis(5,3, True) 
					#theta, self.ex, self.ey, self.ez = matrixToEulerAxisAngle(matrix.rotateXYZ(self.angleList))
				self.indexChanged = False

				self.modelView = matrix.rotateXYZ(self.angleList)
				glMultMatrixf(self.modelView)
			else:
				m = numpy.asarray(matrix.getRotationMatrix(self.angInc, self.index))
				if self.modelView is None:
					self.modelView = self.g_Transform.T

				theta, self.ex, self.ey, self.ez = matrixToEulerAxisAngle(m)
				if self.rotationDirection == "Extrinsic":
					# use just the camera transformation
					self.drawEulerAxis(5,3,True) 
					# extrinsic rotation (external or global axes)
					self.modelView = matrix.dot(self.modelView,m.T)
					glMultMatrixf(self.modelView)
				else:
					# intrinsic rotation (internal or local axes)
					self.modelView = matrix.dot(self.modelView,m)
					glMultMatrixf(self.modelView.T)

			self.rotating = True
		else:
			# draw the arcball rotation axis
			self.drawEulerAxis(5, 3)
			if self.rotating:
				if self.modelView is not None:
					if self.rotationDirection == "Intrinsic":
						self.g_ThisRot = self.matrix4To3(self.modelView.T)
						self.g_Transform = self.modelView.T
					else:
						self.g_ThisRot = self.matrix4To3(self.modelView)
						self.g_Transform = self.modelView
					glMultMatrixf(self.g_Transform)	
				self.rotating = False
			else:
				glMatrixMode (GL_MODELVIEW)
				glMultMatrixf(self.g_Transform)

		if not self.arcBall:
			# use all three transformations
			self.drawEulerAxis(5,3)
		self.setupTexture()
		self.drawCube()
		self.drawCoordAxes(1.5, 3.0)


	## Refresh the context.
	# Function must be called in a timed callback manner.
	def refresh( self ):
		"""Request refresh of the context"""
		self.updateGL()

	## This method encapsulates the functions required to set up for textured rendering. 
	#  The original tutorial made these calls once for the entire program. 
	#  This organization makes more sense if you are likely to have multiple textures.
	def setupTexture( self ):
		"""Render-time texture environment setup"""

		# Configure the texture rendering parameters
		glEnable(GL_TEXTURE_2D)

		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
		# Re-select our texture, could use other generated textures if we had generated them earlier...
		glBindTexture(GL_TEXTURE_2D, self.imageID)

	## Draw axis "x" in red, "y" in green and "z" in blue.
	#
	#  @param len length of each axis.
	#  @param wid line width for drawing.
	#
	def drawCoordAxes(self, len = 2.0, wid = 1.0):
		"""Draw the three coordinate axes."""

		lw = glGetFloatv(GL_LINE_WIDTH)
		glLineWidth(wid)
		glBegin(GL_LINES)

		# red
		glColor3f(1.0,0.0,0.0)    
		glVertex3f(0, 0, 0)
		glVertex3f(len, 0, 0)

		# green
		glColor3f(0.0,1.0,0.0)
		glVertex3f(0, 0, 0)
		glVertex3f(0, len, 0)

		# blue
		glColor3f(0.0,0.0,1.0) 
		glVertex3f(0, 0, 0)
		glVertex3f(0, 0, len)
	 
		glEnd()
		glLineWidth(lw)

	## Draw the Euler axis of rotation in white.
	#  It uses the instance vector self.ex(ey)(ez).
	#
	#  @param scale scale factor applied to the vector.
	#  @param wid line width.
	#  @param reset whether to set the instance vector to None.
	#
	def drawEulerAxis(self, scale=3, wid = 1, reset = False):
		""""Draw the Euler axis of rotation."""

		if self.isEulerAxisDefined():
			self.drawAxis(self.ex, self.ey, self.ez, scale, wid)

		if reset:
			self.ex = self.ey = self.ez = None

	## Draw an axis (a line) in white, given its direction and passing through the origin.
	#  The axis is drawn from (-x,-y,-z)*scale to (x,y,z)*scale.
	#
	#  @param x vector coordinate x.
	#  @param y vector coordinate y.
	#  @param z vector coordinate z.
	#  @param scale scale factor applied to the vector.
	#  @param wid line width.
	#
	def drawAxis(self,x,y,z,scale=3, wid = 1):	
		""""Draw an axis of rotation."""

		lw = glGetFloatv(GL_LINE_WIDTH)
		glLineWidth(wid)
		x *= scale
		y *= scale
		z *= scale
		glBegin(GL_LINES)
		# white
		glColor3f(1.0,1.0,1.0)    
		glVertex3f(-x,-y,-z)
		glVertex3f(x,y,z)
		glEnd()
		glLineWidth(lw)

	## Drawing the cube has changed slightly, because we now need to specify the texture 
	#  coordinates for each vertex. This is all just taken from the original tutorial.
	def drawCube( self ):
		"""Draw a cube with texture coordinates"""

		glBegin(GL_QUADS)
		glColor3f(1.0,0.0,0.0)
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
		glColor3f(0.0,1.0,0.0)
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
		glColor3f(0.0,0.0,1.0)
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,  1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		glColor3f(1.0,1.0,0.0)
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0, -1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		glColor3f(0.0,1.0,1.0)
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		glColor3f(1.0,0.0,1.0)
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
		glEnd()

		glDisable(GL_TEXTURE_2D)
		#glutSwapBuffers()

	## This virtual function is called once before the first call to paintGL() or resizeGL().
	# This function should set up any required OpenGL resources and state. 
	def initializeGL(self):

		glutInit(sys.argv)

		# Select type of Display mode:   
		#  Double buffer 
		#  RGBA color
		#  Alpha components supported 
		#  Depth buffer
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)

		# This Will Clear The Background Color To Black
		glClearColor(0.0, 0.0, 0.0, 1.0)
		# Enables Clearing Of The Depth Buffer
		glClearDepth(1.0)
		# The Type Of Depth Test To Do
		glDepthFunc(GL_LEQUAL)
		# Enables Depth Testing
		glEnable(GL_DEPTH_TEST)
		# Select Flat Shading (Nice Definition Of Objects)
		glShadeModel (GL_SMOOTH)
		# Really Nice Perspective Calculations
		glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

		# mcolor will be applied to both ambient and diffuse components of the material.
		# This is done for convenience because in most cases Ambient and Diffuse properties
		# of a material should be set to the same color.
		mcolor = [ 1.0, 0.0, 0.0, 1.0 ]
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor)

		# enable color tracking (specify material properties by merely calling the glColor)
		glEnable (GL_COLOR_MATERIAL)
		# set material properties which will be assigned by glColor
		glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)
		glFrontFace(GL_CCW)

		self.imageID = self.loadImage ()

	## Returns the cursor coordinates in widget coordinates
	def convCursorToWidgetCoord (self):
		cursor = QtGui.QCursor()
		cursor_x = cursor.pos().x()
		cursor_y = cursor.pos().y()

		pt = QtCore.QPoint(cursor_x, cursor_y)
		return self.mapFromGlobal(pt)

	## Called when a mouse button is clicked. 
	#  This function is currently used to pass a start and end coordinate to Arcball. 
	def mousePressEvent(self, event):
	
		if (self.arcBall):
			widgetHeight = self.geometry().height()
			# Set Last Static Rotation To Last Dynamic One
			self.g_LastRot = copy.copy (self.g_ThisRot)
			# Prepare For Dragging
			self.g_isDragging = True

			convPt = self.convCursorToWidgetCoord()
			# if putting the camera in the negative z axis: mouse_pt = Point2fT (convPt.x(), widgetHeight - convPt.y())
			mouse_pt = Point2fT (convPt.x(), convPt.y())
			# Update Start Vector And Prepare For Dragging
			self.g_ArcBall.click (mouse_pt)

	## Reports the position of the mouse cursor, relative to this widget.
	def mouseMoveEvent(self, event):
		""" Mouse cursor is moving
			 Glut calls this function (when mouse button is down)
			 and passes the mouse cursor postion in window coords as the mouse moves.
		"""

		if (self.g_isDragging and self.arcBall):
			widgetHeight = self.geometry().height()
			convPt = self.convCursorToWidgetCoord()

			# if putting the camera in the negative z axis: mouse_pt = Point2fT (convPt.x(), widgetHeight - convPt.y())
			mouse_pt = Point2fT (convPt.x(), convPt.y())
			# Update End Vector And Get Rotation As Quaternion
			ThisQuat = self.g_ArcBall.drag (mouse_pt)
			theta, self.ex, self.ey, self.ez = quatToEulerAxisAngle(ThisQuat)
			# Convert Quaternion Into Matrix3fT
			self.g_ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)
			# Use correct Linear Algebra matrix multiplication C = A * B
			# Accumulate Last Rotation Into This One
			self.g_ThisRot = Matrix3fMulMatrix3f (self.g_LastRot, self.g_ThisRot)
			# Set Our Final Transform's Rotation From This One
			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)

		return

	## Restructure the aspect ratio when resizing the window
	def resizeGL(self, width, height):
		if height == 0: height = 1

		self.g_ArcBall.setBounds(width, height)

		glViewport(0, 0, width, height)

		self.setCamera()

	## This function is called whenever the widget needs to be painted.
	#  Before invoking this function, the context and the framebuffer are bound, 
	#  and the viewport is set up by a call to glViewport().
	def paintGL(self):
		self.Render()

class MainWindow(QtGui.QMainWindow):

	def __init__(self):
		QtGui.QMainWindow.__init__(self)

		self.resize(WIDTH, HEIGHT)
		self.setWindowTitle('Texture Mapping to Cube')

		glWidget = GLWidget(self)
		self.glWidget = glWidget

		# QT requires a timed refresh when performing animations
		timer = QtCore.QTimer(self)
		timer.setInterval(20)
		QtCore.QObject.connect(timer, QtCore.SIGNAL('timeout()'), glWidget.refresh)
		timer.start()
		
		self.controlScheme = "rotation around x-axis ( %s )" % self.glWidget.rotationDirection
		self.initLayout(glWidget)

	## Initializes the layout of the UI.
	def initLayout(self, glWidget):
		self.createMenus()
		self.createDockWindows()
		self.widget = glWidget

		main_layout = QtGui.QVBoxLayout()
		main_layout.addStretch()

		central_widget = glWidget
		central_widget.setLayout(main_layout)
		self.setCentralWidget(central_widget)	

	## Create the tool bar menu.
	def createMenus(self):

		menubar = self.menuBar()

		exitAction = QtGui.QAction(QtGui.QIcon('exit.png'), '&Exit', self)        
		exitAction.setShortcut('Q')
		exitAction.setStatusTip('Exit application')
		exitAction.triggered.connect(QtGui.qApp.quit)

		openFileAction = QtGui.QAction(QtGui.QIcon('exit.png'), '&Open File', self)        
		openFileAction.setShortcut('Ctrl+O')
		openFileAction.setStatusTip('Open file')
		openFileAction.triggered.connect(self.openFileDialog)

		fileMenu = menubar.addMenu('&File')
		fileMenu.addAction(openFileAction)
		fileMenu.addAction(exitAction)

		self.viewMenu = self.menuBar().addMenu("&View")		

	## Create dockable windows for the controller.
	def createDockWindows(self):
		dock = QtGui.QDockWidget("Options", self)
		# Set allowable areas for where the window can be docked. 
		dock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea | QtCore.Qt.TopDockWidgetArea | QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)
		self.controls = QtGui.QWidget(dock)

		self.buttonX = QtGui.QPushButton('x-axis', self)
		self.buttonY = QtGui.QPushButton('y-axis', self)
		self.buttonZ = QtGui.QPushButton('z-axis', self)
		self.buttonArc = QtGui.QPushButton('arcball', self)

		self.buttonX.clicked.connect(self.setRotX)
		self.buttonY.clicked.connect(self.setRotY)
		self.buttonZ.clicked.connect(self.setRotZ)
		self.buttonArc.clicked.connect(self.setArcball)

		button_layout = QtGui.QHBoxLayout()
		button_layout.addWidget(self.buttonX)
		button_layout.addWidget(self.buttonY)
		button_layout.addWidget(self.buttonZ)
		button_layout.addWidget(self.buttonArc)

		self.controls.setLayout(button_layout)

		dock.setWidget(self.controls)
		self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)
		self.viewMenu.addAction(dock.toggleViewAction())

		dock = QtGui.QDockWidget("Current Action", self)
		self.info = QtGui.QWidget(dock)

		# -------------- Buttons ----------------------- #
		control_layout = QtGui.QGridLayout()
		self.buttonInt = QtGui.QPushButton('Intrinsic', self)
		self.buttonExt = QtGui.QPushButton('Extrinsic', self)
		self.buttonNone = QtGui.QPushButton('ZYX', self)

		self.buttonInt.clicked.connect(self.setIntrinsic)
		self.buttonExt.clicked.connect(self.setExtrinsic)
		self.buttonNone.clicked.connect(self.setNone)

		control_layout.addWidget(self.buttonInt,  1, 0)
		control_layout.addWidget(self.buttonExt,  2, 0)
		control_layout.addWidget(self.buttonNone, 3, 0)

		self.lInfo = QtGui.QLabel()
		self.lInfo.setText("Current control scheme: \n\n" + self.controlScheme)	
		self.lInfo.setAlignment(QtCore.Qt.AlignCenter)

		font = QtGui.QFont()
		font.setBold(True)
		self.lInfo.setFont(font)
		control_layout.addWidget(self.lInfo)

		self.info.setLayout(control_layout)
		# ------------------------------------------------ #

		dock.setWidget(self.info)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
		self.viewMenu.addAction(dock.toggleViewAction())

	## Open an image file to modify the texture.
	def openFileDialog(self):
		filter = "Image Files (*.png *.jpg *.bmp)"
		fileName = QtGui.QFileDialog.getOpenFileNameAndFilter(self, "Open File", "~/Programming/Python/PyQt/TextureMap/images/", filter)

		self.widget.imgName = str(fileName[0])

		if (self.widget.imgName is not None and len(self.widget.imgName) > 0): 
			# reload image
			self.widget.imageID = self.widget.loadImage ()
	
	def setIntrinsic(self):
		self.glWidget.setRotationDirection (0)
		axis = ("x-axis", "y-axis", "z-axis")
		self.controlScheme = "rotation around %s ( %s )" % (axis[self.widget.index], self.glWidget.rotationDirection)

	def setExtrinsic(self):
		self.glWidget.setRotationDirection (1)
		axis = ("x-axis", "y-axis", "z-axis")
		self.controlScheme = "rotation around %s ( %s )" % (axis[self.widget.index], self.glWidget.rotationDirection)

	def setNone(self):
		self.glWidget.setRotationDirection (2)
		axis = ("x-axis", "y-axis", "z-axis")
		self.controlScheme = "rotation around %s ( %s )" % (axis[self.widget.index], self.glWidget.rotationDirection)

	## Clears previous rotations. 
	# Call this function before selecting a new axis for rotation.
	def resetArcball(self):
		self.widget.arcBall = False

	## Set the cube to rotate around its x-axis.
	def setRotX(self):
		""" Set the cube to rotate around its x-axis. """
		self.resetArcball()
		self.widget.setIndex(0)
		self.controlScheme = "rotation around x-axis ( %s )" % self.glWidget.rotationDirection

	## Set the cube to rotate around its y-axis.
	def setRotY(self):
		""" Set the cube to rotate around its y-axis. """
		self.resetArcball()
		self.widget.setIndex(1)
		self.controlScheme = "rotation around y-axis ( %s )" % self.glWidget.rotationDirection

	## Set the cube to rotate around its z-axis.
	def setRotZ(self):
		""" Set the cube to rotate around its z-axis. """
		self.resetArcball()
		self.widget.setIndex(2)
		self.controlScheme = "rotation around z-axis ( %s )" % self.glWidget.rotationDirection

	def setArcball(self):
		self.widget.arcBall = True
		self.controlScheme = "arcball"

def main():
	"""Instantiate the QT objects."""

	app = QtGui.QApplication(sys.argv)	
	win = MainWindow()
	win.show()
	timer = QtCore.QTimer()
	timer.timeout.connect(updateLabel)
	timer.start(500)  

	return app, win, timer

def updateLabel():
	"""Update the information label."""
	win.lInfo.setText("Current control scheme: \n\n" + win.controlScheme)	

if __name__ == "__main__":
	app, win, timer = main()
	sys.exit(app.exec_())
