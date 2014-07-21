#!/usr/bin/env python

## Imports
## ^^^^^^^

## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy
from math import *
## Then load sys to get sys.argv.
import sys
import copy

## Next import all the Qt bindings into the current namespace, for
## convenience. This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed. The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

#Get moving!
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID

## Finally import the RViz bindings themselves.
import rviz
import tf

import rospkg

## The MyViz class is the main container widget.
class MyViz( QWidget ):

	## MyViz Constructor
	def __init__(self):

		QWidget.__init__(self)
	#The visualizer
		self.frame = rviz.VisualizationFrame()
		self.frame.setSplashPath( "" )
		self.frame.initialize()



	## The reader reads config file data into the config object.
		## VisualizationFrame reads its data from the config object.
		reader = rviz.YamlConfigReader()
		config = rviz.Config()

		# We use rospack to find the filepath for remote_nav.
		rospack = rospkg.RosPack()
		package_path = rospack.get_path('remote_nav')
		#Now you can grab this filepath from either roslaunch remote_nav myviz and using the launch file or just rosrun.
		config_file = rospy.get_param('remote_nav/rviz_config', package_path + "/rviz/map_and_img.rviz")
		reader.readFile( config, config_file )
		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
		self.setWindowIcon(QIcon(package_path +'/images/icon.png'))


	#For sending nav goals.
		self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
	#A publisher to literally tell dis bisnatch to cancel all goals.
		self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID)
		self.listener = tf.TransformListener()

	#Is the robot facing forward along our track?
		self.isForward = True
	#Get the track_length for our 1D start track.
		self.track_length = rospy.get_param('remote_nav/track_length', 5.0)
		self.robot_frame = rospy.get_param('remote_nav/robot_frame', "/base_footprint")
	

	#Disable unneeded views and more visualization setup
		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )
		self.manager = self.frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		
	##LAYOUT
	##^^^^^^
	# 	layout = QVBoxLayout()
	# 	layout.addWidget( self.frame )
		layout = QGridLayout()
		layout.setSpacing(10)

		layout.addWidget(self.frame, 1, 0, 4, 3)
		
	 	h_layout = QHBoxLayout()
		
	#Buttons and attached commands
		# 1. Create Button
		# 2. Connect Signal to Slot
		# 3. Add to layout

		self.stop_button = QPushButton( "STOP" )
		self.stop_button.clicked.connect( self.onStopButtonClick )
		self.stop_button.setToolTip('Press this to immediately <b>STOP</b> the robot')
		self.stop_button.setStyleSheet("background-color: #700000 ; font-weight: bold; color: white")
		layout.addWidget( self.stop_button, 6, 1 )
		
		self.fwd_button = PicButton(QPixmap(package_path + "/images/up.png"))
		self.fwd_button.setClickPix(QPixmap(package_path + "/images/upDark.png"))
		# self.fwd_button = QPushButton("Move Forward")
		self.fwd_button.pressed.connect( self.onFwdPress )
		self.fwd_button.setToolTip('While held, the robot will move forward')
		layout.addWidget( self.fwd_button, 4, 1 )
		layout.setAlignment(self.fwd_button, Qt.AlignHCenter)

		turn_button = PicButton(QPixmap(package_path + "/images/rotate.png"))
		turn_button.setClickPix(QPixmap(package_path + "/images/rotateDark.png"))
		# turn_button = QPushButton( "Turn Around[ALEX DEBUG - Nav Goals]" )
		turn_button.clicked.connect( self.onTurnButtonClick )
		turn_button.setToolTip('The robot will turn around 180 degrees')




		look_left_btn = PicButton(QPixmap(package_path + "/images/left.png"))
		look_left_btn.setClickPix(QPixmap(package_path + "/images/leftDark.png"))
		# layout.addWidget(look_left_btn, 2, 0)
		# layout.setAlignment(look_left_btn, Qt.AlignLeft)

		look_right_btn = PicButton(QPixmap(package_path + "/images/right.png"))
		look_right_btn.setClickPix(QPixmap(package_path + "/images/rightDark.png"))
		# layout.addWidget(look_right_btn, 2, 2)
		# layout.setAlignment(look_right_btn, Qt.AlignRight)
		

		#Finalizing layout and placing components
		h_layout.addWidget(look_left_btn)
		h_layout.setAlignment(look_left_btn, Qt.AlignRight)
		h_layout.addWidget(turn_button)
		h_layout.setAlignment(turn_button, Qt.AlignHCenter)
		h_layout.addWidget(look_right_btn)
		h_layout.setAlignment(look_right_btn, Qt.AlignLeft)

		layout.addLayout( h_layout, 5, 1 )	
		self.setLayout( layout )


## Handle GUI events
## ^^^^^^^^^^^^^^^^^

	def closeEvent(self, event):

		reply = QMessageBox.question(self, 'Message',
		"Are you sure to quit?", QMessageBox.Yes | 
		QMessageBox.No, QMessageBox.No)

		if reply == QMessageBox.Yes:
			event.accept()
		else:
			event.ignore()

	def switchToView( self, view_name ):
		view_man = self.manager.getViewManager()
		for i in range( view_man.getNumViews() ):
			if view_man.getViewAt( i ).getName() == view_name:
				view_man.setCurrentFrom( view_man.getViewAt( i ))
				return
		print( "Did not find view named %s." % view_name )

	# BUTTON CALLBACKS
	# ^^^^^^^^^^^^^^^^
	def onFwdPress(self):
		self.moveNav()


	def onDebugButtonClick(self):
	#Tells robot to return to home base.
		goal = self._get_start_pose()

		self._send_nav_goal(goal)
		self.isForward = True

	def onStopButtonClick(self):
		QApplication.processEvents()
		self._cancel_goals()

#
	def onTurnButtonClick(self):
		if self.isForward:
			self.faceBackward()
		else:
			self.faceForward()
	def onTurnTwistButtonClick(self):
		self.turnAround()

	def onResetDirButtonClick(self):
		self.faceForward()

## NAVIGATION FUNCTIONS
## ^^^^^^^^^^^^^^^^^^^^
	#Face forward along our track.
	def faceForward(self):
		self.isForward = True
		goal = self._get_pose_from_start()
		goal.pose.position.y = 0
		goal.pose.orientation.z = 0.0
		goal.pose.orientation.w = 1.0

		self._send_nav_goal(goal)
		print ("Now facing forward.")

	#Face 180 degrees from the forward position.
	def faceBackward(self):
		self.isForward = False
		#Grab where we currently are.
		goal = self._get_pose_from_start()
		#realign and turn around.
		goal.pose.position.y = 0
		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)
		print ("Now facing backward.")
	def moveNav(self):
		if (self.isForward):
			goal = self._get_end_pose()
			self._send_nav_goal(goal)
		else:
			goal = PoseStamped()
			goal.header.frame_id = "/start"
			goal.pose.orientation.z = 1.0
			goal.pose.orientation.w = 0.0

			self._send_nav_goal(goal)

		while self.fwd_button.isDown():
			QApplication.processEvents()
		self._cancel_goals()
		# if self.isForward:
		# 	self.faceForward()
		# else:
		# 	self.faceBackward()



#PRIVATE FUNCTIONS
#^^^^^^^^^^^^^^^^
	# Sends a nav goal to the bot. This is like sending it a position in space to go.
	# If it is necessary to transform to the /map frame, specify so with transform=True
	def _send_nav_goal(self, pose):
		self.nav_pub.publish(pose)

	#Returns transform of robot relative to /start pose.
	def _get_pose_from_start(self):
		(trans, rot) = self.listener.lookupTransform("/start", self.robot_frame, rospy.Time(0))
		start_trans = PoseStamped()
		start_trans.header.frame_id = "/start"
		start_trans.header.stamp = rospy.Time.now()
		start_trans.pose.position.x = trans[0]
		start_trans.pose.position.y = trans[1]
		start_trans.pose.position.z = trans[2]
		start_trans.pose.orientation.x = rot[0]
		start_trans.pose.orientation.y = rot[1]
		start_trans.pose.orientation.z = rot[2]
		start_trans.pose.orientation.w = rot[3]
		return start_trans
	#Get start frame's pose with parent frame /map.
	def _get_start_pose(self):
		(trans, rot) = self.listener.lookupTransform("/map","/start", rospy.Time(0))
		goal = PoseStamped()
		goal.header.frame_id = "/map"
		goal.pose.position.x = trans[0]
		goal.pose.position.y = trans[1]

		goal.pose.orientation.z = rot[2]
		goal.pose.orientation.w = rot[3]
		return goal
	#Returns the pose of the END of the track defined by start.
	def _get_end_pose(self):
		pose = PoseStamped()
		pose.header.frame_id = "/start"
		pose.pose.position.x += self.track_length
		pose.pose.orientation.w = 1.0
		return pose

	def _cancel_goals(self):
		goalID = GoalID()
		self.cancel_pub.publish(goalID)


#SPECIAL SNOWFLAKE CLASSES
#^^^^^^^^^^^^^^^^
class PicButton(QAbstractButton):
	def __init__(self, pixmap, parent=None):
		super(PicButton, self).__init__(parent)
		self.pixmap = pixmap
		self.clickpix = pixmap
		if self.pixmap.width() > 50:
			self.pixmap = self.pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)

	def setClickPix(self, pixmap2):
		self.clickpix = pixmap2

	def paintEvent(self, event):
		painter = QPainter(self)
		if self.isDown():
			painter.drawPixmap(event.rect(), self.clickpix)
		else:
			painter.drawPixmap(event.rect(), self.pixmap)

	def sizeHint(self):
		return self.pixmap.size()


## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
if __name__ == '__main__':
	app = QApplication( sys.argv )
	rospy.init_node('move')

	myviz = MyViz()
	myviz.resize( 1000, 500 )
	myviz.show()

	app.exec_()
