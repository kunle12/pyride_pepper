/*
 *  PyPepperModule.cpp
 *
 */

#include <pthread.h>
#include <string>
#include "PyPepperModule.h"
#include "PepperProxyManager.h"

namespace pyride {

#define PYRIDE_ROBOT_MODEL  "Pepper"

PyDoc_STRVAR( PyPepper_doc, \
             "PyPepper is an extension module for PyRIDE on Pepper robot." );

/*! \class PyPepper
 *  \brief PyPepper is the main Python extension module of PyRIDE on Aldebaran Pepper robot.
 *
 *  PyPepper module consists of a set of callable Python methods specifically related to
 *  Pepper low level functionalities and a set of callback functions that should be implemented
 *  by Pepper programmers.
 */

PyPepperModule * PyPepperModule::s_pyPepperModule = NULL;

static const char *kLeftArmKWlist[] = { "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_elbow_yaw_joint", "l_elbow_roll_joint", "l_wrist_yaw_joint",
    "frac_max_speed", "is_blocking", NULL };
static const char *kRightArmKWlist[] = { "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_elbow_yaw_joint", "r_elbow_roll_joint", "r_wrist_yaw_joint",
    "frac_max_speed", "is_blocking", NULL };

static const char *kLegKWlist[] = { "hip_roll_joint", "hip_pitch_joint", "knee_pitch_joint",
    "frac_max_speed", NULL };

static const char *kBodyRawJointDataKWlist[] = { "joints", "keyframes", "timestamps", "is_blocking", NULL };

static const char *kBodyKWlist[] = { "head_yaw_joint", "head_pitch_joint",
  "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_elbow_yaw_joint",
  "l_elbow_roll_joint", "l_wrist_yaw_joint", "l_hand_joint",
  "hip_roll_joint", "hip_pitch_joint", "knee_pitch_joint",
  "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_elbow_yaw_joint",
  "r_elbow_roll_joint", "r_wrist_yaw_joint", "r_hand_joint",
  "frac_max_speed", NULL };

// helper function

static bool colourStr2ID( const char * colourStr, NAOLedColour & colourID )
{
  if (!colourStr)
    return false;

  if (!strcasecmp( colourStr, "red" )) {
    colourID = RED;
  }
  else if (!strcasecmp( colourStr, "green" )) {
    colourID = GREEN;
  }
  else if (!strcasecmp( colourStr, "blue" )) {
    colourID = BLUE;
  }
  else if (!strcasecmp( colourStr, "white" )) {
    colourID = WHITE;
  }
  else if (!strcasecmp( colourStr, "yellow" )) {
    colourID = YELLOW;
  }
  else if (!strcasecmp( colourStr, "pink" )) {
    colourID = PINK;
  }
  else if (!strcasecmp( colourStr, "blank" )) {
    colourID = BLANK;
  }
  else {
    return false;
  }
  return true;
}

static bool parseIntFloatObj( PyObject * obj, float & number )
{
  if (!obj)
    return false;

  if (PyFloat_Check( obj )) {
    number = PyFloat_AsDouble( obj );
  }
  else if (PyInt_Check( obj )) {
    number = (float)PyInt_AsLong( obj );
  }
  else {
    return false;
  }
  return true;
}

static bool parseIntObj( PyObject * obj, int & number )
{
  if (!obj)
    return false;

  if (PyInt_Check( obj )) {
    number = (int)PyInt_AsLong( obj );
  }
  else {
    return false;
  }
  return true;
}

static PyObject * PyModule_write( PyObject *self, PyObject * args )
{
  char * msg;
  std::string outputMsg;

  if (!PyArg_ParseTuple( args, "s", &msg )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  // Next send it to all (active) clients.
  while (*msg) {
    if (*msg == '\n')
      outputMsg += "\r\n";
    else
      outputMsg += *msg;
    msg++;
  }

  PyPepperModule::instance()->write( outputMsg.c_str() );

  Py_RETURN_NONE;
}

static PyObject * PyModule_SetTeamMemberID( PyObject *self, PyObject * args )
{
  int teamID, teamColour;
  if (!PyArg_ParseTuple( args, "ii", &teamID, &teamColour )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (teamID < 0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setTeamMemberID: invalid "
                 "team member ID %d!", teamID );

    return NULL;
  }
  if (teamColour < BlueTeam || teamColour > PinkTeam) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setTeamMemberID: invalid "
                 "team colour %d! Valid format: %d = Pink; %d = Blue", teamColour, PinkTeam, BlueTeam );

    return NULL;
  }

  ServerDataProcessor::instance()->setTeamMemberID( teamID, (TeamColour)teamColour );
  PyPepperModule::instance()->clientID( ServerDataProcessor::instance()->clientID() );

  Py_RETURN_NONE;
}

static PyObject * PyModule_sendTeamMessage( PyObject *self, PyObject * args )
{
  char * dataStr = NULL;

  if (!PyArg_ParseTuple( args, "s", &dataStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  PyPepperModule::instance()->sendTeamMessage( dataStr );
  Py_RETURN_NONE;
}

/*! \fn say(text, volume, animated, is_blocking)
 *  \memberof PyPepper
 *  \brief Let Pepper to speak an input text.
 *  \param string text. Text that be spoken by Pepper
 *  \param bool animated. True == speak with arm movements; False == plain speaking. Optional, default is False.
 *  \param bool is_blocking. True == blocking function call; False == Non-blocking call. Optional, default is False.
 *  \return None.
 */
static PyObject * PyModule_PepperSayWithVolume( PyObject * self, PyObject * args )
{
  char * dataStr = NULL;
  bool toBlock = false;
  bool toAnimate = false;

  PyObject * toAnimateObj = NULL;
  PyObject * toBlockObj = NULL;

  if (!PyArg_ParseTuple( args, "s|OO", &dataStr, &toAnimateObj, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (toAnimateObj) {
    if (PyBool_Check( toAnimateObj )) {
      toAnimate = PyObject_IsTrue( toAnimateObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.say: second parameter must be a boolean!" );
      return NULL;
    }
  }
  if (toBlockObj) {
    if (PyBool_Check( toBlockObj )) {
      toBlock = PyObject_IsTrue( toBlockObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.say: third parameter must be a boolean!" );
      return NULL;
    }
  }

  if (dataStr) {
    PepperProxyManager::instance()->say( string( dataStr ),
                                               toAnimate, toBlock );
  }
  Py_RETURN_NONE;
}

/*! \fn moveHeadTo(head_yaw, head_pitch, relative, frac_speed)
 *  \memberof PyPepper
 *  \brief Move the Pepper head to a specific yaw and pitch position. This new position could be either absolute head position or a relative position w.r.t the current head position.
 *  \param float head_yaw. Must be in radian.
 *  \param float head_pitch. Must be in radian.
 *  \param bool relative. True == relative angle values; False == absolute angle values. Optional, default is False.
 *  \param float frac_speed. Fraction of the maximum speed, i.e. within (0..1] Optional, default = 0.05.
 *  \return None.
 */
static PyObject * PyModule_PepperMoveHeadTo( PyObject * self, PyObject * args )
{
  float yaw = 0.0;
  float pitch = 0.0;
  bool absolute = false;
  float frac_speed = 0.05;
  bool isRelative = false;
  PyObject * isYesObj = NULL;

  if (!PyArg_ParseTuple( args, "ff|Of", &yaw, &pitch, &isYesObj, &frac_speed )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (frac_speed <= 0 || frac_speed > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveHeadTo: frac_speed must be within (0..1]." );
    return NULL;
  }

  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isRelative = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveHeadTo: the third optional parameter must be a boolean!" );
      return NULL;
    }
  }

  PepperProxyManager::instance()->moveHeadTo( yaw, pitch, isRelative, frac_speed );
  Py_RETURN_NONE;
}

static PyObject * PyModule_PepperUpdateHeadPos( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;
  double speed = 0.1;

  if (!PyArg_ParseTuple( args, "dd|f", &yaw, &pitch, &speed )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  PepperProxyManager::instance()->updateHeadPos( yaw, pitch, speed );
  Py_RETURN_NONE;
}

/*! \fn gotoStation()
 *  \memberof PyPepper
 *  \brief Move the robot into its charging station.
 *  \return None.
 */
static PyObject * PyModule_PepperGotoStation( PyObject * self )
{
  PepperProxyManager::instance()->gotoStation();
  Py_RETURN_NONE;
}

/*! \fn leaveStation()
 *  \memberof PyPepper
 *  \brief Move the robot off its charging station.
 *  \return None.
 */
static PyObject * PyModule_PepperLeaveStation( PyObject * self )
{
  PepperProxyManager::instance()->leaveStation();
  Py_RETURN_NONE;
}

/*! \fn stand(is_init)
 *  \memberof PyPepper
 *  \brief Move the robot into a standing pose.
 *  \param bool is_init. Optional. True = A standing pose ready for walk; False = Default standing pose.
 *  \return None.
 */
static PyObject * PyModule_PepperStand( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.stand: the parameter must be a boolean!" );
      return NULL;
    }
  }
  PepperProxyManager::instance()->stand( isYes );
  Py_RETURN_NONE;
}

/*! \fn navigateBodyTo(target_position)
 *  \memberof PyPepper
 *  \brief Navigate Pepper body to a specified position.
 *  \param tuple target_position. Position w.r.t the current pose in the form of (x,y).
 *  \return bool. True == navigation successful; False == navigation failed.
 */
static PyObject * PyModule_PepperNavigateBodyTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;

  if (!PyArg_ParseTuple( args, "ff", &xcoord, &ycoord )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.navigateBodyTo: input position parameters." );
    return NULL;
  }

  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = 0.0;

  if (PepperProxyManager::instance()->navigateBodyTo( pose ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyTo(x,y,theta,best_timecancel_previous_move, is_blocking)
 *  \memberof PyPepper
 *  \brief Move the PR2 body to a pose at (x,y,theta) w.r.t the current pose.
 *  \param float x. X coordinate w.r.t. the current pose.
 *  \param float y. Y coordinate w.r.t. the current pose.
 *  \param float theta. Angular position w.r.t. the current pose.
 *  \param float best_time. Optional, ask the robot try its best to reach the input pose in this time frame.
 *  \param bool cancel_previous_move. Optional cancel previous move command if it is still executing (default False).
 *  \param bool is_blocking. Optional. True = blocking call; False = unblocking call (Default False).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveBodyTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  float bestTime = 5.0; //seconds

  bool cancelMove = false;
  bool inpost = true;

  PyObject * boolObj = NULL;
  PyObject * isBlockObj = NULL;

  if (!PyArg_ParseTuple( args, "fff|fOO", &xcoord, &ycoord, &theta, &bestTime, &boolObj, &isBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (!PyBool_Check( boolObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyTo: optional input parameters must be a boolean!" );
      return NULL;
    }
    cancelMove = PyObject_IsTrue( boolObj );
  }
  if (isBlockObj) {
    if (!PyBool_Check( isBlockObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyTo: optional input parameters must be a boolean!" );
      return NULL;
    }
    inpost = !PyObject_IsTrue( isBlockObj );
  }

  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;

  if (PepperProxyManager::instance()->moveBodyTo( pose, bestTime, cancelMove, inpost ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn crouch()
 *  \memberof PyPepper
 *  \brief Move the robot into a crouch pose.
 *  \return None.
 *  \note Stiffness will be turned off.
 */
static PyObject * PyModule_PepperCrouch( PyObject * self )
{
  PepperProxyManager::instance()->crouch();
  Py_RETURN_NONE;
}

/*! \fn getHeadPos()
 *  \memberof PyPepper
 *  \brief Get the current robot head yaw and pitch in radian.
 *  \return tuple(head_yaw, head_pitch)
 */
static PyObject * PyModule_PepperGetHeadPos( PyObject * self )
{
  float yaw = 0.0;
  float pitch = 0.0;
  if (PepperProxyManager::instance()->getHeadPos( yaw, pitch )) {
    return Py_BuildValue( "(ff)", yaw, pitch );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPepper.getHeadPos: unable to get head position!" );
    return NULL;
  }
}

/*! \fn setHeadStiffness(stiffness)
 *  \memberof PyPepper
 *  \brief Set the stiffness of the Pepper head.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_PepperSetHeadStiffness( PyObject * self, PyObject * args )
{
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "f", &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setHeadStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  PepperProxyManager::instance()->setHeadStiffness( stiff );
  Py_RETURN_NONE;
}

/*! \fn setBodyStiffness(stiffness)
 *  \memberof PyPepper
 *  \brief Set the stiffness of the Pepper body.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_PepperSetBodyStiffness( PyObject * self, PyObject * args )
{
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "f", &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setBodyStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  PepperProxyManager::instance()->setBodyStiffness( stiff );
  Py_RETURN_NONE;
}

/*! \fn setArmStiffness(left_arm,stiffness)
 *  \memberof PyPepper
 *  \brief Set the stiffness of a Pepper' arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_PepperSetArmStiffness( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "Of", &isYesObj, &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( isYesObj )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setArmStiffness: the first parameter must be a boolean!" );
    return NULL;
  }
  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setArmStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  PepperProxyManager::instance()->setArmStiffness( PyObject_IsTrue( isYesObj ), stiff );
  Py_RETURN_NONE;
}

/*! \fn setLegStiffness(left_leg,stiffness)
 *  \memberof PyPepper
 *  \brief Set the stiffness of a Pepper' leg.
 *  \param bool left_leg. True for left leg; False for right leg.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_PepperSetLegStiffness( PyObject * self, PyObject * args )
{
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "f", &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setLegStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  PepperProxyManager::instance()->setLegStiffness( stiff );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectory(joint_trajectory,is_blocking)
 *  \memberof PyPepper
 *  \brief Move a Pepper arm to a sequence of joint positions, i.e. trajectory.
 *  \param list joint_trajectory. A list of joint position dictionaries with the same structure of the PyPepper.moveArmWithJointPos.
 *  \param bool is_blocking. Optional. True = blocking call; False = unblocking call (Default false).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveArmWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  PyObject * isYesObj = NULL;
  bool inpost = true;

  if (!PyArg_ParseTuple( args, "O|O", &trajObj, &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  int listSize = 0;

  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }

  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      inpost = !PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: the second parameter must be a boolean!" );
      return NULL;
    }
  }

  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right

  std::vector< std::vector<float> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 4) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 4 joint entries for a Pepper arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: input list contains"
                     " values not related to Pepper arms!" );
        return NULL;
      }
    }

    std::vector<float> arm_joint_pos( 5, 0.0 );

    for (int k = 0; k < 5; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( arm_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }

  if (PepperProxyManager::instance()->moveArmWithJointTrajectory( (armsel == 1), trajectory, times_to_reach, inpost ) )
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveArmWithJointPos(joint_position, frac_max_speed, is_blocking)
 *  \memberof PyPepper
 *  \brief Move a Pepper's arm to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of arm joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getArmJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \param bool is_blocking. Optional. True = blocking call; False = unblocking call (Default False).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveArmWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float s_p_j, s_r_j, e_y_j, e_r_j, w_y_j;
  float frac_max_speed = 0.5;
  PyObject * boolObj = NULL;

  bool isLeftArm = false;
  bool inpost = true;

  if (PyArg_ParseTupleAndKeywords( args, keywds, "fffff|fO", (char**)kLeftArmKWlist,
                                  &s_p_j, &s_r_j, &e_y_j, &e_r_j, &w_y_j, &frac_max_speed, &boolObj ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "fffff|fO", (char**)kRightArmKWlist,
                                     &s_p_j, &s_r_j, &e_y_j, &e_r_j, &w_y_j, &frac_max_speed, &boolObj ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }

  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }

  if (boolObj) {
    if (PyBool_Check( boolObj )) {
      inpost = !PyObject_IsTrue( boolObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointPos: is_blocking parameter must be a boolean!" );
      return NULL;
    }
  }

  std::vector<float> positions( 5, 0.0 );
  positions[0] = s_p_j;
  positions[1] = s_r_j;
  positions[2] = e_y_j;
  positions[3] = e_r_j;
  positions[4] = w_y_j;

  if (PepperProxyManager::instance()->moveArmWithJointPos( isLeftArm, positions, frac_max_speed, inpost ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveLegWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyPepper
 *  \brief Move a Pepper's leg to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of leg joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getLegJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveLegWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float h_r_j, h_p_j, k_p_j;
  float frac_max_speed = 0.5;

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "fff|f", (char**)kLegKWlist,
                                  &h_r_j, &h_p_j, &k_p_j, &frac_max_speed ))
  {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveLegWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }

  std::vector<float> positions( 3, 0.0 );
  positions[0] = h_r_j;
  positions[1] = h_p_j;
  positions[2] = k_p_j;

  if (PepperProxyManager::instance()->moveLegWithJointPos( positions, frac_max_speed ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyPepper
 *  \brief Move the Pepper body joints to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of body joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getBodyJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveBodyWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float bh_r_j, bh_p_j, bk_p_j;
  float l_s_p_j, l_s_r_j, l_e_y_j, l_e_r_j, l_w_y_j, l_hand_j;
  float r_s_p_j, r_s_r_j, r_e_y_j, r_e_r_j, r_w_y_j, r_hand_j;
  float h_y_j, h_p_j;

  float frac_max_speed = 0.5;

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "fffffffffffffffff|f",
                                   (char**)kBodyKWlist, &h_y_j, &h_p_j,
                                   &l_s_p_j, &l_s_r_j, &l_e_y_j, &l_e_r_j, &l_w_y_j, &l_hand_j,
                                   &bh_r_j, &bh_p_j, &bk_p_j,
                                   &r_s_p_j, &r_s_r_j, &r_e_y_j, &r_e_r_j, &r_w_y_j, &r_hand_j,
                                   &frac_max_speed ))
  {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }

  std::vector<float> positions( 17, 0.0 );
  positions[0] = h_y_j;
  positions[1] = h_p_j;

  positions[2] = l_s_p_j;
  positions[3] = l_s_r_j;
  positions[4] = l_e_y_j;
  positions[5] = l_e_r_j;
  positions[6] = l_w_y_j;
  positions[7] = l_hand_j;

  positions[8] = bh_r_j;
  positions[9] = bh_p_j;
  positions[10] = bk_p_j;

  positions[11] = r_s_p_j;
  positions[12] = r_s_r_j;
  positions[13] = r_e_y_j;
  positions[14] = r_e_r_j;
  positions[15] = r_w_y_j;
  positions[16] = r_hand_j;

  if (PepperProxyManager::instance()->moveBodyWithJointPos( positions, frac_max_speed ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyWithRawTrajectoryData(joint_trajectory_data)
 *  \memberof PyPepper
 *  \brief Move the Pepper body joints in specified trajectories.
 *  \param dict joint_trajectory_data. A dictionary of {joints, keyframes, timestamps, is_blocking} where
 *  joints is a list of joint names that are factory defined, keyframes is a list of corresponding joint values (trajectory)
 *  for each joint specified in joints; timestamps is a list of corresponding time to reach values for the keyframes
 *  for each joint specified in joints; is_blocking is a boolean for whether the call is blocking.
 *  \return bool. True == valid command; False == invalid command.
 *  \warning This method is not for general use. You need to know what you are doing.
 */
static PyObject * PyModule_PepperMoveBodyWithRawTrajectoryData( PyObject * self, PyObject * args, PyObject * keywds )
{
  PyObject * jointsObj = NULL;
  PyObject * timesObj = NULL;
  PyObject * keyframesObj = NULL;
  PyObject * boolObj = NULL;
  bool inpost = false;
  bool isbezier = false;

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "OOO|O",
                                   (char**)kBodyRawJointDataKWlist, &jointsObj, &keyframesObj, &timesObj, &boolObj ))
  {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  int listSize = 0, sublistSize = 0;
  PyObject * obj = NULL;

  if (!PyList_Check( jointsObj ) || (listSize = PyList_Size( jointsObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: joint list parameter must be a non empty list of joint names defined by Pepper spec!" );
    return NULL;
  }

  if (!PyList_Check( keyframesObj ) || (PyList_Size( keyframesObj )) != listSize) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: keyframes parameter must be a list of joint values that matches with input joints!" );
    return NULL;
  }

  if (!PyList_Check( timesObj ) || (PyList_Size( timesObj )) != listSize) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: times parameter must be a list of timestamps that matches with input joints!" );
    return NULL;
  }

  std::vector<std::string> joint_names;
  for (int i = 0; i < listSize; ++i) {
    obj = PyList_GetItem( jointsObj, i );
    if (!PyString_Check( obj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d "
                   "must be a string that corresponding to a Pepper joint!", i );
      return NULL;
    }
    joint_names.push_back( PyString_AsString( obj ) );
  }

  std::vector< std::vector<AngleControlPoint> > key_frames;
  for (int i = 0; i < listSize; ++i) {
    obj = PyList_GetItem( keyframesObj, i );
    if (!PyList_Check( obj ) || (sublistSize = PyList_Size( obj )) == 0) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d "
                   "must be a list of joint values for Pepper joint %s!", i, joint_names[i].c_str() );
      return NULL;
    }
    std::vector<AngleControlPoint> joint_values;
    joint_values.resize( sublistSize );
    // check the first element to determine whether the inputs are bezier control points.
    isbezier = PyList_Check( PyList_GetItem( obj, 0 ) );
    PyObject * jval = NULL;
    for (int j = 0; j < sublistSize; ++j) {
      jval = PyList_GetItem( obj, j );
      if (isbezier) {
        if (PyList_Check( jval ) && PyList_Size( jval ) == 3) {
          PyObject * val0 = PyList_GetItem( jval, 0 );
          PyObject * val1 = PyList_GetItem( jval, 1 );
          PyObject * val2 = PyList_GetItem( jval, 2 );

          if (!parseIntFloatObj( val0, joint_values[j].angle )) {
            PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d has an"
                         " invalid angle value in joint control point for Pepper joint %s!", j, joint_names[i].c_str() );
            return NULL;
          }
          if (!PyList_Check( val1 ) || PyList_Size( val1 ) != 3 ||
              !PyList_Check( val2 ) || PyList_Size( val2 ) != 3)
          {
            PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d has an"
                         " invalid bezier parameters in joint control point for Pepper joint %s!", j, joint_names[i].c_str() );
            return NULL;
          }
          PyObject * v0 = PyList_GetItem( val1, 0 );
          PyObject * v1 = PyList_GetItem( val1, 1 );
          PyObject * v2 = PyList_GetItem( val1, 2 );

          if (!parseIntObj( v0, joint_values[j].bparam1.type ) ||
              !parseIntFloatObj( v1, joint_values[j].bparam1.dtime ) ||
              !parseIntFloatObj( v2, joint_values[j].bparam1.dangle ) )
          {
            PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d has an"
                         " invalid first bezier parameter in joint control point for Pepper joint %s!", j, joint_names[i].c_str() );
            return NULL;
          }
          v0 = PyList_GetItem( val2, 0 );
          v1 = PyList_GetItem( val2, 1 );
          v2 = PyList_GetItem( val2, 2 );

          if (!parseIntObj( v0, joint_values[j].bparam2.type ) ||
              !parseIntFloatObj( v1, joint_values[j].bparam2.dtime ) ||
              !parseIntFloatObj( v2, joint_values[j].bparam2.dangle ) )
          {
            PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d has an"
                         " invalid second bezier parameter in joint control point for Pepper joint %s!", j, joint_names[i].c_str() );
            return NULL;
          }
        }
        else {
          PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d is an"
                       " invalid joint control point for Pepper joint %s!", j, joint_names[i].c_str() );
          return NULL;
        }
      }
      else {
        if (!parseIntFloatObj( jval, joint_values[j].angle )) {
          PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d is an"
                       " invalid joint value for Pepper joint %s!", j, joint_names[i].c_str() );
          return NULL;
        }
      }
    }
    key_frames.push_back( joint_values );
  }

  std::vector< std::vector<float> > time_stamps;
  for (int i = 0; i < listSize; ++i) {
    obj = PyList_GetItem( timesObj, i );
    if (!PyList_Check( obj ) || (sublistSize = PyList_Size( obj )) == 0) {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d "
                   "must be a list of timestamps for Pepper joint %s!", i, joint_names[i].c_str() );
      return NULL;
    }
    std::vector<float> time_values( sublistSize, 0.0 );
    PyObject * tval = NULL;
    for (int j = 0; j < sublistSize; ++j) {
      tval = PyList_GetItem( obj, j );
      if (!parseIntFloatObj( tval, time_values[j] )) {
        PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: input list item %d is an"
                     " invalid timestamp for Pepper joint %s!", j, joint_names[i].c_str() );
        return NULL;
      }
    }
    time_stamps.push_back( time_values );
  }

  if (boolObj) {
    if (PyBool_Check( boolObj )) {
      inpost = !PyObject_IsTrue( boolObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveBodyWithRawTrajectoryData: the last parameter must be a boolean!" );
      return NULL;
    }
  }

  if (PepperProxyManager::instance()->moveBodyWithRawTrajectoryData( joint_names, key_frames, time_stamps, isbezier, inpost ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn openHand(which_hand, keep_stiffness)
 *  \memberof PyPepper
 *  \brief Opens one or both Pepper hands.
 *  \param int which_hand. 1 = left hand, 2 = right hand and 3 = both hand.
 *  \param bool keep_stiffness. Optional, keep stiffness on after opening the hand (default False).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperOpenHand( PyObject * self, PyObject * args )
{
  int mode = 0;
  bool keepStiffness = false;
  PyObject * boolObj = NULL;

  if (!PyArg_ParseTuple( args, "i|O", &mode, &boolObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (!PyBool_Check( boolObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.openHand: last optional input parameter must be a boolean!" );
      return NULL;
    }
    keepStiffness = PyObject_IsTrue( boolObj );
  }

  switch (mode) {
    case 1:
      if (PepperProxyManager::instance()->setHandPosition( true, 1.0, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 2:
      if (PepperProxyManager::instance()->setHandPosition( false, 1.0, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 3:
      if (PepperProxyManager::instance()->setHandPosition( true, 1.0, keepStiffness ) &&
          PepperProxyManager::instance()->setHandPosition( false, 1.0, keepStiffness ))
      {
        Py_RETURN_TRUE;
      }
      break;
    default:
      PyErr_Format( PyExc_ValueError, "PyPepper.openHand: invalid hand number! 1 = left hand, 2 = right hand and 3 = both hand." );
      return NULL;
  }
  Py_RETURN_FALSE;
}

/*! \fn closeHand(which_hand, keep_stiffness)
 *  \memberof PyPepper
 *  \brief Closes one or both Pepper hands.
 *  \param int which_hand. 1 = left hand, 2 = right hand and 3 = both hands.
 *  \param bool keep_stiffness. Optional, keep stiffness on after closing the hand (default False).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperCloseHand( PyObject * self, PyObject * args )
{
  int mode = 0;
  bool keepStiffness = false;
  PyObject * boolObj = NULL;

  if (!PyArg_ParseTuple( args, "i|O", &mode, &boolObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (!PyBool_Check( boolObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.closeHand: last optional input parameter must be a boolean!" );
      return NULL;
    }
    keepStiffness = PyObject_IsTrue( boolObj );
  }

  switch (mode) {
    case 1:
      if (PepperProxyManager::instance()->setHandPosition( true, 0.0, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 2:
      if (PepperProxyManager::instance()->setHandPosition( false, 0.0, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 3:
      if (PepperProxyManager::instance()->setHandPosition( true, 0.0, keepStiffness ) &&
          PepperProxyManager::instance()->setHandPosition( false, 0.0, keepStiffness ))
      {
        Py_RETURN_TRUE;
      }
      break;
    default:
      PyErr_Format( PyExc_ValueError, "PyPepper.closeHand: invalid hand number! 1 = left hand, 2 = right hand and 3 = both hand." );
      return NULL;
  }
  Py_RETURN_FALSE;
}

/*! \fn setHandPosition(which_hand, hand_joint_ratio, keep_stiffness)
 *  \memberof PyPepper
 *  \brief open one of Pepper hands to the specified ratio [0..1.0].
 *  \param int which_hand. 1 = left hand, 2 = right hand and 3 = both hands.
 *  \param float hand_joint_ratio. Hand opening ratio [0..1.0].
 *  \param bool keep_stiffness. Optional, keep stiffness on after closing the hand (default False).
 */
static PyObject * PyModule_PepperSetHandPosition( PyObject * self, PyObject * args, PyObject * keywds  )
{
  int mode = 0;
  float ratio = 0.0;
  PyObject * boolObj = NULL;

  bool keepStiffness = false;

  if (!PyArg_ParseTuple( args, "if|O", &mode, &ratio, &boolObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (ratio < 0.0 || ratio > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setHandPosition: ratio parameter must be within [0..1.0]!" );
    return NULL;
  }

  if (boolObj) {
    if (!PyBool_Check( boolObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.setHandPosition: last optional input parameter must be a boolean!" );
      return NULL;
    }
    keepStiffness = PyObject_IsTrue( boolObj );
  }

  switch (mode) {
    case 1:
      if (PepperProxyManager::instance()->setHandPosition( true, ratio, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 2:
      if (PepperProxyManager::instance()->setHandPosition( false, ratio, keepStiffness ))
        Py_RETURN_TRUE;
      break;
    case 3:
      if (PepperProxyManager::instance()->setHandPosition( true, ratio, keepStiffness ) &&
          PepperProxyManager::instance()->setHandPosition( false, ratio, keepStiffness ))
      {
        Py_RETURN_TRUE;
      }
      break;
    default:
      PyErr_Format( PyExc_ValueError, "PyPepper.setHandPosition: invalid hand number! 1 = left hand, 2 = right hand and 3 = both hand." );
      return NULL;
  }
  Py_RETURN_FALSE;
}

/*! \fn getArmJointPositions(left_arm)
 *  \memberof PyPepper
 *  \brief Get the current joint positions of one of the Pepper arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \return dictionary(arm_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_PepperGetArmJointPositions( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  PyObject * usbObj = NULL;

  bool useSensor = false;

  if (!PyArg_ParseTuple( args, "O|O", &armsel, &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.getArmJointPositions: first input parameter must be a boolean!" );
    return NULL;
  }

  bool isLeftArm = PyObject_IsTrue( armsel );

  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.getArmJointPositions: second input parameter must be a boolean!" );
      return NULL;
    }
  }

  std::vector<float> positions( 5, 0.0 );

  PepperProxyManager::instance()->getArmJointsPos( isLeftArm, positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 5; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, (isLeftArm ? kLeftArmKWlist[i] : kRightArmKWlist[i]), numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

/*! \fn getLegJointPositions()
 *  \memberof PyPepper
 *  \brief Get the current joint positions of the Pepper leg.
 *  \return dictionary(leg_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_PepperGetLegJointPositions( PyObject * self, PyObject * args )
{
  PyObject * usbObj = NULL;

  bool useSensor = false;

  if (!PyArg_ParseTuple( args, "|O", &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.getLegJointPositions: input parameter must be a boolean!" );
      return NULL;
    }
  }

  std::vector<float> positions( 3, 0.0 );

  PepperProxyManager::instance()->getLegJointsPos( positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 3; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, kLegKWlist[i], numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

/*! \fn getBodyJointPositions(use_sensor_data)
 *  \memberof PyPepper
 *  \brief Get the current joint positions of Pepper body joints.
 *  \param bool use_sensor_data. True = using sensor reading; False = not using sensor.
 *  \return dictionary(body_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_PepperGetBodyJointPositions( PyObject * self, PyObject * args )
{
  PyObject * usbObj = NULL;

  bool useSensor = false;

  if (!PyArg_ParseTuple( args, "|O", &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.getBodyJointPositions: the optional input parameter must be a boolean!" );
      return NULL;
    }
  }

  std::vector<float> positions( 17, 0.0 );

  PepperProxyManager::instance()->getBodyJointsPos( positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 17; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, kBodyKWlist[i], numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

static PyObject * PyModule_PepperSetAutonomousAbility( PyObject * self, PyObject * args )
{
  char * ability = NULL;
  PyObject * boolObj = NULL;


  if (!PyArg_ParseTuple( args, "sO", &ability, &boolObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( boolObj )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setAutonomousAbility: second parameter must be a boolean!" );
    return NULL;
  }

  PepperProxyManager::instance()->setAutonomousAbility( ability, PyObject_IsTrue( boolObj ) );
  Py_RETURN_NONE;
}

/** @name Audio Management Functions
 *
 */
/**@{*/
/*! \fn loadAudioFile(file_name)
 *  \memberof PyPepper
 *  \brief Load an audio file into the robot system.
 *  \param str file_name. Audio file path. Must be a full path.
 *  \return int audio ID.
 */
static PyObject * PyModule_PepperLoadAudioFile( PyObject * self, PyObject * args )
{
  char * text = NULL;
  int audioID = -1;

  if (!PyArg_ParseTuple( args, "s", &text )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (text) {
    audioID = PepperProxyManager::instance()->loadAudioFile( text );
  }
  if (audioID == -1) {
    PyErr_Format( PyExc_ValueError, "PyPepper.loadAudioFile: unable to load audio file %s!", text );
    return NULL;
  }
  return Py_BuildValue( "i", audioID );
}

/*! \fn unloadAudioFile(audio_id)
 *  \memberof PyPepper
 *  \brief Unload an audio from the robot system (free memory).
 *  \param int audio_id. Loaded audio file ID.
 *  \return None.
 */
static PyObject * PyModule_PepperUnloadAudioFile( PyObject * self, PyObject * args )
{
  int audioID = 0;

  if (!PyArg_ParseTuple( args, "i", &audioID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (audioID > 0) {
    PepperProxyManager::instance()->unloadAudioFile( audioID );
  }
  Py_RETURN_NONE;
}

/*! \fn unloadAllAudioFile()
 *  \memberof PyPepper
 *  \brief Unload all audio from the robot system.
 *  \return None.
 */
static PyObject * PyModule_PepperUnloadAllAudioFiles( PyObject * self )
{
  PepperProxyManager::instance()->unloadAllAudioFiles();
  Py_RETURN_NONE;
}

static PyObject * PyModule_PepperPlayWebAudio( PyObject * self, PyObject * args )
{
  char * text = NULL;

  if (!PyArg_ParseTuple( args, "s", &text )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (text) {
    PepperProxyManager::instance()->playWebAudio( text );
  }
  Py_RETURN_NONE;
}

/*! \fn playAudioID(audio_id)
 *  \memberof PyPepper
 *  \brief Play a loaded audio file.
 *  \param int audio_id. Loaded audio file ID.
 *  \return None.
 */
static PyObject * PyModule_PepperPlayAudioID( PyObject * self, PyObject * args )
{
  int audioID = 0;
  PyObject * toBlockObj = NULL;

  if (!PyArg_ParseTuple( args, "i|O", &audioID, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (toBlockObj && !PyBool_Check( toBlockObj )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.playAudioID: second parameter should be a boolean!" );
    return NULL;
  }
  if (audioID > 0) {
    PepperProxyManager::instance()->playAudioID( audioID, (toBlockObj && PyObject_IsTrue( toBlockObj )) );
  }
  Py_RETURN_NONE;
}

/*! \fn getAudioVolume()
 *  \memberof PyPepper
 *  \brief Get the current master audio volume.
 *  \return int audio volume between [0,100].
 */
static PyObject * PyModule_PepperGetAudioVolume( PyObject * self )
{
  int volume = PepperProxyManager::instance()->getAudioVolume();
  return Py_BuildValue( "i", volume );
}

/*! \fn setAudioVolume(volume)
 *  \memberof PyPepper
 *  \brief Set the master audio volume.
 *  \param int volume. Must be between [0,100]
 *  \return None.
 */
static PyObject * PyModule_PepperSetAudioVolume( PyObject * self, PyObject * args )
{
  int volume = 50;

  if (!PyArg_ParseTuple( args, "i", &volume )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (volume < 0 || volume > 100) {
    PyErr_Format( PyExc_ValueError, "PyPepper.setAudioVolume: invalid audio volume, must be within [0..100]!" );
    return NULL;
  }

  PepperProxyManager::instance()->setAudioVolume( volume );
  Py_RETURN_NONE;
}

/*! \fn pauseAudioID(audio_id)
 *  \memberof PyPepper
 *  \brief Pause a playing audio.
 *  \param int audio_id. Audo file ID.
 *  \return None.
 */
static PyObject * PyModule_PepperPauseAudioID( PyObject * self, PyObject * args )
{
  int audioID = 0;

  if (!PyArg_ParseTuple( args, "i", &audioID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (audioID > 0) {
    PepperProxyManager::instance()->pauseAudioID( audioID );
  }
  Py_RETURN_NONE;
}

/*! \fn stopAllAudio()
 *  \memberof PyPepper
 *  \brief Stop playing all audio files.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_PepperStopAllAudio( PyObject * self )
{
  PepperProxyManager::instance()->stopAllAudio();
  Py_RETURN_NONE;
}

/** @name Choregraphe Behaviour Management Functions
 *
 */
/**@{*/
/*! \fn startBehaviour(name)
 *  \memberof PyPepper
 *  \brief Start playing a behaviour.
 *  \param str name. The name of the behaviour.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperStartBehaviour( PyObject * self, PyObject * args )
{
  char * name = NULL;

  if (!PyArg_ParseTuple( args, "s", &name )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (PepperProxyManager::instance()->startBehaviour( name )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn runBehaviour(name, is_blocking)
 *  \memberof PyPepper
 *  \brief Run a behaviour.
 *  \param str name. The name of the behaviour.
 *  \param bool is_blocking. Optional, True = blocking call, False = non blocking. Default: False.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperRunBehaviour( PyObject * self, PyObject * args )
{
  char * name = NULL;
  bool inpost = true;
  PyObject * boolObj = NULL;

  if (!PyArg_ParseTuple( args, "s|O", &name, boolObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (!PyBool_Check( boolObj )) {
      PyErr_Format( PyExc_ValueError, "PyPepper.runBehaviour: last optional input parameter must be a boolean!" );
      return NULL;
    }
    inpost = !PyObject_IsTrue( boolObj );
  }

  if (PepperProxyManager::instance()->runBehaviour( name, inpost )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn stopBehaviour(name)
 *  \memberof PyPepper
 *  \brief Stop a currently playing behaviours.
 *  \param str name. The name of the behaviour.
 *  \return None.
 */
static PyObject * PyModule_PepperStopBehaviour( PyObject * self, PyObject * args )
{
  char * name = NULL;

  if (!PyArg_ParseTuple( args, "s", &name )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  PepperProxyManager::instance()->stopBehaviour( name );
  Py_RETURN_NONE;
}

/*! \fn stopAllBehaviours()
 *  \memberof PyPepper
 *  \brief Stop all playing behaviours.
 *  \return None.
 */
static PyObject * PyModule_PepperStopAllBehaviours( PyObject * self )
{
  PepperProxyManager::instance()->stopAllBehaviours();
  Py_RETURN_NONE;
}

/*! \fn getBehaviourList(installed)
 *  \memberof PyPepper
 *  \brief Return a list of loaded (or installed) default behaviours on Pepper.
 *  \param bool installed. Optional. True = Installed behaviours; False = Loaded behaviours. Default: False
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_PepperGetBehaviourList( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.getBehavourList: the parameter must be a boolean!" );
      return NULL;
    }
  }
  std::vector<std::string> list = PepperProxyManager::instance()->getBehaviourList( isYes );
  size_t list_size = list.size();
  PyObject * retObj = PyList_New( list_size );
  for (size_t i = 0; i < list_size; i++) {
    PyObject * strObj = PyString_FromString( list.at( i ).c_str() );
    PyList_SetItem( retObj, i, strObj );
  }
  return retObj;
}

/** @name Tablet Management Functions
 *
 */
/**@{*/
/*! \fn directToWeb(uri)
 *  \memberof PyPepper
 *  \brief direct Pepper tablet to a specified web site.
 *  \param str uri. URI for the web site.
 *  \return bool. True == successfully load the url; False == url invalid or unreachable.
 */
static PyObject * PyModule_PepperDirectToWeb( PyObject * self, PyObject * args )
{
  char * uriStr = NULL;

  if (!PyArg_ParseTuple( args, "s", &uriStr ) || strlen(uriStr) == 0) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (PepperProxyManager::instance()->directToWeb( uriStr ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn reloadWebpage()
 *  \memberof PyPepper
 *  \brief Reload the current webpage on Pepper's tablet.
 *  \return None.
 */
static PyObject * PyModule_PepperReloadWebpage( PyObject * self )
{
  PepperProxyManager::instance()->reloadWebpage();
  Py_RETURN_NONE;
}

/*! \fn clearWebpage(hide)
 *  \memberof PyPepper
 *  \brief Clear the webpage displayed on the tablet's web browser
 *  \param bool hide. True == Hide the browser; False == otherwise (default False)
 *  \return None
 */
static PyObject * PyModule_PepperClearWebpage( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.clearWebpage: the input parameter must be a boolean!" );
      return NULL;
    }
  }
  PepperProxyManager::instance()->clearWebpage( isYes );
  Py_RETURN_NONE;
}

/*! \fn turnTabletOn(on)
 *  \memberof PyPepper
 *  \brief Turn Pepper tablet on or off
 *  \param bool on. True == turn on the tablet; False == turn off the tablet.
 *  \return None
 */
static PyObject * PyModule_PepperTurnTabletOn( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (PyBool_Check( isYesObj )) {
    isYes = PyObject_IsTrue( isYesObj );
  }
  else {
    PyErr_Format( PyExc_ValueError, "PyPepper.turnTabletOn: the input parameter must be a boolean!" );
    return NULL;
  }
  PepperProxyManager::instance()->turnTabletOn( isYes );
  Py_RETURN_NONE;
}

/*! \fn resetTablet()
 *  \memberof PyPepper
 *  \brief Reset Pepper's tablet.
 *  \return None.
 */
static PyObject * PyModule_PepperResetTablet( PyObject * self )
{
  PepperProxyManager::instance()->resetTablet();
  Py_RETURN_NONE;
}

/*! \fn getTabletWiFiInfo()
 *  \memberof PyPepper
 *  \brief Retrieve Pepper's tablet status information and its MAC address.
 *  \return tuple. (status, mac address)
 */
static PyObject * PyModule_PepperGetTabletWiFiInfo( PyObject * self )
{
  std::string status = PepperProxyManager::instance()->getTabletWiFiStatus();
  std::string macaddr = PepperProxyManager::instance()->getTabletWiFiMacAddress();

  return Py_BuildValue( "(ss)", status.c_str(), macaddr.c_str() );
}

/*! \fn connectTabletWiFiTo(ssid, key)
 *  \memberof PyPepper
 *  \brief Connect Pepper's tablet to a specific WPA WiFi hotspot.
 *  \param str ssid. SSID of the WiFi hotspot.
 *  \param str key. WAP security key of the hotspot.
 *  \return bool. True == successfully connected; False == unable to connect.
 *  \note Only WPA secure protocol is supported.
 */
static PyObject * PyModule_PepperConnectTabletWiFiTo( PyObject * self, PyObject * args )
{
  char * ssid = NULL;
  char * wpakey = NULL;

  if (!PyArg_ParseTuple( args, "ss", &ssid, &wpakey ) || strlen(ssid) == 0 || strlen(wpakey) == 0) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (PepperProxyManager::instance()->connectTabletWiFiTo( ssid, wpakey ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn disconnectTabletWiFi()
 *  \memberof PyPepper
 *  \brief Disconnect Pepper's tablet WiFi connection with a hotspot
 *  \return bool. True == successful disconnection; False = failed.
 */
static PyObject * PyModule_PepperDisconnectTabletWiFi( PyObject * self )
{
  if (PepperProxyManager::instance()->disconnectTabletWiFi())
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn turnTabletWiFiOn(on)
 *  \memberof PyPepper
 *  \brief Turn Pepper tablet WiFi on or off
 *  \param bool on. True == turn on the WiFi; False == turn off the WiFi.
 *  \return None
 */
/**@}*/
static PyObject * PyModule_PepperTurnTabletWiFiOn( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (PyBool_Check( isYesObj )) {
    isYes = PyObject_IsTrue( isYesObj );
  }
  else {
    PyErr_Format( PyExc_ValueError, "PyPepper.turnTabletWiFiOn: the input parameter must be a boolean!" );
    return NULL;
  }
  PepperProxyManager::instance()->turnTabletWiFiOn( isYes );
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn setChestLED(colour)
 *  \memberof PyPepper
 *  \brief Set Pepper's chest LED to a colour.
 *  \param str colour. Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 *  \return None.
 */
static PyObject * PyModule_PepperSetChestLED( PyObject * self, PyObject * args )
{
  char * colourStr = NULL;
  NAOLedColour colourID;

  if (!PyArg_ParseTuple( args, "s", &colourStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (colourStr2ID( colourStr, colourID )) {
    PepperProxyManager::instance()->setChestLED( colourID );
  }
  else {
    PyErr_Format( PyExc_ValueError, "PyPepper.setChestLED: invalid input colour."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }
  Py_RETURN_NONE;
}

/*! \fn setFaceLED(colour)
 *  \memberof PyPepper
 *  \brief Set Pepper's face/eye LED to a colour.
 *  \param str colour. Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 *  \return None.
 */
static PyObject * PyModule_PepperSetFaceLED( PyObject * self, PyObject * args )
{
  char * colourStr = NULL;
  NAOLedColour colourID;

  if (!PyArg_ParseTuple( args, "s", &colourStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (colourStr2ID( colourStr, colourID )) {
    PepperProxyManager::instance()->setFaceLED( colourID );
  }
  else {
    PyErr_Format( PyExc_ValueError, "PyPepper.setFaceLED: invalid input colour."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }
  Py_RETURN_NONE;
}

/*! \fn pulseChestLED(colour_one, colour_two, period)
 *  \memberof PyPepper
 *  \brief Periodically switch Pepper's chest LED between the two input colours.
 *  \param str colour_one. Colour label one.
 *  \param str colour_one. Colour label two.
 *  \param int period. Time (in seconds) before switching LED colour.
 *  \return None.
 *  \note Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 */
static PyObject * PyModule_PepperPulseChestLED( PyObject * self, PyObject * args )
{
  char * colourStr1 = NULL;
  char * colourStr2 = NULL;

  float period = 0.5;

  NAOLedColour colourID1, colourID2;

  if (!PyArg_ParseTuple( args, "ss|f", &colourStr1, &colourStr2, &period )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!colourStr2ID( colourStr1, colourID1 ) || !colourStr2ID( colourStr2, colourID2 )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.pulseChestLED: invalid input colour(s)."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }

  if (period <= 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.pulseChestLED: invalid pulse period." );
    return NULL;
  }

  PepperProxyManager::instance()->pulsatingChestLED( colourID1, colourID2, period );
  Py_RETURN_NONE;
}

/*! \fn setCameraParameter(parameter_id, value)
 *  \memberof PyPepper
 *  \brief Set a camera parameter value.
 *  \param int parameter_id. The integer ID of the parameter. If parameter_id is not set, this method will set all camera parameters to their default values.
 *  \param int value. The value for the parameter. If value is not set, this method will set the specified camera parameter to its default value.
 *  \return boolean. True == successful; otherwise failed.
 */
static PyObject * PyModule_PepperSetCameraParameter( PyObject * self, PyObject * args )
{
  int pid = -1;
  int value = -1;

  if (!PyArg_ParseTuple( args, "|ii", &pid, &value )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (ServerDataProcessor::instance()->setCameraParameter( 0, pid, value )) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

/*! \fn getBatteryStatus()
 *  \memberof PyPepper
 *  \brief Return the current robot battery status.
 *  \return tuple(battery percentage, is_plugged_in, is_(dis)charging).
 */
static PyObject * PyModule_PepperGetBatteryStatus( PyObject * self )
{
  int batpercent = 0;
  bool isplugged = false;
  bool ischarging = false;
  bool isdischarging = false;

  PepperProxyManager::instance()->getBatteryStatus( batpercent, isplugged, ischarging, isdischarging );

  if (ischarging) {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "charging" );
  }
  else if (isdischarging) {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "discharging" );

  }
  else {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "not charging" );
  }
}

/*! \fn shutdownRobot(reboot)
 *  \memberof PyPepper
 *  \brief Shutting down the robot.
 *  \param bool reboot. True == Reboot the robot; False == otherwise (default False)
 *  \return None
 */
/**@}*/
static PyObject * PyModule_PepperShutdownRobot( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.shutdownRobot: the input parameter must be a boolean!" );
      return NULL;
    }
  }
  PepperProxyManager::instance()->shutdownRobot( isYes );
  Py_RETURN_NONE;
}

#define INCLUDE_COMMON_PYMODULE_MEHTODS
#include "../libsrc/pyridecore/PyModulePyCommon.cpp"

static PyMethodDef PyModule_methods[] = {
  { "write", (PyCFunction)PyModule_write, METH_VARARGS,
    "standard output for UTS Pepper Python console." },
  { "setTeamMemberID", (PyCFunction)PyModule_SetTeamMemberID, METH_VARARGS,
    "Set Pepper team member ID and team colour." },
  { "sendTeamMessage", (PyCFunction)PyModule_sendTeamMessage, METH_VARARGS,
    "Send a message to the rest team members." },
  { "say", (PyCFunction)PyModule_PepperSayWithVolume, METH_VARARGS,
    "Let Pepper speak with an optional volume and animation." },
  { "moveHeadTo", (PyCFunction)PyModule_PepperMoveHeadTo, METH_VARARGS,
    "Move Pepper head to a new position." },
  { "updateHeadPos", (PyCFunction)PyModule_PepperUpdateHeadPos, METH_VARARGS,
    "Change Pepper head position with a specific angle in radian." },
  { "getHeadPos", (PyCFunction)PyModule_PepperGetHeadPos, METH_NOARGS,
    "Get Pepper's head position." },
  { "setHeadStiffness", (PyCFunction)PyModule_PepperSetHeadStiffness, METH_VARARGS,
    "Set the stiffness of the Pepper's head. " },
  { "setBodyStiffness", (PyCFunction)PyModule_PepperSetBodyStiffness, METH_VARARGS,
    "Set the stiffness of the Pepper's body. " },
  { "gotoStation", (PyCFunction)PyModule_PepperGotoStation, METH_NOARGS,
    "Get Pepper to its charging station. " },
  { "leaveStation", (PyCFunction)PyModule_PepperGotoStation, METH_NOARGS,
    "Move Pepper off its charging station. " },
  { "stand", (PyCFunction)PyModule_PepperStand, METH_VARARGS,
    "Get Pepper to stand up in standard or ready to walk mode (set optional input to True). " },
  { "crouch", (PyCFunction)PyModule_PepperCrouch, METH_NOARGS,
    "Get Pepper to crouch. " },
  { "navigateBodyTo", (PyCFunction)PyModule_PepperNavigateBodyTo, METH_VARARGS,
    "Navigate Pepper to a specific position w.r.t the current robot pose. " },
  { "moveBodyTo", (PyCFunction)PyModule_PepperMoveBodyTo, METH_VARARGS,
    "Move Pepper to a specific position w.r.t the current robot pose. " },
  { "setArmStiffness", (PyCFunction)PyModule_PepperSetArmStiffness, METH_VARARGS,
    "Set the stiffness of the one of Pepper's arms. " },
  { "setLegStiffness", (PyCFunction)PyModule_PepperSetLegStiffness, METH_VARARGS,
     "Set the stiffness of the one of Pepper's legs. " },
  { "moveArmWithJointPos", (PyCFunction)PyModule_PepperMoveArmWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of Pepper arms with specific joint positions." },
  { "moveArmWithJointTrajectory", (PyCFunction)PyModule_PepperMoveArmWithJointTraj, METH_VARARGS,
    "Move one of Pepper arms with specific joint trajectory (a list of joint positions)." },
  { "moveLegWithJointPos", (PyCFunction)PyModule_PepperMoveLegWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of Pepper legs with specific joint positions." },
  { "moveBodyWithJointPos", (PyCFunction)PyModule_PepperMoveBodyWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move Pepper all body joint positions." },
  { "moveBodyWithRawTrajectoryData", (PyCFunction)PyModule_PepperMoveBodyWithRawTrajectoryData, METH_VARARGS|METH_KEYWORDS,
    "Move Pepper all body joints in fully specified trajectories." },
  { "getArmJointPositions", (PyCFunction)PyModule_PepperGetArmJointPositions, METH_VARARGS,
    "Get joint positions of Pepper's arms." },
  { "getLegJointPositions", (PyCFunction)PyModule_PepperGetLegJointPositions, METH_VARARGS,
    "Get joint positions of Pepper's torso." },
  { "getBodyJointPositions", (PyCFunction)PyModule_PepperGetBodyJointPositions, METH_VARARGS,
    "Get full joint positions of Pepper." },
  { "setAutonomousAbility", (PyCFunction)PyModule_PepperSetAutonomousAbility, METH_VARARGS,
     "Set the autonomous ability of the Pepper. " },
  { "openHand", (PyCFunction)PyModule_PepperOpenHand, METH_VARARGS,
    "Open one or both Pepper hands. Parameters: which_hand. 1 = left hand, 2 = right hand and 3 = both hand, (optional) keep stiffness on after closing the hand (default False)" },
  { "closeHand", (PyCFunction)PyModule_PepperCloseHand, METH_VARARGS,
    "Close one or both Pepper hand. Parameter: which_hand. 1 = left hand, 2 = right hand and 3 = both hand, (optional) keep stiffness on after closing the hand (default False)" },
  { "setHandPosition", (PyCFunction)PyModule_PepperSetHandPosition, METH_VARARGS,
    "Set specific opening ratio on one or both Pepper hands. Parameters: which_hand. 1 = left hand, 2 = right hand and 3 = both hands. Float [0.0, 1.0] amount of closing, (optional) keep stiffness on after closing the hand (default False)." },
  { "loadAudioFile", (PyCFunction)PyModule_PepperLoadAudioFile, METH_VARARGS,
    "Load an audio file on Pepper." },
  { "unloadAudioFile", (PyCFunction)PyModule_PepperUnloadAudioFile, METH_VARARGS,
    "Unload an audio file from Pepper." },
  { "unloadAllAudioFiles", (PyCFunction)PyModule_PepperUnloadAllAudioFiles, METH_NOARGS,
    "Unload all audio files from Pepper." },
  { "playWebAudio", (PyCFunction)PyModule_PepperPlayWebAudio, METH_VARARGS,
    "Play a web audio stream on Pepper." },
  { "playAudioID", (PyCFunction)PyModule_PepperPlayAudioID, METH_VARARGS,
    "Play an audio file on Pepper." },
  { "getAudioVolume", (PyCFunction)PyModule_PepperGetAudioVolume, METH_NOARGS,
    "Get master audio volume on Pepper." },
  { "setAudioVolume", (PyCFunction)PyModule_PepperSetAudioVolume, METH_VARARGS,
    "Set master audio volume on Pepper." },
  { "pauseAudioID", (PyCFunction)PyModule_PepperPauseAudioID, METH_VARARGS,
    "Pause a playing audio file." },
  { "stopAllAudio", (PyCFunction)PyModule_PepperStopAllAudio, METH_NOARGS,
    "Stop playing all audio on Pepper." },
  { "startBehaviour", (PyCFunction)PyModule_PepperStartBehaviour, METH_VARARGS,
    "Start playing a behaviour on Pepper. Parameter: string name of the behaviour." },
  { "runBehaviour", (PyCFunction)PyModule_PepperRunBehaviour, METH_VARARGS,
    "Run a behaviour on Pepper. Parameter: string name of the behaviour, optional boolean. True = blocking call, False = blocking. Default: False." },
  { "stopBehaviour", (PyCFunction)PyModule_PepperStopBehaviour, METH_VARARGS,
    "Stop a current playing behaviour on Pepper. Parameter: string name of the behaviour." },
  { "stopAllBehaviours", (PyCFunction)PyModule_PepperStopAllBehaviours, METH_NOARGS,
    "Stop all playing behaviours on Pepper." },
  { "getBehaviourList", (PyCFunction)PyModule_PepperGetBehaviourList, METH_VARARGS,
    "Get a list of loaded (or installed) behaviours on Pepper." },
  { "directToWeb", (PyCFunction)PyModule_PepperDirectToWeb, METH_VARARGS,
    "Direct Pepper tablet browser to a URI." },
  { "reloadWebpage", (PyCFunction)PyModule_PepperReloadWebpage, METH_NOARGS,
    "Reload the current webpage on the Pepper tablet." },
  { "clearWebpage", (PyCFunction)PyModule_PepperClearWebpage, METH_VARARGS,
    "Clear the current webpage showing on the Pepper tablet." },
  { "turnTabletOn", (PyCFunction)PyModule_PepperTurnTabletOn, METH_VARARGS,
    "Turn Pepper tablet on or off." },
  { "resetTablet", (PyCFunction)PyModule_PepperResetTablet, METH_NOARGS,
    "Reset Pepper tablet." },
  { "getTabletWiFiInfo", (PyCFunction)PyModule_PepperGetTabletWiFiInfo, METH_NOARGS,
    "Retrieve Pepper tablet's wifi status and MAC address." },
  { "connectTabletWiFiTo", (PyCFunction)PyModule_PepperConnectTabletWiFiTo, METH_VARARGS,
    "Connect Pepper tablet WiFi to a specific WPA WiFi hotspot." },
  { "disconnectTabletWiFi", (PyCFunction)PyModule_PepperDisconnectTabletWiFi, METH_NOARGS,
    "Disconnect Pepper tablet WiFi from the current hotspot." },
  { "turnTabletWiFiOn", (PyCFunction)PyModule_PepperTurnTabletWiFiOn, METH_VARARGS,
    "Turn Pepper tablet WiFi on or off." },
  { "setChestLED", (PyCFunction)PyModule_PepperSetChestLED, METH_VARARGS,
    "Set the colour of the chest LEDs on Pepper." },
  { "setFaceLED", (PyCFunction)PyModule_PepperSetFaceLED, METH_VARARGS,
    "Set the colour of the face/eye LEDs on Pepper." },
  { "pulseChestLED", (PyCFunction)PyModule_PepperPulseChestLED, METH_VARARGS,
    "Pulse the chest LED of Pepper between two colours." },
  { "getBatteryStatus", (PyCFunction)PyModule_PepperGetBatteryStatus, METH_NOARGS,
    "Get the current battery status." },
  { "shutdownRobot", (PyCFunction)PyModule_PepperShutdownRobot, METH_VARARGS,
    "Shutdown the Pepper robot." },
  { "setCameraParameter", (PyCFunction)PyModule_PepperSetCameraParameter, METH_VARARGS,
    "Set Pepper camera parameter. Input: int parameter id, int value. Check Pepper document for details. Default with no inputs sets parameter(s) to default values" },
#define DEFINE_COMMON_PYMODULE_METHODS
#include "../libsrc/pyridecore/PyModulePyCommon.cpp"
  { NULL, NULL, 0, NULL }           /* sentinel */
};

//#pragma mark PyPepperModule implmentation
PyPepperModule::PyPepperModule() : PyModuleExtension( "PyPepper" )
{

}

PyObject * PyPepperModule::createPyModule()
{
  return Py_InitModule3( "PyPepper", PyModule_methods, PyPepper_doc );
}

PyPepperModule * PyPepperModule::instance()
{
  if (!s_pyPepperModule)
    s_pyPepperModule = new PyPepperModule();

  return s_pyPepperModule;
}
} // namespace pyride
