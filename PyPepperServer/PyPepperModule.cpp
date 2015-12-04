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

static const char *kLeftArmKWlist[] = { "l_shoulder_pitch_joint", "l_shoulder_roll_joint",
		"l_elbow_yaw_joint", "l_elbow_roll_joint", "l_wrist_yaw_joint", "frac_max_speed", NULL };
static const char *kRightArmKWlist[] = { "r_shoulder_pitch_joint", "r_shoulder_roll_joint",
		"r_elbow_yaw_joint", "r_elbow_roll_joint", "r_wrist_yaw_joint", "frac_max_speed", NULL };
static const char *kLegKWlist[] = { "hip_roll_joint", "hip_pitch_joint", "knee_pitch_joint",
    "frac_max_speed", NULL };

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

static PyObject * PyModule_PepperSayWithVolume( PyObject * self, PyObject * args )
{
  float volume = 0.0;
  char * dataStr = NULL;
  bool toBlock = false;
  bool toAnimate = true;

  PyObject * toAnimateObj = NULL;
  PyObject * toBlockObj = NULL;

  if (!PyArg_ParseTuple( args, "s|fOO", &dataStr, &volume, &toAnimateObj, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (toAnimateObj) {
    if (PyBool_Check( toAnimateObj )) {
      toAnimate = PyObject_IsTrue( toAnimateObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.say: third parameter must be a boolean!" );
      return NULL;
    }
  }
  if (toBlockObj) {
    if (PyBool_Check( toBlockObj )) {
      toBlock = PyObject_IsTrue( toBlockObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.say: fouth parameter must be a boolean!" );
      return NULL;
    }
  }

  if (volume < 0.0 || volume > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.say: invalid voice volume!" );
    return NULL;
  }
  if (dataStr) {
    PepperProxyManager::instance()->sayWithVolume( string( dataStr ), volume,
                                               toAnimate, toBlock );
  }
  Py_RETURN_NONE;
}

/*! \fn moveHeadTo(head_yaw, head_pitch,is_absolute)
 *  \memberof PyPepper
 *  \brief Move the Pepper head to a specific yaw and pitch position. This new position could be either absolute head position or a relative position w.r.t the current head position.
 *  \param float head_yaw. Must be in radian.
 *  \param float head_pitch. Must be in radian.
 *  \param bool is_absolute. Optional. True = inputs represent absolute value for the new head position; False = inputs represent relative head position w.r.t. to the current head position.
 *  \return None.
 */
static PyObject * PyModule_PepperMoveHeadTo( PyObject * self, PyObject * args )
{
  float yaw = 0.0;
  float pitch = 0.0;
  bool absolute = false;
  PyObject * isYesObj = NULL;

  if (!PyArg_ParseTuple( args, "ff|O", &yaw, &pitch, &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      absolute = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.moveHeadTo: the third optional parameter must be a boolean!" );
      return NULL;
    }
  }

  PepperProxyManager::instance()->moveHeadTo( yaw, pitch, absolute );
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

/*! \fn moveBodyTo(x,y,theta,best_time)
 *  \memberof PyPepper
 *  \brief Move the PR2 body to a pose at (x,y,theta) w.r.t the current pose.
 *  \param float x. X coordinate w.r.t. the current pose.
 *  \param float y. Y coordinate w.r.t. the current pose.
 *  \param float theta. Angular position w.r.t. the current pose.
 *  \param float best_time. Optional, ask the robot try its best to reach the input pose in this time frame.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PepperMoveBodyTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  float bestTime = 5.0; //seconds

  if (!PyArg_ParseTuple( args, "fff|f", &xcoord, &ycoord, &theta, &bestTime )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;

  if (PepperProxyManager::instance()->moveBodyTo( pose, bestTime ))
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
 *  \param bool is_blocking. Optional. True = blocking call; False = unblocking call.
 *  \return None.
 */
static PyObject * PyModule_PepperMoveArmWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  PyObject * isYesObj = NULL;
  bool inpost = false;

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
      inpost = PyObject_IsTrue( isYesObj );
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

    std::vector<float> arm_joint_pos( 4, 0.0 );

    for (int k = 0; k < 4; k++) {
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

  PepperProxyManager::instance()->moveArmWithJointTrajectory( (armsel == 1), trajectory,
                                                          times_to_reach, inpost );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyPepper
 *  \brief Move a Pepper's arm to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of arm joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getArmJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
 */
static PyObject * PyModule_PepperMoveArmWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float s_p_j, s_r_j, e_y_j, e_r_j, w_y_j;
  float frac_max_speed = 0.5;

  bool isLeftArm = false;

  if (PyArg_ParseTupleAndKeywords( args, keywds, "fffff|f", (char**)kLeftArmKWlist,
                                  &s_p_j, &s_r_j, &e_y_j, &e_r_j, &w_y_j, &frac_max_speed ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "fffff|f", (char**)kRightArmKWlist,
                                     &s_p_j, &s_r_j, &e_y_j, &e_r_j, &w_y_j, &frac_max_speed ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }

  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.moveArmWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }

  std::vector<float> positions( 5, 0.0 );
  positions[0] = s_p_j;
  positions[1] = s_r_j;
  positions[2] = e_y_j;
  positions[3] = e_r_j;
  positions[4] = w_y_j;

  PepperProxyManager::instance()->moveArmWithJointPos( isLeftArm, positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn moveLegWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyPepper
 *  \brief Move a Pepper's leg to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of leg joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getLegJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
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

  PepperProxyManager::instance()->moveLegWithJointPos( positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn moveBodyWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyPepper
 *  \brief Move the Pepper body joints to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of body joint positions in radian.
 *  The dictionary must the same structure as the return of PyPepper.getBodyJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
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

  PepperProxyManager::instance()->moveBodyWithJointPos( positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn openHand(left_hand, ratio, keep_stiffness)
 *  \memberof PyPepper
 *  \brief Open one of the Pepper hands to a ratio of allowed hand joint limit value.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \param float ratio. Optional, ratio between 0.0 and 1.0. 1.0 means the hand is fully open. Default is 1.0
 *  \param bool keep_stiffness. Optional, True for keep the stiffness on after the action; otherwise False.
 *  \return None.
 */
static PyObject * PyModule_PepperOpenHand( PyObject * self, PyObject * args )
{
  PyObject * armSelObj = NULL;
  PyObject * keepStiffObj = NULL;
  float oratio = 1.0;

  bool keep_stiff = false;

  if (!PyArg_ParseTuple( args, "O|fO", &armSelObj, &oratio, &keepStiffObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( armSelObj )) {
    PyErr_Format( PyExc_ValueError, "PyPepper.openHand: first input parameter must be a boolean!" );
    return NULL;
  }

  if (oratio > 1.0 || oratio < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.openHand: second input parameter must be a float with in [0..1.0]!" );
    return NULL;
  }

  bool isLeftArm = PyObject_IsTrue( armSelObj );

  if (keepStiffObj) {
    if (PyBool_Check( keepStiffObj )) {
      keep_stiff = PyObject_IsTrue( keepStiffObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPepper.openHand: third input parameter must be a boolean!" );
      return NULL;
    }
  }

  PepperProxyManager::instance()->openHand( isLeftArm, oratio, keep_stiff );
  Py_RETURN_NONE;
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

/*! \fn pluseChestLED(colour_one, colour_two, period)
 *  \memberof PyPepper
 *  \brief Periodically switch Pepper's chest LED between the two input colours.
 *  \param str colour_one. Colour label one.
 *  \param str colour_one. Colour label two.
 *  \param int period. Time (in seconds) before switching LED colour.
 *  \return None.
 *  \note Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 */
static PyObject * PyModule_PepperPluseChestLED( PyObject * self, PyObject * args )
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
    PyErr_Format( PyExc_ValueError, "PyPepper.pluseChestLED: invalid input colour(s)."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }

  if (period <= 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPepper.pluseChestLED: invalid pulse period." );
    return NULL;
  }

  PepperProxyManager::instance()->pulsatingChestLED( colourID1, colourID2, period );
  Py_RETURN_NONE;
}

/*! \fn getBatteryStatus()
 *  \memberof PyPepper
 *  \brief Return the current robot battery status.
 *  \return tuple(battery percentage, is_plugged_in, is_(dis)charging).
 */
/**@}*/
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
  { "moveBodyTo", (PyCFunction)PyModule_PepperNavigateBodyTo, METH_VARARGS,
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
  { "getArmJointPositions", (PyCFunction)PyModule_PepperGetArmJointPositions, METH_VARARGS,
    "Get joint positions of Pepper's arms." },
  { "getLegJointPositions", (PyCFunction)PyModule_PepperGetLegJointPositions, METH_VARARGS,
    "Get joint positions of Pepper's torso." },
  { "getBodyJointPositions", (PyCFunction)PyModule_PepperGetBodyJointPositions, METH_VARARGS,
    "Get full joint positions of Pepper." },
  { "openHand", (PyCFunction)PyModule_PepperOpenHand, METH_VARARGS,
     "Open one of the Pepper's hands to an allowed size. " },
  { "setAutonomousAbility", (PyCFunction)PyModule_PepperSetAutonomousAbility, METH_VARARGS,
     "Set the autonomous ability of the Pepper. " },
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
  { "setChestLED", (PyCFunction)PyModule_PepperSetChestLED, METH_VARARGS,
    "Set the colour of the chest LEDs on Pepper." },
  { "pluseChestLED", (PyCFunction)PyModule_PepperPluseChestLED, METH_VARARGS,
    "Pluse the chest LED of Pepper between two colours." },
  { "getBatteryStatus", (PyCFunction)PyModule_PepperGetBatteryStatus, METH_NOARGS,
    "Get the current battery status." },
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
