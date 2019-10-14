/*
 *  PepperProxyManager.cpp
 *  PyPepperServer
 *
 *  Created by Xun Wang on 18/11/15.
 *  Copyright 2012, 2015 Galaxy Network. All rights reserved.
 *
 */
#include <sys/time.h>
#include <algorithm>
#include <PyRideCommon.h>
#include "PepperProxyManager.h"
#include "PyPepperModule.h"

namespace pyride {

static const long kMotionCommandGapTolerance = 2 * 1000000 / kMotionCommandFreq;

static const char *kPepperBodyJoints[] = {
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHand",
    "HipRoll",
    "HipPitch",
    "KneePitch",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "RHand"
};

static const int kPepperJointNo = sizeof( kPepperBodyJoints ) / sizeof( kPepperBodyJoints[0] );

PepperProxyManager * PepperProxyManager::s_pPepperProxyManager = NULL;

void * pulse_thread( void * controller )
{
  ((PepperProxyManager *)controller)->continuePulseChestLED();
  return NULL;
}

void * timeout_thread( void * controller )
{
  ((PepperProxyManager *)controller)->timeoutCheck();
  return NULL;
}

PepperProxyManager::PepperProxyManager() :
  moveInitialised_( false ),
  isChestLEDPulsating_( false ),
  speechCtrl_( false ),
  headCtrl_( false ),
  lArmCtrl_( false ),
  rArmCtrl_( false ),
  lHandCtrl_( false ),
  rHandCtrl_( false ),
  bodyCtrl_( false ),
  behaviourCtrl_( false ),
  audioCtrl_( false ),
  runningThread_( (pthread_t)NULL ),
  timeoutThread_( (pthread_t)NULL ),
  speechThread_( NULL ),
  headmoveThread_( NULL ),
  larmmoveThread_( NULL ),
  rarmmoveThread_( NULL ),
  lhandmoveThread_( NULL ),
  rhandmoveThread_( NULL ),
  bodymoveThread_( NULL ),
  legmoveThread_( NULL ),
  behaviourThread_( NULL ),
  audioThread_( NULL )
{
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_, &t_mta );
}

PepperProxyManager::~PepperProxyManager()
{
  pthread_mutex_destroy( &t_mutex_ );
  pthread_mutexattr_destroy( &t_mta );
}

PepperProxyManager * PepperProxyManager::instance()
{
  if (!s_pPepperProxyManager) {
    s_pPepperProxyManager = new PepperProxyManager();
  }
  return s_pPepperProxyManager;
}

void PepperProxyManager::initWithBroker( boost::shared_ptr<ALBroker> broker, boost::shared_ptr<ALMemoryProxy> memoryProxy )
{
  memoryProxy_ = memoryProxy;

  try {
    speechProxy_ = boost::shared_ptr<ALTextToSpeechProxy>(new ALTextToSpeechProxy( broker ));
    speechProxy_->setLanguage( "English" );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALTextToSpeech.\n");
    speechProxy_.reset();
  }
  if (speechProxy_) {
    INFO_MSG( "Pepper text to speech is successfully initialised.\n" );
  }
  try {
    animateSpeechProxy_ = boost::shared_ptr<ALAnimatedSpeechProxy>(new ALAnimatedSpeechProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAnimatedSpeechProxy.\n");
    animateSpeechProxy_.reset();
  }
  if (animateSpeechProxy_) {
    INFO_MSG( "Pepper Animated speech is successfully initialised.\n" );
  }

  try {
    ledProxy_ = boost::shared_ptr<ALLedsProxy>(new ALLedsProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALLeds.\n");
    ledProxy_.reset();
  }
  if (ledProxy_) {
    INFO_MSG( "Pepper LED control is successfully initialised.\n" );
  }

  try {
    audioDeviceProxy_ = boost::shared_ptr<ALAudioDeviceProxy>(new ALAudioDeviceProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAudioDevice.\n");
    audioDeviceProxy_.reset();
  }
  if (audioDeviceProxy_) {
    INFO_MSG( "Pepper ALAudioDevice is successfully initialised.\n" );
  }

  try {
    audioPlayerProxy_ = boost::shared_ptr<ALAudioPlayerProxy>(new ALAudioPlayerProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAudioPlayer.\n");
    audioPlayerProxy_.reset();
  }
  if (audioPlayerProxy_) {
    INFO_MSG( "Pepper ALAudioPlayer is successfully initialised.\n" );
  }

  try {
    behaviourManagerProxy_ = boost::shared_ptr<ALBehaviorManagerProxy>(new ALBehaviorManagerProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALBehaviourManager.\n");
    behaviourManagerProxy_.reset();
  }
  if (behaviourManagerProxy_) {
    INFO_MSG( "Pepper ALBehaviourManager is successfully initialised.\n" );
  }

  try {
    motionProxy_ = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALMotion.\n");
    motionProxy_.reset();
  }
  if (motionProxy_) {
    moveInitialised_ = false;
    AL::ALValue joints; // make sure we get joint limits in correct order
    jointLimits_.arraySetSize( 17 );
    jointLimits_[0] = motionProxy_->getLimits( "HeadYaw" )[0];
    jointLimits_[1] = motionProxy_->getLimits( "HeadPitch" )[0];
    jointLimits_[2] = motionProxy_->getLimits( "LShoulderPitch" )[0];
    jointLimits_[3] = motionProxy_->getLimits( "LShoulderRoll" )[0];
    jointLimits_[4] = motionProxy_->getLimits( "LElbowYaw" )[0];
    jointLimits_[5] = motionProxy_->getLimits( "LElbowRoll" )[0];
    jointLimits_[6] = motionProxy_->getLimits( "LWristYaw" )[0];
    jointLimits_[7] = motionProxy_->getLimits( "LHand" )[0];
    jointLimits_[8] = motionProxy_->getLimits( "HipRoll" )[0];
    jointLimits_[9] = motionProxy_->getLimits( "HipPitch" )[0];
    jointLimits_[10] = motionProxy_->getLimits( "KneePitch" )[0];
    jointLimits_[11] = motionProxy_->getLimits( "RShoulderPitch" )[0];
    jointLimits_[12] = motionProxy_->getLimits( "RShoulderRoll" )[0];
    jointLimits_[13] = motionProxy_->getLimits( "RElbowYaw" )[0];
    jointLimits_[14] = motionProxy_->getLimits( "RElbowRoll" )[0];
    jointLimits_[15] = motionProxy_->getLimits( "RWristYaw" )[0];
    jointLimits_[16] = motionProxy_->getLimits( "RHand" )[0];
    //INFO_MSG( "hand limit (%f,%f)\n", (float)jointLimits_[16][0], (float)jointLimits_[16][1] );
    INFO_MSG( "Pepper Motion is successfully initialised.\n" );
  }

  try {
    postureProxy_ = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALRobotPosture.\n");
    postureProxy_.reset();
  }
  if (postureProxy_) {
    postureProxy_->goToPosture( "Stand", 0.6 );
    /*
    if (motionProxy_) {
      motionProxy_->rest();
    }
    */
    INFO_MSG( "Pepper Robot Posture is successfully initialised.\n" );
  }

  try {
    navigationProxy_ = boost::shared_ptr<ALNavigationProxy>(new ALNavigationProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALNavigationProxy.\n");
    navigationProxy_.reset();
  }
  if (navigationProxy_) {
    INFO_MSG( "Pepper navigation is successfully initialised.\n" );
  }

  try {
    autoLifeProxy_ = boost::shared_ptr<ALAutonomousLifeProxy>(new ALAutonomousLifeProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALAutonomousLifeProxy.\n");
    autoLifeProxy_.reset();
  }
  if (autoLifeProxy_) {
    INFO_MSG( "Pepper Autonomous life is successfully initialised.\n" );
  }

  try {
    basicAwarenessProxy_ = boost::shared_ptr<ALBasicAwarenessProxy>(new ALBasicAwarenessProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALBasicAwarenessProxy.\n");
    basicAwarenessProxy_.reset();
  }
  if (basicAwarenessProxy_) {
    INFO_MSG( "Pepper Basic Awareness is successfully initialised.\n" );
  }
  try {
    rechargeProxy_ = boost::shared_ptr<ALRechargeProxy>(new ALRechargeProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALRechargeProxy.\n");
    rechargeProxy_.reset();
  }
  if (rechargeProxy_) {
    INFO_MSG( "Pepper Auto Recharge is successfully initialised.\n" );
  }
  try {
    systemProxy_ = boost::shared_ptr<ALSystemProxy>(new ALSystemProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALSystemProxy.\n");
    systemProxy_.reset();
  }
  if (systemProxy_) {
    INFO_MSG( "Pepper system proxy is successfully initialised.\n" );
  }
  try {
    autoblinkingProxy_ = broker->getProxy( "ALAutonomousBlinking" );
    backgroundMovementProxy_ = broker->getProxy( "ALBackgroundMovement" );
    listenMovementProxy_ = broker->getProxy( "ALListeningMovement" );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Unable to create proxy to the standard automonous abilities.\n" );
    autoblinkingProxy_.reset();
    backgroundMovementProxy_.reset();
    listenMovementProxy_.reset();
  }
  if (autoblinkingProxy_) {
    INFO_MSG( "Pepper standard automonous abilities are successfully initialised.\n" );
  }
  try {
    tabletProxy_ = broker->getProxy( "ALTabletService" );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Unable to create proxy to tablet service.\n" );
  }
  if (tabletProxy_) {
    INFO_MSG( "Pepper tablet service is successfully initialised.\n" );
  }
}

void PepperProxyManager::setSpeechParameter( const std::string & param, float value )
{
  if (!speechProxy_) {
    return;
  }
  
  //if (param.compare("speech") == 0 && value == 0.0) {
    //speechProxy_->resetSpeed();
  //}
  //else {
    speechProxy_->setParameter( param, value );
  //}
}

bool PepperProxyManager::say( const std::string & text, bool toAnimate )
{
  if (speechCtrl_ || text.length() <= 0)
    return false;

  if (toAnimate) {
    if (!animateSpeechProxy_) {
      return false;
    }
  }
  else if (!speechProxy_) {
    return false;
  }

  if (speechThread_ && speechThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "speech is in progress.\n" );
    return false;
  }
  speechCtrl_ = true;

  if (speechThread_) { // we already in the thread
    blockedSpeech( text, toAnimate );
  }
  else {
    speechThread_ = new boost::thread( &PepperProxyManager::blockedSpeech, this, text, toAnimate );
  }

  return true;
}

void PepperProxyManager::blockedSpeech( const std::string & text, bool toAnimate )
{
  bool isSuccess = true;
  try {
    if (toAnimate) {
      animateSpeechProxy_->say( text );
    }
    else {
      speechProxy_->say( text );
    }
  }
  catch (...) {
    ERROR_MSG( "Unable to speak %s.\n", text.c_str() );
    isSuccess = false;
  }
  speechCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onSpeakSuccess" : "onSpeakFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done speech.\n" );

  delete speechThread_;
  speechThread_ = NULL;
}

bool PepperProxyManager::getHeadPos( float & yaw, float & pitch )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    std::vector<float> curHeadPos = motionProxy_->getAngles( names, true );
    yaw = curHeadPos.at( 0 );
    pitch = curHeadPos.at( 1 );
    return true;
  }
  return false;
}

bool PepperProxyManager::moveHeadTo( const float yaw, const float pitch, bool relative, float frac_speed )
{
  if (!motionProxy_ || headCtrl_)
    return false;

  if (headmoveThread_ && headmoveThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "head movement is in progress.\n" );
    return false;
  }

  headCtrl_ = true;
  if (headmoveThread_) { // we already in the thread
    blockedHeadMove( yaw, pitch, relative, frac_speed );
  }
  else {
    headmoveThread_ = new boost::thread( &PepperProxyManager::blockedHeadMove, this, yaw, pitch, relative, frac_speed );
  }
  return true;
}

void PepperProxyManager::blockedHeadMove( const float yaw, const float pitch, bool relative, float frac_speed )
{
  AL::ALValue names = "Head";
  AL::ALValue stiff = 1.0f;
  AL::ALValue newHeadPos;

  newHeadPos.arraySetSize( 2 );

  if (relative) {
    std::vector<float> curHeadPos;

    curHeadPos = motionProxy_->getAngles( names, true );

    newHeadPos[0] = clamp( yaw + curHeadPos.at( 0 ), HEAD_YAW );
    newHeadPos[1] = clamp( pitch + curHeadPos.at( 1 ), HEAD_PITCH );
  }
  else {
    newHeadPos[0] = clamp( yaw, HEAD_YAW );
    newHeadPos[1] = clamp( pitch, HEAD_PITCH );
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( names, stiff );

    motionProxy_->angleInterpolationWithSpeed( names, newHeadPos, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle interpolation to %s.\n", newHeadPos.toString().c_str() );
    isSuccess = false;
  }
  headCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onHeadActionSuccess" : "onHeadActionFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done head movement.\n" );

  delete headmoveThread_;
  headmoveThread_ = NULL;
}

void PepperProxyManager::updateHeadPos( const float yaw, const float pitch, const float speed )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    AL::ALValue newHeadPos;
    // AL::ALValue stiff = 1.0f;
    float yaw_value = 0.0;
    float pitch_value = 0.0;

    newHeadPos.arraySetSize( 2 );

    std::vector<float> curHeadPos;
    //motionProxy_->setStiffnesses( names, stiff );

    curHeadPos = motionProxy_->getAngles( names, true );

    yaw_value = clamp_change( yaw, curHeadPos.at( 0 ), HEAD_YAW );
    pitch_value = clamp_change( pitch, curHeadPos.at( 1 ), HEAD_PITCH );

    newHeadPos[0] = yaw_value / 2.0;
    newHeadPos[1] = pitch_value / 2.0;

    float myspeed = (speed > 1.0 || speed < 0.0 ) ? 0.1 : speed; // default to 0.1
    try {
      // HACK assuming input frequency is 10Hz and 3x commands
      motionProxy_->changeAngles( names, newHeadPos, myspeed );
      usleep( 33000 );
      motionProxy_->changeAngles( names, newHeadPos, myspeed );
      usleep( 33000 );
      motionProxy_->changeAngles( names, newHeadPos, myspeed );
    }
    catch (...) {
      ERROR_MSG( "Unable to change angles to %s", newHeadPos.toString().c_str() );
    }
  }
}

void PepperProxyManager::setHeadStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Head";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::setBodyStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Body";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::setJointStiffness( const int jointID, const float stiff )
{
  if (jointID < 0 || jointID > kPepperJointNo) {
    return;
  }
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = kPepperBodyJoints[jointID];
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::stand( bool init )
{
  if (postureProxy_) {
    postureProxy_->goToPosture( init ? "StandInit" : "Stand", 0.7 );
    moveInitialised_ = false;
  }
}

void PepperProxyManager::crouch()
{
  if (postureProxy_) {
    postureProxy_->goToPosture( "Crouch", 0.7 );
    motionProxy_->rest();
    moveInitialised_ = false;
  }
}

bool PepperProxyManager::moveBodyTo( const RobotPose & pose, float duration, bool cancelPreviousMove )
{
  if (!motionProxy_ || bodyCtrl_) {
    return false;
  }
  if (motionProxy_->moveIsActive()) {
    if (cancelPreviousMove) {
      motionProxy_->stopMove();
      if (bodymoveThread_) {
        bodymoveThread_->join();
        delete bodymoveThread_;
        bodymoveThread_ = NULL;
      }
    }
    else {
      ERROR_MSG( "Unable to issue new work command, robot is moving." );
      return false;
    }
  }
  bodyCtrl_ = true;

  if (bodymoveThread_) { // we already in the thread
    blockedBodyMoveTo( pose, duration );
  }
  else {
    bodymoveThread_ = new boost::thread( &PepperProxyManager::blockedBodyMoveTo, this, pose, duration );
  }

  return true;
}

void PepperProxyManager::blockedBodyMoveTo( const RobotPose & pose, const float duration )
{
  bool isSuccess = true;
  try {
    if (!moveInitialised_) {
      motionProxy_->wakeUp();
      motionProxy_->moveInit();
      moveInitialised_ = true;
    }

    motionProxy_->moveTo( pose.x, pose.y, pose.theta, duration );
  }
  catch (...) {
    ERROR_MSG( "Unable to move body position.\n" );
    isSuccess = false;
  }
  bodyCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onMoveBodySuccess" : "onMoveBodyFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done body movement.\n" );

  delete bodymoveThread_;
  bodymoveThread_ = NULL;
}

void PepperProxyManager::cancelBodyMovement()
{
  if (motionProxy_ && motionProxy_->moveIsActive()) {
    motionProxy_->stopMove();
    if (bodymoveThread_) {
      bodymoveThread_->join();
      delete bodymoveThread_;
      bodymoveThread_ = NULL;
    }
  }
}

bool PepperProxyManager::navigateBodyTo( const RobotPose & pose, bool cancelPreviousMove )
{
  if (!navigationProxy_) {
    ERROR_MSG( "Unable to initialise navigation." );
    return false;
  }
  if (motionProxy_->moveIsActive()) {
    if (cancelPreviousMove) {
      motionProxy_->stopMove();
    }
    else {
      ERROR_MSG( "Unable to issue new work command, robot is moving." );
      return false;
    }
  }
  return navigationProxy_->navigateTo( pose.x, pose.y );
}

void PepperProxyManager::updateBodyPose( const RobotPose & pose )
{
  if (!motionProxy_)
    return;

  gettimeofday( &cmdTimeStamp_, NULL );

  if (!moveInitialised_) {
    motionProxy_->wakeUp();
    motionProxy_->moveInit();
    moveInitialised_ = true;
  }
  motionProxy_->moveToward( pose.x, pose.y, pose.theta );
  if (!timeoutThread_) {
    if (pthread_create( &timeoutThread_, NULL, timeout_thread, this ) ) {
      ERROR_MSG( "Unable to create thread to check motion command expiry.\n" );
      timeoutThread_ = (pthread_t)NULL;
      return;
    }
  }
}

void PepperProxyManager::getBodyJointsPos( std::vector<float> & positions,
                      bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = "Body";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::getArmJointsPos( bool isLeft, std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::getLowerBodyJointsPos( std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = "Leg";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void PepperProxyManager::setArmStiffness( bool isLeft, const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void PepperProxyManager::setLegStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Leg";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

bool PepperProxyManager::moveArmWithJointPos( bool isLeftArm, const std::vector<float> & positions,
                                           float frac_speed )
{
  if (!motionProxy_ || bodyCtrl_)
    return false;

  if (isLeftArm) {
    if (lArmCtrl_)
      return false;

    if (larmmoveThread_ && larmmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "left arm movement is in progress.\n" );
      return false;
    }
  }
  else {
    if (rArmCtrl_)
      return false;

    if (rarmmoveThread_ && rarmmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "right arm movement is in progress.\n" );
      return false;
    }
  }

  if (positions.size() != 5)
    return false;

  if (isLeftArm) {
    lArmCtrl_ = true;

    if (larmmoveThread_) { // we already in the thread
      blockedArmMove( isLeftArm, positions, frac_speed );
    }
    else {
      larmmoveThread_ = new boost::thread( &PepperProxyManager::blockedArmMove, this, isLeftArm, positions, frac_speed );
    }
  }
  else {
    rArmCtrl_ = true;

    if (rarmmoveThread_) { // we already in the thread
      blockedArmMove( isLeftArm, positions, frac_speed );
    }
    else {
      rarmmoveThread_ = new boost::thread( &PepperProxyManager::blockedArmMove, this, isLeftArm, positions, frac_speed );
    }
  }

  return true;
}

void PepperProxyManager::blockedArmMove( bool isLeftArm, const std::vector<float> & positions,
                                      float frac_speed )
{
  int pos_size = positions.size();

  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll",
                                                      "LWristYaw") :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll",
                                                      "RWristYaw");

  AL::ALValue angles;

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], (isLeftArm ? L_SHOULDER_PITCH + i : R_SHOULDER_PITCH + i) );
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( names, 1.0 );

    motionProxy_->angleInterpolationWithSpeed( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s.\n", angles.toString().c_str() );
    isSuccess = false;
  }

  if (isLeftArm) {
    lArmCtrl_ = false;
  }
  else {
    rArmCtrl_ = false;
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", isLeftArm ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onMoveArmActionSuccess" : "onMoveArmActionFailed"), arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done arm movement.\n" );
  if (isLeftArm) {
    delete larmmoveThread_;
    larmmoveThread_ = NULL;
  }
  else {
    delete rarmmoveThread_;
    rarmmoveThread_ = NULL;
  }
}

bool PepperProxyManager::moveArmWithJointTrajectory( bool isLeftArm,
                                                     const std::vector< std::vector<float> > & trajectory,
                                                     const std::vector<float> & times_to_reach )
{
  if (!motionProxy_ || bodyCtrl_)
    return false;

  if (isLeftArm) {
    if (lArmCtrl_)
      return false;

    if (larmmoveThread_ && larmmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "left arm movement is in progress.\n" );
      return false;
    }
  }
  else {
    if (rArmCtrl_)
      return false;

    if (rarmmoveThread_ && rarmmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "right arm movement is in progress.\n" );
      return false;
    }
  }

  size_t traj_size = trajectory.size();

  if (traj_size <= 0 || traj_size != times_to_reach.size())
    return false;

  if (isLeftArm) {
    lArmCtrl_ = true;

    if (larmmoveThread_) { // we already in the thread
      blockedArmMoveTraj( isLeftArm, trajectory, times_to_reach );
    }
    else {
      larmmoveThread_ = new boost::thread( &PepperProxyManager::blockedArmMoveTraj, this, isLeftArm, trajectory, times_to_reach );
    }
  }
  else {
    rArmCtrl_ = true;

    if (rarmmoveThread_) { // we already in the thread
      blockedArmMoveTraj( isLeftArm, trajectory, times_to_reach );
    }
    else {
      rarmmoveThread_ = new boost::thread( &PepperProxyManager::blockedArmMoveTraj, this, isLeftArm, trajectory, times_to_reach );
    }
  }

  return true;
}

void PepperProxyManager::blockedArmMoveTraj( bool isLeftArm,
                                             const std::vector< std::vector<float> > & trajectory,
                                             const std::vector<float> & times_to_reach )
{
  size_t traj_size = trajectory.size();

  //AL::ALValue names = isLeftArm ? "LArm" : "RArm";
  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll",
                                                      "LWristYaw" ) :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll",
                                                      "RWristYaw" );
  AL::ALValue joints;
  AL::ALValue times;

  joints.arraySetSize( 5 );
  times.arraySetSize( 5 );

  for (size_t j = 0; j < 5; ++j) {
    AL::ALValue angles;
    AL::ALValue ttr;

    angles.arraySetSize( traj_size );
    ttr.arraySetSize( traj_size );

    joints[j] = angles;
    times[j] = ttr;

    float time_to_reach_for_pt = 0.0;
    for (size_t jp = 0; jp < traj_size; ++jp) {
      joints[j][jp] = clamp( trajectory[jp][j], (isLeftArm ? L_SHOULDER_PITCH + j : R_SHOULDER_PITCH + j) );
      time_to_reach_for_pt += times_to_reach[jp];
      times[j][jp] = time_to_reach_for_pt;
    }
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( names, 1.0 );

    motionProxy_->angleInterpolation( names, joints, times, true );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle interpolation to %s.\n", joints.toString().c_str() );
    isSuccess = false;
  }

  if (isLeftArm) {
    lArmCtrl_ = false;

  }
  else {
    rArmCtrl_ = false;
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", isLeftArm ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onMoveArmActionSuccess" : "onMoveArmActionFailed"), arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done arm movement.\n" );
  if (isLeftArm) {
    delete larmmoveThread_;
    larmmoveThread_ = NULL;
  }
  else {
    delete rarmmoveThread_;
    rarmmoveThread_ = NULL;
  }
}

bool PepperProxyManager::moveLowerBodyWithJointPos( const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_ || bodyCtrl_)
    return false;

  if (positions.size() != 3) {
    return false;
  }

  if (legmoveThread_ && legmoveThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "lower body movement is in progress.\n" );
    return false;
  }

  bodyCtrl_ = true;

  if (legmoveThread_) { // we already in the thread
    blockedLowerBodyMove( positions, frac_speed );
  }
  else {
    legmoveThread_ = new boost::thread( &PepperProxyManager::blockedLowerBodyMove, this, positions, frac_speed );
  }

  return true;
}

void PepperProxyManager::blockedLowerBodyMove( const std::vector<float> & positions, float frac_speed )
{
  int pos_size = positions.size();

  AL::ALValue names = "Leg";
  AL::ALValue angles;

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], HIP_ROLL + i );
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( names, 1.0 );
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    isSuccess = false;
  }
  bodyCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onMoveLowerBodySuccess" : "onMoveLowerBodyFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done lower body movement.\n" );
  delete legmoveThread_;
  legmoveThread_ = NULL;
}

bool PepperProxyManager::moveLowerBodyWithJointTrajectory( const std::vector< std::vector<float> > & trajectory,
                                                           const std::vector<float> & times_to_reach )
{
  if (!motionProxy_ || bodyCtrl_)
    return false;

  size_t traj_size = trajectory.size();

  if (traj_size <= 0 || traj_size != times_to_reach.size())
    return false;

  if (legmoveThread_ && legmoveThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "lower body movement is in progress.\n" );
    return false;
  }

  bodyCtrl_ = true;

  if (legmoveThread_) { // we already in the thread
    blockedLowerBodyMoveTraj( trajectory, times_to_reach );
  }
  else {
    legmoveThread_ = new boost::thread( &PepperProxyManager::blockedLowerBodyMoveTraj, this, trajectory, times_to_reach );
  }

  return true;
}

void PepperProxyManager::blockedLowerBodyMoveTraj( const std::vector< std::vector<float> > & trajectory,
                                                   const std::vector<float> & times_to_reach )
{
  size_t traj_size = trajectory.size();

  AL::ALValue names = AL::ALValue::array( "HipRoll",
                                          "HipPitch",
                                          "KneePitch" );

  AL::ALValue joints;
  AL::ALValue times;

  joints.arraySetSize( 3 );
  times.arraySetSize( 3 );

  for (size_t j = 0; j < 3; ++j) {
    AL::ALValue angles;
    AL::ALValue ttr;

    angles.arraySetSize( traj_size );
    ttr.arraySetSize( traj_size );

    joints[j] = angles;
    times[j] = ttr;

    float time_to_reach_for_pt = 0.0;
    for (size_t jp = 0; jp < traj_size; ++jp) {
      joints[j][jp] = clamp( trajectory[jp][j], HIP_ROLL + j );
      time_to_reach_for_pt += times_to_reach[jp];
      times[j][jp] = time_to_reach_for_pt;
    }
  }

  bool isSuccess = true;

  try {
    motionProxy_->setStiffnesses( names, 1.0 );
    motionProxy_->angleInterpolation( names, joints, times, true );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle interpolation to %s.\n", joints.toString().c_str() );
    isSuccess = false;
  }
  bodyCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onMoveLowerBodySuccess" : "onMoveLowerBodyFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done lower body movement.\n" );
  delete legmoveThread_;
  legmoveThread_ = NULL;
}

bool PepperProxyManager::moveBodyWithJointPos( const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_ || bodyCtrl_ || lArmCtrl_ || rArmCtrl_)
    return false;

  int pos_size = positions.size();
  if (pos_size != jointLimits_.getSize()) {
    return false;
  }
  AL::ALValue names = "Body";
  AL::ALValue angles;

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], i );
  }
  try {
    motionProxy_->setStiffnesses( names, 1.0 );
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;
}

bool PepperProxyManager::moveBodyWithRawTrajectoryData( const std::vector<std::string> & joint_names,
                                                        const std::vector< std::vector<AngleControlPoint> > & key_frames,
                                                        const std::vector< std::vector<float> > & time_stamps, bool isBezier )
{
  // minimal check in this method. Use under you own risk!
  // TODO: this is a silly wrapper function. Should just do a simple cast.
  if (!motionProxy_ || lArmCtrl_ || rArmCtrl_ || bodyCtrl_)
    return false;

  if (bodymoveThread_ && bodymoveThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "body movement is in progress.\n" );
    return false;
  }

  size_t joint_size = joint_names.size();
  if (joint_size != key_frames.size() || joint_size != time_stamps.size()) {
    ERROR_MSG( "Inconsistent trajectory data specification." );
    return false;
  }
  bodyCtrl_ = true;

  if (bodymoveThread_) { // we already in the thread
    blockedBodyMoveWithData( joint_names, key_frames, time_stamps, isBezier );
  }
  else {
    bodymoveThread_ = new boost::thread( &PepperProxyManager::blockedBodyMoveWithData, this, joint_names, key_frames, time_stamps, isBezier );
  }

  return true;
}

void PepperProxyManager::blockedBodyMoveWithData( const std::vector<std::string> & joint_names,
                                                  const std::vector< std::vector<AngleControlPoint> > & key_frames,
                                                  const std::vector< std::vector<float> > & time_stamps, bool isBezier )
{
  AL::ALValue keys;
  AL::ALValue times;

  size_t joint_size = joint_names.size();

  keys.arraySetSize( joint_size );
  times.arraySetSize( joint_size );

  for (int i = 0; i < joint_size; i++) {
    int key_size = key_frames[i].size();
    int ts_size = time_stamps[i].size();

    keys[i].arraySetSize( key_size );
    for (int j = 0; j < key_size; ++j) {
      if (isBezier) {
        keys[i][j] = AL::ALValue::array( key_frames[i][j].angle, AL::ALValue::array( key_frames[i][j].bparam1.type,
            key_frames[i][j].bparam1.dtime, key_frames[i][j].bparam1.dangle ), AL::ALValue::array( key_frames[i][j].bparam2.type,
                key_frames[i][j].bparam2.dtime, key_frames[i][j].bparam2.dangle ) );
      }
      else {
        keys[i][j] = key_frames[i][j].angle;
      }
    }
    times[i].arraySetSize( ts_size );
    for (int j = 0; j < ts_size; ++j) {
      times[i][j] = time_stamps[i][j];
    }
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( "Body", 1.0 );

    motionProxy_->angleInterpolationBezier( joint_names, times, keys );
  }
  catch (...) {
    ERROR_MSG( "Unable to move joints in specified raw trajectories." );
    isSuccess = false;
  }
  bodyCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onFullBodyMotionSuccess" : "onFullBodyMotionFailed"), NULL );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done body movement.\n" );

  delete bodymoveThread_;
  bodymoveThread_ = NULL;
}

bool PepperProxyManager::setHandPosition( bool isLeft, float openRatio, bool keepStiff )
{
  if (!motionProxy_ || bodyCtrl_)
    return false;

  if (isLeft) {
    if (lHandCtrl_)
      return false;

    if (lhandmoveThread_ && lhandmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "left hand movement is in progress.\n" );
      return false;
    }
  }
  else {
    if (rHandCtrl_)
      return false;

    if (rhandmoveThread_ && rhandmoveThread_->get_id() != boost::this_thread::get_id()) {
      ERROR_MSG( "right hand movement is in progress.\n" );
      return false;
    }
  }

  if (isLeft) {
    lHandCtrl_ = true;

    if (lhandmoveThread_) { // we already in the thread
      blockedHandMove( isLeft, openRatio, keepStiff );
    }
    else {
      lhandmoveThread_ = new boost::thread( &PepperProxyManager::blockedHandMove, this, isLeft, openRatio, keepStiff );
    }
  }
  else {
    rHandCtrl_ = true;

    if (rhandmoveThread_) { // we already in the thread
      blockedHandMove( isLeft, openRatio, keepStiff );
    }
    else {
      rhandmoveThread_ = new boost::thread( &PepperProxyManager::blockedHandMove, this, isLeft, openRatio, keepStiff );
    }
  }

  return true;
}

void PepperProxyManager::blockedHandMove( bool isLeft, float openRatio, bool keepStiff )
{
  float oratio = 1.0;
  if (openRatio <= 1.0 && openRatio >= 0.0) {
    oratio = openRatio;
  }

  AL::ALValue names;
  AL::ALValue angles;

  names.arraySetSize( 1 );
  angles.arraySetSize( 1 );

  if (isLeft) {
    names[0] = "LHand";
    angles[0] = oratio * ((float)jointLimits_[L_HAND][1] - (float)jointLimits_[L_HAND][0]);
  }
  else {
    names[0] = "RHand";
    angles[0] = oratio * ((float)jointLimits_[R_HAND][1] - (float)jointLimits_[R_HAND][0]);
  }

  bool isSuccess = true;
  try {
    motionProxy_->setStiffnesses( (isLeft ? "LHand" : "RHand"), 1.0 );

    motionProxy_->angleInterpolationWithSpeed( names, angles, 0.8 );

    if (!keepStiff) {
      motionProxy_->setStiffnesses( (isLeft ? "LHand" : "RHand"), 0.0 );
    }
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s.\n", angles.toString().c_str() );
    isSuccess = false;
  }

  if (isLeft) {
    lHandCtrl_ = false;
  }
  else {
    rHandCtrl_ = false;
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", isLeft ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onHandActionSuccess" : "onHandActionFailed"), arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done hand movement.\n" );
  if (isLeft) {
    delete lhandmoveThread_;
    lhandmoveThread_ = NULL;
  }
  else {
    delete rhandmoveThread_;
    rhandmoveThread_ = NULL;
  }
}

int PepperProxyManager::loadAudioFile( const std::string & text )
{
  int audioID = -1;
  if (audioPlayerProxy_) {
    try {
      audioID = audioPlayerProxy_->loadFile( text );
    } catch (const std::exception &error) {
      ERROR_MSG( "Unable to load audio %s.\n", text.c_str() );
    }
  }
  return audioID;
}

void PepperProxyManager::unloadAudioFile( const int audioID )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadFile( audioID );
  }
}

void PepperProxyManager::unloadAllAudioFiles()
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadAllFiles();
  }
}

void PepperProxyManager::playWebAudio( const std::string & url )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.playWebStream( url, 1.0, 0.0 );
  }
}

bool PepperProxyManager::playAudioID( const int audioID )
{
  if (!audioPlayerProxy_ || audioCtrl_) {
    return false;
  }

  if (audioThread_ && audioThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "audio play is in progress.\n" );
    return false;
  }
  audioCtrl_ = true;

  if (audioThread_) { // we already in the thread
    blockedPlayAudio( audioID );
  }
  else {
    audioThread_ = new boost::thread( &PepperProxyManager::blockedPlayAudio, this, audioID );
  }

  return true;
}

void PepperProxyManager::blockedPlayAudio( const int audioID )
{
  bool isSuccess = true;

  try {
    audioPlayerProxy_->play( audioID );
  }
  catch (...) {
    ERROR_MSG( "Unable to play audio id %d.\n", audioID );
    isSuccess = false;
  }
  audioCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(i)", audioID );

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onPlayAudioSuccess" : "onPlayAudioFailed"), arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );

  //DEBUG_MSG( "done audio.\n" );

  delete audioThread_;
  audioThread_ = NULL;
}

int PepperProxyManager::getAudioVolume()
{
  if (audioDeviceProxy_) {
    return audioDeviceProxy_->getOutputVolume();
  }
  else {
    return 0.0;
  }
}

void PepperProxyManager::setAudioVolume( const int vol )
{
  if (vol < 0 || vol > 100 )
    return;

  if (audioDeviceProxy_) {
    audioDeviceProxy_->setOutputVolume( vol );
  }
}

void PepperProxyManager::pauseAudioID( const int audioID )
{
  if (!audioCtrl_)
    return;

  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.pause( audioID );
  }
}

void PepperProxyManager::stopAllAudio()
{
  if (!audioCtrl_)
    return;

  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.stopAll();
  }
}

bool PepperProxyManager::playVideo( const std::string & url )
{
  if (tabletProxy_) {
    return tabletProxy_->call<bool>( "playVideo", url );
  }
  return false;
}

bool PepperProxyManager::stopVideo()
{
  if (tabletProxy_) {
    return tabletProxy_->call<bool>( "stopVideo" );
  }
  return false;
}

bool PepperProxyManager::pauseVideo()
{
  if (tabletProxy_) {
    return tabletProxy_->call<bool>( "pauseVideo" );
  }
  return false;
}

bool PepperProxyManager::resumeVideo()
{
  if (tabletProxy_) {
    return tabletProxy_->call<bool>( "resumeVideo" );
  }
  return false;
}

bool PepperProxyManager::startBehaviour( const std::string & behaviour )
{
  if (!behaviourManagerProxy_)
    return false;

  try {
    behaviourManagerProxy_->startBehavior( behaviour );
  }
  catch (const ALError& e) {
    ERROR_MSG( "Unable to start behaviour %s.\n", behaviour.c_str() );
    return false;
  }
  return true;
}

bool PepperProxyManager::runBehaviour( const std::string & behaviour )
{
  if (!behaviourManagerProxy_ || behaviourCtrl_)
    return false;

  if (behaviourThread_ && behaviourThread_->get_id() != boost::this_thread::get_id()) {
    ERROR_MSG( "behaviour is in progress.\n" );
    return false;
  }

  behaviourCtrl_ = true;

  if (behaviourThread_) { // we already in the thread
    blockedBehaviourRun( behaviour );
  }
  else {
    behaviourThread_ = new boost::thread( &PepperProxyManager::blockedBehaviourRun, this, behaviour );
  }
  return true;
}

void PepperProxyManager::blockedBehaviourRun( const std::string & behaviour )
{
  bool isSuccess = true;
  try {
    behaviourManagerProxy_->runBehavior( behaviour );
  }
  catch (const ALError& e) {
    ERROR_MSG( "Unable to run behaviour %s.\n", behaviour.c_str() );
    isSuccess = false;
  }
  behaviourCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( (isSuccess ? "onBehaviourComplete" : "onBehaviourFailed"), NULL );


  PyGILState_Release( gstate );

  //DEBUG_MSG( "done behaviour.\n" );
  delete behaviourThread_;
  behaviourThread_ = NULL;
}

void PepperProxyManager::stopBehaviour( const std::string & behaviour )
{
  if (behaviourManagerProxy_) {
    try {
      behaviourManagerProxy_->stopBehavior( behaviour );
      if (behaviourThread_) {
        behaviourThread_->join();
        delete behaviourThread_;
        behaviourThread_ = NULL;
      }
    }
    catch (const ALError& e) {
      ERROR_MSG( "Unable to stop behaviour %s.\n", behaviour.c_str() );
    }
  }
}

void PepperProxyManager::stopAllBehaviours()
{
  if (behaviourManagerProxy_) {
    try {
      behaviourManagerProxy_->stopAllBehaviors();
    }
    catch (const ALError& e) {
      ERROR_MSG( "Unable to stop all behaviours.\n" );
    }
  }
}

std::vector<std::string> PepperProxyManager::getBehaviourList( bool installed )
{
  std::vector<std::string> list;
  if (behaviourManagerProxy_) {
    if (installed) {
      return behaviourManagerProxy_->getInstalledBehaviors();
    }
    else {
      return behaviourManagerProxy_->getLoadedBehaviors();
    }
  }
  return list;
}

bool PepperProxyManager::directToWeb( const std::string & url )
{
  if (!tabletProxy_)
    return false;

  return tabletProxy_->call<bool>( "showWebview", url );
}

void PepperProxyManager::reloadWebpage( bool bypassCache )
{
  if (tabletProxy_) {
    tabletProxy_->call<void>( "reloadPage", bypassCache );
  }
}

void PepperProxyManager::clearWebpage( bool hide )
{
  if (tabletProxy_) {
    tabletProxy_->call<void>( "cleanWebview" );
    if (hide)
      tabletProxy_->call<void>( "hideWebview" );
  }
}

void PepperProxyManager::turnTabletOn( bool turnOn )
{
  if (tabletProxy_) {
    if (turnOn) {
      tabletProxy_->call<void>( "wakeUp" );
      tabletProxy_->call<void>( "turnScreenOn", true );
    }
    else {
      tabletProxy_->call<void>( "turnScreenOn", false );
      tabletProxy_->call<void>( "goToSleep" );
    }
  }
}

void PepperProxyManager::resetTablet()
{
  if (tabletProxy_) {
    tabletProxy_->call<void>( "resetTablet" );
  }
}

std::string PepperProxyManager::getTabletWiFiStatus()
{
  if (tabletProxy_) {
    return tabletProxy_->call<std::string>( "getWifiStatus" );
  }
  return "";
}

std::string PepperProxyManager::getTabletWiFiMacAddress()
{
  if (tabletProxy_) {
    return tabletProxy_->call<std::string>( "getWifiMacAddress" );
  }
  return "";
}

bool PepperProxyManager::connectTabletWiFiTo( const std::string ssid, const std::string wapkey )
{
  if (!tabletProxy_)
    return false;

  if (tabletProxy_->call<bool>( "configureWifi", "wpa", ssid, wapkey )) {
    return tabletProxy_->call<bool>( "connectWifi", ssid );
  }
  else {
    ERROR_MSG( "Invalid WiFi hotspot configuration!\n" );
  }
  return false;
}

bool PepperProxyManager::disconnectTabletWiFi()
{
  if (!tabletProxy_)
    return false;

  return tabletProxy_->call<bool>( "disconnectWifi" );
}

void PepperProxyManager::turnTabletWiFiOn( bool turnOn )
{
  if (tabletProxy_) {
    if (turnOn) {
      tabletProxy_->call<void>( "enableWifi" );
    }
    else {
      tabletProxy_->call<void>( "disableWifi" );
    }
  }
}

void PepperProxyManager::setChestLED( const NAOLedColour colour )
{
  if (ledProxy_) {
    if (isChestLEDPulsating_) {
      isChestLEDPulsating_ = false;
      pthread_join( runningThread_, NULL );
    }
    switch (colour) {
      case WHITE:
        ledProxy_->on( "ChestLeds" );
        break;
      case BLANK:
        ledProxy_->off( "ChestLeds" );
        break;
      case RED:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case BLUE:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case GREEN:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case YELLOW:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case PINK:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      default:
        break;
    }
  }
}

void PepperProxyManager::pulsatingChestLED( const NAOLedColour colour1, const NAOLedColour colour2, const float period )
{
  if (!ledProxy_)
    return;

  pthread_mutex_lock( &t_mutex_ );

  ledColourHex_.arraySetSize( 2 );
  ledColourHex_[0] = this->colour2Hex( colour1 );
  ledColourHex_[1] = this->colour2Hex( colour2 );
  ledChangePeriod_.arraySetSize( 2 );
  ledChangePeriod_[0] = ledChangePeriod_[1] = period;

  pthread_mutex_unlock( &t_mutex_ );

  if (isChestLEDPulsating_)
    return;

  isChestLEDPulsating_ = true;

  if (pthread_create( &runningThread_, NULL, pulse_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to pulse LED.\n" );
    return;
  }
}

void PepperProxyManager::setFaceLED( const NAOLedColour colour )
{
  if (ledProxy_) {
    switch (colour) {
      case WHITE:
        ledProxy_->on( "FaceLeds" );
        break;
      case BLANK:
        ledProxy_->off( "FaceLeds" );
        break;
      case RED:
        ledProxy_->off( "RightFaceLedsBlue" );
        ledProxy_->off( "RightFaceLedsGreen" );
        ledProxy_->on( "RightFaceLedsRed" );
        ledProxy_->off( "LeftFaceLedsBlue" );
        ledProxy_->off( "LeftFaceLedsGreen" );
        ledProxy_->on( "LeftFaceLedsRed" );
        break;
      case BLUE:
        ledProxy_->on( "RightFaceLedsBlue" );
        ledProxy_->off( "RightFaceLedsGreen" );
        ledProxy_->off( "RightFaceLedsRed" );
        ledProxy_->on( "LeftFaceLedsBlue" );
        ledProxy_->off( "LeftFaceLedsGreen" );
        ledProxy_->off( "LeftFaceLedsRed" );
        break;
      case GREEN:
        ledProxy_->off( "RightFaceLedsBlue" );
        ledProxy_->on( "RightFaceLedsGreen" );
        ledProxy_->off( "RightFaceLedsRed" );
        ledProxy_->off( "LeftFaceLedsBlue" );
        ledProxy_->on( "LeftFaceLedsGreen" );
        ledProxy_->off( "LeftFaceLedsRed" );
        break;
      case YELLOW:
        ledProxy_->off( "RightFaceLedsBlue" );
        ledProxy_->on( "RightFaceLedsGreen" );
        ledProxy_->on( "RightFaceLedsRed" );
        ledProxy_->off( "LeftFaceLedsBlue" );
        ledProxy_->on( "LeftFaceLedsGreen" );
        ledProxy_->on( "LeftFaceLedsRed" );
        break;
      case PINK:
        ledProxy_->on( "RightFaceLedsBlue" );
        ledProxy_->off( "RightFaceLedsGreen" );
        ledProxy_->on( "RightFaceLedsRed" );
        ledProxy_->on( "LeftFaceLedsBlue" );
        ledProxy_->off( "LeftFaceLedsGreen" );
        ledProxy_->on( "LeftFaceLedsRed" );
        break;
      default:
        break;
    }
  }
}

void PepperProxyManager::getBatteryStatus( int & percentage, bool & isplugged, bool & ischarging, bool & isdischarging )
{
  if (memoryProxy_) {
    percentage = memoryProxy_->getData( "BatteryChargeChanged" );
    isplugged = memoryProxy_->getData( "BatteryPowerPluggedChanged" );
    ischarging = memoryProxy_->getData( "BatteryChargingFlagChanged" );
    isdischarging = memoryProxy_->getData( "BatteryDisChargingFlagChanged" );
  }
}

void PepperProxyManager::setAutonomousAbility( const std::string & ability, bool enable )
{
  /*
  if (autoLifeProxy_) {
    autoLifeProxy_->setAutonomousAbilityEnabled( ability, enable ); // TODO: method is not available in SDK 2.4. yet
  }
  */
  std::string cmd = ability;
  std::transform( cmd.begin(), cmd.end(), cmd.begin(), ::tolower );
  if (ability.compare( "basicawareness" ) == 0 && basicAwarenessProxy_) {
    if (enable) {
      basicAwarenessProxy_->startAwareness();
    }
    else {
      basicAwarenessProxy_->stopAwareness();
    }
  }
  else if (ability.compare( "autonomousblinking" ) == 0 && autoblinkingProxy_) {
    autoblinkingProxy_->call<void>( "setEnabled", enable );
  }
  else if (ability.compare( "backgroundmovement" ) == 0 && backgroundMovementProxy_) {
    backgroundMovementProxy_->call<void>( "setEnabled", enable );
  }
  else if (ability.compare( "listeningmovement" ) == 0 && listenMovementProxy_) {
    listenMovementProxy_->call<void>( "setEnabled", enable );
  }
}

void PepperProxyManager::gotoStation()
{
  if (rechargeProxy_) {
    rechargeProxy_->goToStation();
  }
}

void PepperProxyManager::leaveStation()
{
  if (rechargeProxy_) {
    rechargeProxy_->leaveStation();
  }
}

void PepperProxyManager::shutdownRobot( bool restart )
{
  if (systemProxy_) {
    if (restart)
      systemProxy_->reboot();
    else
      systemProxy_->shutdown();
  }
}

void PepperProxyManager::fini()
{
  if (isChestLEDPulsating_) {
    isChestLEDPulsating_ = false;
    pthread_join( runningThread_, NULL );
  }
  if (speechProxy_) {
    speechProxy_.reset();
  }
  if (animateSpeechProxy_) {
    animateSpeechProxy_.reset();
  }
  if (motionProxy_) {
    motionProxy_.reset();
  }
  if (ledProxy_) {
    ledProxy_.reset();
  }
  if (audioDeviceProxy_) {
    audioDeviceProxy_.reset();
  }
  if (audioPlayerProxy_) {
    audioPlayerProxy_.reset();
  }
  if (behaviourManagerProxy_) {
    behaviourManagerProxy_.reset();
  }
  if (postureProxy_) {
    postureProxy_.reset();
  }
  if (navigationProxy_) {
    navigationProxy_.reset();
  }
  if (autoLifeProxy_) {
    autoLifeProxy_.reset();
  }
  if (basicAwarenessProxy_) {
    basicAwarenessProxy_.reset();
  }
  if (rechargeProxy_) {
    rechargeProxy_.reset();
  }
  if (systemProxy_) {
    systemProxy_.reset();
  }
  if (autoblinkingProxy_) {
    autoblinkingProxy_.reset();
    backgroundMovementProxy_.reset();
    listenMovementProxy_.reset();
  }
  if (tabletProxy_) {
    tabletProxy_.reset();
  }
}

// helper function
void PepperProxyManager::continuePulseChestLED()
{
  while (isChestLEDPulsating_) {
    pthread_mutex_lock( &t_mutex_ );
    ledProxy_->fadeListRGB( "ChestLeds", ledColourHex_, ledChangePeriod_ );
    pthread_mutex_unlock( &t_mutex_ );

    usleep( 10000 );
  }
}

void PepperProxyManager::timeoutCheck()
{
  struct timeval now;

  do {
    usleep( 10000 );
    gettimeofday( &now, NULL );
  } while (((now.tv_sec - cmdTimeStamp_.tv_sec) * 1000000 + now.tv_usec - cmdTimeStamp_.tv_usec) < kMotionCommandGapTolerance);
  this->cancelBodyMovement();
  timeoutThread_ = (pthread_t)NULL;
}

float PepperProxyManager::clamp( float val, int jointInd )
{
  if (jointInd >= jointLimits_.getSize()) {
    ERROR_MSG( "invalid joint index %d\n", jointInd );
    return val;
  }

  if (val < (float)jointLimits_[jointInd][0]) {
    return (float)jointLimits_[jointInd][0];
  }
  else if (val > (float)jointLimits_[jointInd][1]) {
    return (float)jointLimits_[jointInd][1];
  }
  else {
    return val;
  }
}

inline float PepperProxyManager::clamp_change( float chg, float val, int jointInd )
{
  if (jointInd >= jointLimits_.getSize()) {
    ERROR_MSG( "invalid joint index %d\n", jointInd );
    return 0.0;
  }
  float sum = chg + val;
  if (sum < (float)jointLimits_[jointInd][0]) {
    return (float)jointLimits_[jointInd][0] - val;
  }
  else if (sum > (float)jointLimits_[jointInd][1]) {
    return (float)jointLimits_[jointInd][1] - val;
  }
  else {
    return chg;
  }
}

int PepperProxyManager::colour2Hex( const NAOLedColour colour )
{
  int retval = 0;
  switch (colour) {
    case WHITE:
      retval = 0x00ffffff;
      break;
    case RED:
      retval = 0x00ff0000;
      break;
    case BLUE:
      retval = 0x000000ff;
      break;
    case GREEN:
      retval = 0x0000ff00;
      break;
    case YELLOW:
      retval = 0x00ffff00;
      break;
    case PINK:
      retval = 0x00ff00ff;
      break;
    default:
      break;
  }
  return retval;
}
} // namespace pyride
