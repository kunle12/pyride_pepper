/*
 *  PyPepperServer.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 18/11/15.
 *  Copyright 2009, 2015 Galaxy Network. All rights reserved.
 *
 */

#include <sys/time.h>
#include <sys/select.h>
#include <stdio.h>

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

#include <alproxies/alaudiodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <althread/alcriticalsection.h>

#include <PyRideCommon.h>
#include <PythonServer.h>

#include "PyPepperServer.h"
#include "PepperProxyManager.h"
#include "AppConfigManager.h"
#include "VideoToWebBridge.h"
#include "AudioFeedbackStream.h"
#include "PyPepperModule.h"

PYRIDE_LOGGING_DECLARE( "/home/nao/log/tin.log" );

namespace pyride {

static const int kMaxAudioSamples = 16384;
static const int kAudioSampleRate = 16000;
static const float kHFOV = 57.2;
static const float kVFOV = 44.3;

void * videograb_thread( void * controller )
{
  ((NaoCam *)controller)->continueProcessing();
  return NULL;
}

NaoCam::NaoCam( boost::shared_ptr<ALBroker> pBroker, const std::string & name, const int cameraID ) :
  broker_( pBroker ),
  procThread_( (pthread_t)NULL ),
  gvmName_( "" ),
  takeSnapShot_( false ),
  cameraID_( cameraID )
{
  pthread_attr_init( &threadAttr_ );
  pthread_attr_setinheritsched( &threadAttr_, PTHREAD_EXPLICIT_SCHED );
  pthread_attr_setschedpolicy( &threadAttr_, SCHED_OTHER );
  devInfo_.deviceID = "PEPPERCAM" + cameraID;
  devInfo_.deviceName = devInfo_.deviceLabel = name;
  devInfo_.index = cameraID;
  devInfo_.shouldBeActive = true;
}

NaoCam::~NaoCam()
{
  pthread_attr_destroy( &threadAttr_ );
}

bool NaoCam::initDevice()
{
  if (isInitialised_) {
    return isInitialised_;
  }

  try {
    videoProxy_ = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy( broker_ ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALVideoDevice.\n");
    videoProxy_.reset();
    return isInitialised_;
  }

  std::string GVMName = "Controller_GVM";

  try {
    gvmName_ = videoProxy_->subscribeCamera( GVMName, cameraID_, AL::kVGA, kYUV422ColorSpace,
                                      /*kRGBColorSpace,*/ 20 );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not register GVM to ALVideoDevice.\n" );
    videoProxy_.reset();
    return isInitialised_;
  }

  if (!this->getDefaultVideoSettings()) {
    return isInitialised_;
  }

  INFO_MSG( "camera %d resolution = %d and is %s\n", cameraID_, videoProxy_->getResolution( gvmName_ ),
      (videoProxy_->isCameraOpen(cameraID_) ? "open" : "closed") );
  packetStamp_ = 0;
  clientNo_ = 0;
  isStreaming_ = false;
  isInitialised_ = true;

  INFO_MSG( "Nao camera is successfully initialised.\n" );
  return isInitialised_;
}

void NaoCam::finiDevice()
{
  if (!isInitialised_) {
    return;
  }

  this->finiWorkerThread();

  if (videoProxy_) {
    videoProxy_->unsubscribe( gvmName_ );
    videoProxy_.reset();
  }

  isInitialised_ = false;
}

bool NaoCam::initWorkerThread()
{
  if (pthread_create( &procThread_, &threadAttr_, videograb_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to grab Pepper camera images.\n" );
    return false;
  }
  return true;
}

void NaoCam::finiWorkerThread()
{
  isStreaming_ = false;

  if (procThread_) {
    pthread_join( procThread_, NULL ); // allow thread to exit
    procThread_ = (pthread_t)NULL;
  }
}

void NaoCam::continueProcessing()
{
  ALImage * results;

  //int width, height, nbLayers, colorSpace, seconds = 0;
  int rawDataSize = 0;
  unsigned char * rawData = NULL;
  //struct timeval pr1, pr2;
  //long wait;

  while (isStreaming_) {
    //gettimeofday( &pr1, NULL );

    results = (ALImage *)(videoProxy_->getDirectRawImageLocal( gvmName_ ));
    if (results) {
      rawData = (unsigned char *)(results->getData());
      rawDataSize = results->getSize();
      processAndSendImageData( rawData, rawDataSize, RAW );
      if (takeSnapShot_) {
        saveToJPEG( rawData, rawDataSize, RAW );
        takeSnapShot_ = false;
      }
    }
    videoProxy_->releaseDirectRawImage( gvmName_ );
  }
}

void NaoCam::takeSnapshot( const VideoDeviceDataHandler * dataHandler )
{
  dataHandler_ = (VideoDeviceDataHandler *)dataHandler;

  if (isStreaming_) {
    takeSnapShot_ = true;
  }
  else {
    ALImage * results = (ALImage *)(videoProxy_->getDirectRawImageLocal( gvmName_ ));
    if (results) {
      unsigned char * rawData = (unsigned char *)(results->getData());
      int rawDataSize = results->getSize();
      saveToJPEG( rawData, rawDataSize, RAW );
    }
    videoProxy_->releaseDirectRawImage( gvmName_ );
  }
}

bool NaoCam::setCameraParameter( int pid, int value )
{
  if (pid < -1 || pid > 34) {
    ERROR_MSG( "PyPepperServer: Invalid camera parameter %d.\n", pid );
    return false;
  }
  switch (pid) {
    case -1:
      return videoProxy_->setAllCameraParametersToDefault( gvmName_ );
    case 3:
    case 14:
    case 15:
    case 24:
      ERROR_MSG( "PyPepperServer: Disabled camera parameter %d.\n", pid );
      return false;
  }
  if (value < 0) {
    return videoProxy_->setCameraParameterToDefault( gvmName_, pid );
  }
  return videoProxy_->setCameraParameter( gvmName_, pid, value );
}

bool NaoCam::getDefaultVideoSettings()
{
  vSettings_.fps = 5;
  vSettings_.format = RGB;
  vSettings_.resolution = 2; // 640x480
  vSettings_.reserved = 0;

  this->setProcessParameters();

  return true;
}

bool PyPepperServer::initDevice()
{
  if (isInitialised_) {
    return isInitialised_;
  }

  try {
    audioDevice->callVoid( "setClientPreferences", getName(),
                          kAudioSampleRate, (int)FRONTCHANNEL, 0 );
    audioDevice->callVoid( "setParameter", std::string("outputSampleRate"),
                               kAudioSampleRate );
  }
  catch (const std::exception &error) {
    ERROR_MSG( "PyPepperServer: Could not set parameters to audio device.\n");
    return isInitialised_;
  }
  packetStamp_ = 0;
  clientNo_ = 0;
  isStreaming_ = false;
  isInitialised_ = true;
  aSettings_.reserved = (char)audioDevice->call<int>( "getOutputVolume" );
  INFO_MSG( "Nao audio device is successfully initialised.\n" );
  return isInitialised_;

}

void PyPepperServer::finiDevice()
{
  if (!isInitialised_) {
    return;
  }

  this->finiWorkerThread();

  isInitialised_ = false;

}

bool PyPepperServer::initWorkerThread()
{
  try {
    if (audioBuffer_)
      delete [] audioBuffer_; // reset buffer just in case

    audioBuffer_ = new AL_SOUND_FORMAT[kMaxAudioSamples];
    this->startDetection();
  }
  catch (const std::exception &error) {
    ERROR_MSG( "Unable to subscribe to audio capturing!\n" );
    return false;
  }
  return true;
}

void PyPepperServer::finiWorkerThread()
{
  isStreaming_ = false;

  try {
    this->stopDetection();
    if (audioBuffer_) {
      delete [] audioBuffer_;
      audioBuffer_ = NULL;
    }
  }
  catch (const std::exception &error) {
    //ERROR_MSG( "Unable to unsubscribe to audio capturing!\n" );
  }
}

void PyPepperServer::process( const int &pNbOfInputChannels, const int &pNbrSamples,
                       const AL_SOUND_FORMAT *pDataInterleaved, const AL::ALValue &pTimeStamp )
{
  if (!audioBuffer_)
    return;

  memcpy( audioBuffer_, pDataInterleaved, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels );

  /*char nofSkippedChannels = 3;

  const AL_SOUND_FORMAT * iterAudioDataSource = pDataInterleaved;
  const AL_SOUND_FORMAT * iterAudioDataSourceEnd = pDataInterleaved+pNbrSamples*pNbOfInputChannels;

  AL_SOUND_FORMAT * iterAudioDataSelectedChannel = audioBuffer_;
  // take the 1st left channel
  while (iterAudioDataSource < iterAudioDataSourceEnd) {
    (*iterAudioDataSelectedChannel++) = (*iterAudioDataSource++);
    iterAudioDataSource += 3; //nofSkippedChannels;
  }
  */

  //printf( "captured %d audio samples data size %d nofchan %d\n", pNbrSamples, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels, pNbOfInputChannels );
  this->processAndSendAudioData( audioBuffer_, pNbrSamples );
}

// implementation of PyPepperServer
PyPepperServer::PyPepperServer( boost::shared_ptr<ALBroker> pBroker, const std::string & pName ) :
  ALSoundExtractor( pBroker, pName ),
  AudioDevice(),
  audioBuffer_( NULL ),
  callbackMutex_( ALMutex::createALMutex() )
{
  setModuleDescription( "This is a server module for TIN." );
  functionName( "onRightBumperPressed", getName(), "Method called when the right bumper is pressed.");
  BIND_METHOD( PyPepperServer::onRightBumperPressed );
  functionName("onLeftBumperPressed", getName(), "Method called when the left bumper is pressed.");
  BIND_METHOD( PyPepperServer::onLeftBumperPressed );
  functionName("onBackBumperPressed", getName(), "Method called when the back bumper is pressed.");
  BIND_METHOD( PyPepperServer::onBackBumperPressed );

  functionName("onFrontTactilTouched", getName(), "Method called when the head front tactile is touched.");
  BIND_METHOD( PyPepperServer::onFrontTactilTouched );
  functionName("onMiddleTactilTouched", getName(), "Method called when the head middle tactile is touched.");
  BIND_METHOD( PyPepperServer::onMiddleTactilTouched );
  functionName("onRearTactilTouched", getName(), "Method called when the head rear tactile is touched.");
  BIND_METHOD( PyPepperServer::onRearTactilTouched );

  functionName("onRightHandBackTouched", getName(), "Method called when the back tactile of the right hand is touched.");
  BIND_METHOD( PyPepperServer::onRightHandBackTouched );
  functionName("onLeftHandBackTouched", getName(), "Method called when the back tactile of the left hand is touched.");
  BIND_METHOD( PyPepperServer::onLeftHandBackTouched );

  functionName("onMovementFailed", getName(), "Method called when body movement failed.");
  BIND_METHOD( PyPepperServer::onMovementFailed );

  functionName("onSingleChestButtonPressed", getName(), "Method called when the chest button pressed once.");
  BIND_METHOD( PyPepperServer::onSingleChestButtonPressed );
  functionName("onDoubleChestButtonPressed", getName(), "Method called when the chest button pressed twice.");
  BIND_METHOD( PyPepperServer::onDoubleChestButtonPressed );
  functionName("onTripleChestButtonPressed", getName(), "Method called when the chest button pressed three times.");
  BIND_METHOD( PyPepperServer::onTripleChestButtonPressed );

  functionName("onPowerHatchOpened", getName(), "Method called when the power hatch is opened or closed.");
  BIND_METHOD( PyPepperServer::onPowerHatchOpened );
  functionName("onConnectedToChargingStation", getName(), "Method called when the robot is connected or disconnected from the charging station.");
  BIND_METHOD( PyPepperServer::onConnectedToChargingStation );
  functionName("onBatteryNearlyEmpty", getName(), "Method called when the robot is connected or disconnected from the charging station.");
  BIND_METHOD( PyPepperServer::onBatteryNearlyEmpty );
  functionName("onBatteryPowerPlugged", getName(), "Method called when the battery charger is plugged or unplugged.");
  BIND_METHOD( PyPepperServer::onBatteryPowerPlugged );
  functionName("onBatteryChargeChanged", getName(), "Method called when a change in battery level.");
  BIND_METHOD( PyPepperServer::onBatteryChargeChanged );
}

void PyPepperServer::init()
{
  PYRIDE_LOGGING_INIT;

  NaoCam * topCam = new NaoCam( getParentBroker(), "Pepper Top Head Cam", AL::kTopCamera );
  if (topCam->initDevice()) {
    pepperCams_.push_back( topCam );
  }
  NaoCam * bottomCam = new NaoCam( getParentBroker(), "Pepper Bottom Head Cam", AL::kBottomCamera );
  if (bottomCam->initDevice()) {
    pepperCams_.push_back( bottomCam );
  }

  if (this->initDevice()) {
    pepperAudio_.push_back( this );
  }

  try {
    memoryProxy_ = boost::shared_ptr<ALMemoryProxy>(new ALMemoryProxy( getParentBroker() ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALMemory.\n");
    memoryProxy_.reset();
  }

  PepperProxyManager::instance()->initWithBroker( getParentBroker(), memoryProxy_ );
  ServerDataProcessor::instance()->init( pepperCams_, pepperAudio_ );
  ServerDataProcessor::instance()->addCommandHandler( this );
  AppConfigManager::instance()->loadConfigFromFile( DEFAULT_CONFIGURATION_FILE );
  ServerDataProcessor::instance()->setClientID( AppConfigManager::instance()->clientID() );
  ServerDataProcessor::instance()->setDefaultRobotInfo( PEPPER, AppConfigManager::instance()->startPosition(),
      (RobotCapability)(MOBILITY|VIDEO_FEEDBACK|AUDIO_FEEBACK|MANIPULATION));

  PythonServer::instance()->init( AppConfigManager::instance()->enablePythonConsole(), PyPepperModule::instance() );
  VideoToWebBridge::instance()->setPyModuleExtension( PyPepperModule::instance() );
  ServerDataProcessor::instance()->discoverConsoles();

  if (memoryProxy_) {
    INFO_MSG( "Pepper memory proxy is successfully initialised.\n" );
    /* subscribe to sensor events */
    memoryProxy_->subscribeToEvent( "RightBumperPressed", "PyPepperServer", "onRightBumperPressed" );
    memoryProxy_->subscribeToEvent( "LeftBumperPressed", "PyPepperServer", "onLeftBumperPressed" );
    memoryProxy_->subscribeToEvent( "BackBumperPressed", "PyPepperServer", "onBackBumperPressed" );

    memoryProxy_->subscribeToEvent( "FrontTactilTouched", "PyPepperServer", "onFrontTactilTouched" );
    memoryProxy_->subscribeToEvent( "MiddleTactilTouched", "PyPepperServer", "onMiddleTactilTouched" );
    memoryProxy_->subscribeToEvent( "RearTactilTouched", "PyPepperServer", "onRearTactilTouched" );

    memoryProxy_->subscribeToEvent( "HandRightBackTouched", "PyPepperServer", "onRightHandBackTouched" );
    memoryProxy_->subscribeToEvent( "HandLeftBackTouched", "PyPepperServer", "onLeftHandBackTouched" );

    memoryProxy_->subscribeToEvent( "ALMotion/MoveFailed", "PyPepperServer", "onMovementFailed" );

    memoryProxy_->subscribeToEvent( "ALChestButton/SimpleClickOccurred", "PyPepperServer", "onSingleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/DoubleClickOccurred", "PyPepperServer", "onDoubleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/TripleClickOccurred", "PyPepperServer", "onTripleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "BatteryPowerPluggedChanged", "PyPepperServer", "onBatteryPowerPlugged" );
    memoryProxy_->subscribeToEvent( "BatteryChargeChanged", "PyPepperServer", "onBatteryChargeChanged" );
    memoryProxy_->subscribeToEvent( "BatteryTrapIsOpen", "PyPepperServer", "onPowerHatchOpened" );
    memoryProxy_->subscribeToEvent( "ALBattery/ConnectedToChargingStation", "PyPepperServer", "onConnectedToChargingStation" );
    memoryProxy_->subscribeToEvent( "BatteryNearlyEmpty", "PyPepperServer", "onBatteryNearlyEmpty" );
  }
}

void PyPepperServer::fini()
{
  this->notifySystemShutdown();

  for (int i = 0; i < pepperCams_.size(); i++) {
    VideoDevice * naocam = pepperCams_.at( i );
    naocam->finiDevice();
    delete naocam;
  }
  pepperCams_.clear();

  this->finiDevice();
  pepperAudio_.clear();

  PepperProxyManager::instance()->fini();
  AppConfigManager::instance()->fini();
  ServerDataProcessor::instance()->fini();

  /*
  if (memoryProxy_) {
    memoryProxy_->unsubscribeToEvent( "RightBumperPressed", getName() );
    memoryProxy_->unsubscribeToEvent( "LeftBumperPressed", getName() );
    memoryProxy_->unsubscribeToEvent( "BackBumperPressed", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/SimpleClickOccurred", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/DoubleClickOccurred", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/TripleClickOccurred", getName() );
    memoryProxy_.reset();
  }*/
}

bool PyPepperServer::executeRemoteCommand( PyRideExtendedCommand command, int & retVal,
                                            const unsigned char * optionalData,
                                            const int optionalDataLength )
{
  // implement the routine to handle commands defined in PyRideExtendedCommand
  // in PyRideCommon.h
  // for example:
  bool status = true;
  retVal = 0;
  switch (command) {
    case SPEAK:
    {
      float volume = *((float *)optionalData);
      //DEBUG_MSG( "received volume %f\n", volume );
      char * text = (char *)optionalData + sizeof( float );
      // TODO: volume is not currently used.
      PepperProxyManager::instance()->say( std::string( text, optionalDataLength - sizeof( float ) ) );
    }
      break;
    case HEAD_MOVE_TO:
    {
      float newHeadYaw = *((float *)optionalData);
      float newHeadPitch = *((float *)optionalData+1);
      newHeadYaw = newHeadYaw * kHFOV * kDegreeToRAD;
      newHeadPitch = newHeadPitch * kVFOV * kDegreeToRAD;
      PepperProxyManager::instance()->moveHeadTo( newHeadYaw, newHeadPitch );
      PepperProxyManager::instance()->setHeadStiffness( 0.0 );
    }
      break;
    case UPDATE_BODY_POSE:
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      PepperProxyManager::instance()->updateBodyPose( newPose );
    }
      break;
    case UPDATE_AUDIO_SETTINGS:
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      int volume;
      memcpy( &volume, dataPtr, sizeof( int ) );
      PepperProxyManager::instance()->setAudioVolume( volume );
    }
      break;
    case VIDEO_FEEDBACK:
    {
      bool ison = (bool)optionalData[0];
      if (ison) { // only the client with the exclusive control can do video feedback
        VideoToWebBridge::instance()->start();
        retVal = 1;
      }
      else {
        VideoToWebBridge::instance()->stop();
        retVal = 0;
      }
    }
      break;
    case AUDIO_FEEDBACK:
    {
      bool ison = (bool)optionalData[0];
      if (ison) { // only the client with the exclusive control can do video feedback
        AudioFeedbackStream::instance()->addClient();
        retVal = 1;
      }
      else {
        AudioFeedbackStream::instance()->removeClient();
        retVal = 0;
      }
    }
      break;
    default:
      status = false;
      break;
  }
  return status;
}

void PyPepperServer::cancelCurrentOperation()
{
  PepperProxyManager::instance()->say( "Emergency Stop!" );
}

PyPepperServer::~PyPepperServer()
{
  this->fini();
}

#pragma callback functions from alproxies.

/** @name Event Callback Functions
 *
 */
/**@{*/
/*! \typedef onBumperPressed(side)
 *  \memberof PyPepper.
 *  \brief Callback function when one of Pepper's bumpers is pressed.
 *  \param str side: side = "right" for the right bumper or  "left" for the left bumper or "back" for the back bumper
 *  \return None.
 */
void PyPepperServer::onRightBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "RightBumperPressed" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "right" );

    PyPepperModule::instance()->invokeCallback( "onBumperPressed", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyPepperServer::onLeftBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "LeftBumperPressed" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "left" );

    PyPepperModule::instance()->invokeCallback( "onBumperPressed", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyPepperServer::onBackBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "BackBumperPressed" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "back" );

    PyPepperModule::instance()->invokeCallback( "onBumperPressed", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

/*! \typedef onHeadTactileTouched(side)
 *  \memberof PyPepper.
 *  \brief Callback function when one of Pepper's head tactile sensor is touched.
 *  \param str side: side = "front" for the front tactile,  "middle" for the middle tactile and "rear" from the back tactile.
 *  \return None.
 */
void PyPepperServer::onFrontTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "FrontTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "front" );

    PyPepperModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyPepperServer::onMiddleTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "MiddleTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "middle" );

    PyPepperModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyPepperServer::onRearTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "RearTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "rear" );

    PyPepperModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

/*! \typedef onRightHandTouched(status)
 *  \memberof PyPepper.
 *  \brief Callback function when one of Pepper's right hand tactile sensor is touched.
 *  \param bool status. True == left hand touched; False == otherwise.
 *  \return None.
 */
void PyPepperServer::onRightHandBackTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandRightBackTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", (stat > 0.5f ? Py_True : Py_False) );

  PyPepperModule::instance()->invokeCallback( "onRightHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onLeftHandTouched(status)
 *  \memberof PyPepper.
 *  \brief Callback function when one of Pepper's left hand tactile sensor is touched.
 *  \param bool status. True == left hand touched; False == otherwise.
 *  \return None.
 */
void PyPepperServer::onLeftHandBackTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandLeftBackTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", (stat > 0.5f ? Py_True : Py_False) );

  PyPepperModule::instance()->invokeCallback( "onLeftHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onMovementFailed()
 *  \memberof PyPepper.
 *  \brief Callback function when movement or navigation command failed.
 *  \return None.
 */
void PyPepperServer::onMovementFailed()
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( "onMovementFailed", NULL );

  PyGILState_Release( gstate );
}

/*! \typedef onChestButtonPressed(noftimes)
 *  \memberof PyPepper.
 *  \brief Callback function when Pepper's chest button is pressed.
 *  \param int noftimes. Number of button presses (between 1 and 3).
 *  \return None.
 */
void PyPepperServer::onSingleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the button is pressed.
   */
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(i)", 1 );

  PyPepperModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyPepperServer::onDoubleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(i)", 2 );

  PyPepperModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyPepperServer::onTripleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(i)", 3 );

  PyPepperModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onPowerPluggedChange(status)
 *  \memberof PyPepper.
 *  \brief Callback function when one of Pepper's left hand tactile sensor is touched.
 *  \param bool status. True == power plugged in; False == otherwise.
 *  \return None.
 */
void PyPepperServer::onBatteryPowerPlugged()
{
  ALCriticalSection section( callbackMutex_ );

  bool isplugged =  memoryProxy_->getData( "BatteryPowerPluggedChanged" );
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", isplugged ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( "onPowerPluggedChange", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onPowerHatchOpened(status)
 *  \memberof PyPepper.
 *  \brief Callback function when the power hatch is opened or closed.
 *  \param bool status. True == power hatch opened; False == otherwise.
 *  \return None.
 */
void PyPepperServer::onPowerHatchOpened()
{
  ALCriticalSection section( callbackMutex_ );

  bool isopened =  memoryProxy_->getData( "BatteryTrapIsOpen" );
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", isopened ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( "onPowerHatchOpened", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onConnectedToChargingStation(status)
 *  \memberof PyPepper.
 *  \brief Callback function when the robot is connected or disconnected to the charging station.
 *  \param bool status. True == connected to charing station; False == otherwise.
 *  \return None.
 */
void PyPepperServer::onConnectedToChargingStation()
{
  ALCriticalSection section( callbackMutex_ );

  bool isopened =  memoryProxy_->getData( "ALBattery/ConnectedToChargingStation" );
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", isopened ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( "onConnectedToChargingStation", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onBatteryNearlyEmpty()
 *  \memberof PyPepper.
 *  \brief Callback function when the battery is near empty and needs to recharge.
 *  \return None.
 */
void PyPepperServer::onBatteryNearlyEmpty()
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( "onBatteryNearlyEmpty", NULL );

  PyGILState_Release( gstate );
}

/*! \typedef onBatteryChargeChange(bat_percent, is_discharging)
 *  \memberof PyPepper.
 *  \brief Callback function when the Pepper battery status has changed.
 *  \param int bat_percent. The remaining battery percentage within [0..100].
 *  \param bool is_discharging. True == the battery is discharging, False == the batter is charging.
  *  \return None.
 */
void PyPepperServer::onBatteryChargeChanged()
{
  ALCriticalSection section( callbackMutex_ );

  int batpercent =  memoryProxy_->getData( "BatteryChargeChanged" );
  bool discharging = memoryProxy_->getData( "BatteryDisChargingFlagChanged" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(iO)", batpercent, discharging ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( "onBatteryChargeChange", arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onSystemShutdown()
 *  \memberof PyPepper.
 *  \brief Callback function when the Pepper is shutting down.
 *  \return None.
 *  \note Currently (v2.4.3) NaoQi does not exit correctly when Pepper is shutting down using the chest button. Init script needs update.
 */
void PyPepperServer::notifySystemShutdown()
{
  INFO_MSG( "PyPepperServer is shutting down..\n" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPepperModule::instance()->invokeCallback( "onSystemShutdown", arg );

  PyGILState_Release( gstate );
}
} // namespace pyride
