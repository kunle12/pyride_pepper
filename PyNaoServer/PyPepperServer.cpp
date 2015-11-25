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
#include "PyPepperModule.h"

PYRIDE_LOGGING_DECLARE( "/home/nao/log/tin.log" );

namespace pyride {

static const int kMaxAudioSamples = 16384;
static const float kHFOV = 60.97;
static const float kVFOV = 34.80;

void * videograb_thread( void * controller )
{
  ((NaoCam *)controller)->continueProcessing();
  return NULL;
}
  
NaoCam::NaoCam( boost::shared_ptr<ALBroker> pBroker, const std::string & pName ) :
  broker_( pBroker ),
  procThread_( (pthread_t)NULL ),
  gvmName_( "" ),
  takeSnapShot_( false )
{
  pthread_attr_init( &threadAttr_ );
  pthread_attr_setinheritsched( &threadAttr_, PTHREAD_EXPLICIT_SCHED );
  pthread_attr_setschedpolicy( &threadAttr_, SCHED_OTHER );
  devInfo_.deviceID = "NAOCAM01";
  devInfo_.deviceName = devInfo_.deviceLabel = "NAO Head Cam";
  devInfo_.index = 0;
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
    gvmName_ = videoProxy_->subscribeCamera( GVMName, AL::kTopCamera, AL::kVGA, kYUV422ColorSpace,
                                      /*kRGBColorSpace,*/ 5 );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not register GVM to ALVideoDevice.\n" );
    videoProxy_.reset();
    return isInitialised_;
  }
  
  if (!this->getDefaultVideoSettings()) {
    return isInitialised_;
  }
 
  INFO_MSG( "camera resolution = %d and is %s\n", videoProxy_->getResolution( gvmName_ ),
      (videoProxy_->isCameraOpen(AL::kTopCamera) ? "open" : "closed") );
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
  
bool NaoCam::getDefaultVideoSettings()
{
  vSettings_.fps = 5;
  vSettings_.format = RGB;
  vSettings_.resolution = 2; // 320x240
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
                          48000, (int)ALLCHANNELS, 0 );
  }
  catch (const std::exception &error) {
    ERROR_MSG( "PyPepperServer: Could not set parameters to audio device.\n");
    return isInitialised_;
  }
  packetStamp_ = 0;
  clientNo_ = 0;
  isStreaming_ = false;
  isInitialised_ = true;
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
  
  //memcpy( audioBuffer_, pDataInterleaved, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels );
  
  //char nofSkippedChannels = 3;
  
  const AL_SOUND_FORMAT * iterAudioDataSource = pDataInterleaved;
  const AL_SOUND_FORMAT * iterAudioDataSourceEnd = pDataInterleaved+pNbrSamples*pNbOfInputChannels;
  
  AL_SOUND_FORMAT * iterAudioDataSelectedChannel = audioBuffer_;
  // take the 1st left channel
  while (iterAudioDataSource < iterAudioDataSourceEnd) {
    (*iterAudioDataSelectedChannel++) = (*iterAudioDataSource++);
    iterAudioDataSource += 3; //nofSkippedChannels;
  }
  
  //printf( "captured %d audio samples data size %d nofchan %d\n", pNbrSamples, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels, pNbOfInputChannels );
  this->processAndSendAudioData( audioBuffer_, pNbrSamples );
}

// implementation of PyPepperServer
PyPepperServer::PyPepperServer( boost::shared_ptr<ALBroker> pBroker, const std::string & pName ) :
  ALSoundExtractor( pBroker, pName ),
  AudioDevice( 1 ),
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
  functionName("onSingleChestButtonPressed", getName(), "Method called when the chest button pressed once.");
  BIND_METHOD( PyPepperServer::onSingleChestButtonPressed );
  functionName("onDoubleChestButtonPressed", getName(), "Method called when the chest button pressed twice.");
  BIND_METHOD( PyPepperServer::onDoubleChestButtonPressed );
  functionName("onTripleChestButtonPressed", getName(), "Method called when the chest button pressed three times.");
  BIND_METHOD( PyPepperServer::onTripleChestButtonPressed );
  functionName("onBatteryPowerPlugged", getName(), "Method called when the battery charger is plugged or unplugged.");
  BIND_METHOD( PyPepperServer::onBatteryPowerPlugged );
  functionName("onBatteryChargeChanged", getName(), "Method called when a change in battery level.");
  BIND_METHOD( PyPepperServer::onBatteryChargeChanged );
}

void PyPepperServer::init()
{
  PYRIDE_LOGGING_INIT;

  NaoCam * topCam = new NaoCam( getParentBroker(), getName() );
  if (topCam->initDevice()) {
    naoCams_.push_back( topCam );
  }

  if (this->initDevice()) {
    naoAudio_.push_back( this );
  }

  try {
    memoryProxy_ = boost::shared_ptr<ALMemoryProxy>(new ALMemoryProxy( getParentBroker() ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyPepperServer: Could not create a proxy to ALMemory.\n");
    memoryProxy_.reset();
  }
  
  // disable default single chest button press
  /*
  try {
    boost::shared_ptr<ALSentinelProxy> sentinelProxy = boost::shared_ptr<ALSentinelProxy>(new ALSentinelProxy( getParentBroker() ));
    sentinelProxy->enableDefaultActionSimpleClick( false );
    sentinelProxy->enableDefaultActionDoubleClick( false );
    sentinelProxy->enableDefaultActionTripleClick( false );
  }
  catch (const ALError& e) {}
  */

  PepperProxyManager::instance()->initWithBroker( getParentBroker(), memoryProxy_ );
  ServerDataProcessor::instance()->init( naoCams_, naoAudio_ );
  ServerDataProcessor::instance()->addCommandHandler( this );
  AppConfigManager::instance()->loadConfigFromFile( DEFAULT_CONFIGURATION_FILE );
  ServerDataProcessor::instance()->setClientID( AppConfigManager::instance()->clientID() );
  ServerDataProcessor::instance()->setDefaultRobotInfo( PEPPER, AppConfigManager::instance()->startPosition() );
  
  PythonServer::instance()->init( AppConfigManager::instance()->enablePythonConsole(), PyPepperModule::instance() );
  ServerDataProcessor::instance()->discoverConsoles();

  if (memoryProxy_) {
    INFO_MSG( "Nao memory proxy is successfully initialised.\n" );
    /* subscribe to sensor events */
    memoryProxy_->subscribeToEvent( "Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value", "PyPepperServer", "onRightBumperPressed" );
    memoryProxy_->subscribeToEvent( "Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value", "PyPepperServer", "onLeftBumperPressed" );
    memoryProxy_->subscribeToEvent( "Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value", "PyPepperServer", "onBackBumperPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/SimpleClickOccurred", "PyPepperServer", "onSingleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/DoubleClickOccurred", "PyPepperServer", "onDoubleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/TripleClickOccurred", "PyPepperServer", "onTripleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "BatteryPowerPluggedChanged", "PyPepperServer", "onBatteryPowerPlugged" );
    memoryProxy_->subscribeToEvent( "BatteryChargeChanged", "PyPepperServer", "onBatteryChargeChanged" );
  }
}

void PyPepperServer::fini()
{
  this->notifySystemShutdown();

  for (int i = 0; i < naoCams_.size(); i++) {
    VideoDevice * naocam = naoCams_.at( i );
    naocam->finiDevice();
    delete naocam;
  }
  naoCams_.clear();
  
  this->finiDevice();
  naoAudio_.clear();

  PepperProxyManager::instance()->fini();
  AppConfigManager::instance()->fini();
  ServerDataProcessor::instance()->fini();

  /*
  if (memoryProxy_) {
    memoryProxy_->unsubscribeToEvent( "Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value", getName() );
    memoryProxy_->unsubscribeToEvent( "Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value", getName() );
    memoryProxy_->unsubscribeToEvent( "Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/SimpleClickOccurred", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/DoubleClickOccurred", getName() );
    memoryProxy_->unsubscribeToEvent( "ALChestButton/TripleClickOccurred", getName() );
    memoryProxy_.reset();
  }*/
}

bool PyPepperServer::executeRemoteCommand( PyRideExtendedCommand command, 
                                            const unsigned char * optionalData,
                                            const int optionalDataLength )
{
  // implement the routine to handle commands defined in PyRideExtendedCommand
  // in PyRideCommon.h
  // for example:
  bool status = true;
  switch (command) {
    case SPEAK:
    {
      float volume = *((float *)optionalData);
      //DEBUG_MSG( "received volume %f\n", volume );
      char * text = (char *)optionalData + sizeof( float );
      PepperProxyManager::instance()->sayWithVolume( std::string( text, optionalDataLength - sizeof( float ) ), volume );
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
    default:
      status = false;
      break;
  }
  return status;
}

void PyPepperServer::cancelCurrentOperation()
{
  PepperProxyManager::instance()->sayWithVolume( "Emergency Stop!" );
}

PyPepperServer::~PyPepperServer()
{
  this->fini();
}

#pragma callback functions from alproxies.

void PyPepperServer::onRightBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value" );
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
  float stat =  memoryProxy_->getData( "Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value" );
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
  float stat =  memoryProxy_->getData( "Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value" );
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

void PyPepperServer::onSingleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
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
  
  /**
   * Check that the bumper is pressed.
   */
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
  
  /**
   * Check that the bumper is pressed.
   */
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", 3 );
  
  PyPepperModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyPepperServer::onBatteryPowerPlugged()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  bool isplugged =  memoryProxy_->getData( "BatteryPowerPluggedChanged" );
  PyObject * arg = NULL;
    
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", isplugged ? Py_True : Py_False );

  PyPepperModule::instance()->invokeCallback( "onPowerPluggedChange", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

void PyPepperServer::onBatteryChargeChanged()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
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