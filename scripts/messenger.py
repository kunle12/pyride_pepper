import PyPepper
import time
import constants
import tinstate
import tininfo
import tinmind

from twitter import *
from timers import *

class TextMessage:
  pass

class Messenger( timerclient.TimerClient ):
  def __init__( self ):
    super(Messenger, self ).__init__()
    self.lastoken = 0
    self.token = 0
    self.twtaccess = None
    self.messages = []
    self.archive = []
    self.timedmessages = []
    self.useraliases = {}
    self.notifmoveTime = None
    self.startupTime = None

    #load last twitter token
    f = None
    try:
      f = open( '/home/nao/recordings/laststate.txt', 'r')
      txtlines = f.readlines()
      for line in txtlines:
        token = line.split('=')
        if len( token ) == 2 and token[0].strip() == 'token':
          self.lastoken = int( token[1].strip() )
          f.close()
    except:
      pass

    f = None
    #load last twitter token
    try:
      f = open( '/home/nao/naoqi/preferences/useralias.txt', 'r')
      txtlines = f.readlines()
      for line in txtlines:
        token = line.split('=')
        if len( token ) == 2:
          self.useraliases[ token[0].strip() ] = token[1].strip()
    except:
      pass

  def checkin( self ):
    try:
      self.twtaccess = Twitter(auth=OAuth(tininfo.TiNTwOAToken, tininfo.TiNTwOASecret, tininfo.TiNTwConKey, tininfo.TiNTwConSecret))
    except:
      print 'Unable to check into TiN twitter app. Try again in 10mins.'
      return False

    self.startupTime = time.localtime()
    self.updatestatus( time.strftime( "I'm back online at %H:%M to serve ", time.localtime() ) + tininfo.TiNLocation + "."  )

    self.token = self.lastoken
    self.getmessages()
    tid = PyPepper.addTimer( 60, -1, 60 )
    self.timercontext[tid] = 'getmsg'
    timermanager.addTimer( tid, self )
    #add purge archive time
    purgetime = timermanager.calcTimePeriodFromNow( "4:00" )
    if purgetime > 0:
      tid = PyPepper.addTimer( purgetime, -1, 24*60*3600 )
      self.timercontext[tid] = 'purgemsg'
      timermanager.addTimer( tid, self )

    return True

  def checkout( self ):
    if self.twtaccess == None:
      return

    self.updatestatus( time.strftime( "I'm going offline at %H:%M for maintenance. See you later.", time.localtime() ) )

    self.savestate()
    self.stoptimers()

    self.twtaccess = None

  def onTimer( self, tid ):
    if tid not in self.timercontext:
      return

    if self.timercontext[tid] == 'getmsg':
      self.getmessages()
    elif self.timercontext[tid] == 'purgemsg':
      self.archive = []
      tinstate.updateStatus( constants.ARCHIVE_MESSAGES, True )

  def getmessages( self ):
    if self.twtaccess == None:
      return

    try:
      if self.token == 0:
        newmessages = self.twtaccess.direct_messages()
      else:
        newmessages = self.twtaccess.direct_messages(since_id=self.token)
    except:
      print "Unable to retrieve messages"
      return

    if len( newmessages ) == 0:
      return

    self.token = newmessages[0]['id']

    #store only the basic info
    modmsg = []
    for newmsg in newmessages:
      msg = TextMessage()
      msg.sender = newmsg['sender']['name']
      msg.senderid = newmsg['sender']['id']
      msg.created_at = time.localtime(time.mktime(time.strptime( newmsg['created_at'], "%a %b %d %H:%M:%S +0000 %Y")) - time.altzone)
      msg.text = newmsg['text'].strip()
      msg.id = newmsg['id']
      if msg.text.startswith( '#now#' ):
        msg.text = msg.text[5:]
        PyPepper.say( "Urgent message announcement \pau=1000\ ")
        self.announcemsg( [msg] )
        tinstate.updateStatus( constants.ARCHIVE_MESSAGES, len(self.archive) == 0 )
      elif msg.text.startswith( '#qa#' ):
        if time.mktime(msg.created_at) >= time.mktime(self.startupTime): #ignore all direct requests before startup
          response = tinmind.respond( msg.text[4:] )
          if response:
            self.replymsg( msg.senderid, response )
        continue
      modmsg.append( msg )

    self.messages = modmsg + self.messages
    tinstate.updateStatus( constants.NEW_MESSAGES, len( self.messages ) == 0 )

    if len(modmsg) > 0:
      self.notifmove()

  def sendmsg( self, name, mesg ):
    try:
      tmo = self.twtaccess.direct_messages.new( user=name, text=mesg )
    except:
      print 'Unable to send message to', name

  def replymsg( self, replyid, mesg ):
    try:
      tmo = self.twtaccess.direct_messages.new( user=replyid, text=mesg )
    except:
      print 'Unable to send message to', replyid

  def updatestatus( self, text ):
    try:
      self.twtaccess.statuses.update( status=text )
    except:
      print 'Unable to update my status'

  def announcemsg( self, mesgs ):
    if len( mesgs ) == 0:
      PyPepper.say( "There is no message to be announced" )
      return

    curTime = time.localtime()
    sayString = ""
    for i in mesgs:
      if i.created_at.tm_yday == curTime.tm_yday:
        sayTime = time.strftime( "Message received at %H hour %M minute", i.created_at )
      else:
        sayTime = time.strftime( "Message received at %H hour %M minute on %A %B %d", i.created_at )
      if i.sender in self.useraliases:
        frmWho = "From %s" % self.useraliases[ i.sender ]
      else:
        frmWho = "From %s" % i.sender
      sayString += "{} {} \pau=1000\ {} \pau=2000\ ".format(sayTime, frmWho, i.text)

    sayString += "Message announcement is complete"
    PyPepper.say( sayString )

  def announce( self ):
    self.archive = self.messages + self.archive
    self.messages = []
    self.announcemsg( self.archive )
    if len(self.archive) > 0:
      self.lastoken = self.archive[0].id
      self.savestate()

    tinstate.updateStatus( constants.NEW_MESSAGES, True )
    tinstate.updateStatus( constants.ARCHIVE_MESSAGES, len(self.archive) == 0 )

  def purgearchive( self ):
    if len(self.archive) == 0:
      PyPepper.say( "No archived message to be deleted." )
      return

    self.archive = []
    tinstate.updateStatus( constants.ARCHIVE_MESSAGES, True )
    PyPepper.say( "All archived message have been deleted." )

  def savestate( self ):
    f = None
    try:
      f = open( '/home/nao/recordings/laststate.txt', 'w')
      f.write( "token = %d\n" % self.lastoken)
    except:
      print "Unable to save last states."
    finally:
      f.close()

  def stoptimers( self ):
    for tid in self.timercontext:
      PyPepper.removeTimer( tid )
      timermanager.delTimer( tid )

    self.timercontext = {}

  def notifmove( self ):
    curTime = time.time()
    if self.notifmoveTime and (curTime - self.notifmoveTime) < 15*60:
      return
    self.notifmoveTime = curTime

    PyPepper.setArmStiffness(True,1)
    h='Hi\pau=1000\ You have a new message. \pau=5000\ Please tap my front head to listen to the message.'
    #notifmove = [{'l_elbow_yaw_joint': -0.7655079960823059, 'l_elbow_roll_joint': -1.2087500095367432, 'l_shoulder_pitch_joint': -0.0920819640159607, 'l_shoulder_roll_joint': -0.035323962569236755, 'time_to_reach' : 0.7},{'l_elbow_yaw_joint': -1.020151972770691, 'l_elbow_roll_joint': -0.621228039264679, 'l_shoulder_pitch_joint': -0.9097039699554443, 'l_shoulder_roll_joint': -0.18258796632289886},{'l_elbow_yaw_joint': -1.2962720394134521, 'l_elbow_roll_joint': -0.6350340843200684, 'l_shoulder_pitch_joint': -0.9112379550933838, 'l_shoulder_roll_joint': 0.29755404591560364},{'l_elbow_yaw_joint': -1.020151972770691, 'l_elbow_roll_joint': -0.621228039264679, 'l_shoulder_pitch_joint': -0.9097039699554443, 'l_shoulder_roll_joint': -0.18258796632289886},{'l_elbow_yaw_joint': -1.2962720394134521, 'l_elbow_roll_joint': -0.6350340843200684, 'l_shoulder_pitch_joint': -0.9112379550933838, 'l_shoulder_roll_joint': 0.29755404591560364},{'l_elbow_yaw_joint': -1.020151972770691, 'l_elbow_roll_joint': -0.621228039264679, 'l_shoulder_pitch_joint': -0.9097039699554443, 'l_shoulder_roll_joint': -0.18258796632289886},{'l_elbow_yaw_joint': -1.2962720394134521, 'l_elbow_roll_joint': -0.6350340843200684, 'l_shoulder_pitch_joint': -0.9112379550933838, 'l_shoulder_roll_joint': 0.29755404591560364}, {'l_elbow_yaw_joint': 0.14415404200553894, 'l_elbow_roll_joint': -1.5446163415908813, 'l_shoulder_pitch_joint': 0.22238804399967194, 'l_shoulder_roll_joint': 0.1855720430612564}]
    #restpos = {'l_elbow_yaw_joint': -0.6811379790306091, 'l_elbow_roll_joint': -1.2102841138839722, 'l_shoulder_pitch_joint': 1.4664621353149414, 'l_shoulder_roll_joint': 0.09966804087162018, 'frac_max_speed' : 0.2}
    PyPepper.say(h)
    #PyPepper.moveArmWithJointTrajectory(notifmove)
    #time.sleep(3)
    #PyPepper.moveArmWithJointPos(**restpos)
