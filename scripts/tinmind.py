import PyPepper
import re
import tininfo
import time
import cPickle

loadedSongs = {}

def respond( question ):
  q = question.strip().lower()

  if 'ip' in q and 'addr' in q:
    return "My IP Address is %s." % PyPepper.getMyIPAddress()
  
  if 'battery' in q and ('how much' in q or 'status' in q):
    (batpc, isplug, ischarging) = PyPepper.getBatteryStatus()
    return "I'm currently %s with %d percent battery power and %s." % (isplug, batpc, ischarging)
  
  m = re.match(r"\W*(play|sing) (?P<song_name>\w+)", q )
  if m:
    try:
      fname = '/home/nao/recordings/%s.mp3' % m.group('song_name')
      if fname not in loadedSongs:
        loadedSongs[fname] = PyPepper.loadAudioFile( fname )

      PyPepper.say( 'Let us play {}!'.format( m.group('song_name') ) )
      if m.group('song_name') == "happy":
        playDanceMove( loadedSongs[fname] )
      else:
        PyPepper.playAudioID( loadedSongs[fname] )
      return "Playing the song."

    except:
      return "I couldn't find song '%s'." % m.group('song_name') 
    
  return "I don't understand your request." 

def eventAction( event ):
  text = event.text.strip().lower()
  m = re.match(r"(?P<name>\w+).*? birthday", text )

  if m:
    bp = m.group('name')
    PyPepper.say( "%s \pau=500\ are you there? \pau=700\ It's your birthday!" % bp, 1.0, True )
    try:
      if 'birthday' not in loadedSongs:
        loadedSongs['birthday'] = PyPepper.loadAudioFile( '/home/nao/recordings/birthday.mp3' )
      PyPepper.playAudioID( loadedSongs['birthday'], True )
    except:
      pass

    PyPepper.say( "Hurray Hurray. Happy birthday, %s!" % bp )
    event.nofnotices = 0
    return "Happy birthday to %s!" % bp

  if event.where:
    sayString = "%s is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, event.where)
    textString = "'%s' is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, event.where)
  else:
    sayString = "%s is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, tininfo.TiNLocation)
    textString = "'%s' is about to start in %d minutes in %s." % (event.text,event.nofnotices*5, tininfo.TiNLocation)
      
  PyPepper.say( sayString )
  event.nofnotices = event.nofnotices - 1

  return textString

def playDanceMove( songid ):
  f = open('/home/nao/naoqi/lib/python/moves.cpl', 'r')
  (larm,rarm)=cPickle.load( f )
  f.close()
  PyPepper.setBodyStiffness(1)
  PyPepper.playAudioID(songid)
  PyPepper.moveArmWithJointTrajectory(larm, True)
  PyPepper.moveArmWithJointTrajectory(rarm)
  PyPepper.stopAllAudio()
  time.sleep( 4 )
  PyPepper.crouch()
