#!/usr/bin/env python3.8
# license removed for brevity
import rospy
import sys, select, termios, tty
from beginner_tutorials.msg import WizardCommand

indications = """
Control Your Robot and Game!
---------------------------
Game:
- t : new Turn
- 1. i : say Intro
- 2. w : welcome first
- 3. r : Welcome Second
- 4. b : Start test
- 5. c : Continue Test
- 6. k : Continue Test Second
- 7. s : Start
- f : Say filler
- h : Time is Out
- g : The player guessed right
- v : Repeat game word
Microphones:
- 1/2: -/+ Left speech threshold
- 9/0: -/+ Right speech threshold

CTRL-C to quit
"""

def getKey():
    settings = termios.tcgetattr(sys.stdin)

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('WizardCommand', log_level=rospy.DEBUG)
    pub = rospy.Publisher('wizard_commands', WizardCommand, queue_size=10)

    try:
        print(indications)
        while(1):
            key = getKey()
            if key == 's':
                msg = WizardCommand()
                msg.command = "start_game"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 't':
                msg = WizardCommand()
                msg.command = "new_turn"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'i':
                msg = WizardCommand()
                msg.command = "say_intro"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'c':
                msg = WizardCommand()
                msg.command = "continueOne"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'b':
                msg = WizardCommand()
                msg.command = "starttest"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))    
            if key == 'k':
                msg = WizardCommand()
                msg.command = "continueTwo"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'h':
                msg = WizardCommand()
                msg.command = "timeOut"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'g':
                msg = WizardCommand()
                msg.command = "GissadeRätt"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'w':
                msg = WizardCommand()
                msg.command = "welcome"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'r':
                msg = WizardCommand()
                msg.command = "welcomesecond"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg)) 
            if key == 'f':
                msg = WizardCommand()
                msg.command = "filler"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == 'v':
                msg = WizardCommand()
                msg.command = "repeat_game_word"
                pub.publish(msg)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))    
   

            ## Microphone ##
            # Left
            if key == '1':
                msg = WizardCommand()
                msg.command = "left_threshold_minus"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == '2':
                msg = WizardCommand()
                msg.command = "left_threshold_plus"
                pub.publish(msg)
                print(key)
            # Right
            if key == '9':
                msg = WizardCommand()
                msg.command = "right_threshold_minus"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))
            if key == '0':
                msg = WizardCommand()
                msg.command = "right_threshold_plus"
                pub.publish(msg)
                print(key)
                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))

                rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'wizard_commands', msg))

            else:
                if (key == '\x03'):
                    break
    except Exception as e:
        print(e)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
