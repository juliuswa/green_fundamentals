#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/k : increase/decrease x velocity
j/l : increase/decrease y velocity
u/o : increase/decrease z velocity

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'i':(1.,0.),
    'k':(-1.,0.),
    'j':(0.,1.),
    'l':(0.,-1.),
    'm':(0.,0.)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(v, w):
    return "currently:\tv: %s\tw: %s" % (v,w)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    v = 0
    w = 0
    
    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                v += moveBindings[key][0]
                w += moveBindings[key][1]
            else:
                v = 0.
                w = 0.
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
            print(twist)

            print(vels(v, w))
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
