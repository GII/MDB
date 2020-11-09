#!/usr/bin/env python3
import rospy
import time
from mdb_common.msg import PNodeMsg

if __name__ == '__main__':
    rospy.loginfo('FUNKA')
    rospy.init_node('mdb_view_talker', anonymous=True)
    try:
        print('Sending data')
        for i in range(1,50):
            pub = rospy.Publisher('/mdb/ltm/p_node', PNodeMsg)
            r = rospy.Rate(1)
            
            msg = PNodeMsg()

            msg.command = 'new'
            msg.id = str(i)
            msg.execute_service = 'a' 
            msg.get_service = 'string'
            msg.class_name = 'PNode'
            msg.language = 'Python'

            pub.publish(msg)
            time.sleep(10)
    except rospy.ROSInterruptException: pass
