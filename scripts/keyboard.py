#!/usr/bin/env python

import rospy
import serial
import string
from std_msgs.msg import String


def KeyControl():
	choice = 0
	pub = rospy.Publisher('keyboard', String, queue_size = 100)
	rospy.init_node('keyboard', anonymous=True)
	rate = rospy.Rate(10)
	is_valid = 0
	print (30 * '-')
	print ("   M A I N - M E N U")
	print (30 * '-')
	print ("0. Start | Stop robot")
	print ("1. Move 4 meters forward")
	print ("2. Move 4 meters back")
	print ("3. Turn to west")
	print ("4. Turn to east")
	print ("5. Speed increse")
	print ("6. Speed decrease")
	print ("7. Define a simple job for testing")
	print ("8. Stop robot from moving")
	print ("9. Test route creating from gps")
	print ("i. Initialize the true north of the robot")
	print ("w. Enter no obstacle mode")
	print ("o. Enter obstacle mode")
	print ("f. Move 10m forward")
	print ("t. 180")
	print ("z. 0")
	print ("b. Enter/Quite from burn mode")
	print ("c. Print config parameters")
	print ("s. Print robot status")
	print ("l. Turn off/on detailed step logs")
	print ("k. Turn lights on/off")
	print ("p. Pause/Resume robot")
	print ("using ctrl + c to exit")
	print (30 * '-')

	while not rospy.is_shutdown():
		choice = raw_input('Enter your choice [0-9] : ')
		if(len(choice) > 1):
			print("You're supposed to enter only one key, try again...")
			continue
		elif (choice == 'i'):
			pub.publish('Init')
		elif choice == '0':
			pub.publish('Switch')
		elif choice == '1':
			pub.publish('Forward')
		elif choice == '2':
			pub.publish('Back')
		elif choice == '3':
			pub.publish('Turn_West')
		elif choice == '4':
			pub.publish('Turn_East')
		elif choice == '5':
			pub.publish('Faster')
		elif choice == '6':
			pub.publish('Slower')
		elif(choice == '7'):
			pub.publish('Demo')
		elif choice == '8':
			pub.publish('Stop')
		elif choice == '9':
			pub.publish('Test')
		elif choice == 'p':
			pub.publish('pause')
		elif choice == 'w':#@yuqing_toggleobstaclemode
			pub.publish('No_obstacle')
		elif choice == 'o':#@yuqing_toggleobstaclemode
			pub.publish('Obstacle')
		elif choice == 'f':#@yuqing_toggleobstaclemode
			pub.publish('10m')
		elif choice == 't':#@yuqing_toggleobstaclemode
			pub.publish('180')
		elif choice == 'z':#@yuqing_toggleobstaclemode
			pub.publish('zero')
		elif choice == 'b':
			pub.publish("Burn")
		elif choice == 'c':
			pub.publish('cfg')
		elif choice == 's':
			pub.publish('status')
		elif choice == 'l':
			pub.publish('log')
		elif choice == 'k':
			pub.publish('light')
		else:
			print ("Invalid/Not defined number. Try again...")
	rate.sleep()

if __name__ == '__main__':
	try:
		KeyControl()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass


