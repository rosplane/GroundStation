#!/usr/bin/env python

import rospy, math, random
from fcu_common.msg import GPS, Obstacles, Obstacle

def talker():

    stationaryObstacles = [
        (40.2518,-111.6493, 100, 100),
        (40.2518,-111.6520, 100,  25)
        ]

    movingObstacles = [
        (40.2498,-111.6453, 70, 100),
        (40.2478,-111.6453, 50, 120)
        ]

    obsPub = rospy.Publisher("/obstacles", Obstacles, queue_size=50)
    # gpsPub = rospy.Publisher("/gps/data", GPS, queue_size=50)
    index = 0
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():

        index += 1
        curVal = (index%628)/100.0

        # # GPS
        # gps = GPS()
        # gps.header.seq = index
        # gps.fix = True
        # gps.NumSat = random.randint(1,7)
        # gps.latitude = 40.2518 + math.sin(curVal)/100.0
        # gps.longitude = -111.6493 + math.sin(curVal)/100.0
        # gps.altitude = math.fabs(200*math.sin(curVal))
        # gps.speed = 5
        # gps.ground_course = 3
        # gps.covariance = 1
        # rospy.loginfo(gps)
        # gpsPub.publish(gps)


        # Obstacles
        stationary_obstacles = []
        moving_obstacles = []
        
        for (lat, lon, radius, height) in stationaryObstacles:
            obstacle = Obstacle()
            obstacle.latitude = lat
            obstacle.longitude = lon
            obstacle.radius = radius
            obstacle.height = height
            stationary_obstacles.append(obstacle)

        for (lat, lon, radius, height) in movingObstacles:
            obstacle = Obstacle()
            obstacle.latitude = lat + math.sin(curVal)/200.0
            obstacle.longitude = lon + math.sin(curVal)/200.0
            obstacle.radius = radius
            obstacle.height = height + math.fabs(200*math.sin(curVal))
            moving_obstacles.append(obstacle)

        obstacles = Obstacles()
        obstacles.stationary_obstacles = stationary_obstacles
        obstacles.moving_obstacles = moving_obstacles
        rospy.loginfo(obstacles)
        obsPub.publish(obstacles)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
