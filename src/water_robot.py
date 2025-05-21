#!/usr/bin/env python3

import rospy, time, random
from math import atan2, sqrt, pow, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class EnergyRobot:
    def __init__(self):
        rospy.init_node("energy_cleaner")
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()

        # Esperar por servicios
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/kill')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/set_pen')

        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.energy_used = []

    def update_pose(self, data):
        self.pose = data

    def reset_robot(self):
        self.set_pen(0, 0, 0, 0, 1)
        self.teleport(1.0, 1.0, 0.0)
        self.set_pen(255, 255, 255, 3, 0)
        rospy.sleep(1)

    def spawn_trash(self):
        x = random.uniform(2.0, 10.0)
        y = random.uniform(2.0, 10.0)
        self.spawn(x, y, 0, 'trash')
        return (x, y)

    def move_to(self, x_goal, y_goal):
        vel = Twist()
        rate = rospy.Rate(10)
        k_linear = 0.5
        k_angular = 3.0
        energy = 0.0
        dt = 1.0 / 10.0

        # Esperar que pose se actualice
        while self.pose.x == 0.0 and self.pose.y == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            dx = x_goal - self.pose.x
            dy = y_goal - self.pose.y
            dist = sqrt(dx**2 + dy**2)

            angle_to_goal = atan2(dy, dx)
            angle_error = angle_to_goal - self.pose.theta
            angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalizar ángulo

            if dist < 0.1:
                break

            vel.linear.x = k_linear if abs(angle_error) < 0.1 else 0.0
            vel.angular.z = k_angular * angle_error
            self.vel_pub.publish(vel)

            energy += (vel.linear.x ** 2 + vel.angular.z ** 2) * dt
            rate.sleep()

        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)

        self.energy_used.append(energy)

        try:
            self.kill("trash")
        except rospy.ServiceException as e:
            rospy.logwarn(f"No se pudo eliminar 'trash': {e}")

    def run(self, trials=3):
        for i in range(trials):
            rospy.loginfo(f"--- Intento {i+1} ---")
            self.reset_robot()
            x, y = self.spawn_trash()
            rospy.sleep(1)
            self.move_to(x, y)

        rospy.loginfo("Energía usada por intento: " + str([f"{e:.2f}" for e in self.energy_used]))
        avg = sum(self.energy_used) / len(self.energy_used)
        rospy.loginfo(f"Energía promedio: {avg:.2f}")

if __name__ == "__main__":
    try:
        robot = EnergyRobot()
        rospy.sleep(2)
        robot.run()
    except rospy.ROSInterruptException:
        pass
