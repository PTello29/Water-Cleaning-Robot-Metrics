#!/usr/bin/env python3
import rospy
import random
import time
from math import atan2, sqrt, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class MiRobot:
    def __init__(self):
        rospy.init_node('robot_basura')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.leer_pos)
        self.pos = Pose()
        self.rate = rospy.Rate(10)

        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/clear')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.wait_for_service('/kill')

        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.clear = rospy.ServiceProxy('/clear', Empty)
        self.tp = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.lapiz = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

        self.lista = []
        self.meta = ['red', 'green']

    def leer_pos(self, data):
        self.pos = data

    def reset(self):
        self.lapiz(0, 0, 0, 0, 1)
        self.tp(10.5, 1.0, atan2(10 - 1, 1 - 10.5))
        self.lapiz(255, 255, 255, 3, 0)

    def color_a_rgb(self, c):
        colores = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'orange': (255, 165, 0),
            'purple': (160, 32, 240),
            'cyan': (0, 255, 255)
        }
        return colores.get(c, (255, 255, 255))

    def generar(self):
        self.clear()
        self.lista = []

        fijos = ['red', 'green']
        otros = ['blue', 'yellow', 'orange', 'purple', 'cyan']
        random.shuffle(otros)
        todos = fijos + otros[:3]

        coords = []
        for i, c in enumerate(todos):
            while True:
                x = round(random.uniform(2, 9), 2)
                y = round(random.uniform(2, 9), 2)
                bien = True
                for a, b in coords:
                    if sqrt((a - x)**2 + (b - y)**2) < 2.0:
                        bien = False
                        break
                if bien:
                    coords.append((x, y))
                    break

            nombre = f"t{i}"
            r, g, b = self.color_a_rgb(c)
            self.spawn(x, y, 0.0, nombre)
            rospy.wait_for_service(f'/{nombre}/set_pen')
            setpen = rospy.ServiceProxy(f'/{nombre}/set_pen', SetPen)
            setpen(r, g, b, 3, 0)

            vel = rospy.Publisher(f'/{nombre}/cmd_vel', Twist, queue_size=10)
            rospy.sleep(0.2)
            mov = Twist()
            mov.linear.x = 3.0
            mov.angular.z = 8.0
            for _ in range(180):
                vel.publish(mov)
                rospy.sleep(0.01)
            vel.publish(Twist())
            try:
                self.kill(nombre)
            except:
                pass

            self.lista.append({'x': x, 'y': y, 'color': c})

    def en_rango(self, x, y):
        dx = x - self.pos.x
        dy = y - self.pos.y
        ang = atan2(dy, dx)
        rel = (ang - self.pos.theta + pi) % (2*pi) - pi
        return abs(rel) <= pi / 3

    def buscar(self):
        cosas = [l for l in self.lista if l['color'] in self.meta and self.en_rango(l['x'], l['y'])]
        if cosas:
            return min(cosas, key=lambda z: sqrt((z['x'] - self.pos.x)**2 + (z['y'] - self.pos.y)**2))
        return None

    def mover(self, x, y):
        v = Twist()
        while not rospy.is_shutdown():
            dx = x - self.pos.x
            dy = y - self.pos.y
            d = sqrt(dx**2 + dy**2)
            ang = atan2(dy, dx)
            ea = (ang - self.pos.theta + pi) % (2 * pi) - pi

            if d < 0.05:
                break

            v.linear.x = 0.5 if abs(ea) < 0.1 else 0.0
            v.angular.z = 3.0 * ea
            self.pub.publish(v)
            self.rate.sleep()
        self.pub.publish(Twist())

    def correr(self):
        self.reset()
        self.generar()

        while not rospy.is_shutdown():
            objetivo = self.buscar()
            if objetivo:
                print(f"\nBasura {objetivo['color']} detectada")
                start = time.perf_counter()
                self.mover(objetivo['x'], objetivo['y'])
                end = time.perf_counter()

                self.lapiz(255, 255, 255, 5, 0)
                self.pub.publish(Twist())
                rospy.sleep(0.1)

                print(f"Listo, basura {objetivo['color']} recogida en {round(end - start, 2)}s")
                self.lista.remove(objetivo)
            else:
                print("No veo basura verde o roja. Esperando...")
                self.pub.publish(Twist())
                rospy.sleep(3)

                if self.buscar():
                    continue

                r1 = input("¿Pongo más basura? (s/n): ").strip().lower()
                r2 = input("¿Regreso al inicio? (s/n): ").strip().lower()

                if r1 == 's':
                    self.generar()
                if r2 == 's':
                    self.reset()

                if r1 != 's' and r2 != 's':
                    print("Nada más que hacer. Adiós.")
                    break

if __name__ == '__main__':
    try:
        bot = MiRobot()
        bot.correr()
    except rospy.ROSInterruptException:
        pass

