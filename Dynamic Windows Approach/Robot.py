"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

import pygame
from pygame.transform import flip, scale, rotate, rotozoom
from sys import exit
BLACK  = (   0,   0,   0)
WHITE  = ( 255, 255, 255)
GREEN  = (   0, 255,   0)
RED    = ( 255,   0,   0)
BLUE   = (   0,   0, 255)

""" Ray-casting  """


cadena=[]
cadena2 =[]
filename = 'Arena1.txt'
with open(filename) as f_obj:
    for line in f_obj:
        line = line.rstrip()
        cadena.append(line.split(','))
for i in cadena:
    i =list(map(lambda x :float(x),i))
    cadena2.append(i)
"""
obstacleList =np.array( [[100.0, 100.0],
                            [200.0, 100.0],
                            [300.0, 100.0],
                            [200.0, 200.0],
                            [300.0, 200.0],
                            [400.0, 200.0],
                            [400.0, 300.0],
                            [500.0, 300.0]])"""

obstacleList =np.array(cadena2)

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    clase de parámetro de simulación
    """

    def __init__(self, pixel):
        # parametros del robot

        self.max_speed = 1.0 * pixel # [m/s]
        self.min_speed = -0.5  * pixel # [m/s]
        self.max_yaw_rate =  ( 40.0 * math.pi / 180.0)  # [rad/s]
        self.max_accel = 0.2  * pixel # [m/ss]
        self.max_delta_yaw_rate = (40.0 * math.pi / 180.0 ) # [rad/ss]
        self.v_resolution = 0.01 * pixel # [m/s]
        self.yaw_rate_resolution = (0.1 * math.pi / 180.0)  # [rad/s]
        self.dt = 0.1  # [s] marca de tiempo para la predicción en movimiento
        self.predict_time = 3.0   # [s]
        self.to_goal_cost_gain = 0.15 * pixel 
        self.speed_cost_gain = 1.0  
        self.obstacle_cost_gain = 100
        self.robot_stuck_flag_cons = 0.001 * pixel # constante para probar que el robot se atasque.
        self.robot_type = RobotType.circle 

        # if robot_type == RobotType.circle
        #También se utiliza para comprobar si se alcanza el objetivo en ambos tipos
        self.robot_radius = 0.5  * pixel # [m] para control de colisión
        self.obs_radius = 0.5 * pixel
        # if robot_type == RobotType.rectangle
        self.robot_width = 1  * pixel# [m] para control de colisión
        self.robot_length = 1.2 * pixel # [m]para control de colisión
        # obstaculos [x(m) y(m), ....]
        """ self.ob = np.array([[100.0, 100.0],
                            [200.0, 100.0],
                            [300.0, 100.0],
                            [200.0, 200.0],
                            [300.0, 200.0],
                            [400.0, 200.0]]) """
        self.ob = "vacio"

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value

""" Robots """
class Robot(pygame.sprite.Sprite):
    """ Estado inicial del vehiculo (parametros iniciales)
         [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)] ,
          x(m) : posición en x
          y(m) : posición en y
          yaw(rad) : posición angular con respecto al eje horizontal 
          v (m/s) : velocidad
          omega (rad/s) : velocidad angular """
    """ posición de la meta  [x(m), y(m)] 
           x(m) : posición en x de la meta
           y(m) : posición de y de la meta"""
    def __init__(self,robot_type, pixel = 10 ):
        super().__init__()
        self.config = Config(pixel)
        self.config.robot_type = robot_type
        self.auto = flip(scale(pygame.image.load("auto.png"), (int(self.config.robot_width), int(self.config.robot_length))), False, True)
      
    def motion(self, x, u):
        x[2] += u[1] * self.config.dt
        x[0] += u[0] * math.cos(x[2]) * self.config.dt
        x[1] += u[0] * math.sin(x[2]) * self.config.dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def dwa_control(self, x):
        dw = self.calc_dynamic_window(x)
        u, trajectory , trayectorias_candidatas= self.calc_control_and_trajectory(x, dw)
        return u, trajectory, trayectorias_candidatas

    def calc_dynamic_window(self,x):
        Vs = [self.config.min_speed, self.config.max_speed,
        -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        Vd = [x[3] - self.config.max_accel * self.config.dt,
        x[3] + self.config.max_accel * self.config.dt,
        x[4] - self.config.max_delta_yaw_rate * self.config.dt,
        x[4] + self.config.max_delta_yaw_rate * self.config.dt]
          
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def predict_trajectory(self, x_init, v, y):
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, y])
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory
    
    def calc_control_and_trajectory(self, x, dw):
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        trayectorias_candidatas = []

        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for y in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y)

                trayectorias_candidatas.append(trajectory)
                
                to_goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

                final_cost = to_goal_cost + speed_cost + ob_cost
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.config.robot_stuck_flag_cons \
                            and abs(x[3]) < self.config.robot_stuck_flag_cons:
                        best_u[1] = -self.config.max_delta_yaw_rate
        
        return best_u, best_trajectory, trayectorias_candidatas
    
    def calc_obstacle_cost(self, trajectory):
        if( self.config.ob =="vacio" ): return 0
        ox = self.config.ob[:, 0]
        oy = self.config.ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)
        if self.config.robot_type == RobotType.rectangle:
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = self.config.ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            local_ob = np.array([local_ob @ x for x in rot])
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            """ Checking si alguno llega a chocar al robot """
            upper_check = local_ob[:, 0] + self.config.obs_radius <= self.config.robot_length / 2
            right_check = local_ob[:, 1] + self.config.obs_radius <= self.config.robot_width / 2
            bottom_check = local_ob[:, 0] + self.config.obs_radius >= -self.config.robot_length / 2
            left_check = local_ob[:, 1] + self.config.obs_radius >= -self.config.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                            np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        elif self.config.robot_type == RobotType.circle:
            if np.array(r + self.config.obs_radius<= self.config.robot_radius).any():
                return float("Inf")
    
        min_r = np.min(r)
        return 1.0 / min_r  # OK

    def calc_to_goal_cost(self, trajectory):
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost
    
    def add_ob(self,o):
        if(self.config.ob=="vacio"):
             self.config.ob = np.array([o])
        else:
            self.config.ob=np.vstack((self.config.ob, o ))

    def reset_ob(self):
        self.config.ob = "vacio"
    
    def run(self, x , goal):
         """  Bucle de avance , mientras el objetivo no alcance la meta. """
         self.inicio = x
         self.goal = goal 
         trajectory = np.array(x)
         while True:
            u, predicted_trajectory, trayectorias_candidatas = self.dwa_control(x)
            x = self.motion(x, u)  # simulación del movimiento del robot
            print(x)
            trajectory = np.vstack((trajectory, x))  # historial de los estados.
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= self.config.robot_radius:
                print("Goal!!")
                break

""" Encontrar obstaculos """
def encontrar_obstaculos(Robot,x,y,large):
    ox = obstacleList[: , 0]
    oy = obstacleList[:, 1]
    dx = x - ox[:, None]
    dy = y - oy[:, None]
    r = np.hypot(dx, dy)
    print(r)
    for ind,r in zip(range(r.shape[0]),r):
        if(r<large):
            Robot.add_ob(obstacleList[ind])
    print("encontrando obstaculos:",Robot.config.ob)

""" Dibujos """    

def dibuja_trayectorias(x,trayectorias_candidatas, screen):
    for trayectoria in trayectorias_candidatas:
        pygame.draw.line(screen,BLACK, (x[0],x[1]), (trayectoria[-1,0],trayectoria[-1,1]), width=1)

def dibuja_trayectoria(x,predicted_trajectory,screen):
    pygame.draw.line(screen,RED ,(x[0],x[1]), (predicted_trajectory[-1,0],predicted_trajectory[-1,1]),2)

def dibuja_obstaculos(ob, screen,tam):
    for i in range(ob.shape[0]):
        obstaculo=scale(pygame.image.load("arbusto.png").convert_alpha(), (int(tam), int(tam)))
        screen.blit(obstaculo, obstaculo.get_rect( center=(ob[i][0], ob[i][1])))     


def dibuja_meta(goal,screen,tam):
    meta=scale(pygame.image.load("inicio-fin.png").convert_alpha(), (int(tam), int(tam)))
    screen.blit(meta, meta.get_rect(center=(goal[0],goal[1])))



""" Simulación sin dependencia del robot """
def simulacion(Robot,x,goal):
    trajectory = np.array(x)
    while True:
        Robot.goal = goal
        u, predicted_trajectory, trayectorias_candidatas = Robot.dwa_control(x)
        x = Robot.motion(x, u)  # simulate robot
        print(x)
        trajectory = np.vstack((trajectory, x))  # store state history
        dist_to_goal = math.hypot(x[0] - Robot.goal[0], x[1] - Robot.goal[1])
        if dist_to_goal <= Robot.config.robot_radius:
            print("Goal!!")
            break



def find_rotation_degrees(radians):
    degree = (radians * (180 / 3.1415) * -1) + 90
    return degree

# método para determinar cuántos radianes rotar la imagen del jugador
def find_rotation_radians(mouse_X, mouse_Y, center_X, center_Y):
    radians = math.atan2(mouse_Y - center_Y, mouse_X - center_X)
    return radians


""" Función grafica """
def SIMULACION(Robot, x , goal):
    Robot.goal = goal
    trajectory = np.array(x)
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((800, 650))
    pygame.display.set_caption("Dynamic window approach")

    #Definiendo x, y para el movimiento con teclado
    x_change=0
    y_change=0


    movement = True
    while True:
        #Acciones para mover al jugador y salir de la simulación
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN:
                key = pygame.key.get_pressed()
                if key[pygame.K_LEFT] or key[pygame.K_a]:
                    x_change = - 3
                if key[pygame.K_RIGHT] or key[pygame.K_d]:
                    x_change = + 3
                if key[pygame.K_UP] or key[pygame.K_w]:
                    y_change = - 3
                if key[pygame.K_DOWN] or key[pygame.K_s]:
                    y_change = + 3
                if key[pygame.K_ESCAPE]:
                    pygame.quit()
            if event.type == pygame.KEYUP:
                if key[pygame.K_LEFT] or key[pygame.K_RIGHT] or key[pygame.K_a] or key[pygame.K_d]:
                    x_change = 0
                if key[pygame.K_UP] or key[pygame.K_DOWN] or key[pygame.K_s] or key[pygame.K_w]:
                    y_change = 0

        #Variando x,y segun el teclado
        x[0] += x_change
        x[1] += y_change

        if movement:
            encontrar_obstaculos(Robot,x[0],x[1],100)
            u, predicted_trajectory, trayectorias_candidatas = Robot.dwa_control(x)
            x = Robot.motion(x, u)  # nuevo estado del robot
            print(x)
            trajectory = np.vstack((trajectory, x))  # guardando estado
            dist_to_goal = math.hypot(x[0] - Robot.goal[0], x[1] - Robot.goal[1])
            if dist_to_goal <= Robot.config.robot_radius:
                print("Goal!!")
                movement = False
            ##############--------ZONA DE DIBUJO------##############
        screen.fill('White') # pintando la ventana
        # screen.blit(rotate(Robot.auto, x[3]),rotate(Robot.auto, x[3]).get_rect( center=(int(x[0]), int(x[1])))) # dibunjando el robot
        dibuja_meta(goal, screen, Robot.config.obs_radius) # dibujando meta
        dibuja_obstaculos(obstacleList, screen, Robot.config.obs_radius) # dibujando obstaculos
        dibuja_trayectorias(x,trayectorias_candidatas, screen)
        dibuja_trayectoria(x,predicted_trajectory,screen)
        #obteniendo variables para la rotación
        radians = find_rotation_radians(predicted_trajectory[-1,0], predicted_trajectory[-1,1], x[0], x[1])
        degrees = find_rotation_degrees(radians)
        playerImgRotated = pygame.transform.rotate(Robot.auto, degrees)
        #Dibujando el player
        screen.blit(playerImgRotated,(x[0]-10,x[1]-10))
        #######################################################
        Robot.reset_ob()
        pygame.display.update()
        clock.tick(60)


""" Función principal """
def main():
    Robot1 =  Robot(RobotType.circle,20)
    x = np.array([10.0, 10.0, math.pi / 8.0, 0.0, 0.0])
    goal = np.array([500.0, 500.0])
    #Robot1.run(x,goal)
    #simulacion(Robot1,x,goal)
    SIMULACION(Robot1,x,goal)

if __name__ == '__main__':
    main()