import pygame
import math
import numpy as np
# from Graphics.graphics import Graphics


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, width, gfx):
        self.gfx = gfx
        self.m2p = 3779.52  # Meter to pixels
        self.w = 90

        self.x = startpos[0]    # x coordinate of the robot
        self.y = startpos[1]    # y coordinate of the robot
        self.heading = 0

        self.vl = 0
        self.vr = 0
        self.last_x = self.x
        self.last_y = self.y
        self.last_vl = 0
        self.last_vr = 0
        self.acc = 0.005

        self.maxspeed = 0.1 * self.m2p
        self.minspeed = self.acc * self.m2p
        # self.reset_speed = False

        self.distances = []
        self.min_obs_dist = 80
        self.min_dust_dist = 35
        self.count_down = 5
        self.closest_obs_size = 0
        self.obs_dist = 0
        self.slide_bool = True

        # TestEnviornment Variables
        self.score = 0

    def avoid_obstacles(self, point_cloud, dust_cloud, dt):
        self.distances = []
        closest_obs = None
        dist = np.inf
        self.closest_obs_size = 0

        if len(dust_cloud) > 0:
            for coords in dust_cloud:
                distance_to_dust = distance([self.x, self.y], coords)
                # print('dist: ',distance_to_dust)

                if distance_to_dust < self.min_dust_dist:
                    self.gfx.remove_dust(dust_cloud)

        # closest_points = []
        if len(point_cloud) > 1:
            for point in point_cloud:   # Figuring out closest obstacle
                #i need to mayberememberthe indexofthepointinthelist?
                # print(f'distance: point{point}', distance([self.x, self.y], point))
                distance_to_obs = distance([self.x, self.y], point)
                self.distances.append(distance_to_obs)

                if dist > distance_to_obs:
                    dist = distance_to_obs

                    # print('distance: ',dist)
                    closest_obs = (point, dist)
                    self.obs_dist = closest_obs[1]

                    if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                        self.closest_obs_size += 1
                        # closest_points.append(closest_obs[0])


            if self.closest_obs_size <= 1:
                self.last_x = self.x
                self.last_y = self.y
            if self.closest_obs_size == 0:
                self.last_vl = self.vl
                self.last_vr = self.vr
                # print('Saving coords: ', (self.last_x, self.last_y))

            # print((closest_points[0][0], closest_points[1][0]) , ' vs ', ( closest_points[0][1], closest_points[1][1]))
            if self.closest_obs_size >= 2:
                self.slide_bool = False
                # print('FALSE SLIDE')

            if closest_obs[1] < self.min_obs_dist:
                # if both sensors are hitting obstacle with same X coordinate
                # or same Y coordinate that means its the same wall !!!!!


                # print('SIZE OF CLOSEST POINTS : ', len(closest_points))
                # print(closest_obs)
                # self.count_down -= dt
                # print('STOP')

                #self.move_backward()
                # self.stop_robot()


                # self.reset_speed = False
                self.stop_robot()
                # print('CHECK ', (self.x, self.y), ' to ', (self.last_x, self.last_y), ' obs size: ', self.closest_obs_size)
                if self.closest_obs_size > 1 and (self.x != self.last_x or self.y != self.last_y):
                    # if len(closest_points) >= 2 and (closest_points[0][0] == closest_points[1][0] or closest_points[0][1] == closest_points[1][1]):
                    # print('reseting from ', (self.x, self.y), ' to ', (self.last_x, self.last_y))
                    self.x = self.last_x
                    self.y = self.last_y

                # IF MORE THAN 1 OBSTACLE DONT SLIDE
                # !!!!!!!!!!!! OR IF PREVIOUS POSITION WAS HITTING 2 WALLS
                hor_wall = True
                if self.closest_obs_size == 1:
                    if np.abs(self.x - closest_obs[0][0]) > np.abs(self.y - closest_obs[0][1]):
                        # print('LEFT RIGHT WALL')
                        hor_wall = True
                    if np.abs(self.x - closest_obs[0][0]) <= np.abs(self.y - closest_obs[0][1]):
                        # print('UP DOWN WALL')
                        hor_wall = False
                    if self.slide_bool:
                        # print('SLIDEEEEEEEEEEEEEEEEEEEEEE')
                        self.slide(hor_wall)
                # print('obst size: ', self.closest_obs_size)
                # if self.reset_speed is False and self.closest_obs_size == 0:
                #     self.reset_speed = True

            else:
                # if self.reset_speed:
                #     print('reset speed')
                #     self.vl = self.last_vl
                #     self.vr = self.last_vr
                # pass
                self.count_down = 5
                #self.move_forward()

    def slide(self, left_right_walls):
        # self.last_x = self.x
        # self.last_y = self.y
        # if wall on the right and angle + => self.y--
        # if wall on the right and angle - => self.y++
        # if wall on the top and angle + => self.x++
        # if wall on the top and angle - => self.x--
        # print('heading: ', math.degrees(self.heading))
        # print('vr: ', self.vr)
        # print('vl: ', self.vl)
        deg = math.degrees(self.heading)
        # print(self.last_vr + self.last_vl)
        if self.last_vr + self.last_vl >= 0:
            # print("FORWARD")
            if left_right_walls:
                if (0 < deg < 180) or (deg < -180):
                        if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                            self.y += 0.5
                        else:
                            self.y -= 0.5
                        # print('up')
                if (deg > 180) or (0 > deg > -180):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.y -= 0.5
                    else:
                        self.y += 0.5
                    # print('down')
            else:
                # print('-----------------')
                if (-90 < deg < 90) or (deg < -270) or (deg > 270):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.x -= 0.5
                    else:
                        self.x += 0.5
                        # print('right')
                if (90 < deg < 270) or (-90 < deg < -270):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.x += 0.5
                    else:
                        self.x -= 0.5
                    # print('left')
        else:
            # print("BACKWARD")
            if left_right_walls:
                if (0 < deg < 180) or (deg < -180):
                        if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                            self.y -= 0.5
                        else:
                            self.y += 0.5
                        # print('up')
                if (deg > 180) or (0 > deg > -180):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.y += 0.5
                    else:
                        self.y -= 0.5
                    # print('down')
            else:
                # print('-----------------')
                if (-90 < deg < 90) or (deg < -270) or (deg > 270):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.x += 0.5
                    else:
                        self.x -= 0.5
                        # print('right')
                if (90 < deg < 270) or (-90 < deg < -270):
                    if (self.vr - self.vl < 0) or (self.vl - self.vr < 0):
                        self.x -= 0.5
                    else:
                        self.x += 0.5
                    # print('left')


    def stop_robot(self):
        self.vr = 0
        self.vl = 0

    def move_backward(self):
        self.vr = - self.minspeed
        self.vl = - self.minspeed

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt
        # self.heading_new = abs(self.heading - 360)

        # self.last_x = self.x
        # self.last_y = self.y

        if self.heading >= 2 * math.pi or self.heading <= -2 * math.pi:
           self.heading = 0
        self.heading_new = abs(math.degrees(self.heading) - 360)
        #self.heading_new = 360 - self.heading
        # print('heading new: ', self.heading_new)
        # print('heading: ', math.degrees(self.heading))
        # self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        # self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

    def controls(self, key):
        # print('TRUE SLIDE')
        self.slide_bool = True
        # W: positive increment of left wheel motor speed
        if key == pygame.K_w:
            # print('W pressed')
            self.vl += self.acc * self.m2p
        # O: positive increment of right wheel motor speed
        elif key == pygame.K_o:
            self.vr += self.acc * self.m2p
        # L: negative increment of right wheel motor speed
        elif key == pygame.K_l:
            self.vr -= self.acc * self.m2p
        # X: both motor speeds are zero
        elif key == pygame.K_x:
            self.stop_robot()
        # S: negative increment of left wheel motor speed
        elif key == pygame.K_s:
            self.vl -= self.acc * self.m2p
        # T: positive increment of both wheels’ motor speed
        elif key == pygame.K_t:
            if self.obs_dist > 50:
                self.vr += self.acc * self.m2p
                self.vl += self.acc * self.m2p
        # G: negative increment of both wheels’ motor speed
        elif key == pygame.K_g:
            self.vr -= self.acc * self.m2p
            self.vl -= self.acc * self.m2p
        # print('Left Motor speed: ', self.vl)
        # print('Right Motor speed: ', self.vr)

    def update_score(self):
        v = (abs(self.vl) + abs(self.vr)) / 2 / self.maxspeed
        d_v = 1 - abs(self.vl - self.vr) / (abs(self.minspeed) + self.maxspeed)

        # 100 is the sensor_range
        i = abs(self.min_obs_dist) / 100
        score_this_frame = v * math.sqrt(d_v) * i
        self.score += score_this_frame
        return self.score
    # End bot control methods

    def set_motor_speed_percentage(self, l_motor, r_motor):
        m_speed = self.maxspeed
        self.vr = r_motor * m_speed
        self.vl = l_motor * m_speed
