import random

import pygame
import math

class Graphics:
    def __init__(self, dimensions, robot_img_path, mao_img_path):
        pygame.init()
        #COLORS
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)

        # load imgs
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(mao_img_path)
        # dimensions
        self.height, self.width = dimensions

        # window settings
        pygame.display.set_caption("AI AVENGERS_Robot Simulator")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.dust_coords = []
        for i in range(100): # Create random dust
            x = random.randint(50, 1100)
            y1 = random.randint(50, 250)
            y2 = random.randint(350, 500)
            self.dust_coords.append((x, y1))
            self.dust_coords.append((x, y2))

        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud, robot_pose):
        for point in point_cloud:
            #pygame.draw.circle(self.map, self.red, point, 3, 0)
            # draw point cloud
            pygame.draw.circle(self.map, self.red, point, 3, 0)

            # draw line from robot to point
            pygame.draw.line(self.map, self.black, robot_pose, point, 1)

            # calculate distance between robot and point
            dx = robot_pose[0] - point[0]
            dy = robot_pose[1] - point[1]
            distance = int(math.sqrt(dx * dx + dy * dy))

            # draw text showing distance
            font = pygame.font.Font('freesansbold.ttf', 12)
            text = font.render(str(distance), True, self.red)
            text_rect = text.get_rect(center=((robot_pose[0] + point[0]) // 2, (robot_pose[1] + point[1]) // 2))
            self.map.blit(text, text_rect)

    def draw_dust(self):
        for coordinate in self.dust_coords:
            pygame.draw.rect(self.map, self.blue, (coordinate[0], coordinate[1], 30,30),0)
        return self.dust_coords

    def remove_dust(self, dust_coor):
        # print('Cloud: ', self.dust_coords)
        # print('sensor: ', dust_coor)
        threshold = 35
        for coord in self.dust_coords:
            for coord2 in dust_coor:
                dist = abs(coord[0] - coord2[0]) + abs(coord[1] - coord2[1])
                if dist <= threshold:
                    # print('remove: ',coord)
                    # print('original: ',self.dust_coords)
                    self.dust_coords.remove(coord)
                    break


