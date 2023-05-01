import math
import pygame

from Robot.robot import Robot
from Graphics.graphics import Graphics
from Sensors.infrared import Ultrasonic
from Robot.TestEnvironment import TestEnvironment

MAP_DIMENSIONS = (600, 1200)

########################

# Load the original PNG image
original_image = pygame.image.load('Robot3.png')

# Create a new Surface with the desired size
new_size = (80, 80)
new_image = pygame.Surface(new_size, pygame.SRCALPHA)

# Scale the original image and blit it onto the new Surface
scaled_image = pygame.transform.smoothscale(original_image, new_size)
new_image.blit(scaled_image, (0, 0))

# Save the new image as a PNG file
pygame.image.save(new_image, 'new.png')

#######################


# the environment graphics
gfx = Graphics(MAP_DIMENSIONS, 'new.png', 'Map3.png')
#env = TestEnvironment()

# the robot
start = (500, 100)
robot = Robot(start, 0.1 * 3779.52, gfx)

# the sensor
sensor_range = 80, math.radians(180)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True
we_play = False

if we_play:
    # simulation Loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                robot.controls(event.key)

        dt = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()
        gfx.map.blit(gfx.map_img, (0, 0))
        robot.kinematics(dt)
        gfx.draw_dust()     # Dust set up
        gfx.draw_robot(robot.x, robot.y, robot.heading)     # Draw Robot
        point_cloud, dust_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
        robot.avoid_obstacles(point_cloud, dust_cloud, dt)
        gfx.draw_sensor_data(point_cloud, [robot.x, robot.y])
        pygame.display.update()
else:
    test_environment = TestEnvironment(10, 10, 120, robot, ultra_sonic, gfx)
    test_environment.run_experiment()
    print("###################")
    print("Training Finished")
    print("###################")

    #t = TestEnvironment(1, 1, 100, robot, ultra_sonic, gfx)
    #t.generation_start_pos = t.best_starts
    print("Best fitness", test_environment.best_fitness)
    print("best genome length", len(test_environment.best_genome))
    print("Best start position", test_environment.best_starts)
    fitness_value = test_environment.evaluate_individual(test_environment.best_genome, test_environment.best_starts)#[[100, 100], [100, 100]])
    print("Best Fitness Value after Evaluation: ", test_environment.best_fitness)



    # self.run_simulator = False
    # t = TestEnvironment(1, 1, 500)
    # t.generation_start_pos = self.best_starts
    # t.evaluate_individual(self.best_genome, [[100, 100], [100, 100]])
    # simulator.run_simulator = False
    # pygame.quit()