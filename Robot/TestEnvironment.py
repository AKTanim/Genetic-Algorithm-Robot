import math
import pickle
import random

import pandas as pd
import pygame

import numpy as np

from EvolutionaryAlgorithm.EvolutionaryAlgorithm import EvolutionaryAlgorithm
from NeuralNetwork2.NeuralNetwork import NeuralNetwork2
from Robot.robot import Robot


class TestEnvironment:
    def __init__(self, num_individuals, num_generations, testing_cycles, robot, ultra_sonic, gfx):
        # Initializing Classes
        self.robot = robot
        self.ultra_sonic = ultra_sonic
        self.gfx = gfx

        # Experiement parameters
        self.num_experiments_conducted = 0
        # Max distance detected by bot sensors
        self.sensor_length = 100    # sensor_range in main file
        self.dt = 0.005 # dt

        # Cycles after which the recurrent input gets updated
        self.delta_cycle_nn = 20

        # Genetic Algorithm Setup
        self.NN = NeuralNetwork2([6, 4, 2], 5, 12, self.delta_cycle_nn)
        self.mutation_rate = 0.3
        self.ev_alg = EvolutionaryAlgorithm(num_individuals, self.NN.create_genomes(num_individuals),
                                            self.mutation_rate)
        self.genomes = self.ev_alg.genomes

        self.num_individuals = num_individuals
        self.num_generations = num_generations
        self.testing_cycles = testing_cycles
        self.num_start_pos = 1

        # Test Environment
        self.win_size = [600, 1200]  # MAP_DIMENSION in
        self.clock = pygame.time.Clock()

        # Test Subject / Bot & Test Maze
        # self.obstacle_list = self.reconstruct_obstacles("maze_files/maze3.pickle")
        # self.obstacle_list2 = self.reconstruct_obstacles("maze_files/maze2.pickle")
        # self.test_mazes = [self.obstacle_list, self.obstacle_list2]

        # self.pSprite = PlayerSprite(30, differential_bot([100, 100], 30, self.obstacle_list, self.sensor_length))
        # self.starting_pos = self.create_random_start()
        self.starting_pos = (self.robot.x, self.robot.y)   # for now we don't use the random starting position
        self.change_pos_every = 2

        # Stats & Data
        self.saved_evals = []
        self.saved_pos = []
        self.saved_genomes = []
        self.best_fitness = 0
        self.best_genome = []
        self.best_starts = []

        # visualize every xth experiement
        self.visualize_every = 5

        # Settings
        print("""####
                Number of generations: """ + str(self.num_generations) + """
                Number of individuals: """ + str(self.num_individuals) + """
                Number of cycles per experiment: """ + str(self.testing_cycles) + """
                New starting positions every x generations: """ + str(self.change_pos_every) + """
                Visualize every xth experiment: """ + str(self.visualize_every) + """
                """)

        # def reconstruct_obstacles(self, filename): # called in above obstacle part
        #     pickle_in = open(filename, "rb")
        #     obstacle_data = pickle.load(pickle_in)
        #     obstacle_list = pygame.sprite.Group()
        #     for obstacle in obstacle_data:
        #         Obstacle(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle_list)
        #
        #     return obstacle_list

        # def create_random_start(self):
        #     starting_positions = []
        #     for obstacle_list in self.test_mazes:
        #         found_pos = False
        #         for _ in range(self.num_start_pos):
        #             while not found_pos:
        #                 # Introduce a padding so the bot does not spawn at the edge
        #                 border_with = 60
        #                 x = np.random.randint(border_with, self.win_size[0] - border_with)
        #                 y = np.random.randint(border_with, self.win_size[1] - border_with)
        #
        #                 self.pSprite.bot.obstacle_list = obstacle_list
        #                 self.pSprite.bot.pos = [x, y]
        #
        #                 # Determine if position is suitable by checking distance sensors
        #                 if min(self.pSprite.bot.get_distances()) > 10:
        #                     random_theta = np.random.uniform(0, 2 * math.pi)
        #                     starting_positions.append([x, y, random_theta])
        #                     found_pos = True
        #     return starting_positions

    def run_experiment(self):
        self.num_experiments_conducted = 0
        for generation_num in range(self.num_generations):
            evals = []
            # if len(self.gfx.dust_coords) == 0:
            #     for i in range(100):  # Create random dust
            #         x = random.randint(50, 1100)
            #         y1 = random.randint(50, 250)
            #         y2 = random.randint(350, 500)
            #         self.gfx.dust_coords.append((x, y1))
            #         self.gfx.dust_coords.append((x, y2))
            #     #self.gfx.dust_coords = dust_coords
            print("####")
            print("Generation num: " + str(generation_num))
            print("####")
            # define new starting positions each generation
            if generation_num % self.change_pos_every == 0:
                # self.generation_start_pos = self.create_random_start()
                self.generation_start_pos = self.starting_pos
            #### Evaluation ####
            for i in range(self.num_individuals):
                print("individual num: " + str(i))

                genome = self.genomes[i]
                fittnes_value = self.evaluate_individual(genome, self.starting_pos)

                evals.append(fittnes_value)
                self.num_experiments_conducted += 1

                # Save data for each generation
            self.saved_evals.extend(evals)
            self.saved_pos.append(self.starting_pos)
            self.saved_genomes.extend(self.ev_alg.genomes)
            ####################

            # Apply evolutionary algorithm
            self.genomes = self.ev_alg.evolve(evals)

        # Save data to excel
        # self.save_data()

    def evaluate_individual(self, genome, starting_positions):
        # self.pSprite.bot.score = 0

        i = 0
        total_dust = 0
        num_collisions = 0
        # for obstacle_list in self.test_mazes:
        #
        #     # Give obstacles to bot
        #     self.pSprite.bot.obstacle_list = obstacle_list

        for test_num in range(2):
            # Let the bot roam randomly
            # self.pSprite.bot.pos = self.generation_start_pos[i][0:2]
            # self.robot.x = self.robot.x
            # self.robot.y = self.robot.y
            self.robot.x = random.randint(100, 1000)
            random_y = [random.randint(100, 200), random.randint(400, 500)]
            self.robot.y = random_y[random.randint(0,1)]
            # set sprite center to bot pose so dust does not collide with old position -> would be chnaged on move
            # self.pSprite.rect.center = self.generation_start_pos[i][0:2]
            # self.pSprite.bot.theta = self.generation_start_pos[i][2]
            self.robot.vl = 0
            self.robot.vr = 0

            # dust
            starting_dust_am = len(self.gfx.dust_coords)
            # starting_dust_am = 1

            cycle_num = 0
            while cycle_num < self.testing_cycles:
                # Calc NN
                # all sensor distances (might be shorter if obstacle)
                distance = self.robot.distances
                #print(distance)
                #print(genome)
                motor_speed_perc = self.NN.calculate(distance, genome, cycle_num)
                #motor_speed_perc = [random.uniform(0.5, 1), random.uniform(0.5, 1)]
                self.robot.set_motor_speed_percentage((motor_speed_perc[0] * 2) - 1,
                                                            (motor_speed_perc[1] * 2) - 1)
                # Move bot
                self.gfx.map.blit(self.gfx.map_img, (0, 0))
                self.robot.kinematics(self.dt)
                self.gfx.draw_dust()  # Dust set up
                self.gfx.draw_robot(self.robot.x, self.robot.y, self.robot.heading)  # Draw Robot
                point_cloud, dust_cloud = self.ultra_sonic.sense_obstacles(self.robot.x, self.robot.y, self.robot.heading)
                self.robot.avoid_obstacles(point_cloud, dust_cloud, self.dt)
                self.gfx.draw_sensor_data(point_cloud, [self.robot.x, self.robot.y])
                pygame.display.update()
                cycle_num += 1


            i += 1
            num_collisions += self.robot.closest_obs_size
            total_dust += starting_dust_am - len(self.gfx.dust_coords)
            print('collisions ',num_collisions)
            print('dust ', total_dust)

        fitness_value = (100 * total_dust - 0.1 * num_collisions) / len(starting_positions)
        print("fitness " + str(fitness_value))
        print("###############")
        if fitness_value > self.best_fitness:
            self.best_fitness = fitness_value
            self.best_genome = genome
            self.best_starts = (self.robot.x, self.robot.y)
        return fitness_value

    #def Final_test(self):


