################################################################################
# Author:  Jonathan Lee
# Date:  2016.08.25
# Revision: 1.17 
# Description:  Initial release to Udacity Project Submission
################################################################################


import numpy as np

class Robot(object):
    def __init__(self, maze_dim):
        # These wall costs are used in the sense_wall function
        self.wall_costs = {'up': [8,1,2],
                            'down': [2,4,8],
                            'left': [4,8,1],
                            'right': [1,2,4]
                            }
        # These binary indices are used in the sense_wall function
        self.bin_index = {'up': [0,3,2],
                            'down': [2,1,0],
                            'left': [1,0,3],
                            'right':  [3,2,1]}

        # These arrows are used in an attemp to print out the path for fun
        self.direction = {'up': '>',
                          'down':'<',
                          'left': '^',
                          'right': 'v',

        }
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim

        # Initialize all the squares to 15 which indicates no walls
        self.walls = [[15 for i in range(maze_dim)] for j in range(maze_dim)]

        # Initialize all distance to goal as 99
        self.distance_to_goal = [[99 for i in range(maze_dim)] for j in range(maze_dim)]
        self.found_goal = False
        # This array contains the locations where a wall change has been made, initially 0
        self.wall_updated = [[0 for i in range(maze_dim)] for j in range(maze_dim)]
        # This array contains the locations where the robot has been moved to, initially 0
        self.moved_to = [[0 for i in range(maze_dim)] for j in range(maze_dim)]
        self.moved_to[self.location[0]][self.location[1]] = 1
        # this is the goal defined in the tester
        self.chosen_goal = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.count = 0
        self.learned_path = False
        self.next_round = 0
        self.already_moved = False
        self.already_chosen = False
        self.final_moves = []
        self.final_path = [[' ' for i in range(maze_dim)] for j in range(maze_dim)]
        self.set_learned_path = False

    # checks the limits of the either x or y location to ensure it is within the maze dimensions
    def check_limits(self, param1):
        if int(param1) >=0 and int(param1) < self.maze_dim:
            return True
        else:
            return False

    # This function was used in the exploratory algorithm
    def choose_new_goal(self, my_list):
        sorted_list = sorted(my_list,reverse=True)
        chosen_item = sorted_list.pop(0)
        newx=chosen_item[0]
        newy=chosen_item[1]
        return newx,newy

    # the flood fill algorithm which plots the distances to the goal from any square in the grid
    def flood_fill(self,xlocation,ylocation):
        moves= [[-1,0], #west
                [0,-1], #south
                [1,0],  #east
                [0,1]   #north
                ]
        self.distance_to_goal = [[99 for i in range(self.maze_dim)] for j in range(self.maze_dim)]

        self.distance_to_goal[xlocation][ylocation] = 0

        # creating a 2-D array which indicates which indicates all squares have not been traveled to
        traveled = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]

        traveled[xlocation][ylocation] = 1

        next_list = [[xlocation,ylocation]]

        while len(next_list)>0:
            current = next_list.pop(0)
            binary_wall = np.binary_repr(self.walls[current[0]][current[1]],width=4)
            for i in range(len(binary_wall)):
                if int(binary_wall[i]) == 1:
                    next_move = [(current[0]+moves[i][0]),(current[1]+moves[i][1])]
                    if (next_move[0]>=0 and next_move[0]<self.maze_dim and next_move[1]>=0 and next_move[1]<self.maze_dim):
                        if (traveled[next_move[0]][next_move[1]] != 1):
                            self.distance_to_goal[next_move[0]][next_move[1]] = self.distance_to_goal[current[0]][current[1]] + 1
                            next_list.append(next_move)
                            traveled[next_move[0]][next_move[1]] = 1

    # detects any wall at any distance and updates the walls array
    def sense_wall(self, direction, sense):
        if direction == 'up':
            xparam1 = -1
            xparam2 = 0 
            xparam3 = 1
            yparam1 = 0
            yparam2 = 1
            yparam3 = 0
            opposite = 'down'
            add_param1 = 1
            add_param2 = 0
        if direction == 'down':
            xparam1 = 1
            xparam2 = 0 
            xparam3 = -1
            yparam1 = 0
            yparam2 = -1
            yparam3 = 0
            opposite = 'up'
            add_param1 = -1
            add_param2 = 0
        if direction == 'left':
            xparam1 = 0
            xparam2 = -1 
            xparam3 = 0
            yparam1 = -1
            yparam2 = 0
            yparam3 = 1
            opposite = 'right'
            add_param1 = 0
            add_param2 = 1
        if direction == 'right':
            xparam1 = 0
            xparam2 = 1 
            xparam3 = 0
            yparam1 = 1
            yparam2 = 0
            yparam3 = -1
            opposite = 'left'
            add_param1 = 0
            add_param2 = -1

        # Find the location of the grid which the sensor is sensing a wall
        left_sq_loc = self.walls[self.location[0]+xparam1*sense[0]][self.location[1]+yparam1*sense[0]]
        front_sq_loc = self.walls[self.location[0]+xparam2*sense[1]][self.location[1]+yparam2*sense[1]]
        right_sq_loc = self.walls[self.location[0]+xparam3*sense[2]][self.location[1]+yparam3*sense[2]]

        # Find the x and y coordinate of the neighbor of the grid which the sensor sensed
        left_add_x = self.location[0]+xparam1*sense[0]-add_param1
        left_add_y = self.location[1]+yparam1*sense[0]-add_param2
        front_add_x = self.location[0]+xparam2*sense[1]-add_param2
        front_add_y = self.location[1]+yparam2*sense[1]+add_param1
        right_add_x = self.location[0]+xparam3*sense[2]+add_param1
        right_add_y = self.location[1]+yparam3*sense[2]+add_param2

        # checking the limits of the x and y coordinate 
        if self.check_limits(left_add_x) and self.check_limits(left_add_y):
            left_sq_plus_loc = self.walls[left_add_x][left_add_y]
        if self.check_limits(front_add_x) and self.check_limits(front_add_y):
            front_sq_plus_loc = self.walls[front_add_x][front_add_y]
        if self.check_limits(right_add_x) and self.check_limits(right_add_y):
            right_sq_plus_loc = self.walls[right_add_x][right_add_y]

        # converting the wall grid into binary form
        far_left_walls = np.binary_repr(left_sq_loc, width=4)
        far_front_walls = np.binary_repr(front_sq_loc, width=4)
        far_right_walls = np.binary_repr(right_sq_loc, width=4)

        # update the left wall
        if int(far_left_walls[self.bin_index[direction][0]]) == 1:
            self.walls[self.location[0]+xparam1*sense[0]][self.location[1]+yparam1*sense[0]] -= self.wall_costs[direction][0]
            self.wall_updated[self.location[0]+xparam1*sense[0]][self.location[1]+yparam1*sense[0]] = 1
            # update the left wall + 1
            if self.check_limits(left_add_x) and self.check_limits(left_add_y):
                self.walls[left_add_x][left_add_y] -= self.wall_costs[opposite][0]
                self.wall_updated[left_add_x][left_add_y] = 1

        # update the front wall
        if int(far_front_walls[self.bin_index[direction][1]]) == 1:
            self.walls[self.location[0]+xparam2*sense[1]][self.location[1]+yparam2*sense[1]] -= self.wall_costs[direction][1]
            self.wall_updated[self.location[0]+xparam2*sense[1]][self.location[1]+yparam2*sense[1]] = 1
            # update the front wall + 1
            if self.check_limits(front_add_x) and self.check_limits(front_add_y):
                self.walls[front_add_x][front_add_y] -= self.wall_costs[opposite][1]
                self.wall_updated[front_add_x][front_add_y] = 1

        # update the right wall
        if int(far_right_walls[self.bin_index[direction][2]]) == 1:
            self.walls[self.location[0]+xparam3*sense[2]][self.location[1]+yparam3*sense[2]] -= self.wall_costs[direction][2]
            self.wall_updated[self.location[0]+xparam3*sense[2]][self.location[1]+yparam3*sense[2]] = 1
            # update the right wall + 1
            if self.check_limits(right_add_x) and self.check_limits(right_add_y):
                self.walls[right_add_x][right_add_y] -= self.wall_costs[opposite][2]
                self.wall_updated[right_add_x][right_add_y] = 1

    # The rotation and movement from this will be returned to the tester
    # in first run, there is a 1st movement algorithm since 
    # the robot will be sensing and exploring one step at a time
    # after the robot reaches the goal everything is reset
    # paths that the robot didn't travel is blocked
    # robot will follow a 2nd movement algorithm which allows the robot to move more than one step
    # in the 2nd movement algorithm the robot also does not sense or alter the map
    def next_move(self, sensors):
        already_moved = False
        # Definining the goal locations for the flood fill algorithm
        goal_a = [(self.maze_dim-1)/2,(self.maze_dim-1)/2]
        goal_b = [(self.maze_dim-1)/2,(self.maze_dim)/2]
        goal_c = [(self.maze_dim)/2,(self.maze_dim-1)/2]
        goal_d = [(self.maze_dim)/2,(self.maze_dim)/2]

        goal_e = [self.maze_dim/2 - 1, self.maze_dim/2]

        headings = ['up', 'right', 'down', 'left']

        # Updating the walls with the sense_wall function
        if self.heading == 'up':
            self.sense_wall('up', sensors)
        if self.heading == 'down':
            self.sense_wall('down', sensors)
        if self.heading =='left':
            self.sense_wall('left', sensors)
        if self.heading =='right':
            self.sense_wall('right', sensors)

        # Initializing the variable for the move 
        wall_bin_updated = np.binary_repr(self.walls[self.location[0]][self.location[1]], width=4)
        west_wall_updated = int(wall_bin_updated[0])
        south_wall_updated = int(wall_bin_updated[1])
        east_wall_updated = int(wall_bin_updated[2])
        north_wall_updated = int(wall_bin_updated[3])

        # Has the goal been reach?
        if self.location == self.chosen_goal:
            self.found_goal = True

        # If the goal has been reached we choose a new location to move the robot
        # we do this until a true path has been learned
        w_moves = [[-1,0],
                    [0,-1],
                    [1,0],
                    [0,1]]
        new_list = []
        new_array=[[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        for i in range(len(self.moved_to)):
                for j in range(len(self.moved_to)):
                    if int(self.moved_to[i][j]) == 1 or int(self.wall_updated[i][j]) == 1:
                        new_array[i][j] += 1
        if self.found_goal == False:
            self.flood_fill(self.chosen_goal[0], self.chosen_goal[1])
        else:
            #########   Implementing the wall blocking Alogrithm  ############
            for i in range(len(self.moved_to)):
                for j in range(len(self.moved_to)):
                    if new_array[i][j] == 0:
                        new_list.append([i,j])
            if len(new_list) > 0:
                for item in new_list:
                    self.walls[item[0]][item[1]] = 0
                    for i in range(len(self.walls)):
                        for j in range(len(self.walls)):
                            if self.moved_to[i][j] == 0:
                                if self.check_limits(i+w_moves[0][0]) and self.check_limits(j+w_moves[0][1]):
                                    if int(np.binary_repr(self.walls[i+w_moves[0][0]][j+w_moves[0][1]],width=4)[2]) == 1:
                                        self.walls[i+w_moves[0][0]][j+w_moves[0][1]] -= 2

                                if self.check_limits(i+w_moves[1][0]) and self.check_limits(j+w_moves[1][1]):
                                    if int(np.binary_repr(self.walls[i+w_moves[1][0]][j+w_moves[1][1]],width=4)[3]) == 1:
                                        self.walls[i+w_moves[1][0]][j+w_moves[1][1]] -= 1

                                if self.check_limits(i+w_moves[2][0]) and self.check_limits(j+w_moves[2][1]):
                                    if int(np.binary_repr(self.walls[i+w_moves[2][0]][j+w_moves[2][1]],width=4)[0]) == 1:
                                        self.walls[i+w_moves[2][0]][j+w_moves[2][1]] -= 8

                                if self.check_limits(i+w_moves[3][0]) and self.check_limits(j+w_moves[3][1]):
                                    if int(np.binary_repr(self.walls[i+w_moves[3][0]][j+w_moves[3][1]],width=4)[1]) == 1:
                                        self.walls[i+w_moves[3][0]][j+w_moves[3][1]] -= 4  
                                
                self.set_learned_path = True

        # Directions on where to move after calculating new distances to goal
        r_moves = [[-1,0],
                    [0,-1],
                    [1,0],
                    [0,1]]

        move_list = []
        dist_list = []

        ##########        Movement implementation for the First Run    ################
        if self.heading == 'up' and not already_moved and self.learned_path == False:
            if self.location[1] >= 0 and self.location[1] < self.maze_dim:
                for i in range(len(wall_bin_updated)):
                    if int(wall_bin_updated[i]) == 1:
                        movex = (self.location[0]+r_moves[i][0])
                        movey = (self.location[1]+r_moves[i][1])
                        if (movex>=0 and movex<self.maze_dim and movey>=0 and movey<self.maze_dim):
                            move_list.append([movex,movey])
                            dist_list.append(self.distance_to_goal[movex][movey])
                    if len(move_list) >0:
                        chosen_move = move_list[np.argmin(dist_list)]
                deltax = chosen_move[0]-self.location[0]
                deltay = chosen_move[1]-self.location[1]

                if deltax == 0 and deltay == 1:
                    rotation = 0
                    movement = 1

                if deltax == 1 and deltay == 0:
                    rotation = 90
                    movement = 1
                    self.heading = 'right'
                if deltax == 0 and deltay == -1:
                    rotation = 90
                    movement = 0
                    self.heading = 'right'
                    
                if deltax == -1 and deltay == 0:
                    rotation = -90
                    movement = 1
                    self.heading = 'left'

                already_moved = True

        if self.heading == 'down' and not already_moved and self.learned_path == False:
            if self.location[1] >=0 and self.location[1] < self.maze_dim:
                for i in range(len(wall_bin_updated)):
                    if int(wall_bin_updated[i]) == 1:
                        movex = (self.location[0]+r_moves[i][0])
                        movey = (self.location[1]+r_moves[i][1])
                        if (movex>=0 and movex<self.maze_dim and movey>=0 and movey<self.maze_dim):
                            move_list.append([movex,movey])
                            dist_list.append(self.distance_to_goal[movex][movey])

                    if len(move_list) >0:
                        chosen_move = move_list[np.argmin(dist_list)]

                deltax = chosen_move[0]-self.location[0]
                deltay = chosen_move[1]-self.location[1]

                if deltax == 0 and deltay == 1:
                    rotation = -90
                    movement = 0
                    self.wall_updated[self.location[0]][self.location[1]] = 1
                    self.heading = 'right'
                if deltax == 1 and deltay == 0:
                    rotation = -90
                    movement = 1
                    self.heading = 'right'
                if deltax == 0 and deltay == -1:
                    rotation = 0
                    movement = 1
                if deltax == -1 and deltay == 0:
                    rotation = 90
                    movement = 1
                    self.heading = 'left'

                already_moved = True

        if self.heading == 'left' and not already_moved and self.learned_path == False:
            if self.location[0] >= 0 and self.location[0] < self.maze_dim:
                for i in range(len(wall_bin_updated)):
                    if int(wall_bin_updated[i]) == 1:
                        movex = (self.location[0]+r_moves[i][0])
                        movey = (self.location[1]+r_moves[i][1])
                        if (movex>=0 and movex<self.maze_dim and movey>=0 and movey<self.maze_dim):
                            move_list.append([movex,movey])
                            dist_list.append(self.distance_to_goal[movex][movey])
                    if len(move_list) >0:
                        chosen_move = move_list[np.argmin(dist_list)]

                deltax = chosen_move[0]-self.location[0]
                deltay = chosen_move[1]-self.location[1]

                if deltax == 0 and deltay == 1:
                    rotation = 90
                    movement = 1
                    self.heading = 'up'
                if deltax == 1 and deltay == 0:
                    rotation = 90
                    movement = 0
                    self.wall_updated[self.location[0]][self.location[1]] = 1
                    self.heading = 'up'
                if deltax == 0 and deltay == -1:
                    rotation = -90
                    movement = 1
                    self.heading = 'down'
                if deltax == -1 and deltay == 0:
                    rotation = 0
                    movement = 1

                already_moved = True

        if self.heading == 'right' and not already_moved and self.learned_path == False:
            if self.location[0] >=0 and self.location[0] < self.maze_dim:
                for i in range(len(wall_bin_updated)):
                    if int(wall_bin_updated[i]) == 1:
                        movex = (self.location[0]+r_moves[i][0])
                        movey = (self.location[1]+r_moves[i][1])
                        if (movex>=0 and movex<self.maze_dim and movey>=0 and movey<self.maze_dim):
                            move_list.append([movex,movey])
                            dist_list.append(self.distance_to_goal[movex][movey])
                    if len(move_list) >0:
                        chosen_move = move_list[np.argmin(dist_list)]

                deltax = chosen_move[0]-self.location[0]
                deltay = chosen_move[1]-self.location[1]

                if deltax == 0 and deltay == 1:
                    rotation = -90
                    movement = 1
                    self.heading = 'up'
                if deltax == 1 and deltay == 0:
                    rotation = 0
                    movement = 1
                if deltax == 0 and deltay == -1:
                    rotation = 90
                    movement = 1
                    self.heading = 'down'
                if deltax == -1 and deltay == 0:
                    rotation = -90
                    movement = 0
                    self.wall_updated[self.location[0]][self.location[1]] = 1
                    self.heading = 'up'
                already_moved = True

        ##########        Movement implementation for the Second Run    ################
        if self.learned_path == True:
            robot_loc = self.location

            possible_moves = { 'up': [[-1,0],
                                     [0,1],
                                     [1,0],
                                     ],
                               'down': [[1,0],
                                       [0,-1],
                                       [-1,0]
                                       ],
                               'left': [[0,-1],
                                       [-1,0],
                                       [0,1],
                                       ],
                               'right': [[0,1],
                                        [1,0],
                                        [0,-1],
                                        ],
                                }

            potential_move_list = []

            for i in range(len(possible_moves[self.heading])):
                # If sensors detect a large enough space robot checks to see if it an move 3 spaces
                # if it can't move 3 then checks if it can move 2
                # if it can't move 2 then moves 1 space
                if sensors[i] >= 3:
                    dist_mult = 3
                    check_point = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1]]
                    if self.distance_to_goal[check_point[0]][check_point[1]] + 1*dist_mult == self.distance_to_goal[robot_loc[0]][robot_loc[1]]:
                        next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                    else:
                        dist_mult = 2
                        check_point = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1]]
                        if self.distance_to_goal[check_point[0]][check_point[1]] + 1*dist_mult == self.distance_to_goal[robot_loc[0]][robot_loc[1]]:
                            next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                        else:
                            dist_mult = 1
                            next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                # If sensors detect a large enough space robot checks to see if it an move 1 spaces
                # if it can't move 2 then moves 1 space
                elif sensors[i] == 2:
                    dist_mult = 2
                    check_point = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1]]
                    if self.distance_to_goal[check_point[0]][check_point[1]] + 1*dist_mult == self.distance_to_goal[robot_loc[0]][robot_loc[1]]:
                        next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                    else:
                        dist_mult = 1
                        next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                # Otherwise just move one space
                elif sensors[i] == 1:
                        dist_mult = 1
                        check_point = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1]]
                        next_possible_move = [robot_loc[0]+dist_mult*possible_moves[self.heading][i][0],robot_loc[1]+dist_mult*possible_moves[self.heading][i][1],dist_mult]
                # Otherwise just move one space           
                elif sensors[i] == 0:
                    next_possible_move = [robot_loc[0]+1*possible_moves[self.heading][i][0],robot_loc[1]+1*possible_moves[self.heading][i][1],1]
                if self.check_limits(next_possible_move[0]) and self.check_limits(next_possible_move[1]):
                    if int(sensors[i]) != 0:
                        potential_move_list.append(next_possible_move)

            while len(potential_move_list) >0 and self.location != self.chosen_goal:
                the_chosen_one = potential_move_list.pop(0)

                if self.distance_to_goal[the_chosen_one[0]][the_chosen_one[1]] + 1*the_chosen_one[2] == self.distance_to_goal[robot_loc[0]][robot_loc[1]]:
                    if self.already_chosen == False:
                        the_real_chosen_one = the_chosen_one
                        self.final_moves.append(the_real_chosen_one)
                        self.final_path[self.location[0]][self.location[1]] = self.direction[self.heading]
                        self.already_chosen = True
                    if self.heading == 'up' and self.already_moved == False:
                        dx = the_real_chosen_one[0]-self.location[0]
                        dy = the_real_chosen_one[1]-self.location[1]

                        if dx == 0 and dy == 1*the_real_chosen_one[2]:
                            rotation = 0
                            movement = 1*the_real_chosen_one[2]
                            self.already_moved = True

                        elif dx == 1*the_real_chosen_one[2] and dy == 0:
                            rotation = 90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'right'
                            self.already_moved = True

                        elif dx == 0 and dy == -1*the_real_chosen_one[2]:
                            rotation = 0
                            movement = -1
                            self.already_moved = True
            
                        elif dx == -1*the_real_chosen_one[2] and dy == 0:
                            rotation = -90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'left'
                            self.already_moved = True
                    elif self.heading == 'down' and self.already_moved == False:
                        dx = the_real_chosen_one[0]-self.location[0]
                        dy = the_real_chosen_one[1]-self.location[1]

                        if dx == 0 and dy == 1*the_real_chosen_one[2]:
                            rotation = 0
                            movement = -1
                            self.already_moved = True
                        elif dx == 1*the_real_chosen_one[2] and dy == 0:
                            rotation = -90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'right'
                            self.already_moved = True
                        elif dx == 0 and dy == -1*the_real_chosen_one[2]:
                            rotation = 0
                            movement = 1*the_real_chosen_one[2]
                            self.already_moved = True
                        elif dx == -1*the_real_chosen_one[2] and dy == 0:
                            rotation = 90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'left'
                            self.already_moved = True
                    elif self.heading == 'left' and self.already_moved == False:
                        dx = the_real_chosen_one[0]-self.location[0]
                        dy = the_real_chosen_one[1]-self.location[1]

                        if dx == 0 and dy == 1*the_real_chosen_one[2]:
                            rotation = 90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'up'
                            self.already_moved = True

                        elif dx == 1*the_real_chosen_one[2] and dy == 0:
                            rotation = 0
                            movement = -1
                            self.already_moved = True
                        elif dx == 0 and dy == -1*the_real_chosen_one[2]:
                            rotation = -90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'down'
                            self.already_moved = True
                        elif dx == -1*the_real_chosen_one[2] and dy == 0:
                            rotation = 0
                            movement = 1*the_real_chosen_one[2]
                            self.already_moved = True
                    elif self.heading == 'right' and self.already_moved == False:
                        dx = the_real_chosen_one[0]-self.location[0]
                        dy = the_real_chosen_one[1]-self.location[1]

                        if dx == 0 and dy == 1*the_real_chosen_one[2]:
                            rotation = -90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'up'
                            self.already_moved = True
                        elif dx == 1*the_real_chosen_one[2] and dy == 0:
                            rotation = 0
                            movement = 1*the_real_chosen_one[2]
                            self.already_moved = True
                        elif dx == 0 and dy == -1*the_real_chosen_one[2]:
                            rotation = 90
                            movement = 1*the_real_chosen_one[2]
                            self.heading = 'down'
                            self.already_moved = True
                        elif dx == -1*the_real_chosen_one[2] and dy == 0:
                            rotation = 0
                            movement = -1
                            self.already_moved = True

        # If the goal is found then we reset
        # Regenerate the distance array one last time
        # Regenerate the wall array one last time
        if self.set_learned_path == True and self.learned_path==False:
            print "Resetting positions..."
            rotation = 'Reset'
            movement = 'Reset'
            the_real_chosen_one = [0,0]
            self.chosen_goal = [self.maze_dim/2 - 1, self.maze_dim/2]
            self.heading = 'up'

            self.flood_fill(self.maze_dim/2 - 1, self.maze_dim/2)
            print "Final distances:"
            for item in self.distance_to_goal:
                print item
            print "\n Final wall:"
            for item in self.walls:
                print item

            self.learned_path = True

        if self.learned_path == False:
            if movement == 1:
                self.location = chosen_move
        else:
            self.location = the_real_chosen_one

        self.moved_to[self.location[0]][self.location[1]] = 1
        self.already_moved = False
        self.already_chosen = False

        return rotation, movement
