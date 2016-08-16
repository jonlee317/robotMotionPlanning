import numpy as np

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim

        self.walls = [[15 for i in range(maze_dim)] for j in range(maze_dim)]
        self.distance_to_goal = [[0 for i in range(maze_dim)] for j in range(maze_dim)]
        
        #maze_len = len(maze_dim)
        #print maze_dim

    def next_move(self, sensors):
        goal_a = [(self.maze_dim-1)/2,(self.maze_dim-1)/2]
        goal_b = [(self.maze_dim-1)/2,(self.maze_dim)/2]
        goal_c = [(self.maze_dim)/2,(self.maze_dim-1)/2]
        goal_d = [(self.maze_dim)/2,(self.maze_dim)/2]

        headings = ['up', 'left', 'down', 'right']




        g_score = 0
        # update walls
        # note that I had to invert the north and south since the array is facing opposite direction
        #print sensors
        wall_bin = np.binary_repr(self.walls[self.location[0]][self.location[1]], width=4)
        #print wall_bin
        west_wall = int(wall_bin[0])
        south_wall = int(wall_bin[1])
        east_wall = int(wall_bin[2])
        north_wall = int(wall_bin[3])

        if self.heading == 'up':
            if sensors[0] == 0 and west_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 8
            if sensors[1] == 0 and north_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 1
            if sensors[2] == 0 and east_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 2
        if self.heading == 'down':
            if sensors[0] == 0 and east_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 2
            if sensors[1] == 0 and south_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 4
            if sensors[2] == 0 and west_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 8
        if self.heading == 'left':
            if sensors[0] == 0 and north_wall ==1:
                self.walls[self.location[0]][self.location[1]] -= 1
            if sensors[1] == 0 and west_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 8
            if sensors[2] == 0 and south_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 4
        if self.heading == 'right':
            if sensors[0] == 0 and south_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 4
            if sensors[1] == 0 and east_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 2
            if sensors[2] == 0 and north_wall == 1:
                self.walls[self.location[0]][self.location[1]] -= 1

        # Flood filling
        for item in self.walls:
            print item
        moves= [[0,-1], #west
                [1,0], #south
                [0,1],  #east
                [-1,0]   #north
                ]

        self.distance_to_goal[goal_a[0]][goal_a[1]] = 0
        self.distance_to_goal[goal_b[0]][goal_b[1]] = 0
        self.distance_to_goal[goal_c[0]][goal_c[1]] = 0
        self.distance_to_goal[goal_d[0]][goal_d[1]] = 0

        traveled = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]

        # need to take g score into account and start expanding the ones with lowest gscore

        traveled[goal_a[0]][goal_a[1]] = 1
        traveled[goal_b[0]][goal_b[1]] = 1
        traveled[goal_c[0]][goal_c[1]] = 1
        traveled[goal_d[0]][goal_d[1]] = 1

        next_list = [[goal_a[0],goal_a[1]],
            [goal_b[0],goal_b[1]],
            [goal_c[0],goal_c[1]],
            [goal_d[0],goal_d[1]]]

        #print next_list

        test_count = 0
        #while test_count <200:

        while len(next_list)>0:
            current = next_list.pop(0)
            test_count +=1
            binary_wall = np.binary_repr(self.walls[current[0]][current[1]],width=4)
            #for b in binary_wall:
            #    print int(b)
            for i in range(len(binary_wall)):
                if int(binary_wall[i]) == 1:
                    next_move = [(current[0]+moves[i][0]),(current[1]+moves[i][1])]
                    if (next_move[0]>=0 and next_move[0]<self.maze_dim and next_move[1]>=0 and next_move[1]<self.maze_dim):
                        if (traveled[next_move[0]][next_move[1]] != 1):
                            self.distance_to_goal[next_move[0]][next_move[1]] = self.distance_to_goal[current[0]][current[1]] + 1
                            next_list.append(next_move)
                            traveled[next_move[0]][next_move[1]] = 1

        for item in self.distance_to_goal:
            print item
        #for item in self.walls:
        #    print item




        #distance_to_goal

        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        r_moves = [[-1,0],
                    [0,-1],
                    [1,0],
                    [0,1]]


        rotation = 0
        #print self.heading
        #print headings.index(self.heading)
        if rotation == 90:
            self.heading = headings[(headings.index(self.heading)+1)%4]
        if rotation == -90:
            self.heading = headings[(headings.index(self.heading)-1)%4]
        movement = 0
        print self.heading
        #
        move_list = []
        if self.heading == 'up':
            if self.location[1] >= 0 and self.location[1] < self.maze_dim-1:

                for i in range(len(wall_bin)):
                    #print i
                    if int(wall_bin[i]) == 1:
                        movex = (self.location[0]+r_moves[i][0])
                        movey = (self.location[1]+r_moves[i][1])
                        if (movex>=0 and movex<self.maze_dim and movey>=0 and movey<self.maze_dim):
                            move_list.append([movex,movey])
                            #print move_list
                    if len(move_list) >0:
                        chosen_move = move_list[np.argmin(self.distance_to_goal[movex][movey])]
                        move_list = []
                deltax = chosen_move[0]-self.location[0]
                deltay = chosen_move[1]-self.location[1]

                if deltax == 0 and deltay == 1:
                    movement = 1
                    rotation = 0
                if deltax == 1 and deltay == 0:
                    rotation = 90
                    movement = 1
                if deltax == 0 and deltay == -1:
                    movement = -1
                    rotation = 0
                if deltax == -1 and deltay == 0:
                    rotation = -90
                    movement = 1

                
                print chosen_move
                self.location[1] += movement
        if self.heading == 'down':
            if self.location[1] >=0 and self.location[1] < self.maze_dim-1:
                self.location[1] -= movement
        if self.heading == 'left':
            if self.location[0] >= 0 and self.location[0] < self.maze_dim-1:
                self.location[0] -= movement
        if self.heading == 'right':
            if self.location[0] >=0 and self.location[0] < self.maze_dim-1:
                self.location[0] += movement


        return rotation, movement
