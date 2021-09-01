import time
import pygame
import sys
import copy
from collections import deque
import numpy as np

#____________________________Optimization results___________________________
# The deque class from collections module is used as the data structure to store the nodes because it is faster compared to the python's inbuilt list.
# String is fast in comparion but searching and conversion from list to string is time consuming(experimentally verified)
# Cannot implement binay search to improve searching speeds because we have to search in an unsorted array.
    
class Node:
    """ This class stores the attributes(like state and parent node) and methods(like check_obstacle_space, actions, generate children etc.) 
        for the each Node in the Map.""" 

    def __init__(self, state, parent):
        
        self.state = state                      # state is the list of x,y coordinates of the node
        self.parent = parent                    # parent attribute stores the parent node of current node

    def __repr__(self):
        return str(self.state)                  # This method returns the state information of the node

    def check_obstacle_space(self, pot_node):
        """Defining obstacle space constraints using half plane equations.
           IMP_NOTE: For concave obstacles divide the obstacles into smaller convex obstacles and take 'OR' between them to find the constraints.
           Example obstacle 3 and 5."""

        x, y = pot_node[0], pot_node[1]

        if (x < 0) or (x > 400) or (y < 0) or (y > 300): # Boundary condition
            return True
        elif (x-90)**2 + (y-70)**2 - 1225 <= 0:   # Obstacle 1 (Circle)
            return True
        elif (y- 176.07 + 1.418*x >= 0) and (y - 98.71 - 0.7*x <= 0) and (y - 436.35 + 1.418*x <= 0) and (y - 74.39 - 0.7*x >= 0): # Obstacle 2 (Rectangle) 
            return True
        elif (x >= 200 and x <= 210 and y <= 280 and y >= 230) or (x >= 210 and x <= 230 and y <= 280 and y >= 270) or (x >= 210 and x <= 230 and y <=240 and y >= 230):      # Obstacle 3 (C section)
            return True
        elif ((x-246)**2)/60**2 + ((y-145)**2)/30**2 - 1 <= 0: # Obstacle 4 (Ellipse)
            return True
        elif ((y + 0.99985*x- 390.95 >= 0) and (y - 0.9866*x + 176.339 <= 0) and (y + 0.2477*x -225.6869 <= 0) and (y + 0.8128*x -425.7314 <=0) and (y - x + 265 >=0)) or ((y - 1.229*x + 294.58 <= 0) and (x- 381.03 <= 0) and (y + 0.8128*x -425.7314 >=0)):  # Obstacle 5 (Irregular polygon)
            return True
        else:
            return False # Node in Freespace

    def up(self):
        # This method performs the up action on the current node
        
        pot_node = (self.state[0], self.state[1] + 1)
        if not self.check_obstacle_space(pot_node):
            up_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            up_node.state[1] = up_node.state[1] + 1
            return up_node
        return None

    def down(self):
        # This method performs the down action on the current node

        pot_node = (self.state[0], self.state[1] - 1)
        if not self.check_obstacle_space(pot_node):
            down_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            down_node.state[1] = down_node.state[1] - 1
            return down_node
        return None

    def left(self):
        # This method performs the left action on the current node

        pot_node = (self.state[0] - 1, self.state[1])
        if not self.check_obstacle_space(pot_node):
            left_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            left_node.state[0] = left_node.state[0] - 1
            return left_node
        return None

    def right(self):
        # This method performs the right action on the current node

        pot_node = (self.state[0] + 1, self.state[1])
        if not self.check_obstacle_space(pot_node):
            right_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            right_node.state[0] = right_node.state[0] + 1
            return right_node
        return None
    
    def up_left(self):
        # This method performs the up_left action on the current node

        pot_node = (self.state[0] - 1, self.state[1] + 1)
        if not self.check_obstacle_space(pot_node):
            up_left_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            up_left_node.state[0], up_left_node.state[1]  = up_left_node.state[0] - 1, up_left_node.state[1] + 1
            return up_left_node
        return None
    
    def up_right(self):
        # This method performs the up_right action on the current node

        pot_node = (self.state[0] + 1, self.state[1] + 1)
        if not self.check_obstacle_space(pot_node):
            up_right_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            up_right_node.state[0], up_right_node.state[1]  = up_right_node.state[0] + 1, up_right_node.state[1] + 1
            return up_right_node
        return None

    def down_left(self):
        # This method performs the down_left action on the current node

        pot_node = (self.state[0] - 1, self.state[1] - 1)
        if not self.check_obstacle_space(pot_node):
            down_left_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            down_left_node.state[0], down_left_node.state[1]  = down_left_node.state[0] - 1, down_left_node.state[1] - 1
            return down_left_node
        return None

    def down_right(self):
        # This method performs the down_right action on the current node

        pot_node = (self.state[0] + 1, self.state[1] - 1)
        if not self.check_obstacle_space(pot_node):
            down_right_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent))
            down_right_node.state[0], down_right_node.state[1]  = down_right_node.state[0] - 1, down_right_node.state[1] - 1
            return down_right_node
        return None

    
    def generate_children(self):
        # This method applies the actions functions to generate the children nodes of the current node 

        children = []
        up_move = self.up()
        down_move = self.down()
        left_move = self.left()
        right_move = self.right()
        up_left_move = self.up_left()
        up_right_move = self.up_right()
        down_left_move = self.down_left()
        down_right_move = self.down_right()
        
        if up_move:
            children.append(up_move) 
        if down_move:
            children.append(down_move)
        if left_move:
            children.append(left_move)
        if right_move:
            children.append(right_move)
        if up_left_move:
            children.append(up_left_move) 
        if up_right_move:
            children.append(up_right_move)
        if down_left_move:
            children.append(down_left_move)
        if down_right_move:
            children.append(down_right_move)
        
        return children
    

if __name__== "__main__":

    while(1):
        # Start node
        x1, y1 = map(int, input("Please input the X and Y coordinates of the start node!\n").split())
        input_node = Node([x1, y1], None)
        
        # Goal node
        x2, y2 = map(int, input("Please input the X and Y coordinates of the goal node!\n").split())
        goal_node = Node([x2, y2], None)
        
        if goal_node.check_obstacle_space(goal_node.state) or input_node.check_obstacle_space(input_node.state):
            print("Input Coordinates are in obstacle space!")
        else:
            break    

    # Using python's inbuilt queue from collections module, because it performs faster enqueue and dequeue operations compared to  a list
    queue = deque()                    
    queue.append(input_node)

    visited_states = []
    t = time.time()

    # Breadth First Search(BFS) Implementation
    while(1):
        current_node = queue.popleft()
        print(current_node, end = '\n')
        
        if current_node.state == [x2, y2]:
            print("Goal Found\n")
            print("Shortest path:\n")
            print(current_node.state)
    
            # Backtracking the parent node to find the shortest path
            # Print sequence GOAL node to START node
            path = []
            while(current_node.state != [x1, y1]):
                current_node = current_node.parent
                path.append(current_node.state)
                print(current_node)
            break
    #____________________________________________________

        if current_node.state not in visited_states:
            visited_states.append(current_node.state)
            children = current_node.generate_children() 

            for child in children:
                queue.append(child)

    print("Execution time", time.time()-t)

#______________________Pygame Animation_________________________
    print("Running pygame animation..................")
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    counter = 0
    while True:

        # Map Generation in pygame
        screen.fill((0,0,0))
        pygame.draw.circle(screen, (255,0,0), (90, 300-70), 35)
        pygame.draw.polygon(screen, (255,0,0), ((48, 300-108), (36.53, 300-124.38),(159.4 ,300-210.416), (170.87, 300-194.036)))
        pygame.draw.polygon(screen, (255,0,0), ((200,20),(230,20),(230,30),(210,30),(210,60), (230,60), (230, 70), (200, 70)))
        pygame.draw.ellipse(screen, (255,0,0), ((186,125, 120, 60)))
        pygame.draw.polygon(screen, (255,0,0), ((328, 300-63), (381.03, 300-116.03), (381.03, 300-171.03), (354, 300-138), (325.17, 300-145.01), (285.574, 300-105.42)))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        if counter ==0:
            for state in visited_states:
                # time.sleep(0.1)
                #print(state)        
                pygame.draw.circle(screen, (255,255,255), (state[0], 300-state[1]), 1) 
                pygame.display.update()
                    
            for state in path:
                pygame.draw.circle(screen, (0,0,255), (state[0], 300-state[1]), 1)
            pygame.display.update()
        counter +=1    
