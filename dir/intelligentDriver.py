'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools, random
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError
from collections import defaultdict

# Class: Graph
# -------------
# Utility class
class Graph(object):
    def __init__(self, nodes, edges,graph):
        self.nodes = nodes
        self.edges = edges
        self.graph = graph

# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):
    MIN_PROB = 0.3
    pm = {}
    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout 
        self.iter = 0
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        self.mp = defaultdict(list)
        for k in self.transProb:
            self.mp[k[0]].append(k[1])
        V = defaultdict(float)
        V_prime = defaultdict(float)
        policy = defaultdict(list)
        policy_prime = defaultdict(list)
        numRows = self.layout.getBeliefRows()
        numCols = self.layout.getBeliefCols()
        state_list =  [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        for state in state_list:
            V[state] = 0
            V_prime[state] = 0
            # policy[state] = random.choice(list(self.worldGraph.graph[state]))
            policy[state] = state
            policy_prime[state] = policy[state]
        self.V = V
        self.V_prime = V_prime
        self.policy = policy
        self.policy_prime = policy_prime

    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            # adjNodes = [(x-1, y-1),(x-1, y+1), (x+1, y-1), (x+1, y+1),(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                edges.append((tile, node))
        graph = defaultdict(set)
        for edge in edges:
            graph[edge[0]].add(edge[1])
            graph[edge[1]].add(edge[0])
            
        return Graph(nodes, edges,graph)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getdist(self, row1,row2, col1, col2):
        return pow((util.rowToY(row1)-util.rowToY(row2))**2+(util.colToX(col1)-util.colToX(col2))**2, 0.5)


    def getreward(self, currPos, nextPos, beliefOfOtherCars, parkedCars, chkPtsSoFar):
        list_of_goal = self.checkPoints
        # list_of_goal.sort(key = lambda x: self.getdist(currPos[0], x[0], currPos[1], x[1]))
        # print("goals: ", list_of_goal)
        goal = list_of_goal[chkPtsSoFar]
        goal_row,goal_col = goal[0], goal[1]
        curr_row, curr_col = currPos[0], currPos[1]
        next_row, next_col = nextPos[0], nextPos[1]
        negative_reward_factor = 100
        negative_reward = 0 
        positive_reward = 0

        #block tiles

        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)
        for ele in list_of_goal:
            if ele in block:
                blockTiles.remove(ele)
        ##################################
        if nextPos == goal:
            positive_reward += 500
        if nextPos in blockTiles:
            positive_reward += -100
        if nextPos[0] >= self.layout.getBeliefRows() or nextPos[0] < 0 or nextPos[1] >= self.layout.getBeliefCols() or nextPos[1] < 0:
            positive_reward += -100
        else:
            curr_dist = self.getdist(curr_row, goal_row, curr_col, goal_col)
            next_dist = self.getdist(next_row, goal_row, next_col, goal_col)
            if next_dist < curr_dist:
                positive_reward += 50
            else:
                positive_reward += 20  
        for i in range(len(beliefOfOtherCars)):    
            negative_reward += negative_reward_factor*beliefOfOtherCars[i].getProb(next_row, next_col)
            if parkedCars[i]:
                neagtive_reward += 100
        # print('positive_reward: ', positive_reward, 'negative_reward: ', negative_reward)
        return positive_reward - negative_reward


    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!
        
        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        ''' 
        numRows = self.layout.getBeliefRows()
        numCols = self.layout.getBeliefCols()
        list_of_goals = self.checkPoints
        final_goal = list_of_goals[chkPtsSoFar]
        moveForward = True
        currPos = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        currState = (util.yToRow(currPos.y),util.xToCol(currPos.x) )
        goalPos = currPos
        goalstate = (util.yToRow(goalPos[1]), util.xToCol(goalPos[0]))
        # BEGIN_YOUR_CODE 
        next_states = self.worldGraph.graph[currState]
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)
        next_states = list(next_states)
        te = []
        for idx in range(len(next_states)):
            if next_states[idx] not in blockTiles:
                te.append(next_states[idx])
        next_states = te
        next_states.sort(key = lambda x: self.getreward(currState, x, beliefOfOtherCars, parkedCars, chkPtsSoFar), reverse = True)
        # print(currState,next_states)
        # print(" ")
        # for i in next_states:
        #     print(i, self.getreward(currPos, i, beliefOfOtherCars, parkedCars, chkPtsSoFar))

        #Handle idx error
        if len(next_states) == 0:
            poss = [(currState[0],currState[1]+1), (currState[0],currState[1]-1), (currState[0]+1,currState[1]), (currState[0]-1,currState[1])]
            for i in range(len(poss)):
                if poss[i][0]>=0 and poss[i][1]>=0 and poss[i][0]<numRows and poss[i][1]<numCols:
                    if poss[i] not in blockTiles:
                        next_states.append(poss[i])
        # print(next_states)
        for idx in range(len(next_states)):
            prob =0
            flag = False
            for i in range(len(parkedCars)):
                if parkedCars[i] == True:
                    temp_prob = self.isCloseToOtherCar(beliefOfOtherCars[i])
                    prob += temp_prob
                    if temp_prob >=0.1:   #  IntelligentDriver.MIN_PROB:
                        flag = True
                        break
                else:
                    temp_prob = self.isCloseToOtherCar(beliefOfOtherCars[i])
                    prob += temp_prob
            if  not flag:
                if prob < 0.1:
                    goalstate = next_states[idx]
                    break 
                else:
                    goalstate = next_states[idx]
                    moveForward = False
                    break
            else:
                goalstate = next_states[idx]

        # END_YOUR_CODE
        poss_goalStates = [goalstate]
        weights = [-1*self.getdist(goalstate[0], final_goal[0], goalstate[1], final_goal[1])]
        u_reward = self.getreward(currState,goalstate,beliefOfOtherCars, parkedCars, chkPtsSoFar)
        for i in range(idx+1,len(next_states)):
            if self.getreward(currState,next_states[i],beliefOfOtherCars, parkedCars, chkPtsSoFar)==u_reward:
                poss_goalStates.append(next_states[i])
                weights.append(-1*self.getdist(next_states[i][0], final_goal[0], next_states[i][1], final_goal[1]))
            
        goalstate = random.choices(poss_goalStates, weights=weights, k = 1)[0]
        
        

        # check for loop
        if currState in IntelligentDriver.pm:
            previous_taken_state = IntelligentDriver.pm[currState][0]
            num_visited = IntelligentDriver.pm[currState][2]
            if goalstate == previous_taken_state and num_visited>10:
                possible_available_states = IntelligentDriver.pm[currState][1]
                if len(possible_available_states) != 1:
                    for ele in possible_available_states:
                        if ele != previous_taken_state:
                            goalstate = ele
                            break
            IntelligentDriver.pm[currState][2] +=1
            IntelligentDriver.pm[currState][0] = goalstate
        else:
            IntelligentDriver.pm[currState] = [goalstate, next_states, 1]
        # future_next_states = self.worldGraph.graph[goalstate]
        # future_next_states = list(future_next_states)
        
        # for ele in future_next_states:
        #     flag2 = True
        #     temp_pr = 0    
        #     for i in range(len(parkedCars)):
        #         if parkedCars[i] == True:
        #             temp_pr += self.isCloseToOtherCar(beliefOfOtherCars[i])
        #             if temp_pr >=0.1:
        #                 flag2 = False
        #                 break
        #         else:
        #             temp_pr += self.isCloseToOtherCar(beliefOfOtherCars[i])
        #             if temp_pr >=0.7:
        #                 flag2 = False
        #                 break

            

        if self.getreward(currState,goalstate,beliefOfOtherCars, parkedCars, chkPtsSoFar) < 10:
            moveForward = False
        if goalstate in blockTiles:
            moveForward = False
        if goalstate[0] == numRows - 1 or goalstate[1] == numCols - 1 or goalstate[0] == 0 or goalstate[1] == 0:
            moveForward = False
        goalPos = (util.rowToY(goalstate[1]), util.colToX(goalstate[0]))
        # print(currState, currPos, goalstate, goalPos)
        # print(goalstate, moveForward)
        return goalPos, moveForward

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def isCloseToOtherCar(self, beliefOfOtherCars):
        newBounds = []
        offset = self.dir.normalized() * 2 * Car.LENGTH
        newPos = self.pos + offset
        row = util.yToRow(newPos.y)
        col = util.xToCol(newPos.x)
        if row> self.layout.getBeliefRows()-1 or col> self.layout.getBeliefCols()-1 or row<0 or col<0:
            return 1
        p = beliefOfOtherCars.getProb(row, col)
        return p

    def getNextGoalPos2(self, beliefOfOtherCars, parkedCars, chkPtsSoFar):
        numRows= self.layout.getBeliefRows()
        numCols= self.layout.getBeliefCols()
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        list_of_goals = self.checkPoints

        currPos = self.getPos()
        currState = (util.yToRow(currPos.y), util.xToCol(currPos.x))


        goalState = list_of_goals[chkPtsSoFar]
        goalPos = (util.rowToY(goalState[1]), util.colToX(goalState[0]))

        ###################
        if chkPtsSoFar != 0 and currState == list_of_goals[chkPtsSoFar-1]:

            V = defaultdict(float)
            V_prime = defaultdict(float)
            policy = defaultdict(list)
            policy_prime = defaultdict(list)
            state_list = self.worldGraph.nodes
            for state in state_list:
                V[state] = 0
                V_prime[state] = 0
                policy[state] = random.choice(list(self.worldGraph.graph[state]))
                policy_prime[state] = policy[state]
            self.V = V
            self.V_prime = V_prime
            self.policy = policy
            self.policy_prime = policy_prime
        ####################


        policy, V, iter = self.policy_iteration(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        immediate_goalState = policy[currState]
        if len(immediate_goalState) == 0:
            poss = [(currState[0],currState[1]+1), (currState[0],currState[1]-1), (currState[0]+1,currState[1]), (currState[0]-1,currState[1])]
            for i in range(len(poss)):
                if poss[i][0]>=0 and poss[i][1]>=0 and poss[i][0]<numRows and poss[i][1]<numCols:
                    if poss[i] not in blockTiles:
                        immediate_goalState.append(poss[i])
            immediate_goalState = random.choice(immediate_goalState)
        print(currState, immediate_goalState, goalState)
        immediate_goalPos = (util.rowToY(immediate_goalState[1]), util.colToX(immediate_goalState[0]))

        moveForward = True
        prob = 0
        flag = False
        for i in range(len(parkedCars)):
            if parkedCars[i]:
                temp_prob = self.isCloseToOtherCar(beliefOfOtherCars[i])
                prob += temp_prob
                if temp_prob >=0.1:
                    moveForward = False
                    flag = True
                    break
            else:
                temp_prob = self.isCloseToOtherCar(beliefOfOtherCars[i])
                prob += temp_prob
                if temp_prob >=0.7:
                    moveForward = False
                    flag = True
                    break
        if not flag:
            if prob < 0.1:
                moveForward = True
            else:
                moveForward = False
        if flag:
            immediate_goalState = random.choice(list(self.worldGraph.graph[currState]))
            immediate_goalPos = (util.rowToY(immediate_goalState[1]), util.colToX(immediate_goalState[0]))
        
        return immediate_goalPos, moveForward

        pass


    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        # print(self.checkPoints)
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]

        goalPos, df = self.getNextGoalPos2(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        # print(self.pos, goalPos, "vectorToGoal: ", vectorToGoal)
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        # else:
        #     actions[Car.DRIVE_FORWARD] = -1.0
        return actions

    # Policy Iteration Algorithm for finding the Optimal Policy
    def policy_iteration(self,beliefOfOtherCars, parkedCars, chkPtsSoFar, epsilon = 0.4, discount = 0.9):
        """
        Find the optimal policy for the given layout and epsilon.
        """
        iter = 0
        # Initialize the value function
        # V = {}
        # V_prime = {}
        state_list = self.worldGraph.nodes
        # for state in state_list:
        #     V[state] = 0
        #     V_prime[state] = 0 
        V = self.V
        V_prime = self.V_prime

        # Initialize the policy
        # policy = {}
        # policy_prime = {}
        # for state in state_list:
        #     policy[state] = state
        #     policy_prime[state] = policy[state]
        policy = self.policy
        policy_prime = self.policy_prime
        while True:
            # Copy the policy to the new one
            for state in state_list:
                policy_prime[state] = policy[state]
            while True:
                # Copy the value function to the new one
                for state in state_list:
                    V_prime[state] = V[state]
                # Find the optimal policy
                for state in state_list:
                    # Find the optimal value
                    action = policy[state]
                    value = 0
                    reward = self.getreward(state,action,beliefOfOtherCars, parkedCars, chkPtsSoFar)
                    for next_state in self.mp[state]:
                        value += self.transProb[state , next_state] * (reward + discount * V_prime[next_state])

                    # for next_state, t in self.transProb[(state, action)]:
                    #     reward = 0
                    #     for s, r in reward_dict[(state, action)]:
                    #         if s == next_state:
                    #             reward = r
                    #             break
                    #     value += t * (reward + discount * V_prime[next_state])
                    V[state] = value
                # Check if the value function converges
                converged = True
                for state in state_list:
                    if abs(V[state] - V_prime[state]) > epsilon:
                        converged = False
                if converged:
                    break
            for state in state_list:
                max_value = -float('inf')
                for action in self.worldGraph.graph[state]:
                    value = 0
                    reward = self.getreward(state,action,beliefOfOtherCars, parkedCars, chkPtsSoFar)
                    for next_state in self.mp[state]:
                        value += self.transProb[state , next_state] * (reward + discount * V_prime[next_state])
                    # for next_state, t in transition_dict[(state, action)]:
                    #     reward = 0
                    #     for s, r in reward_dict[(state, action)]:
                    #         if s == next_state:
                    #             reward = r
                    #             break
                    #     value += t * (reward + discount * V[next_state])
                    if value > max_value:
                        max_value = value
                        policy[state] = action
            # Check if the policy converges
            iter += 1
            converged = True
            for state in state_list:
                if policy_prime[state] != policy[state]:
                    converged = False
            if converged:
                return policy, V, iter


    
    