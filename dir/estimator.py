import util 
from util import Belief, pdf 
from engine.const import Const
from collections import defaultdict
import random

# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    
    

    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb() 
        rows = self.belief.numRows
        cols = self.belief.numCols
        #print("rows: ", rows)
        pf = defaultdict(tuple)
        n = 3
        self.numpoints = n*rows*cols # sample size of points in particle filter 
        point = 0
        for i in range(rows):
            for j in range(cols):
                for k in range(n):
                    pf[point] = (i,j) 
                    point += 1
        self.pf = pf
        self.mp = defaultdict(list)
        # for i in range(rows):
        #     for j in range(cols):
        #         for next_i in range(rows):
        #             for next_j in range(cols):
        #                 if ((i,j),(next_i, next_j)) in self.transProb:
        #                     self.mp[(i,j)].append((next_i, next_j))
        # # for k in self.transProb:

        for k in self.transProb:
            self.mp[k[0]].append(k[1])

        
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    # 
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################
    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        # BEGIN_YOUR_CODE

        #1st approach

        # print("f: ", self.mp)
        #particle filtering
        rows = self.belief.numRows
        cols = self.belief.numCols
        num_points = self.numpoints
        if isParked:
            #if the vehicle is parked, then the car doesnot move, so while elapsing samples into next state we just keep the same sample
            #perform the weighted sampling then
            weight = defaultdict(int)
            for n in range(num_points):
                p_row = self.pf[n][0]
                p_col = self.pf[n][1]
                #mean is the distance between (posX, posY) and (p_row, p_col)
                mean = pow((posY-util.rowToY(p_row))**2+(posX-util.colToX(p_col))**2, 0.5)
                p = pdf(mean, Const.SONAR_STD, observedDist)
                weight[n] = p
            new_pf = defaultdict(tuple)
            resample_Arr = []
            resample_prob = []
            for n in range(num_points):
                resample_Arr.append(self.pf[n])
                resample_prob.append(weight[n])
            for n in range(num_points):
                temp = random.choices(resample_Arr, weights = resample_prob, k = 1)
                new_pf[n] = temp[0]
            self.pf = new_pf

            #update the belief
            new_belief = [[0 for _ in range(cols)] for _ in range(rows)]
            for n in range(num_points):
                p_row = self.pf[n][0]
                p_col = self.pf[n][1]
                new_belief[p_row][p_col] += 1
            for i in range(rows):
                for j in range(cols):
                    self.belief.setProb(i,j,new_belief[i][j])

        else:
            #elapse
            for n in range(num_points):
                p_row = self.pf[n][0]
                p_col = self.pf[n][1]
                # next_row = []
                # next_col = []
                # for i in {-3,-2,-1,0,1,2,3}:
                #     if (p_row+i)>= 0 and (p_row+i)<rows:
                #         next_row.append(p_row+i) 
                #     if (p_col+i)>= 0 and (p_col+i)<cols:
                #         next_col.append(p_col+i)
                # print(p_row,p_col)
                if (p_row, p_col) not in self.mp:
                    continue
                next_grid = self.mp[(p_row,p_col)]
                # print(next_grid)
                prob = []
                for grid in next_grid:
                    prob.append(self.transProb[((p_row,p_col),grid)])
                # next_grid2 = []
                # prob2 = []
                # for next_r in range(rows):
                #     for next_c in range(cols):
                #         next_grid2.append((next_r,next_c))
                #         temp = 0
                #         if ((p_row, p_col), (next_r, next_c)) in self.transProb:
                #             temp = self.transProb[(p_row, p_col), (next_r, next_c)]
                #         prob2.append(temp)
                # for next_r in next_row:
                #     for next_c in next_col:
                #         next_grid.append((next_r,next_c))
                #         temp = 0
                #         if ((p_row,p_col),(next_r,next_c) )in self.transProb:
                #             temp =  self.transProb[(p_row,p_col),(next_r,next_c)]
                #         prob.append(temp)
                #update the mp[n]
                
                next_cell = random.choices(next_grid, weights = prob, k = 1)
                next_cell = next_cell[0] 
                self.pf[n] = next_cell
            #weight
            weight = defaultdict(int)
            for n in range(num_points):
                p_row = self.pf[n][0]
                p_col = self.pf[n][1]
                #mean is the distance between (posX, posY) and (p_row, p_col)
                mean = pow((posY-util.rowToY(p_row))**2+(posX-util.colToX(p_col))**2, 0.5)
                p = pdf(mean, Const.SONAR_STD, observedDist)
                weight[n] = p
            #resample
            new_pf = defaultdict(tuple)
            resample_Arr = []
            resample_prob = []
            for n in range(num_points):
                resample_Arr.append(self.pf[n])
                resample_prob.append(weight[n])
            for n in range(num_points):
                temp = random.choices(resample_Arr, weights = resample_prob, k = 1)
                new_pf[n] = temp[0]
            self.pf = new_pf

            #update the belief
            new_belief = [[0 for _ in range(cols)] for _ in range(rows)]
            for n in range(num_points):
                p_row = self.pf[n][0]
                p_col = self.pf[n][1]
                new_belief[p_row][p_col] += 1
            for i in range(rows):
                for j in range(cols):
                    self.belief.setProb(i,j,new_belief[i][j])
        self.belief.normalize()


        # END_YOUR_CODE
        return
  
    def getBelief(self) -> Belief:
        return self.belief

   