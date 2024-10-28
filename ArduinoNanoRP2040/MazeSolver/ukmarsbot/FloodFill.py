from ApiInterface import ApiInterface

from Logger import Logger
from _Config import Config
from Heading import Heading
from Direction import Direction
class FloodFill:
    def __init__(self,maze,maze_start,maze_goal):
        self.maze = maze
        self.maze_start = maze_start
        self.maze_goal = maze_goal
        self.queue = []
    
    def fill_goal(self,first_pass):
        Logger.log("flood_fill...")    
        self.queue = []
        for x in range(Config.MAZE_WIDTH):
            for y in range(Config.MAZE_HEIGHT):
                ApiInterface.setText(x, y, Config.MAZE_MAX_COST)
                self.maze.cells[x][y].cost = Config.MAZE_MAX_COST

        ApiInterface.setText(self.maze_goal.x, self.maze_goal.y, 0)  
        ApiInterface.setText(self.maze_goal.x-1, self.maze_goal.y, 0)  
        ApiInterface.setText(self.maze_goal.x, self.maze_goal.y-1, 0)  
        ApiInterface.setText(self.maze_goal.x-1, self.maze_goal.y-1, 0)  
        
        self.maze.cells[self.maze_goal.x][self.maze_goal.y].cost = 0    
        #self.maze.cells[self.maze_goal.x-1][self.maze_goal.y].cost = 0    
        #self.maze.cells[self.maze_goal.x][self.maze_goal.y-1].cost = 0    
        #self.maze.cells[self.maze_goal.x-1][self.maze_goal.y-1].cost = 0    

        #Logger.log("self.maze_goal:[%d][%d] / cost:%d"%(self.maze_goal.x,self.maze_goal.y,self.maze_goal[7][7].cost))            

        # set queue first element
        self.queue.append(self.maze.cells[self.maze_goal.x][self.maze_goal.y])
        #self.queue.append(self.maze.cells[self.maze_goal.x-1][self.maze_goal.y])
        #self.queue.append(self.maze.cells[self.maze_goal.x][self.maze_goal.y-1])
        #self.queue.append(self.maze.cells[self.maze_goal.x-1][self.maze_goal.y-1])
        
        while len(self.queue)>0:
            #Logger.log("##########len(queue):%d#############"%len(self.queue))
            current_cell = self.queue.pop(0)
            #Logger.log("current cell:[%d][%d] / cost:%d / Walls:N[%s]-S[%s]-E[%s]-W[%s]"%(current_cell.x,current_cell.y,current_cell.cost,current_cell.walls["NORTH"],current_cell.walls["SOUTH"],current_cell.walls["EAST"],current_cell.walls["WEST"]))            
            new_cost = self.maze.cells[current_cell.x][current_cell.y].cost + 1
            #Logger.log("new_cost:%d"%(new_cost))
            # 
            for heading in Heading.LIST :
                #Logger.log(heading)
                is_exit = False
                if first_pass:
                    is_exit = True
                else:
                    is_exit = (current_cell.is_exit(heading) == 1)
                if is_exit:
                    #Logger.log("==> is_exit")
                    next_cell_coordinates = current_cell.neighbour(heading,Direction.FORWARD)
                    if next_cell_coordinates is not None:
                        next_cell = self.maze.cells[next_cell_coordinates.x][next_cell_coordinates.y]                        
                        #Logger.log("next cell:[%d][%d]-->cost:%d"%(next_cell.x,next_cell.y,next_cell.cost))
                        
                        if next_cell.cost > new_cost :
                            next_cell.cost = new_cost
                            ApiInterface.setText(next_cell.x, next_cell.y, new_cost)
                            self.queue.append(next_cell)
                            #Logger.log("cell added to queue")

    def fill_start(self,first_pass=False):
        #Logger.log("flood fill start...")    
        self.queue = []
        for x in range(Config.MAZE_WIDTH):
            for y in range(Config.MAZE_HEIGHT):
                ApiInterface.setText(x, y, Config.MAZE_MAX_COST)
                self.maze.cells[x][y].cost = Config.MAZE_MAX_COST

        ApiInterface.setText(self.maze_start.x, self.maze_start.y, 0)  
        
        self.maze.cells[self.maze_start.x][self.maze_start.y].cost = 0    

        #Logger.log("self.maze_goal:[%d][%d] / cost:%d"%(self.maze_goal.x,self.maze_goal.y,self.maze_goal[7][7].cost))            

        # set queue first element
        self.queue.append(self.maze.cells[self.maze_start.x][self.maze_start.y])
        
        while len(self.queue)>0:
            #Logger.log("##########len(queue):%d#############"%len(self.queue))
            current_cell = self.queue.pop(0)
            #Logger.log("current cell:[%d][%d] / cost:%d / Walls:N[%s]-S[%s]-E[%s]-W[%s]"%(current_cell.x,current_cell.y,current_cell.cost,current_cell.walls["NORTH"],current_cell.walls["SOUTH"],current_cell.walls["EAST"],current_cell.walls["WEST"]))            
            new_cost = self.maze.cells[current_cell.x][current_cell.y].cost + 1
            #Logger.log("new_cost:%d"%(new_cost))
            # 
            for heading in Heading.LIST :
                #Logger.log(heading)
                is_exit = False
                if first_pass:
                    is_exit = True
                else:
                    is_exit = current_cell.is_exit(heading) == 1
                if is_exit:
                    #Logger.log("==> is_exit")
                    next_cell_coordinates = current_cell.neighbour(heading,Direction.FORWARD)
                    if next_cell_coordinates is not None:
                        next_cell = self.maze.cells[next_cell_coordinates.x][next_cell_coordinates.y]                        
                        #Logger.log("next cell:[%d][%d]-->cost:%d"%(next_cell.x,next_cell.y,next_cell.cost))
                        
                        if next_cell.cost > new_cost :
                            next_cell.cost = new_cost
                            ApiInterface.setText(next_cell.x, next_cell.y, new_cost)
                            self.queue.append(next_cell)
                            #Logger.log("cell added to queue")        