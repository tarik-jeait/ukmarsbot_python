from ApiInterface import ApiInterface

from Location import Location
from WallState import WallState
from Heading import Heading
from Config import Config
from Logger import Logger
from Direction import Direction

class Cell:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.walls = {"NORTH":WallState.UNKNOWN,
                      "EAST":WallState.UNKNOWN,
                      "WEST":WallState.UNKNOWN,
                      "SOUTH":WallState.UNKNOWN
                      }
        self.visited = 0
        self.cost = Config.MAZE_MAX_COST
        self.neighbours = {"NORTH":None,
                           "EAST":None,
                           "WEST":None,
                           "SOUTH":None
                           }

    def neighbour(self,heading,direction):
        #Logger.log("cell [%d][%d] neighbour:%s-%s"%(self.x,self.y,heading.name,direction.name))
        if direction == Direction.FORWARD:
            return self.neighbour_front(heading)
        elif direction == Direction.LEFT:
            return self.neighbour_left(heading)
        elif direction == Direction.RIGHT:
            return self.neighbour_right(heading)

    def neighbour_front(self,heading):
        #Logger.log("neighbour_front")               
        if heading == Heading.NORTH:
            if self.y >= Config.MAZE_HEIGHT-1: 
                return None
            else: 
                return Location(self.x, self.y + 1)            
        if heading == Heading.EAST:
            if self.x >= Config.MAZE_WIDTH - 1:
                return None
            else:
                return Location(self.x+1, self.y)            
        if heading == Heading.SOUTH:
            if self.y <= 0:
                return None
            else:
                return Location(self.x, self.y - 1)        
        if heading == Heading.WEST:
            if self.x<=0:
                return None
            else:
                return Location(self.x - 1, self.y)
    
    def neighbour_left(self,heading):
        if heading == Heading.NORTH:
            if self.x<=0:
                return None
            else:
                return Location(self.x - 1, self.y)
        if heading == Heading.EAST:
            if self.y >= Config.MAZE_HEIGHT-1: 
                return None
            else: 
                return Location(self.x, self.y + 1)            
        if heading == Heading.SOUTH:
            if self.x >= Config.MAZE_WIDTH - 1:
                return None
            else:
                return Location(self.x+1, self.y)          
        if heading == Heading.WEST:
            if self.y <= 0:
                return None
            else:
                return Location(self.x, self.y - 1)

    def neighbour_right(self,heading):
        if heading == Heading.NORTH:
            if self.x >= Config.MAZE_WIDTH - 1:
                return None
            else:
                return Location(self.x+1, self.y)
        if heading == Heading.EAST:
            if self.y <= 0:
                return None
            else:
                return Location(self.x, self.y - 1)
        if heading == Heading.SOUTH:
            if self.x<=0:
                return None
            else:
                return Location(self.x - 1, self.y)
        if heading == Heading.WEST:
            if self.y >= Config.MAZE_HEIGHT-1: 
                return None
            else: 
                return Location(self.x, self.y + 1)
        
    def is_exit(self,heading):
        if self.walls[heading.name] == WallState.EXIT or self.walls[heading.name] == WallState.UNKNOWN:
            return 1
        else:
            return 0

    def set_wall_state(self,heading,state):
        Logger.log("set_wall_state:[%d][%d]:%s-%s-current state:%s"%(self.x,self.y,heading.name,state.name,self.walls[heading.name].name))

        if heading == Heading.NORTH:
            if self.walls["NORTH"] == WallState.UNKNOWN:
                Logger.log("state == WallState.UNKNOWN")
                self.walls["NORTH"] = state
            else:
                Logger.log("state != WallState.UNKNOWN")

            ###################
            # UPDATE NORTH CELL
            ###################
            cell_neighbour = self.neighbours["NORTH"]
            if cell_neighbour is not None and cell_neighbour.walls["SOUTH"]==WallState.UNKNOWN:
                cell_neighbour.walls["SOUTH"] = state                                    
        elif heading == Heading.WEST:
            if self.walls["WEST"] == WallState.UNKNOWN :
                self.walls["WEST"] = state
            ###################
            # UPDATE WEST CELL
            ###################
            cell_neighbour = self.neighbours["WEST"] 
            if cell_neighbour is not None and cell_neighbour.walls["EAST"] == WallState.UNKNOWN :
                cell_neighbour.walls["EAST"] = state
        elif heading == Heading.EAST:
            if self.walls["EAST"] == WallState.UNKNOWN:
                Logger.log("state == WallState.UNKNOWN")
                self.walls["EAST"] = state
            else:
                Logger.log("state != WallState.UNKNOWN")
            ###################
            # UPDATE EAST CELL
            ###################
            cell_neighbour = self.neighbours["EAST"]
            #Logger.log("east cell_neighbour [%d][%d] - West Wall:%s"%(cell_neighbour.x,cell_neighbour.y,cell_neighbour.walls["WEST"]))
            if cell_neighbour and cell_neighbour.walls["WEST"] == WallState.UNKNOWN :
                #Logger.log("cell_neighbour is not None and WEST wall is Unknown")
                cell_neighbour.walls["WEST"] = state
            else:
                #Logger.log("cell_neighbour is None or WEST wall is not Unknown")
                pass
        elif heading == Heading.SOUTH:
            if self.walls["SOUTH"] == WallState.UNKNOWN:
                self.walls["SOUTH"] = state
            ###################
            # UPDATE SOUTH CELL
            ###################
            cell_neighbour = self.neighbours["SOUTH"]
            if cell_neighbour is not None and cell_neighbour.walls["NORTH"] == WallState.UNKNOWN :
                cell_neighbour.walls["NORTH"] = state                                    

    #############################################################################################################
    # TODO : to be completed
    #############################################################################################################
    def update_wall_states(self,heading,wall_front,wall_left,wall_right):
        self.visited = 1
        if heading == Heading.NORTH:
            if wall_front:
                #Logger.log(str("["+str(self.x)+"]["+str(self.y)+"] / ["+str(heading)+"]==> Wall Front"))
                self.set_wall_state(Heading.NORTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'n')
            else:
                self.set_wall_state(Heading.NORTH,WallState.EXIT)
            if wall_left:
                self.set_wall_state(Heading.WEST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'w')
            else:
                self.set_wall_state(Heading.WEST,WallState.EXIT)
            if wall_right:
                self.set_wall_state(Heading.EAST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'e')
            else:
                self.set_wall_state(Heading.EAST,WallState.EXIT)
            
            self.set_wall_state(Heading.SOUTH,WallState.EXIT)

        if heading == Heading.EAST:
            if wall_front:
                self.set_wall_state(Heading.EAST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'e')
            else:
                self.set_wall_state(Heading.EAST,WallState.EXIT)
            
            if wall_left:
                self.set_wall_state(Heading.NORTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'n')
            else:
                self.set_wall_state(Heading.NORTH,WallState.EXIT)
                
            if wall_right:
                self.set_wall_state(Heading.SOUTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'s')
            else:
                self.set_wall_state(Heading.SOUTH,WallState.EXIT)    
            
            self.set_wall_state(Heading.WEST,WallState.EXIT)

        if heading == Heading.WEST:
            if wall_front:
                self.set_wall_state(Heading.WEST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'w')
            else:
                self.set_wall_state(Heading.WEST,WallState.EXIT)
            
            if wall_left:
                self.set_wall_state(Heading.SOUTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'s')
            else:
                self.set_wall_state(Heading.SOUTH,WallState.EXIT)

            if wall_right:
                self.set_wall_state(Heading.NORTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'n')
            else:
                self.set_wall_state(Heading.NORTH,WallState.EXIT)

            self.set_wall_state(Heading.EAST,WallState.EXIT)

        if heading == Heading.SOUTH:
            if ApiInterface.wallFront():
                self.set_wall_state(Heading.SOUTH,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'s')
            else:
                self.set_wall_state(Heading.SOUTH,WallState.EXIT)
            
            if ApiInterface.wallLeft():
                self.set_wall_state(Heading.EAST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'e')
            else:
                self.set_wall_state(Heading.EAST,WallState.EXIT)
            
            if ApiInterface.wallRight():
                self.set_wall_state(Heading.WEST,WallState.WALL)
                ApiInterface.setWall(self.x,self.y,'w')
            else:
                self.set_wall_state(Heading.WEST,WallState.EXIT)
            
            self.set_wall_state(Heading.NORTH,WallState.EXIT)
