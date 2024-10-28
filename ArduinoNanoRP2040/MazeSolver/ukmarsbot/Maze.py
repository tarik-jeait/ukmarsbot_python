from Logger import Logger
from _Config import Config
from Heading import Heading
from Cell import Cell
from WallState import WallState
from Direction import Direction

class Maze:
    def __init__(self,maze_start,maze_goal):
        self.maze_start = maze_start
        self.maze_goal = maze_goal
        # Init Maze
        self.init()

    def init(self):
        Logger.log("###### Maze Init #######")
        # Create Cells
        self.cells = [[Cell(i,j) for j in range(Config.MAZE_HEIGHT)] for i in range(Config.MAZE_WIDTH)]
        
        # Set Walls
        for x in range(Config.MAZE_WIDTH):
            self.cells[x][0].walls["SOUTH"] = WallState.WALL
            self.cells[x][Config.MAZE_HEIGHT - 1].walls["NORTH"] = WallState.WALL
        
        for y in range(Config.MAZE_HEIGHT):
            self.cells[0][y].walls["WEST"] = WallState.WALL
            self.cells[Config.MAZE_WIDTH - 1][y].walls["EAST"] = WallState.WALL

        self.cells[self.maze_start.x][self.maze_start.y].walls["EAST"] = WallState.WALL
        self.cells[self.maze_start.x][self.maze_start.y].walls["WEST"] = WallState.WALL
        self.cells[self.maze_start.x][self.maze_start.y].walls["NORTH"] = WallState.EXIT
        self.cells[self.maze_start.x][self.maze_start.y].walls["SOUTH"] = WallState.WALL
        
        # Set Neighbours
        for x in range(Config.MAZE_WIDTH):
            for y in range(Config.MAZE_HEIGHT):
                # For every cell set neighbours if any
                cell = self.cells[x][y]
                for heading in Heading.LIST:                    
                    cell_neighbour_location = cell.neighbour(heading,Direction.FORWARD)
                    if cell_neighbour_location is not None:
                        cell.neighbours[heading] = self.cells[cell_neighbour_location.x][cell_neighbour_location.y]
                        #Logger.log("cell_neighbour_location is not None")
                    else:
                        #Logger.log("[%d][%d]-heading[%s]:cell_neighbour_location is None"%(cell.x,cell.y,heading.name))
                        pass
        return self

    def get_cell(self,cell_coordinates):
        return self.cells[cell_coordinates.x][cell_coordinates.y]

    def next_move(self,heading,cell,wall_front,wall_left,wall_right):
        # LSRB
        #return self.next_move_LSRB(heading,cell,wall_front,wall_left,wall_right)
        # FFA
        return self.next_move_FFA(heading,cell,wall_front,wall_left,wall_right)

    def next_move_LSRB(self,heading,cell,wall_front,wall_left,wall_right):
        if wall_front and wall_left and wall_right:
            return Direction.BACK
        else:
            if not wall_left:
                return Direction.LEFT
            else:
                if wall_front:
                    return Direction.RIGHT
                else:
                    return Direction.FORWARD

    def get_next_move_return_back_to_start(self,heading,cell,wall_front,wall_left,wall_right):
        
        if wall_front and wall_left and wall_right:
            return Direction.BACK
        else:
            # 1st case : front wall:n
            if not wall_front:                
                # front wall:n | wall_left:y | wall_right:y
                if wall_left and wall_right:
                    return Direction.FORWARD
                else:
                    # front wall:n | wall_left:n | wall_right:y
                    if not wall_left and wall_right:
                        # choose based on cost & visited attributes
                        Logger.log("front wall:n | wall_left:n | wall_right:y")     
                        next_cell_neighbour_front_coordinates = cell.neighbour(heading,Direction.FORWARD)
                        next_cell_neighbour_front = self.get_cell(next_cell_neighbour_front_coordinates)
                        next_cell_neighbour_left_coordinates = cell.neighbour(heading,Direction.LEFT)
                        next_cell_neighbour_left = self.get_cell(next_cell_neighbour_left_coordinates)
                        next_cell_rank = self.select_next_cell_return_to_start(next_cell_neighbour_front,next_cell_neighbour_left)
                        if next_cell_rank == 1:                                
                            return Direction.FORWARD
                        else:
                            if next_cell_rank == 2:
                                return Direction.LEFT
                    else:
                        # front wall:n | wall_left:y | wall_right:n
                        if wall_left and not wall_right:
                            Logger.log("front wall:n | wall_left:y | wall_right:n")
                            # choose based on cost & visited attributes
                            next_cell_neighbour_front_coordinates = cell.neighbour(heading,Direction.FORWARD)
                            next_cell_neighbour_front = self.get_cell(next_cell_neighbour_front_coordinates)                            
                            next_cell_neighbour_right_coordinates = cell.neighbour(heading,Direction.RIGHT)
                            next_cell_neighbour_right = self.get_cell(next_cell_neighbour_right_coordinates)
                            next_cell_rank = self.select_next_cell_return_to_start(next_cell_neighbour_front,next_cell_neighbour_right)
                            if next_cell_rank == 1:                                
                                return Direction.FORWARD
                            else:
                                if next_cell_rank == 2:
                                    return Direction.RIGHT
                        else:
                            # front wall:n | wall_left:n | wall_right:n
                            # choose based on cost & visited attributes
                            return Direction.LEFT
            else:
                # 2nd case : front wall:y
                # front wall:y | wall_left:n | wall_right:y
                if not wall_left and wall_right:
                    # choose based on cost & visited attributes
                    return Direction.LEFT
                else:
                    # front wall:y | wall_left:y | wall_right:n
                    if wall_left and not wall_right:
                        # choose based on cost & visited attributes
                        return Direction.RIGHT
                    else:
                        # front wall:y | wall_left:n | wall_right:n
                        if not wall_left and not wall_right:
                            Logger.log("front wall:y | wall_left:n | wall_right:n")
                            # choose based on cost & visited attributes
                            next_cell_neighbour_left_coordinates = cell.neighbour(heading,Direction.LEFT)
                            next_cell_neighbour_left = self.get_cell(next_cell_neighbour_left_coordinates)                            
                            next_cell_neighbour_right_coordinates = cell.neighbour(heading,Direction.RIGHT)
                            next_cell_neighbour_right = self.get_cell(next_cell_neighbour_right_coordinates)
                            next_cell_rank = self.select_next_cell_return_to_start(next_cell_neighbour_left,next_cell_neighbour_right)
                            if next_cell_rank == 1:                                
                                return Direction.LEFT
                            else:
                                if next_cell_rank == 2:
                                    return Direction.RIGHT

    def next_move_FFA(self,heading,cell,wall_front,wall_left,wall_right):
        if wall_front and wall_left and wall_right:
            return Direction.BACK
        else:
            # 1st case : front wall:n
            if not wall_front:
                # front wall:n | wall_left:y | wall_right:y
                if wall_left and wall_right:
                    return Direction.FORWARD
                else:
                    # front wall:n | wall_left:n | wall_right:y
                    if not wall_left and wall_right:
                        # choose based on cost & visited attributes
                        Logger.log("front wall:n | wall_left:n | wall_right:y")     
                        next_cell_neighbour_front_coordinates = cell.neighbour(heading,Direction.FORWARD)
                        next_cell_neighbour_front = self.get_cell(next_cell_neighbour_front_coordinates)
                        #if next_cell_neighbour_front.visited == 0:
                        #    return Direction.FORWARD
                        next_cell_neighbour_left_coordinates = cell.neighbour(heading,Direction.LEFT)
                        next_cell_neighbour_left = self.get_cell(next_cell_neighbour_left_coordinates)                            
                        #if next_cell_neighbour_left.visited == 0:
                        #    return Direction.LEFT

                        next_cell_rank = self.select_next_cell(next_cell_neighbour_front,next_cell_neighbour_left)
                        if next_cell_rank == 1:                                
                            return Direction.FORWARD
                        else:
                            if next_cell_rank == 2:
                                return Direction.LEFT                            
                        Logger.log("no where to go...!")                        
                    else:
                        # front wall:n | wall_left:y | wall_right:n
                        if wall_left and not wall_right:
                            Logger.log("front wall:n | wall_left:y | wall_right:n")
                            # choose based on cost & visited attributes
                            next_cell_neighbour_front_coordinates = cell.neighbour(heading,Direction.FORWARD)
                            next_cell_neighbour_front = self.get_cell(next_cell_neighbour_front_coordinates)                            
                            next_cell_neighbour_right_coordinates = cell.neighbour(heading,Direction.RIGHT)
                            next_cell_neighbour_right = self.get_cell(next_cell_neighbour_right_coordinates)
                            next_cell_rank = self.select_next_cell(next_cell_neighbour_front,next_cell_neighbour_right)
                            if next_cell_rank == 1:                                
                                return Direction.FORWARD
                            else:
                                if next_cell_rank == 2:
                                    return Direction.RIGHT
                        else:
                            # front wall:n | wall_left:n | wall_right:n
                            # choose based on cost & visited attributes
                            return Direction.LEFT
            else:
                # 2nd case : front wall:y
                # front wall:y | wall_left:n | wall_right:y
                if not wall_left and wall_right:
                    # choose based on cost & visited attributes
                    return Direction.LEFT
                else:
                    # front wall:y | wall_left:y | wall_right:n
                    if wall_left and not wall_right:
                        # choose based on cost & visited attributes
                        return Direction.RIGHT
                    else:
                        # front wall:y | wall_left:n | wall_right:n
                        if not wall_left and not wall_right:
                            Logger.log("front wall:y | wall_left:n | wall_right:n")
                            # choose based on cost & visited attributes
                            next_cell_neighbour_left_coordinates = cell.neighbour(heading,Direction.LEFT)
                            next_cell_neighbour_left = self.get_cell(next_cell_neighbour_left_coordinates)                            
                            next_cell_neighbour_right_coordinates = cell.neighbour(heading,Direction.RIGHT)
                            next_cell_neighbour_right = self.get_cell(next_cell_neighbour_right_coordinates)
                            next_cell_rank = self.select_next_cell(next_cell_neighbour_left,next_cell_neighbour_right)
                            if next_cell_rank == 1:                                
                                return Direction.LEFT
                            else:
                                if next_cell_rank == 2:
                                    return Direction.RIGHT

    def select_next_cell(self,cell1,cell2):
        Logger.log(">select_next_cell:")
        Logger.log(">>cell1:[%d][%d]-visited:%d-cost:%d"%(cell1.x,cell1.y,cell1.visited,cell1.cost))
        Logger.log(">>cell2:[%d][%d]-visited:%d-cost:%d"%(cell2.x,cell2.y,cell2.visited,cell2.cost))
        
        if cell1.visited == 0 and cell2.visited == 0:
            # compare costs
            if cell1.cost >= cell2.cost:
                return 2
            else:
                return 1
        if cell1.visited == 1 and cell2.visited == 1:
            # compare costs
            if cell1.cost >= cell2.cost:
                return 2
            else:
                return 1
        
        if cell1.visited == 0 and cell2.visited == 1:
            return 1

        if cell1.visited == 1 and cell2.visited == 0:
            return 2

    def select_next_cell_return_to_start(self,cell1,cell2):
        Logger.log(">select_next_cell_return_to_start:")
        Logger.log(">>cell1:[%d][%d]-visited:%d-cost:%d"%(cell1.x,cell1.y,cell1.visited,cell1.cost))
        Logger.log(">>cell2:[%d][%d]-visited:%d-cost:%d"%(cell2.x,cell2.y,cell2.visited,cell2.cost))

        if cell1.visited == 0 and cell2.visited == 1:
            return 2

        if cell1.visited == 1 and cell2.visited == 0:
            return 1

        if cell1.visited == 1 and cell2.visited == 1:
            # compare costs
            if cell1.cost >= cell2.cost:
                return 1
            else:
                return 2
        
        if cell1.visited == 0 and cell2.visited == 0:
            # compare costs
            if cell1.cost >= cell2.cost:
                return 2
            else:
                return 1

    def print(self):
        return
        current_cell = self.cells[0][1]
        Logger.log(">> updated current cell:[%d][%d] / cost:%d / Walls:N[%s]-S[%s]-E[%s]-W[%s]"%(current_cell.x,current_cell.y,current_cell.cost,current_cell.walls["NORTH"],current_cell.walls["SOUTH"],current_cell.walls["EAST"],current_cell.walls["WEST"]))
        for y in range(Config.MAZE_HEIGHT):
            for x in range(Config.MAZE_WIDTH):
                Logger.logn("o")
                if self.cells[x][Config.MAZE_HEIGHT-1-y].walls["NORTH"]==WallState.WALL:
                    Logger.logn("---")
                else:
                    Logger.logn("   ")
                Logger.logn("o")
            Logger.log("")
            for x in range(Config.MAZE_WIDTH):
                if self.cells[x][Config.MAZE_HEIGHT-1-y].walls["WEST"]==WallState.WALL:
                    Logger.logn("|")
                else:
                    Logger.logn(" ")
                Logger.logn("   ")
                if self.cells[x][Config.MAZE_HEIGHT-1-y].walls["EAST"]==WallState.WALL:
                    Logger.logn("|")
                else:
                    Logger.logn(" ")
            Logger.log("")
            for x in range(Config.MAZE_WIDTH):
                Logger.logn("o")
                if self.cells[x][Config.MAZE_HEIGHT-1-y].walls["SOUTH"]==WallState.WALL:
                    Logger.logn("---")
                else:
                    Logger.logn("   ")
                Logger.logn("o")
            Logger.log("")

