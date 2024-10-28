import API

from Cell import Cell

from WallState import WallState
from Config import Config
from Heading import Heading
from FloodFill import FloodFill
from Maze import Maze
from Logger import Logger
from Direction import Direction

##########################################################################################
#GLOBAL DEFINITIONS
global maze,flood_fill,maze_goal,maze_start

Config.MAZE_WIDTH = API.mazeWidth()
Config.MAZE_HEIGHT = API.mazeHeight()
#Config.MAZE_GOAL = Cell(3,2)
Config.MAZE_GOAL = Cell(8,8)
Config.MAZE_START = Cell(0,0)
Config.MAZE_CELL_COUNT = Config.MAZE_WIDTH * Config.MAZE_HEIGHT
Config.MAZE_MAX_COST = Config.MAZE_CELL_COUNT - 1

maze_goal = Config.MAZE_GOAL
maze_start = Config.MAZE_START

maze = Maze(maze_start,maze_goal)
flood_fill = FloodFill(maze,maze_start,maze_goal)

##########################################################################################

def main():
    Logger.log("Running...")
    flood_fill.fill_goal(first_pass=True)
    API.setColor(0, 0, "G")
    API.setText(0, 0, "S")
    current_cell = maze.cells[maze_start.x][maze_start.y]
    current_heading = Heading.NORTH
    Logger.log("current cell:[%d][%d]"%(current_cell.x,current_cell.y))
    goal_reached = False
    start_filled = False
    start_reached = False
    second_pass = False
    while True:
        Logger.log("################### LOOP #########################")
        Logger.log(">>current cell:[%d][%d]-visited:%d-cost:%d-WF:%d-WL:%d-WR:%d"%(current_cell.x,current_cell.y,current_cell.visited,current_cell.cost,API.wallFront(),API.wallLeft(),API.wallRight()))
        Logger.log(">>current cell:[%d][%d] / cost:[%d]"%(current_cell.x,current_cell.y,current_cell.cost))
        Logger.log(">>current heading:%s"%current_heading.name)
        current_cell.update_wall_states(current_heading,API.wallFront(),API.wallLeft(),API.wallRight())
        Logger.log(">> updated current cell:[%d][%d] / cost:%d / Walls:N[%s]-S[%s]-E[%s]-W[%s]"%(current_cell.x,current_cell.y,current_cell.cost,current_cell.walls["NORTH"].name,current_cell.walls["SOUTH"].name,current_cell.walls["EAST"].name,current_cell.walls["WEST"].name))            
        maze.print()
        
        if not goal_reached:
            Logger.log(">>##Goal not reached")
            flood_fill.fill_goal(first_pass=False)
            
            if current_cell.x == maze_goal.x and current_cell.y == maze_goal.y and second_pass:
                Logger.log(">>Goal reached")
                exit()
            elif current_cell.x == maze_goal.x and current_cell.y == maze_goal.y and not start_filled:
                Logger.log(">>Goal reached")
                goal_reached=True
                #flood_fill.fill_goal(first_pass=False)
                maze.print()
                #exit()
                #flood_fill.fill_goal(first_pass=False)
                flood_fill.fill_start()
                start_filled = True
            next_move = maze.next_move(current_heading,current_cell,API.wallFront(),API.wallLeft(),API.wallRight())
        else:
            Logger.log(">>##Start not reached")
            if current_cell.x == maze_start.x and current_cell.y == maze_start.y:
                Logger.log(">>##Start reached --> go to the goal")
                start_reached = True
                second_pass = True
                goal_reached = False
                flood_fill.fill_goal(first_pass=False)
                next_move = maze.next_move(current_heading,current_cell,API.wallFront(),API.wallLeft(),API.wallRight())
            else:
                flood_fill.fill_start()
                #next_move = maze.get_next_move_return_back_to_start(current_heading,current_cell,API.wallFront(),API.wallLeft(),API.wallRight())
                next_move = maze.next_move(current_heading,current_cell,API.wallFront(),API.wallLeft(),API.wallRight())
        
        if next_move == Direction.LEFT:
            API.turnLeft()
            current_heading =  Heading.update(current_heading,Direction.LEFT)
            API.moveForward()
            current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
            current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
        else:
            if next_move == Direction.RIGHT:
                API.turnRight()
                current_heading =  Heading.update(current_heading,Direction.RIGHT)
                API.moveForward()
                current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
                current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
            else:
                if next_move == Direction.FORWARD:
                    API.moveForward()
                    current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
                    current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
                else:
                    if next_move == Direction.BACK:
                        API.turnRight()
                        current_heading =  Heading.update(current_heading,Direction.RIGHT)
                        API.turnRight()
                        current_heading =  Heading.update(current_heading,Direction.RIGHT)

if __name__ == "__main__":
    main()
