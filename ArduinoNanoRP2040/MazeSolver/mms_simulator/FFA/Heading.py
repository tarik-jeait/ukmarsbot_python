from enum import Enum
from Direction import Direction

class Heading(Enum):
	NORTH = "NORTH"
	EAST = "EAST"
	SOUTH = "SOUTH"
	WEST = "WEST"
     
	@staticmethod
	def update(heading,direction):
		if direction == Direction.RIGHT:
			if heading == Heading.NORTH:
				return Heading.EAST
			if heading == Heading.EAST:
				return Heading.SOUTH
			if heading == Heading.SOUTH:
				return Heading.WEST
			if heading == Heading.WEST:
				return Heading.NORTH
		elif direction == Direction.LEFT:
			if heading == Heading.NORTH:
				return Heading.WEST
			if heading == Heading.WEST:
				return Heading.SOUTH
			if heading == Heading.SOUTH:
				return Heading.EAST
			if heading == Heading.EAST:
				return Heading.NORTH