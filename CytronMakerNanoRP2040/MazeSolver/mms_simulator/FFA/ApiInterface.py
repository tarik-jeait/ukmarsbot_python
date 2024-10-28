import API

class ApiInterface:

    def mazeWidth():
        return API.mazeWidth()
    def mazeHeight():
        return API.mazeHeight()
    def wallFront():
        return API.wallFront()
    def wallBack():
        return API.wallBack()
    def wallLeft():
        return API.wallLeft()
    def wallRight():
        return API.wallRight()
    def wallFrontLeft():
        return API.wallFrontLeft()
    def wallFrontRight():
        return API.wallFrontRight()
    def wallBackLeft():
        return API.wallBackLeft()
    def wallBackRight():
        return API.wallBackRight()
    def moveForward(distance=None):
        return API.moveForward(distance)    
    def moveForwardHalf():
        return API.moveForwardHalf()
    def turnRight():
        return API.turnRight()
    def turnLeft():
        return API.turnLeft()
    def turnRight90():
        return API.turnRight90()
    def turnLeft90():
        return API.turnLeft90()
    def turnRight45():
        return API.turnRight45()
    def turnLeft45():
        return API.turnLeft45()
    def setWall(x, y, direction):
        return API.setWall(x, y, direction)
    def clearWall(x, y, direction):
        return API.clearWall(x, y, direction)
    def setColor(x, y, color):
        return API.setColor(x, y, color)
    def clearColor(x, y):
        return API.clearColor(x,y)
    def clearAllColor():
        return API.clearAllColor()
    def setText(x, y, text):
        return API.setText(x, y, text)
    def clearText(x, y):
        return API.clearText(x,y)
    def clearAllText():
        return API.clearAllText()
    def wasReset():
        return API.wasReset()
    def ackReset():
        return API.ackReset()