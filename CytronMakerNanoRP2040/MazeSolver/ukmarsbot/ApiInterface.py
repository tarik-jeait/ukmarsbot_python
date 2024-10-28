
class ApiInterface:

    def mazeWidth():
        return 0
    def mazeHeight():
        return 0
    def wallFront():
        return False
    def wallBack():
        return False
    def wallLeft():
        return False
    def wallRight():
        return False
    def wallFrontLeft():
        return False
    def wallFrontRight():
        return False
    def wallBackLeft():
        return False
    def wallBackRight():
        return False
    def moveForward(distance=None):
        return False  
    def moveForwardHalf():
        return False
    def turnRight():
        return False
    def turnLeft():
        return False
    def turnRight90():
        return False
    def turnLeft90():
        return False
    def turnRight45():
        return False
    def turnLeft45():
        return False
    def setWall(x, y, direction):
        return False
    def clearWall(x, y, direction):
        return False
    def setColor(x, y, color):
        return False
    def clearColor(x, y):
        return False
    def clearAllColor():
        return False
    def setText(x, y, text):
        return False
    def clearText(x, y):
        return False
    def clearAllText():
        return False
    def wasReset():
        return False
    def ackReset():
        return False