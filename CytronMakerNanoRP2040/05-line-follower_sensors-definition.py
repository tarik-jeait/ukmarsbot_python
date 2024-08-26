
class Sensors:
    def __init__(self,board):
        self.board = board
        # Sensor Raw Data
        self.leftside = 0
        self.leftfront = 0
        self.rightfront = 0
        self.rightside = 0
        # Sensor Calibrated Data
        leftside_min = 10000
        leftside_max = 0
        
        leftfront_min = 10000
        leftfront_max = 0
        
        rightfront_min = 10000
        rightfront_max = 0
        
        rightside_min = 10000
        rightside_max = 0
        #Normalized values
        self.leftside_normalized = 0
        self.leftfront_normalized = 0
        self.rightfront_normalized = 0
        self.rightside_normalized = 0
    
    def isCrossRoads(self):
        if(self.leftside_normalized<50 and self.rightside_normalized<50):
            return False
        else:
            return True
    
    def isRouteLeft(self):
        if(self.leftside_normalized >= 50):
            return True
        else:
            return False
        
    def isRouteRight(self):
        if(self.rightside_normalized >= 50):
            return True
        else:
            return False
    
    def isTheEnd(self):
        if(self.rightside_normalized >= 50 and self.leftside_normalized >= 50 and self.leftfront_normalized>=50 and self.rightfront_normalized>=50):
            return True
        else:
            return False
    
    def isBlackSurface(self):
        if(self.leftfront_normalized<10 and self.rightfront_normalized<10):
            return True
        else:
            return False


