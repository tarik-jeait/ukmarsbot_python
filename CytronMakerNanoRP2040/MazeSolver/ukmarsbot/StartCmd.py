import time

class StartCmd:
    def __init__(self,board,sensors):
        self.board = board
        self.sensors = sensors
        
    def waitForUserStart(self):
        user_start = False
        card_button = 1
        
        while not user_start:
            self.sensors.readSensorsWallFollower()            
            if self.sensors.front > 5000:
                user_start = True
                return "Start"
            #elif self.sensors.leftside > 10000:
            #    return "LEFT"
            #elif self.sensors.rightside > 10000:
            #    return "RIGHT"
            else:
                card_button = self.board.btn1.value()
                if(card_button == 0):
                    user_start = True
                    return "Calibrate"
                else:
                    time.sleep(0.1)