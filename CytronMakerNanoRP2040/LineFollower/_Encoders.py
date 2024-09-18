from _Config import Config

class Encoders:
    def __init__(self,board):
        self.board = board
        self.left_oldA = 0
        self.left_oldB = 0
        self.right_oldA = 0
        self.right_oldB = 0
        self.m_left_counter = 0
        self.m_right_counter = 0

    def left_input_change(self,pin):
        newB = self.board.ENCODER_LEFT_B.value()
        newA = self.board.ENCODER_LEFT_CLK.value() ^ newB
        delta = Config.ENCODER_LEFT_POLARITY * ((self.left_oldA ^ newB) - (newA ^ self.left_oldB))
        self.m_left_counter += delta
        self.left_oldA = newA
        self.left_oldB = newB
        #encoders_print()

    def right_input_change(self,pin):
        newB = self.board.ENCODER_RIGHT_B.value()
        newA = self.board.ENCODER_RIGHT_CLK.value() ^ newB
        delta = Config.ENCODER_RIGHT_POLARITY * ((self.right_oldA ^ newB) - (newA ^ self.right_oldB))
        self.m_right_counter += delta
        self.right_oldA = newA
        self.right_oldB = newB
        #encoders_print()

    def print(self):
        print("m_left_counter:%d - m_right_counter:%d"% (self.m_left_counter,self.m_right_counter))

    def init(self):
        self.m_left_counter = 0
        self.m_right_counter = 0