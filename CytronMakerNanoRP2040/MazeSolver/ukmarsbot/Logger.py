import sys
from machine import Pin, UART
class Logger:
    LOG_FILE_NAME = None
    CYTRON_UART = None
    @staticmethod
    def init():
        #pass
        #Logger.CYTRON_UART = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
        sequence = None
        try:
            with open('Logs/sequence.txt', 'r') as f:
                sequence = f.read()
                print("sequence:<%s>"%sequence)
                f.close()
                logs_file_name = "Logs/log_"+sequence+".txt"
                Logger.LOG_FILE_NAME = logs_file_name
                with open(logs_file_name, 'wt') as ff:
                    ff.write("###################################")
                    ff.close()                    
        except:
            print("Could open sequence.txt for read or write log.txt")
        
        try:            
            with open('Logs/sequence.txt', 'w') as fff:
                #print('The file', fff.name(), 'is ready for writing:', fff.writable(), '\n')
                #sequence = int(sequence)+1
                new_sequence = int(sequence) + 1
                print("new_sequence:%d"%new_sequence)
                fff.write(str(new_sequence))
                fff.close()            
        except:
            print("Could not open sequence.txt for update")
        
    def log(string):
        #pass
        #sys.stderr.write("{}\n".format(string))
        #sys.stderr.flush()
#         if Logger.CYTRON_UART:
#             Logger.CYTRON_UART.write("{}\n".format(string))
#         else:
#             Logger.CYTRON_UART = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
#             if Logger.CYTRON_UART:
#                 Logger.CYTRON_UART.write("{}\n".format(string))
        try:
            with open(Logger.LOG_FILE_NAME, 'a') as f:
                f.write("{}\n".format(string))       
        except:
            print("Could not read calibrated data")

            
    
    def logn(string):
#         if Logger.CYTRON_UART:
#             Logger.CYTRON_UART.write("{}".format(string))
#         else:
#             Logger.CYTRON_UART = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
#             if Logger.CYTRON_UART:
#                 Logger.CYTRON_UART.write("{}\n".format(string))
        #pass
        #sys.stderr.write("{}".format(string))
        #sys.stderr.flush()        if Logger.CYTRON_UART:
        try:
            with open(Logger.LOG_FILE_NAME, 'a') as f:
                f.write("{}".format(string))       
        except:
            print("Could not read calibrated data")

#         if Logger.CYTRON_UART:
#             Logger.CYTRON_UART.write("{}".format(string))

#         else:
#             Logger.CYTRON_UART = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
#             if Logger.CYTRON_UART:
#                 Logger.CYTRON_UART.write("{}".format(string))
