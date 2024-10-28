import sys
class Logger:
    def log(string):
        sys.stderr.write("{}\n".format(string))
        sys.stderr.flush()
    
    def logn(string):
        sys.stderr.write("{}".format(string))
        sys.stderr.flush()        