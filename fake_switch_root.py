import sys
import time
   
loop = 20

while(loop > 0):
	
    print("looping ... {0}".format(loop))
    sys.stdout.flush()
    
    time.sleep(1)
    
    loop = loop - 1
    
print("... done")
