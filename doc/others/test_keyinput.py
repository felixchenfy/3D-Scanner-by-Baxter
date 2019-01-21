
# See this github: https://gist.github.com/atupal/5865237
if 1:
    import sys, select

    print "You have ten seconds to answer!"

    i, o, e = select.select( [sys.stdin], [], [], 10 )

    if (i):
        print "You said", sys.stdin.readline().strip()
    else:
        print "You said nothing!"
    
elif 0: # error when using raw_input in thread
    import multiprocessing
    import time

    # xx = raw_input() 
    # print xx

    def getKeyPress(time_out=1.0):
        # modified from: https://stackoverflow.com/questions/492519/timeout-on-a-function-call
        # class KeyPress(object):
        #     def __init__(self):
        #         self.key_value=None
        #     def waitKeyPress(self):
        #         # self.key_value = raw_input()
        #         c = raw_input()
        #         print c
        # keypress = KeyPress()
        # p = multiprocessing.Process(target=keypress.waitKeyPress)
        key=[None]
        def waitKeyPress():
            key[0] = raw_input()
        p = multiprocessing.Process(target=waitKeyPress)
        p.start()
        p.join(time_out) # Wait for at most time_out seconds 
        if p.is_alive(): # If thread is still active
            p.terminate()
            p.join() # wait for terminate() finishes
        # return keypress.key_value
        return key[0]
    
    c = getKeyPress(time_out=1.0)
    print c

elif 0: # not working
    import sys
    from select import select

    print "Press any key to configure or wait 5 seconds..."
    timeout = 5
    rlist, wlist, xlist = select([sys.stdin], [], [], timeout)

    if rlist:
        print "Config selected..."
        print rlist
        print wlist
        print xlist
        
    else:
        print "Timed out..."
elif 0: # not working
    import threading
    import time
    import sys
    t = threading.Thread(target=sys.stdin.read(1), args=(1,))
    t.start()
    time.sleep(5)
    t.join()

elif 0: # not working. Only for UNIX???
    import signal

    def input_or_timeout(timeout):
        def nothing(sig, frame): pass
        signal.signal(signal.SIGALRM, nothing)
        signal.alarm(timeout)
        try:
            c = raw_input()
            signal.alarm(0)
            return c
        except (IOError, EOFError):
            return None
    
    c = input_or_timeout(1)
    print c
elif 0: # not working
    import cv2
    import sys
    sys.stdin.flush()
    c=cv2.waitKey(10000)
    c=cv2.waitKey(10000)
    print c
    print "end"

elif 0: # only keyinput. no time delaying
    from evdev import InputDevice
    from select import select
    from enum import Enum
    from time import sleep
    import sys
    sys.stdin.flush() # not working
    def detectInputKey():
        dev = InputDevice('/dev/input/event4')
        while True:
            select([dev], [], [])
            for event in dev.read():
                res=[event.code, event.value]
                return res 

    c = detectInputKey()
    # sleep(0.01)
    c = detectInputKey()
    print c