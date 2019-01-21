
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
   
def getRawInput():
    return raw_input()

def waitRawInput(time_out):
    # import sys, select
    # See this github: https://gist.github.com/atupal/5865237
    i, o, e = select.select( [sys.stdin], [], [], time_out )
    input = None
    if (i):
        input = sys.stdin.readline().strip()
    return input
    
def getKeyPress():
    ''' example:
    import evdev, select, time
    for i in range(10):
        key = getKeyPress()
        if(key=='q'):
            break
        else:
            print key
        time.sleep(0.2)
    '''
    dev = evdev.InputDevice('/dev/input/event4')
    mapping = {
        16:'q',17:'w',18:'e',19:'r',20:'t',21:'y',22:'u',23:'i',24:'o',25:'p',
        30:'a',31:'s',32:'d',33:'f',34:'g',35:'h',36:'j',37:'k',38:'l',
        44:'z',45:'x',46:'c',47:'v',48:'b',49:'n',50:'m',
        } # not completed yet
    while True:
        select.select([dev], [], [])
        for event in dev.read():
            KEY_PRESS_INTERVAL=0.01 # remove the keyrelease
            time.sleep(KEY_PRESS_INTERVAL)
            while dev.read_one() != None:
                pass
            val = [event.code, event.value][1]
            if val in mapping:
                return mapping[val]
            else:
                return val
    return None

def waitKeyPress(time_out=1.0):
    ''' example:
    import multiprocessing
    c = waitKeyPress(time_out=2.0)
    print c
    '''
    key=multiprocessing.Value("c", " ") # c for char, i for int, d for float
    # About setting string instead of char:
    #   https://stackoverflow.com/questions/21290960/how-to-share-a-string-amongst-multiple-processes-using-managers-in-python
    def _getKeyPress(key):
        key.value=getKeyPress()
    p=multiprocessing.Process(target=_getKeyPress,args=(key,))
    p.start()
    p.join(time_out) # Wait for at most time_out seconds 
    if p.is_alive(): # If thread is still active
        p.terminate()
        p.join() # wait for terminate() finishes
        return None
    else:
        return key.value

if __name__=="__main__":

    c = waitKeyPress(time_out=2.0)
    print c

    None
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