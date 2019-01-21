#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' 4 functions for reading input from keyboard
{
    getRawInput(), return string
    waitRawInput(time_out), return string or None
    getKeyPress(), return char
    waitKeyPress(time_out), return char or None
}
Tested on Ubuntu
'''

import numpy as np
import sys, select
import evdev, select, time
import multiprocessing

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
    ''' Example:
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
    ''' Example:
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
    print "\nDetected keypress: ", c

    None