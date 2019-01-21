
import numpy as np
import sys, select
import evdev, select, time

def list2str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

def waitRawInput(time_out):
    # import sys, select
    # See this github: https://gist.github.com/atupal/5865237
    i, o, e = select.select( [sys.stdin], [], [], time_out )
    input = None
    if (i):
        input = sys.stdin.readline().strip()
    return input

def getKeypress():
    ''' example:
    import evdev, select, time
    for i in range(10):
        key = getKeypress()
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



if __name__=="__main__":

    None