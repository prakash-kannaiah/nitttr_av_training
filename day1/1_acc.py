
import cv2
import numpy as np
import time
from pal.products.qcar import QCarRealSense, QCar

#------------- qcar_cmd -------------
throttle = 0.3
steering = 0

#------------- acc_param -------------
stop_distance = 33
nominal_distance = 82

#------------- functions -------------
def acquire_images(myCam):
    myCam.read_RGB()
    rgb = myCam.imageBufferRGB
    myCam.read_depth()
    depth = myCam.imageBufferDepthPX
    return rgb, depth

def ranging(depth):
    if depth is None:
        return None, None
    d = np.squeeze(depth)
    h, w = d.shape
    #uy, ly = h // 3, (2 * h) // 3
    uy, ly = 261, 262
    lx, rx = w // 3, (2 * w) // 3
    crop = d[uy:ly, lx:rx]
    obj_dis = np.min(crop)
    return crop, obj_dis

def acc(obj_dis, throttle):

    if obj_dis is None:
        return throttle

    # ----- AEB (Emergency Braking) -----
    #'''
    if obj_dis <= stop_distance:
        return 0.0
    #'''

    # ----- ACC (Adaptive Cruise Control) -----
    frac = 1.0  # default fraction
    '''
    if stop_distance < obj_dis < nominal_distance:
        frac = (obj_dis - stop_distance) / (nominal_distance - stop_distance)
    #'''

    return throttle * frac

#------------- main -------------
myCam = QCarRealSense(mode='RGB, Depth')
myCar = QCar(readMode=1, frequency=10)

try:
    while True:
        rgb, depth = acquire_images(myCam)
        roi, obj_dis = ranging(depth)
        v_cmd = acc(obj_dis, throttle)

        myCar.write(v_cmd, steering)

        print("distane:", obj_dis, "speed:", v_cmd)
        time.sleep(0.1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


except KeyboardInterrupt:
    print("\nProgram stopped by user (CTRL+C).")
myCar.write(0, 0)
cv2.destroyAllWindows()
