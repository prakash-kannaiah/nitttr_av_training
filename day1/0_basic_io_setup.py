import os
import time
import sys
import math

from qvl.qcar2 import QLabsQCar2
from qvl.system import QLabsSystem
from qvl.real_time import QLabsRealTime
from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera

def setup(
        initialPosition=[0, 0, 0],
        initialOrientation=[0, 0, math.pi],
        cam_location = [-5.266, 12.454, 7.928],
        cam_rotation = [0, 0.473, -1.562]        
    ):

    # Try to connect to Qlabs
    os.system('clear')
    qlabs = QuanserInteractiveLabs()
    # Ensure that QLabs is running on your local machine
    print("Connecting to QLabs...")
    if (not qlabs.open("localhost")):
        print("Unable to connect to QLabs")
        sys.exit()
        return
    print("Connected to QLabs")

    # Delete any previous QCar instances and stop any running spawn models
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()
    time.sleep(0.5)
    # region: QCar spawn description
    
    # Spawn a QCar at the given initial pose
    hqcar = QLabsQCar2(qlabs)
    hqcar.spawn_id(
        actorNumber=0,
        location=[x for x in initialPosition],
        rotation=initialOrientation,
        waitForConfirmation=True
    )
    
    # Create a new camera view and attach it to the QCar
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn(cam_location, cam_rotation)
    hcamera.possess()
    #hqcar.possess()
    # endregion
    
    # Start spawn model
    rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
    QLabsRealTime().start_real_time_model(rtModel)

    return hqcar

if __name__ == '__main__':
    # XXX Add processing of command line arguments
    setup()
