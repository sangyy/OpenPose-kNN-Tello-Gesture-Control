import sys
import traceback
import threading
import tellopy
import av
#import cv2.cv2 as cv2 
import cv2 # for avoidance of pylint error
import numpy
from numpy import array
import os
from time import sleep
import kNNtest
from sklearn.preprocessing import normalize

##code for cvx optimazation
# import CVX_Trajectory as tra_cvx
# path = tra_cvx.trajectory_search()*300


##code for tello basic go function
# plist=[[30,30,30],[30,70,30],[70,70,30],[70,70,70],[30,70,70],[30,70,30],[30,30,30]]
# path = array(plist)
# print('this is the path')
# print(path)


sys.path.append('../../../python')
dir_path = os.path.dirname(os.path.realpath(__file__))

try:
    from openpose import openpose as op
    # from openpose import *
except:
    raise Exception(
        'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')

params = dict()
params["logging_level"] = 3
params["output_resolution"] = "-1x-1"
params["net_resolution"] = "-1x128"
params["model_pose"] = "BODY_25"
params["alpha_pose"] = 0.6
params["scale_gap"] = 0.3
params["scale_number"] = 1
params["render_threshold"] = 0.05
# If GPU version is built, and multiple GPUs are available, set the ID here
params["num_gpu_start"] = 0
params["disable_blending"] = False
# Ensure you point to the correct path where models are located
params["default_model_folder"] = dir_path + "/../../../../models/"
# Construct OpenPose object allocates GPU memory
openpose = op.OpenPose(params)


# this function is used to [test] control tello move in sequential commonns
# [dr.niu] knows whether it is functioning well
'''
def move_to_curve(drone, x, y, z):
    drone.forward(int(x))
    sleep(2)
    drone.down(int(y))
    sleep(2)
    drone.left(int(z))
'''
'''
# function used to control tello to do justure
# according to the idx output of knn
def idx2pose(pastidx):
    if pastidx == 0:    # raise the left arm, lateral raise the right arm
        drone.right(20)
    elif pastidx == 1:  # lateral raise the right arm
        drone.land()
    elif pastidx == 2:  # lateral raise the left arm
        drone.takeoff()
    elif pastidx == 3:  # raise the right arm , lateral raise the left arm
        drone.left(20)
    elif pastidx == 4:  # both arm raised as v
        drone.flip_right()
        # drone.up(20)
    elif pastidx == 5:  # lateral raise both arms
        drone.backward(20)
        # drone.go(50,30,30)

    #         sleep(15)
    #         for i in range(1,numpy.size(path,0)):
    #             x = path[i,0] - path[i-1,0]
    #             y = path[i,1] - path[i-1,1]
    #             z = path[i,2] - path[i-1,2]
    #             print('start to go')
    #             drone.go(x,y,z)
    #             print(x, y, z)
    #             sleep(3)
    #             move_to_curve(drone,x,y,z)

    elif pastidx == 6:  # raise both arms as =
        drone.down(20)
'''

def idx2pose(pastidx):
    if pastidx == 0:    # raise the left arm, lateral raise the right arm
        print('drone.right(20)')
    elif pastidx == 1:  # lateral raise the right arm
        print('drone.land()')
    elif pastidx == 2:  # lateral raise the left arm
        print('drone.takeoff()')
    elif pastidx == 3:  # raise the right arm , lateral raise the left arm
        print('drone.left(20)')
    elif pastidx == 4:  # both arm raised as v
        #print('drone.flip_right()')
        print('drone.up(20)')
        # drone.up(20)
    elif pastidx == 5:  # lateral raise both arms
        print('drone.backward(20)')
        # drone.go(50,30,30)

    #         sleep(15)
    #         for i in range(1,numpy.size(path,0)):
    #             x = path[i,0] - path[i-1,0]
    #             y = path[i,1] - path[i-1,1]
    #             z = path[i,2] - path[i-1,2]
    #             print('start to go')
    #             drone.go(x,y,z)
    #             print(x, y, z)
    #             sleep(3)
    #             move_to_curve(drone,x,y,z)

    elif pastidx == 6:  # raise both arms like |_o_|
        print('drone.down(20)')

def main():
    #drone = tellopy.Tello()

    try:
        #drone.connect()
        #drone.wait_for_connection(60.0)
        
        # drone.startControlCommand()
        # drone.takeoff()
        # drone.takeoff()
        # sleep(3)
        # drone.land()
        # sleep(3)
        #drone.set_video_encoder_rate(1)
        # Open webcam on OS X.
        #container = av.open(format='avfoundation', file='0') 
        #container = av.open(drone.get_video_stream())
        container = av.open('sangyy4.mp4')
        #container.VideoFrame(320, 180, 'rgb24')
        #container.width = 320
        #container.height = 180
        '''
        
         -camera (The camera index for cv::VideoCapture. Integer in the range [0,
      9]. Select a negative number (by default), to auto-detect and open the
      first available camera.) type: int32 default: -1
    -camera_fps (Frame rate for the webcam (also used when saving video). Set
      this value to the minimum value between the OpenPose displayed speed and
      the webcam real frame rate.) type: double default: 30
    -camera_parameter_folder (String with the folder where the camera
      parameters are located.) type: string
      default: "models/cameraParameters/flir/"
    -camera_resolution (Set the camera resolution (either `--camera` or
      `--flir_camera`). `-1x-1` will use the default 1280x720 for `--camera`,
      or the maximum flir camera resolution available for `--flir_camera`)
      type: string default: "-1x-1"

        
        '''
        print('Start Video Stream**********************************')
        # skip first 10 frames

        frame_skip = 20
        #count_frame = 10
        flags = numpy.zeros((1,4))
        pastidx = None  # a var to store info of indx, used in one person ver. to make same movement
        actor = None  # a var to identify which user gets the control of tello

        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip-1
                    continue
                # start_time = time.time()
                interupt = cv2.waitKey(10)  # 10s to read keys? roll call?
                #frame(320, 180, 'rgb24')
                #frame = frame(320, 180)
                #frame = frame.reformat(320, -1, 'rgb24')
                image = cv2.cvtColor(numpy.array(frame.reformat(272, 480).to_image()), cv2.COLOR_RGB2BGR)
                #image = cv2.resize(image, (640, 360));
                keypoints, output_image = openpose.forward(image, True)
                # keypoints is a matrix filled in multi-person data
                # format:[[p1][p2][p3]]

                cv2.imshow("output", output_image)
                # print('get keypoint!')
                # print(keypoints)

                # once test input is not the 7 poses, return idx=6 ?
                # is it because the dist_all<0.7 is too general?
                # solution: modify idx when dist_all>0.7 (multi-person version)

                # for one person data matrix, size=3*25=75
                if 40 < numpy.size(keypoints) < 76:  # ensure that it is a valid data for one person
                    # implement knn
                    (idx, dist_all) = kNNtest.implement_kNN(keypoints)
                    print('One-Person mode')
                    actor = 0
                    # set actor as 0 in one person ver., if next frame is multi-person, we don't know who gets the control
                    # this setting is due to actor cannot be none for logic comparasion in the multi-person actor change stage

                    # print(dist_all)
                    if dist_all[0] < 0.7:
                        print('*****       Pose_Idx=%d       *****' % idx)

                        # if the idx is not the same, change idx
                        # if the idx is the same, do the same movement as the past idx indicates
                        # if the idx of the pose cannot be recgonized, the drone will still move as the pastidx (save energy for actor)
                        if idx != pastidx:
                            pastidx = idx
                            print('pose idx has changed to %d'%(idx))
                        idx2pose(pastidx)


                # for multi-person data matrices, size=n*(3*25)
                if numpy.size(keypoints) > 76:
                    print('multi-person mode')
                    person = len(keypoints) # a var used in person number changed between frames
                    idx_list = []           # a list to store idx of all the person in one frame

                    kp = dict()
                    # apply knn to all the person matrices one by one
                    for i in range(0, len(keypoints)):
                        a = []
                        a.append(keypoints[i])
                        print('seperate kp')
                        name = 'kp{}'.format(i)
                        kp[name] = array(a)

                        # ensure the points are enough for analysis
                        if 40 < numpy.size(kp[name]) < 76:
                            (idx, dist_all) = kNNtest.implement_kNN(kp[name])
                            print('idx, dist done')

                            # if the pose of the person cannot be matched with poseidx 0-6, then idx = none
                            if dist_all[0] > 0.7:
                                idx = None

                            # store the idx only for matrices with enough points
                            idx_list += [idx]

                    print('index list of multi-person:')
                    print(idx_list)

                    # this part is the assignment of actor in multi-person mode
                    # in one person mode, actor = 0 as default

                    # this part is the situation when the plane has not been taken off yet, so actor = none
                    if actor == None:
                        print('Actor is None in multi-person mode')
                        # the person who let the plane take off is assigned as the actor
                        if 2 in idx_list:
                            actor = idx_list.index(2)
                            idx = idx_list[actor]
                            print('take off in multi-person mode by actor:', actor)
                            idx2pose(idx)
                            print('take off in multi-person mode done')

                    # this part is entered when the plane:
                    # 1/ takes off in multi-person mode
                    # 2/ takes off in one-person mode
                    elif actor != None:
                        print('Actor is not None')
                        # what if in the first frame, person=4, and actor=4 (list index=3)
                        # in the next frame, person = 3, actor idx does not changed, list = [0,1,2] act = 3, out of range
                        # base stage:
                        # if person = 3, p-1 = 2, actor = 2 is out of range
                        if person >= 3:
                            if actor >= (person-1):
                                actor = 0
                                print('actor overflow, changed to 0')
                                # actor = 0 is still dangerous, need to be tested
                        if 4 in idx_list:
                            actor = idx_list.index(4)
                            # actor is set to be the first idx of 4 in the list(due to function ofo index)
                            # need to improve: ensure which is should be the actor
                            # why the list is full of 4? is it a bug?
                            print('actor has changed to the person who did pose 4')

                        print('ready to get the idx in [multi-person] actor mode')
                        idx = idx_list[actor]
                        print('actor has set the idx to:',idx)
                        idx2pose(idx)
                        print('actor is :',actor,'pose is:',idx)

                    # print('ready to do pose!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    # idx2pose(idx)
                    # print('do action in multi-person!!!!!!!!!!!!!!!!!!!!!!!')


                elif interupt & 0xFF == ord('q'):
                    cv2.destroyWindow(output_image)
                    print('drone.land()')

                elif numpy.size(keypoints)== 0: ##if UAV can't find any person,turn around until detect one person
                    print('drone.clockwise(20)')

                    # drone.quit()
                    # sleep(1)
                # if interupt & 0xFF == ord('l'):
                #     drone.land()
                #     sleep(2)
                # if interupt & 0xFF == ord('w'):
                #     drone.forward(20)
                #     # sleep(1)
                # if interupt & 0xFF == ord('s'):
                #     drone.backward(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('a'):
                #     drone.left(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('d'):
                #     drone.right(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('z'):
                #     drone.clockwise(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('c'):
                #     drone.flip_right()
                #     sleep(1)
                # if interupt & 0xFF == ord('t'):
                #     drone.takeoff()
                #     sleep(2)
                # if interupt & 0xFF == ord('u'):
                #     drone.up(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('n'):
                #     drone.down(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('v'):
                #     drone.contourclockwise(20)
                #     sleep(1)
                # if interupt & 0xFF == ord('b'):
                #     drone.flip_left()
                #     sleep(1)
                #count_frame = 10
                flags = numpy.zeros((1, 4)) # initial count of each gesture are all 0
                # print('*****       count_frame=%d       *****' % count_frame)
                # frame_skip = int((time.time() - start_time) / frame.time_base)
                frame_skip = 20

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        print('drone.quit()')
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
