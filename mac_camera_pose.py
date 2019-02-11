# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform

# Remember to add your installation path here
# Option a
dir_path = os.path.dirname(os.path.realpath(__file__))
if platform == "win32": sys.path.append(dir_path + '/../../python/openpose/');
else: sys.path.append('../../python');
# Option b
# If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
# sys.path.append('/usr/local/python')

# Parameters for OpenPose. Take a look at C++ OpenPose example for meaning of components. Ensure all below are filled
try:
    #from openpose import *
    from openpose import openpose as op
except:
    raise Exception('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')


def set_params():
    params = dict()
    params["logging_level"] = 3
    params["output_resolution"] = "-1x-1"
    params["net_resolution"] = "128x-1"
    params["model_pose"] = "BODY_25"
    params["alpha_pose"] = 0.6
    params["scale_gap"] = 0.3
    params["scale_number"] = 1
    params["render_threshold"] = 0.05
    # If GPU version is built, and multiple GPUs are available, set the ID here
    params["num_gpu_start"] = 0
    params["disable_blending"] = False
    #params["write_video_fps"] = -1
    # Ensure you point to the correct path where models are located
    params["default_model_folder"] = dir_path + "/../../../models/"
    return params



def main():
    params = set_params()
    
    #Constructing OpenPose object allocates GPU memory
    openpose = op.OpenPose(params)
    
    #Opening OpenCV stream
    stream = cv2.VideoCapture(0)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    

    
    
    while True:
    
        ret,img = stream.read()
        # 转为灰度图
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 改变图片大小
        img = cv2.resize(img, (320, 180));
        
        
        
    
        # Output keypoints and the image with the human skeleton blended on it
           # keypoints, output_image = openpose.forward(img, True)
        keypoints, output_image = openpose.forward(img, True)
        
        # Print the human pose keypoints, i.e., a [#people x #keypoints x 3]-dimensional numpy object with the keypoints of all the people on that image
        if len(keypoints)>0:
            print('Human(s) Pose Estimated!')
            print(keypoints)
        else:
            print('No humans detected!')
        
        
        # Display the stream
        #cv2.putText(output_image,'OpenPose Tello-Gesture-Control',(5,15), font, 0.5,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(output_image,'Tello-Gesture-Control',(55,15), font, 0.5,(255,255,255),1,cv2.LINE_AA)
        
      
        cv2.imshow('Human Pose Estimation',output_image)
        
        if cv2.waitKey(10) & 0xFF == ord('q'):
            print( "I'm done end **********************************")
            break
        
    
    stream.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
        
    


