import sys
import traceback
import threading
import tellopy
import av
import cv2  # for avoidance of pylint error
import numpy
import time
import os
from time import sleep
from sklearn.preprocessing import normalize
import scipy.io as sio

dir_path = os.path.dirname(os.path.realpath(__file__))

def main():
    # drone = tellopy.Tello()

    try:

        # count_frame = 10
        # count = 0
       
        keypoints_collection = numpy.empty((0, 3), float)
        # print(' the size of keypoints_collection is', numpy.shape(keypoints_collection))
        # flags = numpy.zeros((1, 4))
        # print('The path is:' + str(dir_path))
        filename = dir_path + "/testaction_1.mat"

        #  check if there's a action_1.mat file, we will create action_1.mat if it's not exists

        if not os.path.exists(filename):
            sio.savemat('./testaction_1', mdict={'keypoints': keypoints_collection}, oned_as='row')

        # # for each person's action we take his or her action gesture and restore in 4d array [1,image_count,25,3]
        keypoints_collection_import = sio.loadmat('./testaction_1')
        print('********print(keypoints_collection_import)**********')
        print(keypoints_collection_import)
        print('********print(keypoints_collection_import)    end   **********')
        keypoints_collection = keypoints_collection_import.get('keypoints')
     
        #if keypoints_collection.size == 0:
        if numpy.size(keypoints_collection) == 0:
            keypoints_collection = keypoints_collection.reshape(0, 3)
        print(' the size of keypoints_collection is', numpy.shape(keypoints_collection))

      
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        # drone.quit()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
