#!/usr/bin/env python
from config import cfg
from modeling.afm import AFM
import cv2
import numpy as np
import os

class Nodo(object):
    def __init__(self, cfg):
        # Params
        self.system = AFM(cfg)
        self.system.model.eval()
        self.system.load_weight_by_epoch(-1)
        # camera parameters
        #self.mtx=np.array([[cfg.projection_parameters.fx, 0, cfg.projection_parameters.cx],
        #[0, cfg.projection_parameters.fy, cfg.projection_parameters.cy],
        #[0,0,1]])
        #self.dist=np.array([cfg.distortion_parameters.k1,cfg.distortion_parameters.k2,cfg.distortion_parameters.p1, cfg.distortion_parameters.p2])
        #self.width=cfg.width
        #self.height=cfg.height
        #self.newmtx, self.validpixROI=cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width,self.height) , 0, (self.width,self.height))
    
    def start(self,cfg):
        cap = cv2.VideoCapture(r'/home/sergey/work/2D-3D-pose-tracking/datasets/06_mix_1_2020-03-17-14-47-54_stereo_left_image_raw.mp4')
        frame_width = int(cap.get(3))
        frame_height = int(cap.get(4))
        out = cv2.VideoWriter('afm.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
        while True:
            ret, frame = cap.read()
                #dst_img=cv2.undistort(img, self.mtx, self.dist, newCameraMatrix=self.newmtx)        
            feats = self.system.detect(frame, cfg)
            for i in range(feats.shape[0]):
                cv2.line(frame, (feats[i, 0], feats[i, 1]),
                                    (feats[i, 2], feats[i, 3]), (0, 0, 255), 2)
            out.write(frame)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



if __name__ == "__main__":


    config_file = r'/home/sergey/catkin_ws/src/afm/scripts/experiments/afm_unet_euroc.yaml'
    gpu = '0'

    os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu)

    cfg.merge_from_file(config_file)

    my_node = Nodo(cfg)
    my_node.start(cfg)

