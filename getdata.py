from os import sync
import os
from sys import version
import numpy as np
import struct
from pathlib import Path
import open3d

import av
import ffmpeg

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class GetData(QObject) :
    def __init__(self) :
        super(GetData, self).__init__()
        self.sync_count = 0
        self.lidar_sync_count = 0
        self.fps = int()
        
    def setPath(self, path) :
        self.lidar_path = path+"/lidar/ouster/data/" 
        self.gps_path = path+"/gps/data/"
        self.imu_path = path+"/imu/data/"
        self.cam0_path = path+"/cam/cam0/data/"
        self.cam1_path = path+"/cam/cam1/data/"
        self.cam2_path = path+"/cam/cam2/data/"
        self.cam3_path = path+"/cam/cam3/data/"
        #set FPS
        self.lidar_interval = int(10/self.fps)
        self.gps_interval = int(20/self.fps)
        self.imu_interval = int(100/self.fps)
        self.cam_interval = int(30/self.fps)


        
    def getSync(self) :
        self.sendSync()

    send_sync = pyqtSignal(int, int)
    def sendSync(self) :
        self.send_sync.emit(self.sync_count, self.lidar_sync_count)

    @pyqtSlot(int, int, int, int, int, int, int, int, int, int, int, int, int, int)
    def setIndexes(self, lidar_start, lidar_last, gps_start, gps_last, imu_start, imu_last, cam0_start, cam0_last, cam1_start, cam1_last, cam2_start, cam2_last,cam3_start, cam3_last):
        self.lidar_start = lidar_start
        self.lidar_last = lidar_last
        self.gps_start = gps_start
        self.gps_last = gps_last
        self.imu_start = imu_start
        self.imu_last = imu_last
        self.cam0_start = cam0_start
        self.cam0_last = cam0_last
        self.cam1_start = cam1_start
        self.cam1_last = cam1_last
        self.cam2_start = cam2_start
        self.cam2_last = cam2_last
        self.cam3_start = cam3_start
        self.cam3_last = cam3_last
        self.sendLidarNP()
        self.sendDatum()

    def goPrevious(self) :    
        if self.sync_count == 0 :
            pass
        else :
            self.sync_count = self.sync_count - 1
        self.sendDatum()
        
    def goNext(self):
        if self.lidar_start + self.sync_count > self.lidar_last :
            pass
        else :
            self.sync_count = self.sync_count + 1 
        self.sendDatum()

    def goLidarPrevious(self) :    
        if self.lidar_sync_count == 0 :
            pass
        else :
            self.lidar_sync_count = self.lidar_sync_count - 1
        self.sendLidarNP()
        
    def goLidarNext(self):
        if self.lidar_start + self.lidar_sync_count > self.lidar_last :
            pass
        else :
            self.lidar_sync_count = self.lidar_sync_count + 1 
        self.sendLidarNP()

    def sendDatum(self) :
        self.sendGPS()
        self.sendIMU()
        self.sendCam(0)
        self.sendCam(1)
        self.sendCam(2)
        self.sendCam(3)

    #send lidar np data by skipping lidar-interval
    send_lidar = pyqtSignal(object, object) 
    def sendLidarNP(self):
        lidar_path = Path(self.lidar_path).glob("*.bin")
        lidar_bin = list(sorted(lidar_path))
        count = self.lidar_start + self.lidar_sync_count*self.lidar_interval
        if count > self.lidar_last :
            count = self.lidar_start + (self.lidar_sync_count-1) 
        convert_np, intensity = self.convert_bin_to_np(str(lidar_bin[count]))
        self.send_lidar.emit(convert_np,intensity)

    def convert_bin_to_np(self, binFilePath):
        size_float = 4
        list_pcd = []
        list_pcd2 = []
        with open(binFilePath, "rb") as f:
            byte = f.read(size_float * 4)
            while byte:
                x, y, z, intensity = struct.unpack("ffff", byte)
                list_pcd.append([x, y, z])
                list_pcd2.append([x, y, z, intensity])
                byte = f.read(size_float * 4)
        np_pcd = np.asarray(list_pcd)
        np_pcd2 = np.asarray(list_pcd2)
        pcd = open3d.geometry.PointCloud() #open3d.geometry
        pcd.points = open3d.utility.Vector3dVector(np_pcd) #open3d.utility.
        return np_pcd, np_pcd2

    send_gps = pyqtSignal(object)
    def sendGPS(self) :
        gps_txt = Path(self.gps_path+"gps.txt")
        count = self.gps_start + self.sync_count * self.gps_interval
        if count > self.gps_last :
            count = self.gps_start + (self.sync_count - 1)
        gps_line = gps_txt.read_text().split('\n')[count] 
        self.send_gps.emit(gps_line)

    send_imu = pyqtSignal(object)
    def sendIMU(self) :
        imu_txt = Path(self.imu_path+"imu.txt")
        count = self.imu_start + self.sync_count * self.imu_interval
        if count > self.imu_last :
            count = self.imu_start + (self.sync_count - 1)
        imu_line = imu_txt.read_text().split('\n')[count] 
        self.send_imu.emit(imu_line)

    send_cam0 = pyqtSignal(object)
    send_cam1 = pyqtSignal(object)
    send_cam2 = pyqtSignal(object)
    send_cam3 = pyqtSignal(object)
    def sendCam(self, cam) :
        cam_start = 0
        cam_last = 0
        if cam == 0 :
            cam_path = Path(self.cam0_path).glob("*.mp4")
            cam_start = self.cam0_start
            cam_last = self.cam0_last
        elif cam == 1 :
            cam_path = Path(self.cam1_path).glob("*.mp4")
            cam_start = self.cam1_start
            cam_last = self.cam1_last
        elif cam == 2 :
            cam_path = Path(self.cam2_path).glob("*.mp4")
            cam_start = self.cam2_start
            cam_last = self.cam2_last
        elif cam == 3 :
            cam_path = Path(self.cam3_path).glob("*.mp4")
            cam_start = self.cam3_start
            cam_last = self.cam3_last
        
        #video setting 
        height = 1080
        width = 1920
        
        find = False
        count = 0
        for k in sorted(cam_path) :
            if find :
                break
            probe = ffmpeg.probe(str(k))
            video_info = next(x for x in probe['streams'] if x['codec_type'] == 'video')
            num_frames = int(video_info['nb_frames'])
            for i in range(num_frames): 
                calc = count - cam_start - (self.sync_count*self.cam_interval)
                if count > cam_last :
                    break
                if calc >= 0 and calc % self.cam_interval == 0 : 
                    image = self.extract_frame(height, width, k, i)
                    if cam == 0:
                        self.send_cam0.emit(image)
                    elif cam == 1:
                        self.send_cam1.emit(image)
                    elif cam == 2:
                        self.send_cam2.emit(image)
                    elif cam == 3:
                        self.send_cam3.emit(image)
                    find = True
                    break 
                count = count+1
                i = i+1
    def extract_frame(self, height, width, input_vid, frame_num) :
        out, _ = (
            ffmpeg
            .input(str(input_vid))
            .filter_('select', 'gte(n,{})'.format(frame_num))
            .output('pipe:', format='rawvideo', pix_fmt='rgb24', vframes=1)
            .run(capture_stdout=True, capture_stderr=True)
        )
        return np.frombuffer(out, np.uint8).reshape(height, width, 3)
