from os import sync
#import cv2
import threading
import time
import numpy as np
import struct
from pathlib import Path

import open3d 
import ffmpeg 

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

class SaveData(QObject) :
    def __init__(self) :
        super(SaveData, self).__init__()
        self.sync_count = 0
        self.lidar_sync_count = 0
        self.cam_file_count = 0
        self.cam3_file_count = 0
        self.lidar_done = False
        self.cam0_done = False
        self.cam1_done = False
        self.cam2_done = False
        self.cam3_done = False
        self.fps = int()
        
    
    def setPath(self, path) :
        self.lidar_path = path+"/lidar/ouster/data/" 
        self.gps_path = path+"/gps/data/"
        self.imu_path = path+"/imu/data/"
        self.cam0_path = path+"/cam/cam0/data/"
        self.cam1_path = path+"/cam/cam1/data/"
        self.cam2_path = path+"/cam/cam2/data/"
        self.cam3_path = path+"/cam/cam3/data/"
        self.lidar_save_path = path+"/00/pcd/" 
        self.gps_save_path = path+"/00/gps/"
        self.imu_save_path = path+"/00/imu/"
        self.cam0_save_path = path+"/00/image/front/"
        self.cam1_save_path = path+"/00/image/left/"
        self.cam2_save_path = path+"/00/image/right/"
        self.cam3_save_path = path+"/00/image/rear/"
        #set FPS
        self.lidar_interval = int(10/self.fps)
        self.cam_interval = int(30/self.fps)
        self.gps_interval = int(20/self.fps)
        self.imu_interval = int(100/self.fps)
        self.time_to_sleep = float(1/self.fps)
        
    
    send_progress_last = pyqtSignal(int)
    def setSync(self, sync, lidar_sync) :
        self.sync_count = sync
        self.lidar_sync_count = lidar_sync

        self.cam_file_count = int((self.cam0_last - (self.cam0_start+(self.sync_count*self.cam_interval)))/self.cam_interval)
        self.send_progress_last.emit(self.cam_file_count)

        mbox = QMessageBox()
        mbox.setWindowTitle("Save Synced Dataset")
        mbox.setText("Save "+str(self.cam_file_count+1)+" Synchronized Data      \nSave or Not?     ")
        mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        buttonC = mbox.button(QMessageBox.Ok)
        buttonC.setText('Save')
        returnv = mbox.exec_()
        if returnv == QMessageBox.Ok :
            self.saveThread()

    
    def saveThread(self):
        tlidar = threading.Thread(target=self.saveLidar)
        tgps = threading.Thread(target=self.saveGPS)
        timu = threading.Thread(target=self.saveIMU)
        tcam0 = threading.Thread(target=self.saveCAM, args=(0,))
        tcam1 = threading.Thread(target=self.saveCAM, args=(1,))
        tcam2 = threading.Thread(target=self.saveCAM, args=(2,))
        tcam3 = threading.Thread(target=self.saveCAM, args=(3,))
        state = threading.Thread(target=self.check_status)
        tlidar.start()
        tgps.start()
        timu.start()
        tcam0.start()
        tcam1.start()
        tcam2.start()
        tcam3.start()
        state.start()

    def check_status(self) :
        not_yet = True
        while not_yet :
            if self.lidar_done and self.cam0_done and self.cam1_done and self.cam2_done and self.cam3_done :
                mbox = QMessageBox()
                mbox.setWindowTitle("Success")
                mbox.setText("Success to Save Synchronized Data")
                mbox.exec_()
                not_yet = False
    
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

    send_progress = pyqtSignal(int)
    send_lidar = pyqtSignal(object, object) 
    def saveLidar(self) :
        lidar_path = Path(self.lidar_path).glob("*.bin")
        print(self.lidar_sync_count)
        target_idx = self.lidar_start + (self.lidar_sync_count*self.lidar_interval)
        count = 0
        file_count = 0
        for k in sorted(lidar_path) :
            if count > self.lidar_last or file_count > self.cam_file_count :
                self.lidar_done = True
                break
            if count >= target_idx and (count-self.lidar_start)%self.lidar_interval==0:
                np_pcd, pcd = self.convert_bin_to_pcd(str(k))
                self.send_lidar.emit(np_pcd, np_pcd)
                save_name = self.lidar_save_path+"%06d.pcd"%file_count
                open3d.io.write_point_cloud(save_name, pcd)
                self.send_progress.emit(file_count)
                file_count = file_count+1
            count = count+1
    
    def convert_bin_to_pcd(self, binFilePath):
        size_float = 4
        list_pcd = []
        list_pcd2 = []
        with open(binFilePath, "rb") as f:
            byte = f.read(size_float * 4)
            while byte:
                x, y, z, intensity = struct.unpack("ffff", byte)
                list_pcd.append([x, y, z])
                list_pcd2.append([x,y,z,intensity])
                byte = f.read(size_float * 4)
        np_pcd = np.asarray(list_pcd)
        np_pcd2 = np.asarray(list_pcd2)
        pcd = open3d.geometry.PointCloud() # open3d.geometry.
        pcd.points = open3d.utility.Vector3dVector(np_pcd2) #open3d.utility.
        return np_pcd, pcd 

    send_gps = pyqtSignal(object)
    def saveGPS(self) :
        gps_txt = self.gps_path+"gps.txt"
        target_idx = self.gps_start + (self.sync_count*self.gps_interval)
        count = 0
        file_count = 0 
        gpsf = open(str(gps_txt), 'r')
        lines = gpsf.readlines()
        for line in lines :
            if count == 0 :
                save_name = self.gps_save_path+"indexes.txt"
                f = open(save_name, 'w')
                f.write(str(line))
                f.close()
                self.send_gps.emit(line)
            if count > self.gps_last or file_count > self.cam_file_count :
                break
            if count >= target_idx and (count-self.gps_start)%self.gps_interval==0:
                save_name = self.gps_save_path+"%06d.txt"%file_count
                f = open(save_name, 'w')
                f.write(str(line))
                f.close()
                self.send_gps.emit(line)
                time.sleep(self.time_to_sleep)
                file_count = file_count + 1
            count = count + 1

    send_imu = pyqtSignal(object)
    def saveIMU(self) :
        imu_txt = self.imu_path+"imu.txt"
        target_idx = self.imu_start + (self.sync_count*self.imu_interval)
        count = 0
        file_count = 0 
        imuf = open(str(imu_txt), 'r')
        lines = imuf.readlines()
        for line in lines :
            if count == 0 :
                save_name = self.imu_save_path+"indexes.txt"
                f = open(save_name, 'w')
                f.write(str(line))
                f.close()
                self.send_imu.emit(line)
            if count > self.imu_last or file_count > self.cam_file_count :
                break
            if count >= target_idx and (count-self.imu_start)%self.imu_interval==0:
                save_name = self.imu_save_path+"%06d.txt"%file_count
                f = open(save_name, 'w')
                f.write(str(line))
                f.close()
                self.send_imu.emit(line)
                time.sleep(self.time_to_sleep)
                file_count = file_count + 1
            count = count + 1


    send_cam0 = pyqtSignal(object)
    send_cam1 = pyqtSignal(object)
    send_cam2 = pyqtSignal(object)
    send_cam3 = pyqtSignal(object)
    def saveCAM(self, cam) :
        cam_start = 0
        cam_last = 0
        count = 0
        file_count = 0
        pos=""
        if cam == 0 :
            cam_path = Path(self.cam0_path).glob("*mp4")
            cam_start = self.cam0_start
            cam_last = self.cam0_last
            save_path = self.cam0_save_path
            pos = "_front"
        elif cam == 1 :
            cam_path = Path(self.cam1_path).glob("*mp4")
            cam_start = self.cam1_start
            cam_last = self.  cam1_last
            save_path = self.cam1_save_path
            pos="_left"
        elif cam == 2 :
            cam_path = Path(self.cam2_path).glob("*mp4")
            cam_start = self.cam2_start
            cam_last = self.cam2_last
            save_path = self.cam2_save_path
            pos="_right"
        elif cam == 3 :
            cam_path = Path(self.cam3_path).glob("*mp4")
            cam_start = self.cam3_start
            cam_last = self.cam3_last
            save_path = self.cam3_save_path
            pos="_rear"

        #video setting 
        height = 1080
        width = 1920
        pos = ""
        saved = False
        count = 0
        target_index = cam_start+(self.sync_count*self.cam_interval)
        mp4_count = 0
        for k in sorted(cam_path) :
            mp4_count = mp4_count+1
            if saved :
                break
            probe = ffmpeg.probe(str(k))
            video_info = next(x for x in probe['streams'] if x['codec_type'] == 'video')
            num_frames = int(video_info['nb_frames'])
            i=0
            for i in range(num_frames):
                if i>=num_frames :
                    break
                if count > cam_last :
                    saved=True
                    break
                if count >= target_index and (count-cam_start ) % self.cam_interval == 0:
                    last_frames = num_frames
                    if count + 900 >= cam_last :
                        last_frames = cam_last + 1
                    self.extract_jpg(cam_start, pos,k,i,last_frames, save_path, file_count)
                    file_count = file_count+(int((last_frames-i)/self.cam_interval))+1
                    count = count+(int((last_frames-i)/self.cam_interval))*self.cam_interval
                    break
                count = count + 1 
                i = i+1
            
            '''
            if you want visualize, use below code but it's too slow
            i = 0
            for i in range(num_frames) : 
                if i >= num_frames :
                    break
                if count > cam_last :
                    saved = True
                    break
                if count>=target_index and (count-cam_start) % 3 == 0 : 
                    image = self.extract_frame(height, width, k, i)
                    #self.extract_jpg(k, i, num_frames, save_path, file_count)
                    if cam == 0:
                        self.send_cam0.emit(image)
                    elif cam == 1:
                        self.send_cam1.emit(image)
                    elif cam == 2:
                        self.send_cam2.emit(image)
                    elif cam == 3:
                        self.send_cam3.emit(image)
                    zeros = self.returnZeros(file_count)
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(save_path+str(zeros)+str(file_count)+".jpg", image)     # save frame as JPEG file      
                    if cam==3 :
                        self.cam3_file_count = file_count
                        self.send_progress.emit(self.cam3_file_count)
                    #file_count = file_count + 1 
                    file_count = file_count+(num_frames-i+1) 
                count = count + 1
                i = i+1
            '''
        if cam == 0 :
            self.cam0_done = True
        elif cam == 1 :
            self.cam1_done = True
        elif cam == 2 : 
            self.cam2_done = True
        elif cam == 3 :
            self.cam3_done = True

    def extract_jpg(self, start_idx, pos, input_vid, start_num, frame_num, output_name, file_count) :
        out, error = (
            ffmpeg
            .input(str(input_vid))
            .filter_('select', 'between(n,{},{})'.format(start_num-start_idx, frame_num))
            .filter_('fps', fps=self.fps )
            .output(str(output_name)+"%06d"+str(pos)+".jpg", start_number=int(file_count), vsync=0)
            .run()
        )


    '''
    def extract_frame(self, height, width, input_vid, frame_num) :
        out, _ = (
            ffmpeg
            .input(str(input_vid))
            .filter_('select', 'gte(n,{})'.format(frame_num))
            .output('pipe:', format='rawvideo', pix_fmt='rgb24', vframes=1)
            .run(capture_stdout=True, capture_stderr=True)
        )
        if np.frombuffer(out, np.uint8).size() == 0 :
            return np.ones(height, width, 3)
        return np.frombuffer(out, np.uint8).reshape(height, width, 3)

    def extract_jpg2(self, input_vid, frame_num, output_name, file_count) :
        (
            ffmpeg
            .input(str(input_vid))
            .filter_('select', 'gte(n,{})'.format(frame_num) )
            .output(str(output_name)+"%06d.jpg", start_number=file_count, vframes=1)
            .overwrite_output()
            .run(quiet=True)
        )
    '''
    
    
        

    
    
