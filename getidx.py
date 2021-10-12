import sys
from pathlib import Path
from operator import itemgetter

from PyQt5.QtCore import *

class GetIdx(QObject) :
    def __init__(self) :
        super(GetIdx, self).__init__()
        
    
    def setPath(self, path) :
        self.lidar_tspath = Path(path+"/lidar/ouster/timestamps.txt")
        self.gps_path = Path(path+"/gps/data/gps.txt")
        self.imu_path = Path(path+"/imu/data/imu.txt")
        self.cam0_tspath = Path(path+"/cam/cam0/timestamps.txt")
        self.cam1_tspath = Path(path+"/cam/cam1/timestamps.txt")
        self.cam2_tspath = Path(path+"/cam/cam2/timestamps.txt")
        self.cam3_tspath = Path(path+"/cam/cam3/timestamps.txt")

    def sendSync(self):
        self.sendIndexes()
    
    def sendNoSync(self):
        self.sendZeros()

    send_indexes = pyqtSignal(int, int, int, int, int, int, int, int, int, int, int, int, int, int)
    def sendIndexes(self) :
        this_base_time = self.findBaseTime()
        this_last_time = self.findLastTime()
        gps_start, _ = self.getSimilarIdx(this_base_time, self.gps_path, 0.05, 0)
        gps_last, _ = self.getSimilarIdx(this_last_time, self.gps_path, 0.05, 1)
        imu_start, _ = self.getSimilarIdx(this_base_time, self.imu_path, 0.01, 0)
        imu_last, _ = self.getSimilarIdx(this_last_time, self.imu_path, 0.01, 1)
        cam0_start, _ = self.getSimilarIdx(this_base_time, self.cam0_tspath, 0.035, 0)
        cam0_last, _ = self.getSimilarIdx(this_last_time,self.cam0_tspath, 0.035, 1)
        cam1_start, _ = self.getSimilarIdx(this_base_time, self.cam1_tspath, 0.035, 0)
        cam1_last, _ = self.getSimilarIdx(this_last_time,self.cam1_tspath, 0.035, 1)
        cam2_start, _ = self.getSimilarIdx(this_base_time, self.cam2_tspath, 0.035, 0)
        cam2_last, _ = self.getSimilarIdx(this_last_time,self.cam2_tspath, 0.035, 1)
        cam3_start, _ = self.getSimilarIdx(this_base_time, self.cam3_tspath, 0.035, 0)
        cam3_last, _ = self.getSimilarIdx(this_last_time,self.cam3_tspath, 0.035, 1)
        print(cam3_start, cam3_last)
        self.send_indexes.emit(self.lidar_start_idx, self.lidar_last_idx, gps_start, gps_last, imu_start, imu_last, cam0_start, cam0_last, cam1_start, cam1_last, cam2_start, cam2_last,cam3_start, cam3_last)
    
    def sendZeros(self):
        lidar_last, gps_last, imu_last, cam0_last, cam1_last, cam2_last, cam3_last = self.getLastIdx()
        self.send_indexes.emit(0, lidar_last, 0, gps_last, 0, imu_last, 0, cam0_last, 0, cam1_last, 0, cam2_last, 0, cam3_last)

    def findBaseTime(self) :
        self.lidar_start_idx = ""
        lidar_ts = self.lidar_tspath.read_text().split('\n')[0].split('_')[1].split(':')[0]
        gps_ts = self.gps_path.read_text().split('\n')[1].split('_')[1].split(':')[0]
        imu_ts = self.imu_path.read_text().split('\n')[1].split('_')[1].split(':')[0]
        cam0_ts = self.cam0_tspath.read_text().split('\n')[0].split('_')[1].split(':')[0]
        cam1_ts = self.cam1_tspath.read_text().split('\n')[0].split('_')[1].split(':')[0]
        cam2_ts = self.cam2_tspath.read_text().split('\n')[0].split('_')[1].split(':')[0]
        cam3_ts = self.cam3_tspath.read_text().split('\n')[0].split('_')[1].split(':')[0]

        ts_list = [("lidar", lidar_ts), ("gps", gps_ts), ("imu", imu_ts), ("cam0", cam0_ts), ("cam1", cam1_ts), ("cam2", cam2_ts),("cam3", cam3_ts)]
        ts_list.sort(key=itemgetter(1), reverse=True)

        base_time = 0.0

        if ts_list[0][0]=="lidar" :
            base_time = lidar_ts
        elif ts_list[0][0] == "gps" :
            base_time = gps_ts
        elif ts_list[0][0] == "imu" :
            base_time = imu_ts
        elif ts_list[0][0] == "cam0" :
            base_time = cam0_ts
        elif ts_list[0][0] == "cam1" :
            base_time = cam1_ts
        elif ts_list[0][0] == "cam2" :
            base_time = cam2_ts
        elif ts_list[0][0] == "cam3" :
            base_time = cam3_ts
        else : 
            print("this dataset can't synchronized. check your dataset")
            sys.exit() 
        
        base_time = self.change2Sec(base_time)

        start_idx, base_time = self.getSimilarIdx(base_time, self.lidar_tspath, 0.1, 0)
        self.lidar_start_idx = start_idx
        return base_time
    
    def findLastTime(self) :
        self.lidar_last_idx = ""

        with open(self.lidar_tspath, 'r') as lidarf :
            lidar_ts = lidarf.readlines()[-1]
        with open(self.gps_path, 'r') as gpsf :
            gps_ts = gpsf.readlines()[-1]
        with open(self.imu_path, 'r') as imuf :
            imu_ts = imuf.readlines()[-1]
        with open(self.cam0_tspath, 'r') as cam0f :
            cam0_ts = cam0f.readlines()[-1]
        with open(self.cam1_tspath, 'r') as cam1f :
            cam1_ts = cam1f.readlines()[-1]
        with open(self.cam2_tspath, 'r') as cam2f :
            cam2_ts = cam2f.readlines()[-1]
        with open(self.cam3_tspath, 'r') as cam3f :
            cam3_ts = cam3f.readlines()[-1]

        lidar_ts = lidar_ts.split('_')[1].split(':')[0]
        gps_ts = gps_ts.split('_')[1].split(':')[0]
        imu_ts = imu_ts.split('_')[1].split(':')[0]
        cam0_ts = cam0_ts.split('_')[1].split(':')[0]
        cam1_ts = cam1_ts.split('_')[1].split(':')[0]
        cam2_ts = cam2_ts.split('_')[1].split(':')[0]
        cam3_ts = cam3_ts.split('_')[1].split(':')[0]

        
        ts_list = [("lidar", lidar_ts), ("gps", gps_ts), ("imu", imu_ts), ("cam0", cam0_ts), ("cam1", cam1_ts), ("cam2", cam2_ts),("cam3", cam3_ts)]
        ts_list.sort(key=itemgetter(1))

        last_time = 0.0

        if ts_list[0][0]=="lidar" :
            last_time = lidar_ts
        elif ts_list[0][0]=="gps" :
            last_time = gps_ts
        elif ts_list[0][0]=="imu" :
            last_time = imu_ts
        elif ts_list[0][0] == "cam0" :
            last_time = cam0_ts
        elif ts_list[0][0] == "cam1" :
            last_time = cam1_ts
        elif ts_list[0][0] == "cam2" :
            last_time = cam2_ts
        elif ts_list[0][0] == "cam3" :
            last_time = cam3_ts
        else : 
            print("this dataset can't synchronized. check your dataset")
            sys.exit() 
        
        last_time = self.change2Sec(last_time)
       
        last_idx, last_time = self.getSimilarIdx(last_time, self.lidar_tspath, 0.1, 1)
        self.lidar_last_idx = last_idx
        return last_time
    
    def getLastIdx(self) :
        with open(self.lidar_tspath, 'r') as lidarf :
            lidar_len = len(lidarf.readlines())
        with open(self.gps_path, 'r') as gpsf :
            gps_len = len(gpsf.readlines())
        with open(self.imu_path, 'r') as imuf :
            imu_len = len(imuf.readlines())
        with open(self.cam0_tspath, 'r') as cam0f :
            cam0_len = len(cam0f.readlines())
        with open(self.cam1_tspath, 'r') as cam1f :
            cam1_len = len(cam1f.readlines())
        with open(self.cam2_tspath, 'r') as cam2f :
            cam2_len = len(cam2f.readlines())
        with open(self.cam3_tspath, 'r') as cam3f :
            cam3_len = len(cam3f.readlines())
        return  lidar_len-1, gps_len-1, imu_len-1, cam0_len-1, cam1_len-1, cam2_len-1, cam3_len-1
        
    def getSimilarIdx(self, base_time, ts_path, _min, type) :
        ts_texts = open(ts_path, "r")
        temp = base_time
        min = _min
        text_idx = 0
        now = 0
        is_gps = False

        if str(ts_path) == str(self.gps_path) or str(ts_path) == str(self.imu_path):
            is_gps = True

        if type == 0 :
            for line in ts_texts :
                if is_gps==True :
                    is_gps = False
                    text_idx += 1
                    continue
                sensor_time = self.change2Sec(line.split('_')[1].split(':')[0])
                if sensor_time >= base_time and abs(sensor_time-base_time) <= min :
                    min = abs(sensor_time-base_time)
                    temp = sensor_time
                    now = text_idx
                elif abs(sensor_time-base_time) >= 10 :
                    break
                else :
                    pass
                text_idx = text_idx+1
        elif type == 1 :
            for line in ts_texts :
                if is_gps==True :
                    is_gps = False
                    text_idx += 1
                    continue
                sensor_time = self.change2Sec(line.split('_')[1].split(':')[0])
                if sensor_time <= base_time and abs(sensor_time-base_time) <= min :
                    min = abs(sensor_time-base_time)
                    temp = sensor_time
                    now = text_idx
                else :
                    pass
                text_idx = text_idx+1
        else :
            print("Enter the type code ")
            sys.exit()

        base_time = temp
        return now, base_time

    def change2Sec(self, timestamp) :
        hour = float(timestamp[0:2]) * 3600
        minute = float(timestamp[2:4]) * 60
        sec = float(timestamp[4:6])
        millisec = float(timestamp[7:13]) * 0.000001
        return hour+minute+sec+millisec