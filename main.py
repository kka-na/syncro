from PyQt5.uic.properties import QtWidgets
import sys, shutil, os
from pathlib import Path
from operator import itemgetter
from open3d import *

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import vtk
import pyvista as pv
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

form_class = uic.loadUiType("mainwindow.ui")[0]

import getidx
import getdata
import savedata

class WindowClass(QMainWindow, form_class) :
	def __init__(self) : 
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.getidx = getidx.GetIdx()
		self.getdata = getdata.GetData()
		self.savedata = savedata.SaveData()
		self.setConnect()
		
	def setConnect(self) :
		self.actionOpen.triggered.connect(self.setDir)
		self.actionSync.triggered.connect(self.getidx.sendSync)
		self.actionNoSync.triggered.connect(self.getidx.sendNoSync)
		self.getidx.send_indexes.connect(self.getdata.setIndexes)
		self.getdata.send_lidar.connect(self.setLiDAR)
		self.getdata.send_gps.connect(self.setGPS)
		self.getdata.send_imu.connect(self.setIMU)
		self.getdata.send_cam0.connect(self.setCam0)
		self.getdata.send_cam1.connect(self.setCam1)
		self.getdata.send_cam2.connect(self.setCam2)
		self.getdata.send_cam3.connect(self.setCam3)
		self.previous.clicked.connect(self.getdata.goPrevious)
		self.next.clicked.connect(self.getdata.goNext)
		self.sync.clicked.connect(self.getdata.getSync)
		self.lidar_previous.clicked.connect(self.getdata.goLidarPrevious)
		self.lidar_next.clicked.connect(self.getdata.goLidarNext)
		self.getdata.send_sync.connect(self.createDir)
		self.getdata.send_sync.connect(self.savedata.setSync)
		self.getidx.send_indexes.connect(self.savedata.setIndexes) 
		self.savedata.send_lidar.connect(self.setLiDAR)
		self.savedata.send_gps.connect(self.setGPS)
		self.savedata.send_imu.connect(self.setIMU)
		self.savedata.send_cam0.connect(self.setCam0)
		self.savedata.send_cam1.connect(self.setCam1)
		self.savedata.send_cam2.connect(self.setCam2)
		self.savedata.send_cam3.connect(self.setCam3)

		self.savedata.send_progress_last.connect(self.setProgress)
		self.savedata.send_progress.connect(self.handleProgress) 
	
	def setDir(self) :
		self.setVTK()
		self.setFPS()
		path = QFileDialog.getExistingDirectory(None, 'Select Directory of top of datasets',"/media/kana/cde7d52f-f61e-4a99-b164-5108d62c3ceb/210930AI_night", QFileDialog.ShowDirsOnly)
		self.label.setText(str(path))
		self.dir = str(path) #"/home/kana/Documents/ts_project/raw_data" #path
		self.createDir()
		self.getdata.setPath(self.dir)
		self.savedata.setPath(self.dir)
		self.getidx.setPath(self.dir)
	
	def createDir(self):
		make_new = False
		top = str(str(self.dir)+"/00")                                                                                                                                                       
		pcd = str(str(self.dir)+"/00/pcd") 
		gps = str(str(self.dir)+"/00/gps")
		imu = str(str(self.dir)+"/00/imu")
		image = str(str(self.dir)+"/00/image")
		front = str(str(self.dir)+"/00/image/front")
		left = str(str(self.dir)+"/00/image/left")
		right = str(str(self.dir)+"/00/image/right")
		rear = str(str(self.dir)+"/00/image/rear")
		
		if not os.path.exists(top) :
			os.mkdir(top)
		else : 
			mbox = QMessageBox()
			mbox.setWindowTitle("Create New Directory")
			mbox.setText("Create a Directory to Save.     \nCreate or Not?     ")
			mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
			buttonC = mbox.button(QMessageBox.Ok)
			buttonC.setText('Create')
			returnv = mbox.exec_()
			if returnv == QMessageBox.Ok :
				make_new = True
				self.makeDir(top, make_new)
			else :
				sys.exit()
		self.makeDir(pcd, make_new)
		self.makeDir(gps, make_new)
		self.makeDir(imu, make_new)
		self.makeDir(image, make_new)
		self.makeDir(front, make_new)
		self.makeDir(left, make_new)
		self.makeDir(right, make_new)
		self.makeDir(rear, make_new)

	def makeDir(self, path, perm) :
		if not os.path.exists(path) :
			os.mkdir(path)
		else :
			if perm : 
				shutil.rmtree(path)
				os.mkdir(path)
			else :
				print("There is No Permission to Create Top Directory")
				sys.exit()

	def setFPS(self) :
		self.fps = self.spinBox.value()
		print(self.fps)
		self.getdata.fps = self.fps
		self.savedata.fps = self.fps
		
	def setVTK(self):
		self.vtkWidget = QVTKRenderWindowInteractor(self)
		self.layout.addWidget(self.vtkWidget)
		self.ren = vtk.vtkRenderer()
		self.ren.ResetCamera()
		# self.ren.GetActiveCamera().Zoom(5)
		#self.ren.GetActiveCamera().SetFocalPoint(0,0,1)
		self.sphere = vtk.vtkSphereSource()
		self.sphere.SetPhiResolution(5)
		self.sphere.SetThetaResolution(5)
		self.sphere.SetRadius(0.05)
		self.sphere.SetCenter(0,0,0)
				
		self.ren.GetActiveCamera().SetPosition(0,0,100)
		self.renderWindow = self.vtkWidget.GetRenderWindow()
		self.renderWindow.AddRenderer(self.ren)
		self.iren = self.renderWindow.GetInteractor()
		self.points = vtk.vtkPoints()
		self.point_poly_data = pv.PolyData()
		self.point_poly_data.SetPoints(self.points)	
		self.point_mapper = vtk.vtkGlyph3DMapper()
		self.point_mapper.SetInputData(self.point_poly_data)
		self.point_mapper.SetSourceConnection(self.sphere.GetOutputPort())
		self.point_mapper.Update()	

		self.actor = vtk.vtkActor()
		self.actor.SetMapper(self.point_mapper)
		self.colors = vtk.vtkNamedColors()
		self.actor.GetProperty().SetColor(self.colors.GetColor3d("Cyan"))
		self.ren.AddActor(self.actor)

		self.iren.Initialize()
		self.renderWindow.Render()
		self.iren.Start()

	@pyqtSlot(object, object)
	def setLiDAR(self, object, object2) :
		#intensity = object2[:, 3]
		self.point_poly_data.Initialize()
		self.points = pv.vtk_points(object)
		self.point_poly_data.SetPoints(self.points)
		self.point_poly_data.Modified()
		
		self.renderWindow.Render()
		self.iren.Render()
		QApplication.processEvents()
	
	def conv2Qimg(self, object):
		qimage = QImage(object.data, 1920, 1080, 3*1920, QImage.Format_RGB888)	
		qimage = qimage.scaled(self.label0.size(), Qt.KeepAspectRatio)
		return qimage

	@pyqtSlot(object)
	def setIMU(self, object):
		self.label5.setText(str(object))
		self.label5.show()
		QApplication.processEvents()

	@pyqtSlot(object)
	def setGPS(self, object):
		self.label4.setText(str(object))
		self.label4.show()
		QApplication.processEvents()

		
	@pyqtSlot(object)
	def setCam0(self, object):
		self.label0.setPixmap(QPixmap.fromImage(self.conv2Qimg(object)))
		self.label0.show()
		QApplication.processEvents()
	
	@pyqtSlot(object)
	def setCam1(self, object):
		self.label1.setPixmap(QPixmap.fromImage(self.conv2Qimg(object)))
		self.label1.show()
		QApplication.processEvents()
	
	@pyqtSlot(object)
	def setCam2(self, object):
		self.label2.setPixmap(QPixmap.fromImage(self.conv2Qimg(object)))
		self.label2.show()
		QApplication.processEvents()
	
	@pyqtSlot(object)
	def setCam3(self, object):
		self.label3.setPixmap(QPixmap.fromImage(self.conv2Qimg(object)))
		self.label3.show()
		QApplication.processEvents()

	@pyqtSlot(int)
	def setProgress(self, int):
		self.progress_last = int
		self.progressBar.setRange(0, int+1)
		self.progressBar.setValue(0)

	@pyqtSlot(int)
	def handleProgress(self, int):
		self.progressBar.setValue(int+1)


if __name__ == "__main__" :
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()