#!/usr/bin/python3
import os
import sys
import math
import time
import signal
import asyncio

from roslibpy                    import Ros, Service
from ament_index_python.packages import get_package_share_directory

from PyQt5                       import QtCore
from PyQt5.QtGui                 import QIcon, QPixmap, QTransform
from PyQt5.QtWidgets             import QApplication, QWidget, QLabel, QPushButton, QLineEdit, QMainWindow, QDialog
from PyQt5.QtCore                import pyqtSignal

# Variables para compartir datos entre formularios
map_name = ""
ros = Ros(host='localhost', port=9090)  # Conexión a Rosbridge
MAPPING           = "Map"
STRING_SLOW_SPEED = "SLOW_SPEED"

class save_map_form(QWidget):

    package_name = "ground_robot"
    dato_ingresado = pyqtSignal(str)  # Señal personalizada


    def __init__(self):
        super().__init__()

        # GUI funtionality
        self.initUI()


    def initUI(self):
        self.setGeometry(550, 500, 650, 200)
        self.setWindowTitle('Save map')

        stop_button_path = get_package_share_directory(self.package_name) + "/img/save.png"
        button_pixmap = QPixmap(stop_button_path)
        button_img    = QIcon(button_pixmap)

        self.b_save   = QPushButton(self)
        self.le_name  = QLineEdit(self)
        self.l_save   = QLabel(self)
        self.l_status = QLabel(self)

        self.l_save.setGeometry(QtCore.QRect(40, 10, 400, 50))
        self.b_save.setGeometry(QtCore.QRect(550, 70, 50, 50))
        self.le_name.setGeometry(QtCore.QRect(40, 70, 500, 50))
        self.l_status.setGeometry(QtCore.QRect(40, 120, 400, 50))

        self.b_save.setIcon(button_img)
        self.b_save.setIconSize(QtCore.QSize(40, 40))

        self.l_save.setText("Elija un nombre para el mapa a guardar")
        self.b_save.clicked.connect(self.save_map)

        ros.run()  # Iniciar la conexión con Rosbridge

    def save_map(self):
        map_name_text = self.le_name.text()
        if not self.check_if_name_exists(map_name_text):
        # Enter mapping mode
            if map_name_text != "":
                self.dato_ingresado.emit(map_name_text)  # Emitir la señal con el dato
                self.close()
            else:
                pass
        else:
            self.l_status.setText("Nombre repetido")

    def check_if_name_exists(self, name):
        dir_path = get_package_share_directory(self.package_name) + "/maps"
        
        for elem in os.listdir(dir_path):
            if elem.find(name + ".yaml") != -1:
                return True
        return False
    
class GUI(QMainWindow):

    # States
    MANUAL   = "Manual"
    MAPPING  = "Map"
    SAVE_MAP = "Save Map"
    NAVIGATE = "Nav"

    SLOW_SPEED = 0.1
    NORMAL_SPEED = 1.0
    HIGH_SPEED = 2.0

    STRING_SLOW_SPEED   = "SLOW_SPEED"
    STRING_NORMAL_SPEED = "NORMAL_SPEED"
    STRING_HIGH_SPEED   = "HIGH_SPEED"

    BUTTON_STATUS = MANUAL

    ROBOT_STATUS = ""

    package_name = "ground_robot"

    map_name = ""


    def __init__(self):
        super().__init__()

        signal.signal(signal.SIGINT, self.signal_handler)
        
        # GUI funtionality
        self.initUI()

        # ROS functionality
        global ros
        
        self.service_vehicle_movement = Service(ros, 'cmd_vehicle_movement', 'my_robot_interfaces/CmdVehicle')
        self.service_vehicle_velocity = Service(ros, 'cmd_vehicle_velocity', 'my_robot_interfaces/CmdVehicle')
        self.service_vehicle_cmd      = Service(ros, 'nav_cmd'             , 'my_robot_interfaces/CmdVehicle')

        ros.run()  # Iniciar la conexión con Rosbridge


    def initUI(self):

        img_rotation = QTransform()
        arrow_path = get_package_share_directory(self.package_name) + "/img/red_arrow.png"
        pixmap = QPixmap(arrow_path)
        img = QIcon(pixmap)

        stop_button_path = get_package_share_directory(self.package_name) + "/img/stop_button.png"
        button_pixmap = QPixmap(stop_button_path)
        button_img = QIcon(button_pixmap)

        camera_rotation = get_package_share_directory(self.package_name) + "/img/camera_rotation.png"
        camera_pixmap = QPixmap(camera_rotation)
        camera_img = QIcon(camera_pixmap)

        mapping = get_package_share_directory(self.package_name) + "/img/mapping.png"
        self.mapping_img = QIcon(QPixmap(mapping))

        save_map = get_package_share_directory(self.package_name) + "/img/save_map.png"
        self.save_map_img = QIcon(QPixmap(save_map))

        navigate_map = get_package_share_directory(self.package_name) + "/img/navigate.png"
        navigate_map_img = QIcon(QPixmap(navigate_map))

        #Main window
        self.setGeometry(500, 500, 740, 300)
        self.setWindowTitle('Ground robot Joystick')
        self.setWindowIcon(QIcon(QPixmap(get_package_share_directory(self.package_name) + "/img/icon.png")))

        button_width = 50
        button_height = 50
        

        '''
            ___________x
            |
            |
            |
            |
            |y

            QRect(x, y, width, height)

        '''

        # Robot movement
        self.b_forward             = QPushButton(self)
        self.b_backwards           = QPushButton(self)
        self.b_rotateClockwise     = QPushButton(self)
        self.b_rotateAntiClockwise = QPushButton(self)
        self.b_stop                = QPushButton(self)
        
        
        self.b_forward.setGeometry(QtCore.QRect(60, 70, button_width, button_height))
        self.b_backwards.setGeometry(QtCore.QRect(60, 170, button_width, button_height))
        self.b_rotateClockwise.setGeometry(QtCore.QRect(110, 120, button_width, button_height))
        self.b_rotateAntiClockwise.setGeometry(QtCore.QRect(10, 120, button_width, button_height))
        self.b_stop.setGeometry(QtCore.QRect(60, 120, button_width, button_height))


        self.b_rotateClockwise.setIcon(img)
        self.b_rotateClockwise.setIconSize(QtCore.QSize(45, 45))
        
        img_rotation.rotate(-90)
        self.b_forward.setIcon(QIcon(pixmap.transformed(img_rotation)))
        self.b_forward.setIconSize(QtCore.QSize(45, 45))

        img_rotation.rotate(-90)
        self.b_rotateAntiClockwise.setIcon(QIcon(pixmap.transformed(img_rotation)))
        self.b_rotateAntiClockwise.setIconSize(QtCore.QSize(45, 45))

        img_rotation.rotate(-90)
        self.b_backwards.setIcon(QIcon(pixmap.transformed(img_rotation)))
        self.b_backwards.setIconSize(QtCore.QSize(45, 45))

        self.b_stop.setIcon(button_img)
        self.b_stop.setIconSize(QtCore.QSize(30, 30))

        self.b_forward.clicked.connect(self.goForward)
        self.b_backwards.clicked.connect(self.goBackwards)
        self.b_rotateClockwise.clicked.connect(self.rotateClockwise)
        self.b_rotateAntiClockwise.clicked.connect(self.rotateAntiClockwise)
        self.b_stop.clicked.connect(self.stop)



        # Camera movement
        self.b_camera_rotate = QPushButton(self)
        self.le_input_cam_grades = QLineEdit(self)
        self.b_camera_rotate.setIcon(camera_img)
        self.b_camera_rotate.setIconSize(QtCore.QSize(30, 30))
        self.le_input_cam_grades.setText("0")

        self.b_camera_rotate.setGeometry(QtCore.QRect(210, 120, button_width, button_height))
        self.le_input_cam_grades.setGeometry(QtCore.QRect(260, 120, 2*button_width, button_height))

        self.b_camera_rotate.clicked.connect(self.camRotate)


        # Mapping buttons
        self.b_mapping = QPushButton(self)
        self.b_navigate = QPushButton(self)
        self.b_mapping.setGeometry(QtCore.QRect(410 , 70 , 150 , 150))
        self.b_navigate.setGeometry(QtCore.QRect(570, 70, 150, 150))
        self.b_mapping.clicked.connect(self.toogle_button)

        self.b_mapping.setIcon(self.mapping_img)
        self.b_mapping.setIconSize(QtCore.QSize(140, 160))

        self.b_navigate.setIcon(navigate_map_img)
        self.b_navigate.setIconSize(QtCore.QSize(140, 160))        


        # Labels
        self.l_robot_movements = QLabel(self)
        self.l_cam_movements   = QLabel(self)
        self.l_mapping         = QLabel(self)
        self.l_status          = QLabel(self)
        self.l_status_info     = QLabel(self)

        self.l_robot_movements.setText("Robot")
        self.l_cam_movements.setText("Camera")
        self.l_mapping.setText("Mapping")
        self.l_status.setText("Status:")
        self.ROBOT_STATUS = "Robot in manual mode"
        self.l_status_info.setText(self.ROBOT_STATUS)

        self.l_robot_movements.setGeometry(QtCore.QRect(65, 5, 75, 80))
        self.l_cam_movements.setGeometry(QtCore.QRect(255, 5, 75, 80))
        self.l_mapping.setGeometry(QtCore.QRect(455, 3, 75, 80))
        self.l_status.setGeometry(QtCore.QRect(10, 230, 55, 80))
        self.l_status_info.setGeometry(QtCore.QRect(70, 230, 750, 80))

        self.show()

    

    # Movement functions
    # It was first planned to use the keyboard as a joystick (like a teleop_key)
    def goForward(self):
        request = {"command":"w"}
        response = self.service_vehicle_movement.call(request)

        self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    def goBackwards(self):
        request = {"command":"s"}
        response = self.service_vehicle_movement.call(request)

        self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    def rotateClockwise(self):
        request = {"command":"a"}
        response = self.service_vehicle_movement.call(request)

        self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    def rotateAntiClockwise(self):
        request = {"command":"d"}
        response = self.service_vehicle_movement.call(request)

        self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    def stop(self):
        request = {"command":"space_bar"}
        response = self.service_vehicle_movement.call(request)

        self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    def camRotate(self):
        cam_grades = self.le_input_cam_grades.text()
        if self.validate_cam_grades(cam_grades):
            cam_rads = self.normalize_cam_grades(float(cam_grades))
            request = {"command":"-> "+str(cam_rads)}
            response = self.service_vehicle_movement.call(request)

            self.l_status_info.setText(self.ROBOT_STATUS + ": " + response["result"])


    # Validations functions of cam_grades
    def validate_cam_grades(self,cam_grades):
        try:
            cam_grades = float(cam_grades)
            return True
        except Exception as e:
            return False

    def normalize_cam_grades(self,cam_grades):
        if (cam_grades >=-180.0) and (cam_grades <= 180.0):
            cam_rads = math.radians(cam_grades)
        elif cam_grades>180.0:
            res = cam_grades // 180.0
            cam_grades = cam_grades - res*180.0
            cam_rads = math.radians(cam_grades)
        elif cam_grades<-180.0:
            res = -cam_grades // 180.0
            cam_grades = cam_grades + res*180.0
            cam_rads = math.radians(cam_grades)
        
        return cam_rads

    def toogle_button(self):

        if self.BUTTON_STATUS == self.MANUAL:
            # Change button icon to show Save map icon
            self.b_mapping.setIcon(self.save_map_img)
            self.b_mapping.setIconSize(QtCore.QSize(140, 160))
            self.BUTTON_STATUS = self.MAPPING
            
            # Show the Save map form
            self.save_map_form_object = save_map_form()
            self.save_map_form_object.dato_ingresado.connect(self.mapeo)
            self.save_map_form_object.show()

        elif self.BUTTON_STATUS == self.MAPPING:
            self.l_status_info.setText(self.map_name)
            self.b_mapping.setIcon(self.mapping_img)
            self.b_mapping.setIconSize(QtCore.QSize(140, 160))
            self.BUTTON_STATUS = self.SAVE_MAP
            self.save_map(self.map_name)

        elif self.BUTTON_STATUS == self.SAVE_MAP:
            self.BUTTON_STATUS = self.NAVIGATE
        else:
            self.l_status_info.setText("Modo desconocido")


            
    def mapeo(self, dato):
        self.l_status_info.setText("Mapa a guardar: " + str(dato))
        
        self.map_name = str(dato)

        self.speed_cmd(self.STRING_SLOW_SPEED)
        
        self.nav_cmd(MAPPING)


    # Speed command
    def speed_cmd(self,speed):
        request = {"command":str(speed)}
        response = self.service_vehicle_velocity.call(request)

    def callback(self,val):
        self.l_status.setText("Me llego")

    # Save Map
    def save_map(self, map_name):
        
        # Slow velocity for better mapping
        self.speed_cmd(self.STRING_NORMAL_SPEED)

        # Enter mapping mode
        self.nav_cmd(self.SAVE_MAP + "->" + map_name)

    # Navigate
    def Navigate(self):
        
        # Slow velocity for better mapping
        self.speed_cmd(self.STRING_NORMAL_SPEED)

        # Enter mapping mode
        self.nav_cmd(self.NAVIGATE)

    # Speed command
    def speed_cmd(self,speed):
        request = {"command":str(speed)}
        response = self.service_vehicle_velocity.call(request)
        #self.l_status_info.setText(self.ROBOT_STATUS + ": Moving " + response["result"])

    # Speed command
    def nav_cmd(self,cmd):
        request = {"command":cmd}
        response = self.service_vehicle_cmd.call(request)
        self.l_status_info.setText(response["result"])


    def signal_handler(self,signal, frame ):
        self.ros.close()
        exit(0)


def main(args=None):

    app = QApplication(sys.argv)
    ex = GUI()
    
    try:
        sys.exit(app.exec_())

    #except KeyboardInterrupt:
    #    self.ros.close()
    finally:
        pass
        


if __name__ == '__main__':
    main()