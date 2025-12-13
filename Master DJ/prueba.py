from PySide6.QtWidgets import QMainWindow, QFileDialog
from PySide6.QtGui import QPixmap
from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
from PySide6.QtCore import QThread, Signal
from PySide6 import QtWidgets
from Prueba_ui import *
import socket
import pyautogui
import sys
import os
import recursos_rc
import time
#import simplecontroller as sc

class ThreadSocket(QThread):
    global connected
    pot_read = Signal(int)
    ejeX_read = Signal(int)
    ejeY_read = Signal(int)
    atras_read = Signal(int)
    pausa_read = Signal(int)
    adelante_read = Signal(int)
    boton_read = Signal(int)
    def __init__(self, host, port):
        global connected
        super().__init__()
        server.connect((host, port))
        connected = True
        
    def run(self):
        global connected
        try:
            while connected:
                data = server.recv(BUFFER_SIZE).decode("utf-8")
                if data:
                    for message in data.splitlines():  # separa cada línea
                        try:
                            if message.startswith('<pot>'):
                                potenciometro = int(message.removeprefix('<pot>'))
                                self.pot_read.emit(potenciometro)
                            elif message.startswith('<ejeX>'):
                                ejeX = int(message.removeprefix('<ejeX>'))
                                self.ejeX_read.emit(ejeX)
                            elif message.startswith('<ejeY>'):
                                ejeY = int(message.removeprefix('<ejeY>'))
                                self.ejeY_read.emit(ejeY)
                            elif message.startswith('<boton>'):
                                boton = int(message.removeprefix('<boton>'))
                                self.boton_read.emit(boton)
                            elif message.startswith('<atras>'):
                                atras = int(message.removeprefix('<atras>'))
                                self.atras_read.emit(atras)
                            elif message.startswith('<pausa>'):
                                pausa = int(message.removeprefix('<pausa>'))
                                self.pausa_read.emit(pausa)
                            elif message.startswith('<adelante>'):
                                adelante = int(message.removeprefix('<adelante>'))
                                self.adelante_read.emit(adelante)
                        except ValueError:
                            pass
        except ...:
            pass
        finally:
            server.close()
            connected = False
        
    def stop(self):
        global connected
        connected = False
        self.wait()

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        QMainWindow.__init__(self, *args, **kwargs)
        self.setupUi(self)
        pyautogui.FAILSAFE = False
        self.prendido = False
        
        self.ejeX_actual = 2738
        self.ejeY_actual = 2695

        self.ejeX_filtrado = 0.0
        self.ejeY_filtrado = 0.0
        
        self.boton_anterior = 1
        
        self.hilo = ThreadSocket('3.149.222.108', 4062)
        self.hilo.pot_read.connect(self.pot_read)
        self.hilo.ejeX_read.connect(self.ejeX_read)
        self.hilo.ejeY_read.connect(self.ejeY_read)
        self.hilo.atras_read.connect(self.atras_read)
        self.hilo.pausa_read.connect(self.pausa_read)
        self.hilo.adelante_read.connect(self.adelante_read)
        self.hilo.boton_read.connect(self.boton_read)
        
        self.hilo.start()
        
        self.actionSalir.triggered.connect(self.salir)
        
        #self.Elegir_Carpeta.triggered.connect(self.seleccionar_carpeta)
    
    def pot_read(self, value):
        self.Potenciometro.setValue(value * (100/4095))
        
    def ejeX_read(self, value):
        self.EjeX.setValue(value * (100/4095))
        self.ejeX_actual = value
        self.mover_mouse()
        
    def ejeY_read(self, value):
        self.EjeY.setValue(value * (100/4095))
        self.ejeY_actual = value
        self.mover_mouse()
        
    def atras_read(self, value):
        if value:
            self.Atras.setText("Pushado")
        else:
            self.Atras.setText("Despushado")
            
    def pausa_read(self, value):
        if value:
            self.Pausa.setText("Pushado")
        else:
            self.Pausa.setText("Despushado")
    
    def adelante_read(self, value):
        if value:
            self.Adelante.setText("Pushado")
        else:
            self.Adelante.setText("Despushado")
    
    def boton_read(self, value):
        if value:
            self.Boton.setText("Despushado")
        else:
            self.Boton.setText("Pushado")
        
        # Click SOLO cuando pasa de 0 → 1
        if value == 0 and self.boton_anterior == 1:
            pyautogui.click(button='left')

        self.boton_anterior = value
            
    def mover_mouse(self):
        CENTRO_X = 2738
        CENTRO_Y = 2695
        DEADZONE = 200
        MAX_SPEED = 20
        SUAVIZADO = 0.7

        dx_raw = self.ejeX_actual - CENTRO_X
        dy_raw = self.ejeY_actual - CENTRO_Y

        # Deadzone fuerte
        if abs(dx_raw) < DEADZONE:
            dx_raw = 0
        if abs(dy_raw) < DEADZONE:
            dy_raw = 0

        if dx_raw == 0 and dy_raw == 0:
            self.ejeX_filtrado *= SUAVIZADO
            self.ejeY_filtrado *= SUAVIZADO
            return

        # Normalizar (-1 a 1)
        dx_norm = dx_raw / (4095 / 2)
        dy_norm = dy_raw / (4095 / 2)

        # Invertir eje Y
        #dy_norm = -dy_norm

        # Filtro exponencial
        self.ejeX_filtrado = self.ejeX_filtrado * SUAVIZADO + dx_norm * (1 - SUAVIZADO)
        self.ejeY_filtrado = self.ejeY_filtrado * SUAVIZADO + dy_norm * (1 - SUAVIZADO)

        dx = int(self.ejeX_filtrado * MAX_SPEED)
        dy = int(self.ejeY_filtrado * MAX_SPEED)

        if dx != 0 or dy != 0:
            pyautogui.moveRel(dx, dy, duration=0)
    
    """
    def seleccionar_carpeta(self):
        carpeta = QFileDialog.getExistingDirectory(self, "Selecciona la carpeta con música")
        print(carpeta)
        return carpeta
    
    while True:
        print(f"Mouse position: {pyautogui.position()}")  # Get the current mouse position
    
    def leer_mouse(self):
        x, y = pyautogui.position()
        print(f"Posición actual del mouse: X={x}, Y={y}")
    
    """
    def salir(self):
        exit()
    
if __name__ == "__main__":
    BUFFER_SIZE = 1024  # Usamos un número pequeño para tener una respuesta rápida
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
"""
device = sc.Board("COM3")  # Reemplaza "COM3" con el puerto correcto de tu placa
device.pinMode(13, sc.INPUT_PULLUP)
device.pinMode(33, sc.INPUT_PULLUP)
device.pinMode(25, sc.INPUT_PULLUP)
device.pinMode(26, sc.INPUT_PULLUP)
print("hola mundo")
while True:
    os.system("cls")
    print(f"Eje X: {device.analogRead(27)}")
    print(f"Eje Y: {device.analogRead(14)}")
    print(f"Botón: {device.digitalRead(13)}")
    print(f"Atras: {device.digitalRead(33)}")
    print(f"Pausa: {device.digitalRead(25)}")
    print(f"Adelante: {device.digitalRead(26)}")
    print(f"Potenciómetro: {device.analogRead(34)}")
    time.sleep(0.5)
"""