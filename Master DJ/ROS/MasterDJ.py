from PySide6.QtWidgets import QMainWindow, QFileDialog
from PySide6.QtGui import QPixmap
from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
from PySide6.QtCore import QThread, Signal
from PySide6 import QtWidgets
from mutagen.id3 import ID3
from mutagen.mp3 import MP3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32, Bool
from MasterDJ_ui import *
import socket
#import pyautogui
import os
import sys
import recursos_rc

class RosNode(Node):
    def __init__(self, thread):
        super().__init__('mixer_gui')

        self.create_subscription(Int32, 'mixer/pot',
            lambda msg: thread.pot.emit(msg.data), 10)

        self.create_subscription(Int32, 'mixer/eje_x',
            lambda msg: thread.eje_x.emit(msg.data), 10)

        self.create_subscription(Int32, 'mixer/eje_y',
            lambda msg: thread.eje_y.emit(msg.data), 10)

        self.create_subscription(Bool, 'mixer/atras',
            lambda msg: thread.atras.emit(msg.data), 10)

        self.create_subscription(Bool, 'mixer/pausa',
            lambda msg: thread.pausa.emit(msg.data), 10)

        self.create_subscription(Bool, 'mixer/adelante',
            lambda msg: thread.adelante.emit(msg.data), 10)

        self.create_subscription(Bool, 'mixer/boton',
            lambda msg: thread.boton.emit(msg.data), 10)

class RosThread(QThread):
    pot = Signal(int)
    eje_x = Signal(int)
    eje_y = Signal(int)
    atras = Signal(bool)
    pausa = Signal(bool)
    adelante = Signal(bool)
    boton = Signal(bool)

    def run(self):
        rclpy.init(args=None)

        self.node = RosNode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.executor.spin()

        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        QMainWindow.__init__(self, *args, **kwargs)
        self.setupUi(self)
        #pyautogui.FAILSAFE = False
        self.prendido = False
        
        self.ros = RosThread()

        self.ros.pot.connect(self.pot_read)
        #self.ros.eje_x.connect(self.ejeX_read)
        #self.ros.eje_y.connect(self.ejeY_read)
        self.ros.atras.connect(self.atras_read)
        self.ros.pausa.connect(self.pausa_read)
        self.ros.adelante.connect(self.adelante_read)
        #self.ros.boton.connect(self.boton_read)

        self.ros.start()

        
        self.ejeX_actual = 2738
        self.ejeY_actual = 2695

        self.ejeX_filtrado = 0.0
        self.ejeY_filtrado = 0.0
        
        self.boton_anterior = 1
        self.atras_anterior = 0
        self.pausa_anterior = 0
        self.adelante_anterior = 0
        
        self.volumen_anterior = -1.0
        self.pot_filtrado = 0.0
        
        self.lista_canciones = []
        self.index_actual = 0
        
        # Reproductor
        self.player = QMediaPlayer()
        self.audio = QAudioOutput()
        self.player.setAudioOutput(self.audio)
        
        # Conexiones
        self.Elegir_Carpeta.triggered.connect(self.seleccionar_carpeta)
        self.Atras.clicked.connect(self.retroceder_cancion)
        self.Adelante.clicked.connect(self.avanzar_cancion)
        self.Pausa.clicked.connect(self.pausar_cancion)
        self.player.durationChanged.connect(self.actualizar_duracion)
        self.player.positionChanged.connect(self.actualizar_posicion)
        self.BarraProgreso.sliderMoved.connect(self.cambiar_posicion)
        self.actionSalir.triggered.connect(self.salir)
    
    def retroceder_cancion(self):
        if not self.lista_canciones:
            return
        self.index_actual = (self.index_actual - 1) % len(self.lista_canciones)
        self.cargar_cancion()
        
    def avanzar_cancion(self):
        if not self.lista_canciones:
            return
        self.index_actual = (self.index_actual + 1) % len(self.lista_canciones)
        self.cargar_cancion()
        
    def pausar_cancion(self):
        if self.player.isPlaying():
            self.player.pause()
        else:
            self.player.play()
            
    def actualizar_duracion(self, duracion):
        if duracion > 0:
            self.BarraProgreso.setMaximum(duracion)

            # Tiempo total formateado
            minutos = int(duracion // 60000)
            segundos = int((duracion % 60000) // 1000)
            self.Duracion.setText(f"{minutos:02}:{segundos:02}")
            
    def actualizar_posicion(self, posicion):
        # Evita que la barra pegue saltos cuando tú la mueves
        if not self.BarraProgreso.isSliderDown():
            self.BarraProgreso.setValue(posicion)

        # Tiempo pasado formateado
        minutos = int(posicion // 60000)
        segundos = int((posicion % 60000) // 1000)
        self.Posicion.setText(f"{minutos:02}:{segundos:02}")
        
    def cambiar_posicion(self, valor):
        self.player.setPosition(valor)
    
    def seleccionar_carpeta(self):
        carpeta = QFileDialog.getExistingDirectory(self, "Selecciona la carpeta con música")
        if not carpeta:
            return

        # Buscar mp3
        archivos = [os.path.join(carpeta, f) for f in os.listdir(carpeta) if f.endswith(".mp3")]

        self.lista_canciones = [canciones(a) for a in archivos]

        if self.lista_canciones:
            self.index_actual = 0
            self.cargar_cancion()
    
    def cargar_cancion(self):
        cancion = self.lista_canciones[self.index_actual]
        self.player.setSource(QUrl.fromLocalFile(cancion.ruta))
        self.player.play()
        self.BarraProgreso.setValue(0)
        self.Posicion.setText("00:00")
        self.Duracion.setText("00:00")

        # Mostrar datos en la UI
        self.Titulo.setText(cancion.titulo)
        self.Artista.setText(cancion.artista)
        self.Album.setText(cancion.album)

        # Mostrar portada si existe
        if cancion.portada:
            pixmap = QPixmap()
            pixmap.loadFromData(cancion.portada)
            self.Portada.setPixmap(pixmap.scaled(1080, 1080))
        else:
            self.Portada.setPixmap(QPixmap(":/Imagenes/Imagenes/Base.png").scaled(1080, 1080))  # Vacío
    
    def pot_read(self, value):
        self.Volumen.setValue(value * (100/4095))
        
        
        # --------- FILTRO ---------
        SUAVIZADO = 0.4   # MUCHO más rápido
        self.pot_filtrado = self.pot_filtrado * SUAVIZADO + value * (1 - SUAVIZADO)

        # ---------- NORMALIZAR ----------
        norm = self.pot_filtrado / 4095  # 0.0 – 1.0
        norm = max(0.0, min(1.0, norm))

        # ---------- CURVA PERCEPTUAL ----------
        # cuadrática (simple y efectiva)
        volumen = norm ** 2

        # ---------- APLICAR ----------
        if abs(volumen - self.volumen_anterior) > 0.01:
            self.audio.setVolume(volumen)
            self.volumen_anterior = volumen
    """
    def ejeX_read(self, value):
        self.ejeX_actual = value
        self.mover_mouse()
        
    def ejeY_read(self, value):
        self.ejeY_actual = value
        self.mover_mouse()
    """
    def atras_read(self, value):
        if value == 1 and self.atras_anterior == 0:
            self.retroceder_cancion()
        self.atras_anterior = value
            
    def pausa_read(self, value):
        if value == 1 and self.pausa_anterior == 0:
            self.pausar_cancion()
        self.pausa_anterior = value
    
    def adelante_read(self, value):
        if value == 1 and self.adelante_anterior == 0:
            self.avanzar_cancion()
        self.adelante_anterior = value
    """
    def boton_read(self, value):
        
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
    def salir(self):
        exit()
        
    def closeEvent(self, event):
        if self.ros.isRunning():
            self.ros.executor.shutdown()
            self.ros.quit()
            self.ros.wait()
        event.accept()

    
class canciones():
    def __init__(self, ruta):
        self.ruta = ruta
        self.archivo = MP3(ruta, ID3=ID3)
        self.tags = self.archivo.tags

        # Título
        if "TIT2" in self.tags:
            self.titulo = self.tags["TIT2"].text[0]
        else:
            self.titulo = "Desconocido"

        # Artista
        if "TPE1" in self.tags:
            self.artista = self.tags["TPE1"].text[0]
        else:
            self.artista = "Desconocido"

        # Álbum
        if "TALB" in self.tags:
            self.album = self.tags["TALB"].text[0]
        else:
            self.album = "Desconocido"

        # Duración
        self.duracion = self.archivo.info.length

        # Portada (opcional)
        self.portada = None
        for tag in self.tags.values():
            if tag.FrameID == "APIC":
                self.portada = tag.data  # bytes de la imagen
                break
    
if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()

    exit_code = app.exec()

    sys.exit(exit_code)
