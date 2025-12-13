# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MasterDJ.ui'
##
## Created by: Qt User Interface Compiler version 6.9.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QMenu,
    QMenuBar, QProgressBar, QPushButton, QSizePolicy,
    QSlider, QWidget)
import recursos_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(600, 825)
        MainWindow.setMinimumSize(QSize(600, 825))
        MainWindow.setMaximumSize(QSize(600, 825))
        self.Elegir_Carpeta = QAction(MainWindow)
        self.Elegir_Carpeta.setObjectName(u"Elegir_Carpeta")
        self.actionSalir = QAction(MainWindow)
        self.actionSalir.setObjectName(u"actionSalir")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Portada = QLabel(self.centralwidget)
        self.Portada.setObjectName(u"Portada")
        self.Portada.setGeometry(QRect(100, 30, 400, 400))
        self.Portada.setPixmap(QPixmap(u":/Imagenes/Imagenes/Base.png"))
        self.Portada.setScaledContents(True)
        self.Titulo = QLabel(self.centralwidget)
        self.Titulo.setObjectName(u"Titulo")
        self.Titulo.setGeometry(QRect(75, 440, 450, 50))
        font = QFont()
        font.setFamilies([u"Russo One"])
        font.setPointSize(29)
        self.Titulo.setFont(font)
        self.Pausa = QPushButton(self.centralwidget)
        self.Pausa.setObjectName(u"Pausa")
        self.Pausa.setGeometry(QRect(240, 625, 120, 120))
        self.Adelante = QPushButton(self.centralwidget)
        self.Adelante.setObjectName(u"Adelante")
        self.Adelante.setGeometry(QRect(420, 625, 120, 120))
        self.Atras = QPushButton(self.centralwidget)
        self.Atras.setObjectName(u"Atras")
        self.Atras.setGeometry(QRect(60, 625, 120, 120))
        self.Album = QLabel(self.centralwidget)
        self.Album.setObjectName(u"Album")
        self.Album.setGeometry(QRect(80, 490, 450, 30))
        font1 = QFont()
        font1.setPointSize(18)
        self.Album.setFont(font1)
        self.Album.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.Artista = QLabel(self.centralwidget)
        self.Artista.setObjectName(u"Artista")
        self.Artista.setGeometry(QRect(80, 520, 450, 30))
        self.Artista.setFont(font1)
        self.BarraProgreso = QSlider(self.centralwidget)
        self.BarraProgreso.setObjectName(u"BarraProgreso")
        self.BarraProgreso.setGeometry(QRect(60, 560, 480, 22))
        self.BarraProgreso.setOrientation(Qt.Orientation.Horizontal)
        self.Posicion = QLabel(self.centralwidget)
        self.Posicion.setObjectName(u"Posicion")
        self.Posicion.setGeometry(QRect(60, 590, 60, 16))
        self.Duracion = QLabel(self.centralwidget)
        self.Duracion.setObjectName(u"Duracion")
        self.Duracion.setGeometry(QRect(480, 590, 60, 16))
        self.Duracion.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.Volumen = QProgressBar(self.centralwidget)
        self.Volumen.setObjectName(u"Volumen")
        self.Volumen.setGeometry(QRect(60, 765, 480, 23))
        self.Volumen.setValue(24)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 600, 22))
        self.menuElegir_Carpeta = QMenu(self.menubar)
        self.menuElegir_Carpeta.setObjectName(u"menuElegir_Carpeta")
        MainWindow.setMenuBar(self.menubar)

        self.menubar.addAction(self.menuElegir_Carpeta.menuAction())
        self.menuElegir_Carpeta.addAction(self.Elegir_Carpeta)
        self.menuElegir_Carpeta.addAction(self.actionSalir)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.Elegir_Carpeta.setText(QCoreApplication.translate("MainWindow", u"Elegir Carpeta", None))
        self.actionSalir.setText(QCoreApplication.translate("MainWindow", u"Salir", None))
        self.Portada.setText("")
        self.Titulo.setText(QCoreApplication.translate("MainWindow", u"Titulo de la Cancion", None))
        self.Pausa.setText(QCoreApplication.translate("MainWindow", u"Pausa", None))
        self.Adelante.setText(QCoreApplication.translate("MainWindow", u"Adelante", None))
        self.Atras.setText(QCoreApplication.translate("MainWindow", u"Atras", None))
        self.Album.setText(QCoreApplication.translate("MainWindow", u"Album", None))
        self.Artista.setText(QCoreApplication.translate("MainWindow", u"Artista", None))
        self.Posicion.setText(QCoreApplication.translate("MainWindow", u"0:00", None))
        self.Duracion.setText(QCoreApplication.translate("MainWindow", u"0:00", None))
        self.menuElegir_Carpeta.setTitle(QCoreApplication.translate("MainWindow", u"Archivo", None))
    # retranslateUi

