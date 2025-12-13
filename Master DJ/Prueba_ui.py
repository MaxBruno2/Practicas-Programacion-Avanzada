# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'Prueba.ui'
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
    QMenuBar, QProgressBar, QSizePolicy, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(250, 260)
        self.actionSalir = QAction(MainWindow)
        self.actionSalir.setObjectName(u"actionSalir")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Potenciometro = QProgressBar(self.centralwidget)
        self.Potenciometro.setObjectName(u"Potenciometro")
        self.Potenciometro.setGeometry(QRect(110, 30, 118, 23))
        self.Potenciometro.setValue(24)
        self.labelPot = QLabel(self.centralwidget)
        self.labelPot.setObjectName(u"labelPot")
        self.labelPot.setGeometry(QRect(20, 30, 91, 16))
        self.labelEjeY = QLabel(self.centralwidget)
        self.labelEjeY.setObjectName(u"labelEjeY")
        self.labelEjeY.setGeometry(QRect(20, 90, 91, 16))
        self.EjeY = QProgressBar(self.centralwidget)
        self.EjeY.setObjectName(u"EjeY")
        self.EjeY.setGeometry(QRect(110, 90, 118, 23))
        self.EjeY.setValue(24)
        self.labelEjeX = QLabel(self.centralwidget)
        self.labelEjeX.setObjectName(u"labelEjeX")
        self.labelEjeX.setGeometry(QRect(20, 60, 91, 16))
        self.EjeX = QProgressBar(self.centralwidget)
        self.EjeX.setObjectName(u"EjeX")
        self.EjeX.setGeometry(QRect(110, 60, 118, 23))
        self.EjeX.setValue(24)
        self.labelBoton = QLabel(self.centralwidget)
        self.labelBoton.setObjectName(u"labelBoton")
        self.labelBoton.setGeometry(QRect(20, 140, 49, 16))
        self.Boton = QLabel(self.centralwidget)
        self.Boton.setObjectName(u"Boton")
        self.Boton.setGeometry(QRect(110, 140, 81, 16))
        self.labelAtras = QLabel(self.centralwidget)
        self.labelAtras.setObjectName(u"labelAtras")
        self.labelAtras.setGeometry(QRect(20, 160, 49, 16))
        self.Atras = QLabel(self.centralwidget)
        self.Atras.setObjectName(u"Atras")
        self.Atras.setGeometry(QRect(110, 160, 81, 16))
        self.labelPausa = QLabel(self.centralwidget)
        self.labelPausa.setObjectName(u"labelPausa")
        self.labelPausa.setGeometry(QRect(20, 180, 49, 16))
        self.Pausa = QLabel(self.centralwidget)
        self.Pausa.setObjectName(u"Pausa")
        self.Pausa.setGeometry(QRect(110, 180, 81, 16))
        self.labelAdelante = QLabel(self.centralwidget)
        self.labelAdelante.setObjectName(u"labelAdelante")
        self.labelAdelante.setGeometry(QRect(20, 200, 81, 16))
        self.Adelante = QLabel(self.centralwidget)
        self.Adelante.setObjectName(u"Adelante")
        self.Adelante.setGeometry(QRect(110, 200, 81, 16))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 250, 22))
        self.menuMen = QMenu(self.menubar)
        self.menuMen.setObjectName(u"menuMen")
        MainWindow.setMenuBar(self.menubar)

        self.menubar.addAction(self.menuMen.menuAction())
        self.menuMen.addAction(self.actionSalir)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.actionSalir.setText(QCoreApplication.translate("MainWindow", u"Salir", None))
        self.labelPot.setText(QCoreApplication.translate("MainWindow", u"Potenciometro:", None))
        self.labelEjeY.setText(QCoreApplication.translate("MainWindow", u"Eje Y:", None))
        self.labelEjeX.setText(QCoreApplication.translate("MainWindow", u"Eje X:", None))
        self.labelBoton.setText(QCoreApplication.translate("MainWindow", u"Bot\u00f3n:", None))
        self.Boton.setText(QCoreApplication.translate("MainWindow", u"Despushado", None))
        self.labelAtras.setText(QCoreApplication.translate("MainWindow", u"Atr\u00e1s:", None))
        self.Atras.setText(QCoreApplication.translate("MainWindow", u"Despushado", None))
        self.labelPausa.setText(QCoreApplication.translate("MainWindow", u"Pausa:", None))
        self.Pausa.setText(QCoreApplication.translate("MainWindow", u"Despushado", None))
        self.labelAdelante.setText(QCoreApplication.translate("MainWindow", u"Adelante:", None))
        self.Adelante.setText(QCoreApplication.translate("MainWindow", u"Despushado", None))
        self.menuMen.setTitle(QCoreApplication.translate("MainWindow", u"Men\u00fa", None))
    # retranslateUi

