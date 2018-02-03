#!/usr/bin/env python

from datetime import datetime
import numpy as np
import os
import pickle
import sys
import time

from PyQt5 import QtCore, QtWidgets, QtGui
import qdarkstyle

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

#import physics

plt.style.use('fivethirtyeight')
plt.rc('font', family='sansserif')
plt.rc('axes', facecolor='white')
plt.rc('figure', autolayout=True)
plt.rc('xtick', color='white')
plt.rc('ytick', color='white')

RESULT_FOLDER = '/home/gabs48/src/quadruped/tigrillo2/data/analysis/results'


class SimpleFigure(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):

        fig = Figure(figsize=(width, height), dpi=dpi)
        fig.patch.set_facecolor("None")
        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)

        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet("background-color:transparent;")
        self.updateGeometry()


class VizWin(QtWidgets.QGridLayout):

    def __init__(self, win):

        QtWidgets.QGridLayout.__init__(self)

        self.setGeometry(QtCore.QRect(0, 0, 1000, 600))

        self.win = win

    def plotFigure(self, name):

        if name in ["Plot FL", "Plot FR", "Plot BR", "Plot BL"]:
            self.plotSensor(name)
        if name  == "CMA Evolution":
            self.plotEvolution()

    def plotSensor(self, name):

        self.clean()
        data_init = self.win.sel_conf[0]
        data = self.win.sel_conf[self.win.sel_ite]
        l = name.split()[-1]

        self.plot = SimpleFigure()
        self.addWidget(self.plot)

        self.plot.axes.cla()
        # Multi lines
        if len(data_init[l.lower() + "_rob"].shape) > 1:
            rob_mean = np.mean(data_init[l.lower() + "_rob"], axis=1) - np.mean(np.mean(data_init[l.lower() + "_rob"], axis=1))
            sim_mean = np.mean(data[l.lower() + "_sim"], axis=1) - np.mean(np.mean(data[l.lower() + "_sim"], axis=1))
            self.plot.axes.plot(data_init["t"], data_init[l.lower() + "_rob"], linewidth=0.4, color="skyblue")
            self.plot.axes.plot(data_init["t"], rob_mean, linewidth=2.5, color=self.getStyleColors()[0], label="Averaged Centered " + l + " Rob")
            self.plot.axes.plot(data_init["t"], data[l.lower() + "_sim"], linewidth=0.8, color="orchid")
            self.plot.axes.plot(data_init["t"], sim_mean, linewidth=2.5, color=self.getStyleColors()[1], label="Averaged Centered " + l + " Sim")
            self.plot.axes.set_xlim([data_init["t"][0], data_init["t"][-1]])
            self.plot.axes.legend()

        else:
            self.plot.axes.plot(data_init["t"], data_init[l.lower() + "_rob"], linewidth=1, label=l + " Leg Robot")
            self.plot.axes.plot(data_init["t"], data[l.lower() + "_sim"], linewidth=1, label=l + " Leg Simulation")
            self.plot.axes.legend()

        self.plot.draw()

    def plotEvolution(self):

        self.clean()
        scores = [t["score"] for t in self.win.sel_conf]
        pop_size = self.win.sel_conf[0]["pop"]
        try:
            x, y_min, y_max, y_av = self.rearrangePop(scores, pop_size)
        except AssertionError as e:
            print e
            return

        self.plot = SimpleFigure()
        self.addWidget(self.plot)

        self.plot.axes.cla()
        self.plot.axes.plot(x, y_max, linestyle="-", color=self.getStyleColors()[1], linewidth=1, label="Max")
        self.plot.axes.plot(x, y_av, linestyle="-", color=self.getStyleColors()[3], linewidth=1, label="Average")
        self.plot.axes.plot(x, y_min, linestyle="-", color=self.getStyleColors()[0], linewidth=1, label="Min")
        self.plot.axes.set_title("Training score of CMA-ES algorithm with popSize = " + str(pop_size), fontsize=14)
        self.plot.axes.set_ylabel('Score')
        self.plot.axes.set_xlabel('Generation Epoch')
        self.plot.axes.xaxis.label.set_color('white')
        self.plot.axes.yaxis.label.set_color('white')
        self.plot.axes.title.set_color('white')

    def showSimParams(self):

        return

    def showOptParams(self):

        return

    def clean(self):

        for i in reversed(range(self.count())): 
            self.itemAt(i).widget().setParent(None)

    def getStyleColors(self):

        if 'axes.prop_cycle' in plt.rcParams:
            cols = plt.rcParams['axes.color_cycle']
        else:
            cols = ['b', 'r', 'y', 'g', 'k']
        return cols

    def rearrangePop(self, scores, ps):

        array = np.array(scores)
        array = array[0:len(scores) -(len(scores) % ps)]
        # assert len(scores) % ps == 0, "The total number of iteration (" + str(len(scores)) + \
        #                                ") shall be a multiple of the population size (" + \
        #                                str(ps) + "). Please verify the file " +  \
        #                                self.win.sel_exp + " or this sript!"

        matrix = np.reshape(array, (-1, ps))
        y_min = np.min(matrix, axis=1)
        y_max = np.max(matrix, axis=1)
        y_av = np.mean(matrix, axis=1)
        x = np.array(range((y_av.size))) + 1
        return x, y_min, y_max, y_av


class IteListWin(QtWidgets.QGridLayout):

    def __init__(self, win):

        QtWidgets.QGridLayout.__init__(self)
        self.setGeometry(QtCore.QRect(0, 0, 150, 400))

        self.label = QtWidgets.QLabel()
        self.label.setText("Epoch List")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.addWidget(self.label)

        self.list = QtWidgets.QListWidget()
        self.list.setMinimumWidth(100)
        self.list.setMaximumWidth(350)
        self.addWidget(self.list)

        self.win = win

        self.list.itemClicked.connect(self.selectIteration)
        self.list.currentItemChanged.connect(self.selectIteration)

    def loadIterations(self):

        self.list.clear()

        if os.path.isfile(self.win.sel_exp + "/cmaes_evolution.pkl"):
            self.win.sel_conf = pickle.load(open(self.win.sel_exp + "/cmaes_evolution.pkl", "rb"))
            for i in range(len(self.win.sel_conf)):
                item = QtWidgets.QListWidgetItem()
                item.setText("Iteration " + str(i))
                item.setData(QtCore.Qt.UserRole, i)
                self.list.addItem(item)

        else:
            item = QtWidgets.QListWidgetItem()
            item.setText("Experiment produced no data")
            item.setData(QtCore.Qt.UserRole, None)
            self.list.addItem(item)

    def selectIteration(self, item):

        if "data" in dir(item):
            self.win.sel_ite = item.data(QtCore.Qt.UserRole)


class ExpListWin(QtWidgets.QGridLayout):

    def __init__(self, win):

        QtWidgets.QGridLayout.__init__(self)
        self.setGeometry(QtCore.QRect(0, 0, 150, 400))

        self.label = QtWidgets.QLabel()
        self.label.setText("Experiment List")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.addWidget(self.label)

        self.list = QtWidgets.QListWidget()
        self.list.setMinimumWidth(100)
        self.list.setMaximumWidth(350)
        self.addWidget(self.list)

        self.win = win

        self.loadExperiments()
        self.list.itemClicked.connect(self.selectExperiment)
        self.list.currentItemChanged.connect(self.selectExperiment)

    def loadExperiments(self):

        for dirname, dirnames, filenames in os.walk(self.win.folder):

            dirnames.sort(reverse=True)
            for subdirname in dirnames:
                date = datetime.strptime(subdirname, '%Y%m%d-%H%M%S')
                item = QtWidgets.QListWidgetItem()
                item.setText(date.strftime("Exp %d/%m/%Y - %H:%M:%S"))
                item.setData(QtCore.Qt.UserRole, dirname + "/" + subdirname)
                self.list.addItem(item)

    def selectExperiment(self, item):

        self.win.sel_exp = item.data(QtCore.Qt.UserRole)
        self.win.ite_list_lay.loadIterations()


class ExpButWin(QtWidgets.QGridLayout):

    def __init__(self, win):

        QtWidgets.QGridLayout.__init__(self)
        self.setGeometry(QtCore.QRect(0, 0, 1000, 100))

        self.win = win

        self.addLegend()
        self.addButtons()

    def addButtons(self):
        
        self.b1 = QtWidgets.QPushButton("CMA Evolution")
        self.b1.installEventFilter(self)
        self.addWidget(self.b1, 1, 0)

        self.b2 = QtWidgets.QPushButton("Simulate Best")
        self.b2.installEventFilter(self)
        self.addWidget(self.b2, 1, 1)

        self.b3 = QtWidgets.QPushButton("Optim Parameters")
        self.b3.installEventFilter(self)
        self.addWidget(self.b3, 1, 2)

    def addLegend(self):
        
        self.l1 = QtWidgets.QLabel()
        self.l1.setText("Optimization Functions")
        self.l1.setAlignment(QtCore.Qt.AlignCenter)
        self.addWidget(self.l1, 0, 0, 1, 3)

    def eventFilter(self, object, event):

        if event.type() == QtCore.QEvent.MouseButtonPress:
            if self.win.sel_conf:
                if object.text() == "Simulate Best":
                    print "Simulatioooon"
                elif object.text() == "Optim Parameters":
                    print "Show table"
                else:
                    self.win.viz_lay.plotFigure(object.text())
                return True
            else:
                self.win.displayStatus("Please select experiment and iteration before using this function", 3000)

        elif event.type() == QtCore.QEvent.HoverMove:
            if object.text() == "Simulate Best":
                self.win.displayStatus("Replay the simulation of the best individual in the experiment")
            elif object.text() == "Optim Parameters":
                self.win.displayStatus("Display the parameters of the optimization process")
            elif object.text() == "CMA Evolution":
                self.win.displayStatus("Display the optimization evolution across generations")
            return True

        return False


class IteButWin(QtWidgets.QGridLayout):

    def __init__(self, win):

        QtWidgets.QGridLayout.__init__(self)
        self.setGeometry(QtCore.QRect(0, 0, 1000, 100))

        self.win = win

        self.addLegend()
        self.addButtons()

    def addButtons(self):
        
        self.b1 = QtWidgets.QPushButton("Plot FL")
        self.b1.installEventFilter(self)
        self.addWidget(self.b1, 1, 0)

        self.b2 = QtWidgets.QPushButton("Plot FR")
        self.b2.installEventFilter(self)
        self.addWidget(self.b2, 1, 1)

        self.b3 = QtWidgets.QPushButton("Simulate")
        self.b3.installEventFilter(self)
        self.addWidget(self.b3, 1, 2)

        self.b4 = QtWidgets.QPushButton("Plot BL")
        self.b4.installEventFilter(self)
        self.addWidget(self.b4, 2, 0)

        self.b5 = QtWidgets.QPushButton("Plot BR")
        self.b5.installEventFilter(self)
        self.addWidget(self.b5, 2, 1)

        self.b6 = QtWidgets.QPushButton("Spectogram")
        self.b6.installEventFilter(self)
        self.addWidget(self.b6, 2, 2)

        self.b7 = QtWidgets.QPushButton("Sim Parameters")
        self.b7.installEventFilter(self)
        self.addWidget(self.b7, 1, 3)

    def addLegend(self):

        self.l1 = QtWidgets.QLabel()
        self.l1.setText("Iteration Functions")
        self.l1.setAlignment(QtCore.Qt.AlignCenter)
        self.addWidget(self.l1, 0, 0, 1, 4)

    def eventFilter(self, object, event):

        if event.type() == QtCore.QEvent.MouseButtonPress:
            if self.win.sel_conf and (self.win.sel_ite is not None):
                self.win.viz_lay.plotFigure(object.text())
                return True
            else:
                self.win.displayStatus("Please select experiment and iteration before using this function", 3000)

        elif event.type() == QtCore.QEvent.HoverMove:
            if "Plot" in object.text():
                self.win.displayStatus("Plot the sensor signals on robot and in simulation")
            elif object.text() == "Optim Parameters":
                self.win.displayStatus("Display the parameters of the specific iteration")
            elif object.text() == "Spectogram":
                self.win.displayStatus("Plot the Spectogram of the signal difference between robot and simulation")
            return True

        return False


class AppWin(QtWidgets.QMainWindow):

    def __init__(self, folder):

        # Init
        QtWidgets.QMainWindow.__init__(self)

        self.folder = folder
        self.sel_exp = None
        self.sel_ite = None
        self.sel_conf = None

        self.initUI()

    def initUI(self):

        self.resize(1200, 800)
        
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("Tigrillo Optimization Viewer")
        self.displayStatus("This software allows to browse experiment folder and display results", 4000)
        self.setWindowIcon(self.style().standardIcon(getattr(QtWidgets.QStyle, 'SP_ComputerIcon')))

        self.constructUI()
        self.moveUI()
        self.show()

    def constructUI(self):

        # Create top menu and shortcuts
        #self.file_menu = QtWidgets.QMenu('&File', self)
        #self.file_menu.addAction('&Quit', self.quitUI, QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        #self.menuBar().addMenu(self.file_menu)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+C"), self, self.quitUI)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+D"), self, self.quitUI)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+W"), self, self.quitUI)

        # Create frame structure
        self.main_window = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        win_width = self.frameGeometry().width()
        win_height = self.frameGeometry().height()

        self.main_sel_pan = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.main_vis_pan = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        self.ite_list_lay = IteListWin(self)
        self.exp_list_lay = ExpListWin(self)
        self.ite_list = QtWidgets.QWidget()
        self.exp_list = QtWidgets.QWidget()

        self.exp_list.setLayout(self.exp_list_lay)
        self.main_sel_pan.addWidget(self.exp_list)
        self.ite_list.setLayout(self.ite_list_lay)
        self.main_sel_pan.addWidget(self.ite_list)

        self.exp_butt_lay = ExpButWin(self)
        self.ite_butt_lay = IteButWin(self)
        self.viz_lay = VizWin(self)
        self.exp_butt = QtWidgets.QWidget()
        self.ite_butt = QtWidgets.QWidget()
        self.viz = QtWidgets.QWidget()

        self.exp_butt.setLayout(self.exp_butt_lay)
        self.main_vis_pan.addWidget(self.exp_butt)
        self.viz.setLayout(self.viz_lay)
        self.main_vis_pan.addWidget(self.viz)
        self.ite_butt.setLayout(self.ite_butt_lay)
        self.main_vis_pan.addWidget(self.ite_butt)

        self.main_window.addWidget(self.main_sel_pan)
        self.main_window.addWidget(self.main_vis_pan)

        # Set focus
        self.main_window.setFocus()
        self.setCentralWidget(self.main_window)

    def moveUI(self):

        frameGm = self.frameGeometry()
        screen = QtWidgets.QApplication.desktop().screenNumber(QtWidgets.QApplication.desktop().cursor().pos())
        centerPoint = QtWidgets.QApplication.desktop().screenGeometry(screen).center()
        frameGm.moveCenter(centerPoint)
        self.move(frameGm.topLeft())

    def resizeEvent(self, event):

        win_width = self.frameGeometry().width()
        win_height = self.frameGeometry().height()

        self.ite_list.setMinimumWidth(win_width/5)
        self.ite_list.setMaximumWidth(win_width/3)
        self.exp_list.setMinimumWidth(win_width/5)
        self.exp_list.setMaximumWidth(win_width/3)
        self.exp_butt.setMinimumHeight(win_height/10)
        self.ite_butt.setMinimumHeight(win_height/10)
        self.viz.setMinimumHeight(win_height/2)
        self.exp_butt.setMaximumHeight(win_height/7)
        self.ite_butt.setMaximumHeight(win_height/7)
        self.viz.setMaximumHeight(9*win_height/10)

        QtWidgets.QMainWindow.resizeEvent(self, event)

    def quitUI(self):

        self.close()

    def closeEvent(self, ce):

        self.quitUI()

    def displayStatus(self, msg, t=1000):

        self.statusBar().showMessage(msg, t)


def gui():

    app = QtWidgets.QApplication(sys.argv)
    win = AppWin(RESULT_FOLDER)

    # Dark style
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

    sys.exit(app.exec_())


if __name__ == '__main__':

     gui()
