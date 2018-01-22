#!/usr/bin/env python

"""
This script enables plotting data in real time using the pyqtgraph library.


Date Modified   Modified by     Changes
Nov 22, 2016    Vipul Vishnoi   Initial commit
Jan 24, 2017    Vipul Vishnoi   Added writing data to a file
Feb 06, 2017    Vipul Vishnoi   Added multiple IMU support
Feb 10, 2017    Vipul Vishnoi   Minor updates
Feb 11, 2017    ChrisZL		Modified plot defaults

"""

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np

import serial
import sys
import time
import multiprocessing
import os

"""
 Name:      Serial_data_get

 Purpose:   To get serial data from port and modify message before pushing
            to queue.

 Param:     q           - Queue handle
            sample_name - File name to store mag data

"""


def serial_data_get(q, sample_name):
    ser = serial.Serial(sys.argv[1], int(sys.argv[2]))

    fd = open("../../data/%s/%s" % (sample_name, sample_name + "_data.txt"), "w")
    fd2 = open("../../data/%s/%s" % (sample_name, sample_name + "_data2.txt"), "w")
    os.chmod("../../data/%s/%s" % (sample_name, sample_name + "_data.txt"), 0777)
    os.chmod("../../data/%s/%s" % (sample_name, sample_name + "_data2.txt"), 0777)

    while True:
        # Read Serial data
        try:
            line = ser.readline()
            try:
                line = line.rstrip('\n').rstrip('\r')
                ID = int(line.split(' ')[0])
                x = float(line.split(' ')[1])
                y = float(line.split(' ')[2])
                z = float(line.split(' ')[3])
                mag_xyz = [ID, x, y, z]

                if (ID == 1):
                    fd.write("%.5f %.5f %.5f\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                    #fd.write("%i %i %i\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                elif (ID == 2):
                    fd2.write("%.5f %.5f %.5f\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                    #fd2.write("%i %i %i\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                else:
                    print "Invalid IMU ID"
                    raise

                q.put(mag_xyz)
                print "Queuing: " + str(mag_xyz)
                time.sleep(0.1)

            except KeyboardInterrupt:
                raise

            except:
                print "Invalid data"
                pass

        except serial.serialutil.SerialException:
            print "Serial read fail, trying again"
            time.sleep(0.2)
            pass

        except KeyboardInterrupt:
            fd.close()
            fd2.close()
            sys.exit(1)

"""
 Name:      serial_data_qtplot

 Purpose:   To plot real time data from queue.

 Param:     q - Queue handle

"""


def serial_data_qtplot(q):
    kelowna_mag_raw = 367.5
    gain = 0.667
    app = QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.opts['distance'] = 3000 # This is the initial zoom of the plot
    w.show()
    w.setWindowTitle('Sample magnetometer plots')

    g = gl.GLGridItem()
    g.scale(500, 500, 500)
    w.addItem(g)

    ax = gl.GLAxisItem()
    ax.setSize(1000, 1000, 1000)
    w.addItem(ax)

    md = gl.MeshData.sphere(rows=25, cols=25, radius=kelowna_mag_raw/gain)
    m4 = gl.GLMeshItem(meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1,1,1,0.2))
    m4.translate(0,10,0)
    w.addItem(m4)

    # Maximum depth of queue. Increase this number to plot more data
    max_index = 10000
    index = 0
    index_2 = 0
    p = np.zeros((max_index, 3))
    p_2 = np.zeros((max_index, 3))
	
	# White (1,1,1), Red(255,0,0), Blue(0,0,255), Green(0,128,0)
    sp = gl.GLScatterPlotItem(pos=p, color=(255, 0, 0, 0.6), size=50, pxMode=False)
    sp_2 = gl.GLScatterPlotItem(pos=p_2, color=(0, 0, 255, 0.6), size=50, pxMode=False)

    w.addItem(sp)
    w.addItem(sp_2)

    wait = 0
    while True:
        if not q.empty():
            wait = 0
            data = q.get()

            if (data[0]==1):
                p[index] = (data[1], data[2], data[3])
                index += 1
                if index > max_index - 1:
                    index = 0
                sp.setData(pos=p)

            elif (data[0]==2):
                p_2[index] = (data[1], data[2], data[3])
                index_2 += 1
                if index_2 > max_index - 1:
                    index_2 = 0
                sp_2.setData(pos=p_2)

            app.processEvents()

        # If queue is empty, wait for data in queue. Exit if nothing
        else:
            if (wait > 20):
                break
            else:
                time.sleep(0.5)
                wait += 1

    print "No more data to plot, Exiting"
    sys.exit(app.exec_())


"""
 Name:      main

 Purpose:   Initialize threads and wait for valid data to appear,
            then start 2nd process to start plotting real time data in
            queue

 Param:     None

"""


def main():
    print "\n"
    print "#-----------------------------------------------------#"
    print "This script is used to read magnetometer readings only."
    print "Press ctrl + c to stop and save the readings.\nPress ctrl + z to exit the program."
    print "#-----------------------------------------------------#"
    sample_name = raw_input('Enter file name to store data\n')
    if not os.path.exists("../../data/%s" % sample_name):
        os.makedirs("../../data/%s" % sample_name)
        os.chmod("../../data/%s" % sample_name, 0777)

    q = multiprocessing.Queue()

    p1 = multiprocessing.Process(target=serial_data_get, args=(q, sample_name))
    p2 = multiprocessing.Process(target=serial_data_qtplot, args=(q,))
    p1.start()
    while q.empty():
        print "Queue is empty"
        time.sleep(1)
    p2.start()


if __name__ == '__main__':
    main()
