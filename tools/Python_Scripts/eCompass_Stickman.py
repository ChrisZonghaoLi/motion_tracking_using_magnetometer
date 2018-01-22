"""Draw a stickman and control the arms with imus
It's almost perfect, add uncal and cal dynamic reading txt file output; 
add the 3d cal reading scatter plot
add the eCompass and side-view stickman 
"""



import pygame
import numpy as np
import serial
import sys
import time
import multiprocessing
import os
import math
import shutil
from pyqtgraph.Qt import QtCore, QtGui # Require install pyqtgraph
import pyqtgraph.opengl as gl # Require install pyopengl

pygame.init()

# initialize colours
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)

right = 375
thickness = 10
midx = 650
midy = 300


"""
 Name:      Serial_data_get
 Purpose:   To get serial data from port and modify message before pushing to queue
 Param:     q, q2 - Queue handle
		 sample_name - File name to store mag data
"""
def serial_data_get(q, q2, sample_name): 
    ser = serial.Serial(sys.argv[1], int(sys.argv[2]))
    data = []
    
    # For IMU1
    fd = open("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data.txt"), "w") # IMU 1 uncal
    os.chmod("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data.txt"), 0777)   
    fd2 = open("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data_cal.txt"), "w") # IMU 1 cal
    os.chmod("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data_cal.txt"), 0777)
   
    # For IMU2
    fd3 = open("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data.txt"), "w") # IMU 2 uncal
    os.chmod("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data.txt"), 0777)   
    fd4 = open("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data_cal.txt"), "w") # IMU 2 cal
    os.chmod("../../data/%s_dynamic/%s" % (sample_name, sample_name + "_dynamic_data_cal.txt"), 0777)
    
    bias_a_inv = np.loadtxt("../../data/%s/%s_data_Bias_&_A_inv.txt" % (sample_name, sample_name))
    bias_a_inv2 = np.loadtxt("../../data/%s/%s_data2_Bias_&_A_inv.txt" % (sample_name, sample_name))

    gain = 0.667
    x_bias = bias_a_inv[0,0]
    y_bias = bias_a_inv[0,1]
    z_bias = bias_a_inv[0,2]
    bias = [x_bias,y_bias,z_bias]
    A_1 = [[bias_a_inv[1,0], bias_a_inv[1,1], bias_a_inv[1,2]],
           [bias_a_inv[2,0], bias_a_inv[2,1], bias_a_inv[2,2]],
           [bias_a_inv[3,0], bias_a_inv[3,1], bias_a_inv[3,2]]]
           
    x_bias2 = bias_a_inv2[0,0]
    y_bias2 = bias_a_inv2[0,1]
    z_bias2 = bias_a_inv2[0,2]
    bias2 = [x_bias2,y_bias2,z_bias2]
    A_12 = [[bias_a_inv2[1,0], bias_a_inv2[1,1], bias_a_inv2[1,2]],
            [bias_a_inv2[2,0], bias_a_inv2[2,1], bias_a_inv2[2,2]],
            [bias_a_inv2[3,0], bias_a_inv2[3,1], bias_a_inv2[3,2]]]
    
    while True:
        # Read Serial data
        try:
            c = ser.read(1)
            data.append(c)
            if c == "\n":
                line = ''.join(data)
                data = []
                try:    
                    line = line.rstrip('\n').rstrip('\r') # unit of readings is in mG
                    ID = int(line.split(' ')[0])
                    x = float(line.split(' ')[1])
                    y = float(line.split(' ')[2])
                    z = float(line.split(' ')[3])
                    mag_xyz = [ID, x, y, z]
                    
                    np.set_printoptions(suppress=True) # turn off scientific notation
                    
                    if (ID == 1):
                        fd.write("%.5f %.5f %.5f\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                        mag_xyz_1 = [x, y, z] # mG
                        mag_xyz_cal_1 = np.around((np.array(np.dot((np.subtract((np.array(mag_xyz_1)*gain),bias)),A_1))/gain), decimals=3) # set decimal number to 5, they are with no unit
                        fd2.write("%.5f %.5f %.5f\n" % (mag_xyz_cal_1[0], mag_xyz_cal_1[1], mag_xyz_cal_1[2]))
                    elif (ID == 2):
                        fd3.write("%.5f %.5f %.5f\n" % (mag_xyz[1], mag_xyz[2], mag_xyz[3]))
                        mag_xyz_2 = [x, y, z] # mG
                        mag_xyz_cal_2 = np.around((np.array(np.dot((np.subtract((np.array(mag_xyz_2)*gain),bias2)),A_12))/gain), decimals=3) # set decimal number to 5, they are with no unit
                        fd4.write("%.5f %.5f %.5f\n" % (mag_xyz_cal_2[0], mag_xyz_cal_2[1], mag_xyz_cal_2[2]))
                    else:
                        print "Invalid IMU ID"
                        raise                    
                    
                    mag_cal = (mag_xyz_cal_1, mag_xyz_cal_2)
                    q.put(mag_cal)
                    q2.put(mag_cal)
                    
                    print "Queuing: "  + str(mag_cal[0]) + str(mag_cal[1])
                    
                    time.sleep(0.01)
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
            fd3.close()
            fd4.close()
            sys.exit(1)


"""
 Name:      generate stickman and compass objects
 Purpose:   To animate/visulize real time data from queue
 Param:     q - Queue handle
"""
def serial_data_annimate(q):
    #kelowna_mag_raw = 367.5
    #gain = 0.667

    # Set height and width of the screen
    size = [midx * 2, midy * 2]
    screen = pygame.display.set_mode(size)

    pygame.display.set_caption("Stickman_version")

    done = False
    clock = pygame.time.Clock()

    while not done:
        if not q.empty():
            data = q.get()
            x1 = data[0][0]
            y1 = data[0][1]
            z1 = data[0][2]
            
            x2 = data[1][0]
            y2 = data[1][1]
            z2 = data[1][2]
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            # pos = pygame.mouse.get_pos()
            # x = pos[0]
            # y = pos[0]

            screen.fill(white)
            #pygame.draw.rect(screen, blue, [100, 50, 500, 350], 5)
            pygame.draw.circle(screen, (0,0,255), (midx/2, midy), 300, 20)
            draw_compass_figure(screen, x1, y1, z1)
            draw_stick_figure(screen, x1, y1, z1, x2, y2, z2)
            
            myfont = pygame.font.SysFont("monospace", 30)
            
            label = myfont.render("W", 1, (255,0,0))
            screen.blit(label, (50, midy-10))
            label = myfont.render("E", 1, (255,0,0))
            screen.blit(label, (580, midy-10))
            label = myfont.render("S", 1, (255,0,0))
            screen.blit(label, (midx/2-10, 550))
            label = myfont.render("N", 1, (255,0,0))
            screen.blit(label, (midx/2-10, 30))

            # Background (screen, blue, [start height, start w, width, heigt], t)
            pygame.draw.line(screen, blue, [700, 0], [700, 600], 5)

            pygame.display.flip()
            clock.tick(60)

    pygame.quit()


"""
 Name:      generate compass animation
 Purpose:   To animate/visulize real time data from queue
 Param:     q - Queue handle
"""
def draw_compass_figure(screen, x1, y1, z1): # top view, compass
    # if event.type == pygame. MOUSEMOTION:
    # Head (screen, colour, [horizontal, vertical], dia)
    norm_x1 = 200*x1/np.sqrt(x1**2+y1**2)
    norm_y1 = 200*y1/np.sqrt(x1**2+y1**2)
    pygame.draw.circle(screen, black, [midx/2, midy], 10)
    pygame.draw.line(screen, red, [midx/2, midy], [midx/2 + norm_x1, midy - norm_y1], thickness)


"""
 Name:      generate stickman animation
 Purpose:   To animate/visulize real time data from queue
 Param:     q - Queue handle
"""
def draw_stick_figure(screen, x1, y1, z1, x2, y2, z2): # side view
    # Head (screen, colour, [horizontal, vertical], dia)
    pygame.draw.circle(screen, black, [midx+right, 120], 40)
    # Center Line
    pygame.draw.line(screen, black, [midx+right, 120], [midx+right, 320], thickness)
   
    norm_x1 = 100*x1/np.sqrt(x1**2+y1**2)
    norm_y1 = 100*y1/np.sqrt(x1**2+y1**2)
    norm_x2 = 100*x2/np.sqrt(x2**2+y2**2)
    norm_y2 = 100*y2/np.sqrt(x2**2+y2**2)
    
    # Left Arm + forearm
    #pygame.draw.line(screen, black, [midx+250, 160], [midx, 220], thickness)
    #pygame.draw.line(screen, red, [midx+right, 220], [midx+right+100*x/np.sqrt(x**2+y**2), 220+100*y/np.sqrt(x**2+y**2)], thickness) 
    pygame.draw.line(screen, black, [midx+right, 220], [midx+right-100, 220+100], thickness) 
    # Left Thigh + Shin
    pygame.draw.line(screen, red, [midx+right, 320], [midx+right + norm_x1, 320 + norm_y1], thickness)
    pygame.draw.line(screen, red, [midx+right + norm_x1, 320 + norm_y1], [midx+right + norm_x1 + norm_x2, 320+norm_y1 + norm_y2], thickness)
    
    
"""
 Name:      serial_data_qtplot
 Purpose:   To plot real time data from queue.
 Param:     q2 - Queue handle
"""
def serial_data_qtplot(q2):
    kelowna_mag_raw = 367.5
    gain = 0.667
    app = QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.opts['distance'] = 3000 # This is the initial zoom of the plot
    w.show()
    w.setWindowTitle('Sample magnetometer plots')
    # Define grids properties
    g = gl.GLGridItem()
    g.scale(500, 500, 500)
    w.addItem(g)

    # Define axes properties
    ax = gl.GLAxisItem()
    ax.setSize(1000, 1000, 1000)
    w.addItem(ax)
    
    md = gl.MeshData.sphere(rows=20, cols=20, radius=kelowna_mag_raw/gain)
    m4 = gl.GLMeshItem(meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1,1,1,0.2))
    m4.translate(0,10,0)
    w.addItem(m4)

    # Maximum depth of queue. Increase this number to plot more data
    max_index = 10000
    index = 0
    p = np.zeros((max_index, 3))
    # White (1,1,1), Red(255,0,0), Blue(0,0,255), Green(0,128,0)
    sp = gl.GLScatterPlotItem(pos=p, color=(0, 0, 255, 0.6), size=50.0, pxMode=False) 
    w.addItem(sp)
    wait = 0
    while True:
        if not q2.empty():
            wait = 0
            pos = q2.get()
            p[index] = (pos[0][0], pos[0][1], pos[0][2]) # the calibrated readings from IMU1
            index += 1
            if index > max_index - 1:
                index = 0
                print "index reset"
            sp.setData(pos=p)
            app.processEvents()
        # If queue is empty,wait for data in queue. Exit if nothing
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
    print "This script should be executed after the finish of static calibration."
    print "It will show you real-time uncalibrated and calibrated plotting."
    print "The combined bias and the A-1 matrix will be used to calibrate the dynamic magnetometer readings."
    print "Press ctrl + c to stop and save the readings.\nPress ctrl + z to exit the program."
    print "The file name you enter should be identical to the one you named for the static calibration."
    print "#-----------------------------------------------------#"
    while True:
        sample_name = raw_input('Enter file name to store data ("name_dynamic"):\n')
        if not os.path.exists("../../data/%s_dynamic" % sample_name):
            os.makedirs("../../data/%s_dynamic" % sample_name)
            os.chmod("../../data/%s_dynamic" % sample_name, 0777)
            break
        else:
            sel = raw_input('File already exists, delete it? [y/n]\n')
            if (sel == 'y'):
                shutil.rmtree("../../data/%s_dynamic" % sample_name)
                print "File has been deleted"
            else:
                pass

    q = multiprocessing.Queue()
    q2 = multiprocessing.Queue()

    p1 = multiprocessing.Process(target=serial_data_get, args=(q, q2, sample_name))
    p2 = multiprocessing.Process(target=serial_data_annimate, args=(q,))
    p3 = multiprocessing.Process(target=serial_data_qtplot, args=(q2,))
    p1.start()
    while q.empty():
        print "Queue is empty"
        time.sleep(1)
    p2.start()
    p3.start()


if __name__ == '__main__':
    main()
