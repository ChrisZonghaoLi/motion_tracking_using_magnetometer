#!/usr/bin/env python

"""
This script enables static data processing of Magnetometer data

Date Modified   Modified by     Changes
Feb 11, 2017    ChrisZL         Add some instrutions on the purpose of this program;
                                minor changes on the output of "_data_stats" file
                                it will now print out the combined bias and A_inv on the terminal directly
Feb 13, 2017	ChrisZL		Update the gain of the sensor

"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import numpy.matlib
import scipy.linalg as scla
import argparse
import re


""""
 Name:      static_analysis

 Purpose:   Plot calibrated and uncalibrated data sets as well as evaluate the mean and variance

 Param:     raw and cal data
            file_name - file name to store plots

"""
def static_analysis(x_raw, y_raw, z_raw, x_cal, y_cal, z_cal, A_inv, x_bias, y_bias, z_bias, file_name, mag_raw):

    # To plot uncalibrated data
    raw_fig = plt.figure()
    ax1 = raw_fig.gca(projection='3d')

    # To plot calibrated data
    cal_fig = plt.figure()
    ax2 = cal_fig.gca(projection='3d')

    # Data histogram
    diff_fig = plt.figure()
    ax3 = diff_fig.gca()

    ax1.plot_wireframe(x_raw, y_raw, z_raw, color ='r')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')

    ax2.plot_wireframe(x_cal, y_cal, z_cal, color='g')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('z')

    diff = (numpy.sqrt(numpy.square(x_cal) + numpy.square(y_cal) + numpy.square(z_cal))) - mag_raw

    #f_stat = open("../data/%s_stats.txt" % file_name, 'w')
    #f_stat.write("Bias:\n x_bias = %.4f\n y_bias = %.4f\n z_bias = %.4f\n\n" % (x_bias, y_bias, z_bias))
    #f_stat.write("Correction matrix (A_inv):\n%s\n%s\n%s\n\n" % (A_inv[0], A_inv[1], A_inv[2]))
    #f_stat.write("Other Statisstics:\n Mean: %.4f\n Var:  %.4f\n Std:  %.4f" % (numpy.mean(diff), numpy.var(diff), numpy.std(diff)))
    #f_stat.close()

    f_stat = open("../../data/%s_Bias_&_A_inv.txt" % file_name, 'w')
    f_stat.write("%s  %s  %s\n\n" % (x_bias, y_bias, z_bias))
    f_stat.write("%f  %f  %f\n%f  %f  %f\n%f  %f  %f\n\n" % (A_inv[0,0], A_inv[0,1], A_inv[0,2], A_inv[1,0], A_inv[1,1], A_inv[1,2], A_inv[2,0], A_inv[2,1], A_inv[2,2]))
    f_stat.write("%s  %s  %s" % (numpy.mean(diff), numpy.var(diff), numpy.std(diff)))
    f_stat.close()


    hist, bins = np.histogram(diff, bins=50)
    width = 0.7 * (bins[1] - bins[0])
    center = (bins[:-1] + bins[1:]) / 2
    ax3.bar(center, hist, align='center', width=width)

    raw_fig.savefig("../../data/%s_uncal.png" % file_name)
    cal_fig.savefig("../../data/%s_cal.png" % file_name)
    diff_fig.savefig("../../data/%s_hist.png" % file_name)
    # plt.show()


"""
 Name:      static_cal

 Purpose:   Calculate A inverse matrix, bias and calibrate the magnetometer data set

 Param:     sample_path - Path to magnetometer data text file

"""
def static_cal(sample_path):
    mag_raw = 367.5 # Kelowna's raw magnetic field

    # Load data set and parse file_name from sample_path
    data = np.loadtxt(sample_path)
    f_st = re.search('/data/', sample_path)
    f_end = re.search('.txt', sample_path)
    file_name = sample_path[f_st.end():f_end.start()]

    # Write raw data to text file
    gain =  0.667
    raw_data = data* gain
    np.savetxt("../../data/%s_raw.txt" % file_name, raw_data, '%.5f')
    column_num = raw_data.shape[0]

    D = np.ones((10, column_num))
    D[0,:] = np.square(raw_data[:, 0]) # x ^ 2
    D[1,:] = np.square(raw_data[:, 1]) # y ^ 2
    D[2,:] = np.square(raw_data[:, 2]) # z ^ 2
    D[3,:] = 2 * raw_data[:, 1] * raw_data[:, 2] # 2yz
    D[4,:] = 2 * raw_data[:, 0] * raw_data[:, 2] # 2xz
    D[5,:] = 2 * raw_data[:, 0] * raw_data[:, 1] # 2xy
    D[6,:] = 2 * raw_data[:, 0] # 2x
    D[7,:] = 2 * raw_data[:, 1] # 2y
    D[8,:] = 2 * raw_data[:, 2] # 2z
    D[9,:] = np.ones((1, column_num)) #1

    # S = D * DT
    S = np.dot(D, D.transpose())

    # Split S matrix to 4 sub-matrices and evalute the SS matrix
    S_11 = S[0:6,0:6]
    S_12 = S[0:6,6:10]
    S_21 = S_12.transpose()
    S_22 = S[6:10, 6:10]
    S_22_i = np.linalg.inv(S_22)

    # SS = S_11 - S_12 * S_22_i * S_21
    SS = S_11 - np.dot(np.dot(S_12,S_22_i), S_21)

    # Constraint Matrix C
    # C = [-1, 1, 1, 0, 0, 0;
    #       1,-1, 1, 0, 0, 0;
    #       1, 1,-1, 0, 0, 0;
    #       0, 0, 0,-4, 0, 0;
    #       0, 0, 0, 0,-4, 0;
    #       0, 0, 0, 0, 0,-4];
    # Normalized C:
    C = [[0, 0.5, 0.5, 0, 0, 0],
    [0.5, 0, 0.5, 0, 0, 0],
    [0.5, 0.5, 0, 0, 0, 0],
    [0, 0, 0, -0.25, 0, 0],
    [0, 0, 0, 0, -0.25, 0],
    [0, 0, 0, 0, 0, -0.25]]

    C_i = np.linalg.inv(C)

    # Find eigenvalue of matirix E, E * V - V * D = [0]
    E = np.dot(C, SS)
    w, v = np.linalg.eig(E)

    # Find the zero - based position of the only positive eigenvalue.
    # The associated eigenvector will be in the corresponding column of matrix
    max_eigen = w[0]
    i = 0
    for k in w:
        if k>0:
            max_eigen = k
            break
        else:
            i = i+1

    # Check the sign of V1, if it is negative, flip the sign
    V1 = v[:, i]
    if V1[0] < 0:
        V1 = -1 * V1

    V2 = np.dot(np.dot(S_22_i, S_21), V1)

    # Setup Ellipsoid matrix
    Ellipsoid = np.ones((10, 1))
    Ellipsoid[0:6,0] = V1
    Ellipsoid[6:10,0] = -1 * V2[0:5]

    # Normalize Ellipsoid matrix to prevent trivial results
    Ellipsoid = Ellipsoid/np.linalg.norm(Ellipsoid)

    # Calculate the combined bias
    Q = [[Ellipsoid[0,0],Ellipsoid[5,0],Ellipsoid[4,0]], [Ellipsoid[5,0],Ellipsoid[1,0],Ellipsoid[3,0]], [Ellipsoid[4,0],Ellipsoid[3,0],Ellipsoid[2,0]]]
    Q_i = np.linalg.inv(Q)

    U = [[Ellipsoid[6,0]],[Ellipsoid[7,0]],[Ellipsoid[8,0]]]

    B = np.dot(Q_i, U)

    # Combined bias - hard Iron offset
    x_bias = -B[0,0]
    y_bias = -B[1,0]
    z_bias = -B[2,0]

    bias = np.matlib.repmat([x_bias, y_bias, z_bias], raw_data.shape[0], 1)
    
    print"\n"
    print "#-----------------------------------------------------#"
    print "This script will conduct static calibration to the targeted sensor data file."
    print "It will also do some post-processing jobs for you."
    print "Most importantly, it will give you the combined bias and A_inv matrix."
    print "They will be used in the dynamic calibration."
    print "#-----------------------------------------------------#"
    print "Combined Bias:\n%s" %(bias[0])
    
    # Find the correction matrix A_inv
    J = Ellipsoid[9,0]
    A_inv = (scla.sqrtm(Q)) * mag_raw/(np.sqrt(np.dot(np.dot(B.transpose(), Q), B) - J))
    print"\nA_inv:\n%s\n%s\n%s" %(A_inv[0], A_inv[1], A_inv[2])

    # Finally calculate and save the calibrated data
    cal_data = (np.dot(A_inv,(raw_data - bias).transpose())).transpose()
    np.savetxt("../../data/%s_cal.txt" % file_name, cal_data,'%.5e')

    # Store data points to plot and invoke plotting function
    x_raw = raw_data[:, 0]
    y_raw = raw_data[:, 1]
    z_raw = raw_data[:, 2]

    x_cal = cal_data[:, 0]
    y_cal = cal_data[:, 1]
    z_cal = cal_data[:, 2]

    # Analyze calibrated data
    static_analysis(x_raw, y_raw, z_raw, x_cal, y_cal, z_cal, A_inv, x_bias, y_bias, z_bias, file_name, mag_raw)



"""
 Name:      main

 Purpose:   Initialize command line arguments, invoke static calibration function and plot calibrated and uncalibrated
            data

 Param:     None

"""


def main():
    parser = argparse.ArgumentParser(description='Python script for static calibration of magnetometer data')
    parser.add_argument('sample_path', type=str, help='Provide data file to process')

    args = parser.parse_args()
    sample_path = args.sample_path
    static_cal(sample_path)

if __name__ == '__main__':
    main()
