import csv
import sys
import pandas as pd
import numpy as np
import logging
import re
import json
import os
import matplotlib.pyplot as plt

'''
Importing a library for handling,evaluating and comparing the trajectory output of odometry and SLAM algorithms. Ref: https://github.com/MichaelGrupp/evo/
'''
import evo.core.lie_algebra as lie
from evo.core import trajectory,sync
from evo.tools import plot, file_interface, log

# Defining Radius  of earth
R = 6378137.

# Defining quaternions of identity rotation matrix.
Q = np.array([ '0','0','0','1'])

class Point:
    '''
    Class of point
    '''
    def __init__(self,lat,lng,alt):
        self.__lat = (lat)
        self.__lng = (lng)
        self.__alt = alt
        
    def get_location(self):
        return self.__lat, self.__lng,self.__alt 

def calcDist(point1, point2):
    '''
    Calculates earth distances given two gps location.
    :param point1, point2: gps locations
    :return: distance between the two gps locations in float. 
    '''
    if (not isinstance(point1,Point) or  not isinstance(point2,Point)):
        sys.exit( "Points type doesn't match")
    if (point1 == None or point2 == None):
        sys.exit( "Points Cannot be none")
        return 
    lat1,lng1,_ = point1.get_location()
    lat2,lng2,_ = point2.get_location()
    dlng = lng2 - lng1
    dlat = lat2 - lat1
    #print(dlng,dlat)
    a = (np.sin(dlat/2.))**2 +np.cos(lat1/1.)*np.cos(lat2/1.) * (np.sin(dlng/2.))**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    d = R * c
    return float(d)

def idd_to_tum(pred_idd,baseDir):
    '''
    Takes the predicted trajectory in idd format [timestamps, tx,ty,tz] to tum format.
    :param pred_idd: submission file in idd format, baseDir: path to the submission folder
    :return: None
    '''
    idd = pd.read_csv(pred_idd)
    idd['timestamp']= pd.to_datetime(idd['timestamp'],format = '%H-%M-%S-%f')
    pred_idd = os.path.basename(pred_idd)
    path_name = pred_idd.split('.')[0]
    # Generate the /tmp directory which contains temporary file.
    if(os.path.exists(baseDir + "/tmp")):
        pass
    else:
        os.makedirs(baseDir+'/tmp')
    tum = open(baseDir + '/tmp/'+path_name+'.tum','w')
    initial_ts = pd.Timestamp((idd.at[0,'timestamp']))
    for i,r in idd.iterrows():
        delta_time = str((idd.at[i,'timestamp']-initial_ts).total_seconds())
        t = np.array([idd.at[i,'tx'],idd.at[i,'ty'],idd.at[i,'tz']])
        line = np.append(np.append(delta_time,t),Q)
        tum.write(' '.join(line)+'\n')

def align(tum_gt,tum_pred,baseDir):
    '''
    Takes the ground truth and the predicted traj in tum format  from the /tmp folder and aligns the predicted traj with ground truth traj in the best possible way.
    :param tum_gt: ground truth file in tum format, tum_pred: predicted file in tum format, baseDir: path to the submission folder
    :return: None
    '''
    file_name = os.path.basename(tum_pred).split('.')[0]
    traj_gt = file_interface.read_tum_trajectory_file(tum_gt)
    traj_pred = file_interface.read_tum_trajectory_file(tum_pred)
    # Align the predicted trajectory with the ground truth trajectory.
    traj_est_aligned_scaled = trajectory.align_trajectory(traj_pred, traj_gt,correct_scale=True)
    dest = baseDir + '/tmp/'+file_name+'_aligned.tum'
    # Save the aligned trajectory to /tmp folder
    file_interface.write_tum_trajectory_file(dest, traj_est_aligned_scaled, confirm_overwrite=True)
    
def pose_from_gps(packet, scale):
    '''
    Helper method to compute a SE(3) pose matrix from an OXTS packet.
    '''
    # Use a Mercator projection to get the translation vector. Ref: https://github.com/utiasSTARS/pykitti/blob/d3e1bb81676e831886726cc5ed79ce1f049aef2c/pykitti/utils.py#L85
    tx = scale * packet['lng'] * np.pi * R / 180.
    ty = scale * R * np.log(np.tan((90. + packet['lat']) * np.pi / 360.))
    tz = packet['alt']
    t = np.array([tx, ty, tz])
    return t

def gps_to_tum(inpf,opf):
    '''
    Takes the ground truth  in format [timestamps, image_idx,latitude,longitude,altitude] to tum format.
    :param inpf: path to input file i.e. the ground truth d*_gt.csv file opf: path to where you want to store the converted file   
    :return: None
    '''
    file_name = os.path.basename(inpf)
    print(file_name)
    gps = pd.read_csv(inpf)
    scale = None
    origin = None
    f = open(opf,'w')
    for i,r in gps.iterrows():
        line = {}
        line['lat'] = float(gps.at[i,'latitude'])
        line['lng'] = float(gps.at[i,'longitude'])
        line['alt'] = float(gps.at[i,'altitude'])
        # Calculate the scale
        if scale is None:
            scale = np.cos(line['lat'] * np.pi / 180.)
        t = pose_from_gps(line,scale)
        if  origin is None:
            first_time = pd.to_datetime(gps.at[i,'timestamp'], format = "%H-%M-%S-%f")
            origin = t
        # Calculate translation w.r.t origin.
        T_w = (t - origin).reshape((1,3))
        arr = np.append(np.array(((pd.to_datetime(gps.at[i,'timestamp'], format = "%H-%M-%S-%f"))-first_time).total_seconds()),np.append(np.array(T_w,'str'),Q))
        str_arr = " ".join(arr)
        f.write(str_arr+'\n')
                
def calcError(gps_gtFileName,aligned_predFileName):
    '''
    Takes the predicted aligned trajectory in the tum format  and ground truth in format(timestamp,image_idx,latitude,longitude,altitude)
    :param gps_gtFileName: path to ground truth file i.e. d*_gt.csv aligned_predFileName: path to aligned predicted trajectory in tum format
    :return: final error in float
    '''
    gps_gt_csv = pd.read_csv(gps_gtFileName)
    aligned_tum = pd.read_csv(aligned_predFileName, header = None, usecols = [0,1,2,3], names = ['timedelta','tx','ty','tz'], sep = ' ')
    gps_gt_csv['timestamp']= pd.to_datetime(gps_gt_csv['timestamp'], format = '%H-%M-%S-%f')
    aligned_tum['timedelta']= pd.to_timedelta(aligned_tum['timedelta'], unit='s')
    # Get the timestamp of the inital point
    initial_ts = pd.Timestamp((gps_gt_csv.at[0,'timestamp']))
    aligned_tum['timedelta'] += initial_ts
    # Get the gps of the initial point
    lng_origin = float(gps_gt_csv.at[0,'longitude'])
    lat_origin = float(gps_gt_csv.at[0,'latitude'])
    alt_origin = float(gps_gt_csv.at[0,'altitude'])
    scale = np.cos(np.radians(lat_origin))
    E = []
    for i,r in aligned_tum.iterrows():
        tx_i = aligned_tum.at[i,'tx']
        ty_i = aligned_tum.at[i,'ty']
        tz_i = aligned_tum.at[i,'tz']
        # Do the inverse mercator projection i.e. calculate the gps coordinates from translation vector[tx,ty,tz]
        origin = [scale*np.radians(lng_origin)*R,scale * R * np.log(np.tan((90. + lat_origin) * np.pi / 360.)),alt_origin]
        t_abs = np.array([tx_i,ty_i,tz_i]) + np.array(origin)
        lng_i = (t_abs[0]*180.)/(scale*np.pi*R)
        lat_i = (np.arctan(np.e**(t_abs[1]/(scale*R)))*(360.)/(np.pi))-90.
        alt_i = t_abs[2]
        lng_gt_i = gps_gt_csv.at[i,'longitude']
        lat_gt_i = gps_gt_csv.at[i,'latitude']
        alt_gt_i = gps_gt_csv.at[i,'altitude']
        # Calculate the earth distance between ground truth gps location and predicted gps location.
        dist = calcDist(Point(lat_i,lng_i,alt_i),Point(lat_gt_i,lng_gt_i,alt_gt_i))
        E.append(dist)
    return np.mean(E)

def main(baseDir,gt):
    '''
    Takes in the submission directory and the ground truth directory directory and performs the evaluation.
    :param baseDir: path to submission directory, gt: path to ground truth directory
    :return : array of Error
    '''
    routes = ['d0','d1','d2']
    E = []
    for ds in routes:
        submission_file = baseDir + '/' + ds + '.csv'
        tum_gt = baseDir + '/tmp/'+ds+'_gt.tum'
        tum_pred = baseDir + '/tmp/'+ds+'.tum'
        gt_gps = gt + '/' + ds+'_gt.csv'
        aligned_pred_tum = baseDir + '/tmp/' + ds + '_aligned.tum'
        # Check if the submission file exist.
        if(not os.path.exists(submission_file)):
            sys.exit("File not found: " + submission_file + " at location " + baseDir + ".")
        # Check if the ground truth file exist.
        if(not os.path.exists(gt_gps)):
            sys.exit("File not found: " + gt_gps + " at location " + gt + ".")
        data1 = pd.read_csv(submission_file)
        data2 = pd.read_csv(gt_gps)
        # Check if the number of points in ground truth and submitted file are same.
        if(data1.shape[0]!=data2.shape[0]):
            sys.exit("Number of outputs does not match with the ground truth in "+ ds +" submission file.")
        # Check for the format of submitted file.
        if(data1.shape[1]!=4 or (not (data1.columns==['timestamp','tx','ty','tz',]).all())):
            sys.exit("Please check the format for "+ ds +" submission file.")
        # Check for the format of ground truth file.
        if(data2.shape[1]!=4 or (not (data2.columns==['timestamp','latitude','longitude','altitude']).all()) ):
            sys.exit("Please check the format for "+ ds +" ground truth file.")
        idd_to_tum(submission_file,baseDir)
        gps_to_tum(gt_gps,tum_gt)
        align(tum_gt,tum_pred,baseDir)
        e = calcError(gt_gps,aligned_pred_tum)
        E.append(e)
        logger.info("\Error for " + ds + " :" + str(e))
        print(e)
    return E

if __name__ =="__main__":
    logger = logging.getLogger("Evaluation")
    log.configure_logging(verbose = True)
    baseDir = sys.argv[1]
    gt = sys.argv[2]
    E = main(baseDir,gt)
    print("Final Error:")
    print(np.mean(E))

