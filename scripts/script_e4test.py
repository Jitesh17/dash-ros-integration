#!/usr/bin/env python3

import sys
import time
import socket
import datetime
import argparse

import rospy
from std_msgs.msg import String

from e4client import *

parser = argparse.ArgumentParser(description='e4client : Client for E4 streaming server and save the data to a CSV file')
parser.add_argument('--sockHost', action='store', dest='sockHost', default='172.24.40.109' ,help='IP address of the PC where E4 streaming server is running. [default:127.0.0.1]')
parser.add_argument('--sockPort', action='store', dest='sockPort', default=8005, type=int, help='Port number [Default: 28000]')
parser.add_argument('--deviceID', action='store', dest='deviceID', default=0, type=int, help='Device ID to save the data, 0, 1, 2... [Default: 0]')
parser.add_argument('--outCsvFname', action='store', dest='outCsvFname', default='' ,help='Output CSV filename. If vacant, ignored. The filename can include %%d, %%t, and %%s. %%d is substituted by deviceID. %%t is substituted by YYYY_MM_DD-HH_MM_SS. %%s is substituted by ACC, BVP, GSR, TEMP, IBI, HR, BAT, TAG. If %%s is missing, it is added at the end. If -1, the filename is automatically determined as %%t_%%d_%%s.csv. [default:]')
args = parser.parse_args()

dataTypeList = ['ACC','BVP','GSR','TEMP','IBI','HR','BAT','TAG']

outCsvFname = args.outCsvFname
if outCsvFname == "-1":
    outCsvFname = "%t_%d_%s.csv"
if len(outCsvFname.split("%d")) == 2:
    outCsvFnameL = outCsvFname.split("%d")
    outCsvFname = outCsvFnameL[0] + str(args.deviceID) + outCsvFnameL[1]
elif len(outCsvFname.split("%d")) > 2:
    print('Error : len(outCsvFname.split("%%d")) > 2.')
    quit()
if len(outCsvFname.split("%t")) == 2:
    outCsvFnameL = outCsvFname.split("%t")
    now = datetime.datetime.now()
    strTmp = "%02d_%02d_%02d-%02d_%02d_%02d" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
    outCsvFname = outCsvFnameL[0] + strTmp + outCsvFnameL[1]
elif len(outCsvFname.split("%t")) > 2:
    print('Error : len(outCsvFname.split("%%t")) > 2.')
    quit()
fnameList = []
if args.outCsvFname != "":
    if len(outCsvFname.split("%s")) == 1:
        outCsvFname += "%s"
    if len(outCsvFname.split("%s")) > 2:
        print('Error : len(outCsvFname.split("%%s")) > 2.')
        quit()
    for s in dataTypeList:
        fname = outCsvFname % (s)
        fnameList.append(fname)
        print("Opening:", fname)
        fid = open(fname, 'w')
        fid.close()

pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)

def print_sub(stream_id, timestamp, *sample) -> None:
    now = time.time()
    # print(now, stream_id.name, timestamp, *sample)
    if len(fnameList) == 0:
        print(now, stream_id.name, timestamp, *sample)
        rospy.loginfo(sample)
        pub.publish(sample)
    else:
        dataType = stream_id.name
        idx = [i for i in range(len(dataTypeList)) if dataTypeList[i] == dataType]
        if len(idx) == 0:
            print("Warning : dataType(=%s) is not in the list..." % (dataType))
        if len(idx) == 1:
            sampleTmp=",".join(["%s" % (str(i)) for i in list(sample)])
            fid = open(fnameList[idx[0]], 'a')
            fid.write(f'{now},{timestamp},{sampleTmp}\n')
            fid.close()
            if dataType == "TAG":
                print(now, stream_id.name, timestamp, *sample)

if __name__ == '__main__':
    with E4StreamingClient(args.sockHost, args.sockPort) as client:
        devs = client.list_connected_devices()
        print('Num of devices:',len(devs))
        if len(devs) <= args.deviceID:
            print('Error : Num of devices(=%d) <= args.deviceID(=%d)...' % (len(devs), args.deviceID))
            quit()
        with client.connect_to_device(devs[args.deviceID]) as conn:
            conn.subscribe_to_stream(E4DataStreamID.ACC, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.BVP, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.GSR, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.TEMP, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.IBI, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.HR, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.BAT, print_sub)
            conn.subscribe_to_stream(E4DataStreamID.TAG, print_sub)
            input('Press Enter or Ctrl-C to exit...')
