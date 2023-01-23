#!/usr/bin/env python3
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import pandas as pd

import csv

data_gsr = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_GSR.csv", names=['time1', 'time2', 'data'], header=None)
data_hr = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_HR.csv", names=['time1', 'time2', 'data'], header=None)
data_bvp = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_BVP.csv", names=['time1', 'time2', 'data'], header=None)
data_ibi = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_IBI.csv", names=['time1', 'time2', 'data'], header=None)
data_temp = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_TEMP.csv", names=['time1', 'time2', 'data'], header=None)
data_acc = pd.read_csv("recordings/weight/2023_01_11-09_52_20_0_ACC.csv", names=['time1', 'time2', 'datax','datay','dataz'], header=None)

baseline = data_gsr['time2'][0]

data_gsr['time2'] = data_gsr['time2'] - baseline
data_hr['time2'] = data_hr['time2'] - baseline
data_bvp['time2'] = data_bvp['time2'] - baseline
data_ibi['time2'] = data_ibi['time2'] - baseline
data_temp['time2'] = data_temp['time2'] - baseline
data_acc['time2'] = data_acc['time2'] - baseline

print(data_gsr)

print(data_gsr.columns)
print(data_gsr['time2'])
print('baseline')
print(baseline)

fig = make_subplots(rows=3, cols=3, subplot_titles=("Galvanic Skin Response", "Heart Rate", "Accelerometer X",
    "Temperature", "Blood Volume Pulse", "Accelerometer Y", "", "Interbeat Interval","Accelerometer Y"))

fig.add_trace(
    go.Scatter(x=data_gsr['time2'], y=data_gsr['data']), row = 1, col = 1
) 
fig.add_trace(
    go.Scatter(x=data_hr['time2'], y=data_hr['data']), row = 1 , col = 2
)
fig.add_trace(
    go.Scatter(x=data_bvp['time2'], y=data_bvp['data']), row = 2, col = 2
) 
fig.add_trace(
    go.Scatter(x=data_temp['time2'], y=data_temp['data']), row = 2 , col = 1
)
fig.add_trace(
    go.Scatter(x=data_ibi['time2'], y=data_ibi['data']), row = 3 , col = 2
)


fig.add_trace(
    go.Scatter(x=data_acc['time2'], y=data_acc['datax']), row = 1 , col = 3
)
fig.add_trace(
    go.Scatter(x=data_acc['time2'], y=data_acc['datay']), row = 2, col = 3
)
fig.add_trace(
    go.Scatter(x=data_acc['time2'], y=data_acc['dataz']), row = 3 , col = 3
)
fig.show()