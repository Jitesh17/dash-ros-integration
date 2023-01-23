#!/usr/bin/env python3
import plotly.express as px
import pandas as pd

import csv

data = pd.read_csv("2023_01_11-09_52_20_0_GSR.csv", names=['time1', 'time2', 'data'], header=None)

data.columns

print(data)

print(data.columns)
print(data['data'])
rows = []

#df = px.data.gapminder().query("country=='Canada'")
fig = px.line(data, x="time2", y="data", title='Galvanic Skin Response', markers=True)
fig.show()


#df = px.data.gapminder().query("country=='Canada'")
#fig = px.line(df, x="year", y="lifeExp", title='Life expectancy in Canada')
#fig.show()