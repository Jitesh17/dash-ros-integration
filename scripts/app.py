import datetime
import pathlib
from datetime import datetime as dt
from threading import Lock, Thread

import dash
import numpy as np
import pandas as pd
from dash import dcc, html
from dash.dependencies import ClientsideFunction, Input, Output
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import collections
import rospy

from std_msgs.msg import String, Float32


app = dash.Dash(
    __name__,
    meta_tags=[{"name": "viewport",
                "content": "width=device-width, initial-scale=1"}],
)
app.title = "Clinical Analytics Dashboard"

server = app.server
app.config.suppress_callback_exceptions = True

dcc._css_dist[0]['relative_package_path'].append('mycss.css')

# Path
BASE_PATH = pathlib.Path(__file__).parent.resolve()
DATA_PATH = BASE_PATH.joinpath("data").resolve()
app.layout = html.Div(
    id="app-container", children=[

        # Banner
        html.Div(
            id="banner",
            className="banner",
            children=[
                # html.Img(src=app.get_asset_url("plotly_logo.png")),
                html.H2(f"Welcome to the {app.title}"), 
                #             html.Div(
                #     # id="description-card",
                #     children=[
                #         # html.H5("Clinical Analytics"),
                #         html.H3("Welcome to the Clinical Analytics Dashboard"),
                #     ],
                # )
                ],
        ),
        html.Div(id='graph_card', children=[
            html.Div(className="four columns", children=[
                dcc.Graph(id="graph1")  # , style={'display': 'inline-block'}),
            ]),
            html.Div(className="four columns", children=[
                dcc.Graph(id="graph2")  # , style={'display': 'inline-block'}),
            ]),
            html.Div(className="four columns", children=[
                dcc.Graph(id="graph3")  # , style={'display': 'inline-block'})
            ]),
        ]),
        html.Div(id='graph_card', children=[
            html.Div(className="four columns", children=[
                # , style={'display': 'inline-block'}),
                dcc.Graph(id="graph4")
            ]),
            html.Div(className="four columns", children=[
                # , style={'display': 'inline-block'}),
                dcc.Graph(id="graph5")
            ]),
            html.Div(className="four columns", children=[
                # , style={'display': 'inline-block'})
                dcc.Graph(id="graph6")
            ]),
        ]),
    ])

class Listener:
    def __init__(self,):
        self.beats = collections.deque(np.zeros(100))
        print("CPU: {}".format(self.beats))


        fig, self.ax = plt.subplots()
        self.ax.set_ylim(35,38)
        self.mutex = Lock()
        self.listener()


    def callback(self, data):
        with self.mutex:
            #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
            rospy.loginfo('I heard Temp of %s', data.data)
            self.beats.popleft()
            self.beats.append(data.data)
            print("CPU: {}".format(self.beats))

            self.ax.cla()

            self.ax.plot(self.beats)
            self.ax.scatter(len(self.beats)-1, self.beats[-1])
            self.ax.text(len(self.beats)-1, self.beats[-1]+2, "{}".format(int(self.beats[-1])))
            self.ax.set_ylim(35,38)

            plt.draw()

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener_temp', anonymous=True)
        rospy.Subscriber('Temperature', Float32, self.callback)
        print('ayy its your boy')
        plt.show()

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()
    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
def runRos(a):
    print("Ros starts")
    a.spin()

if __name__ == "__main__":
    # listen = Listener()

    # ros_thread = Thread(target=runRos, args=[listen])
    # ros_thread.start()
    app.run_server(debug=True)
