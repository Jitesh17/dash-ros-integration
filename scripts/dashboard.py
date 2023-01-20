#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import division, print_function

import collections
import json
import os
import signal
import sys
import time
import traceback
from threading import Lock

import actionlib
import dash
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Point
# Plotly, Dash, and Flask
import plotly.graph_objs as go
import rospkg
import rospy
from actionlib_msgs.msg import GoalStatus
from dash import dcc, html
from flask import jsonify
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, String
from turtle_actionlib.msg import ShapeAction, ShapeGoal
from turtlesim.msg import Pose
import pathlib

# Helper functions and constants (should ideally be in a utils module)

GOAL_STATUS_TO_TXT = {
    getattr(GoalStatus, x): x for x in dir(GoalStatus) if x.isupper()}


# The app definition

APP = dash.Dash(
    __name__,
    assets_folder=os.path.join(
        rospkg.RosPack().get_path('haru_empatica'), 'dash_assets'),
    # external_stylesheets=[
    #     # {
    # #         'href': 'https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css',
    # #         'rel': 'stylesheet',
    # #         'integrity': 'sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T',
    # #         'crossorigin': 'anonymous',
    #     # },
    #     {'href': 'scripts/assets/base.css',
    #      'rel': 'stylesheet',
    #     },
    #     {'href': 'scripts/assets/clinical-analytics.css',
    #      'rel': 'stylesheet',
    #     },
    # ],, 
    # static_folder='assets',
    meta_tags=[{"name": "viewport",
                "content": "width=device-width, initial-scale=1"}],

)
APP.title = "Clinical Analytics Dashboard"
server = APP.server
APP.config.suppress_callback_exceptions = True

BASE_PATH = pathlib.Path(__file__).parent.resolve()
DATA_PATH = BASE_PATH.joinpath("data").resolve()
# dcc. [0]['relative_package_path'].append('scripts/assets/base.css')


class Dashboard(object):
    """
    Create a Flask server to display the UI and a ROS node to send commands to
    the turtlesim
    """

    # Flask
    APP_HOST = '0.0.0.0'
    APP_PORT = 8080
    APP_STATUS_URL = '/ros_api/status'
    APP_STATUS_ENDPOINT = 'ros_status'

    # Constants that determine the behaviour of the dashboard
    # Pose is published at ~62 Hz; so we'll see ~30 sec of history. Note that
    # these parameters could be set through ROS parameters or services too!
    POSE_UPDATE_INTERVAL = 1
    POSE_MAX_TIMESTEPS = 700
    TEMP_ATTRIBUTES = ['temp']

    # Constants for pertinent output fields
    SERVER_STATUS_OUTPUT_FORMAT = "Shape Server Status: {status}"

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP
        self._flask_server = self._app.server

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        # Initialize the variables that we'll be using to save information
        self._server_status = GoalStatus.LOST
        
        self._temp_history = np.ones(
            (1+len(Dashboard.TEMP_ATTRIBUTES), Dashboard.POSE_MAX_TIMESTEPS)) * np.nan
        self._history_length = 0
        self._temp_history_lock = Lock()
        # self._pose_history_lock = Lock()

        # Setup the subscribers, action clients, etc.
        # self._shape_client = actionlib.SimpleActionClient(Dashboard.TURTLE_SHAPE_ACTION_NAME, ShapeAction)
        # self._pose_sub = rospy.Subscriber(
        #     'Temperature', Float32, self.temp_callback)
        rospy.Subscriber('Blood_Volume_Pulse', Float32, self.temp_callback)

        # rospy.Subscriber('Accelerometer', Point, self.acc_callback)

        # Initialize the application
        self.beats = collections.deque(np.zeros(100))
        print("CPU: {}".format(self.beats))

        fig, self.ax = plt.subplots()
        self.ax.set_ylim(35, 38)
        self._define_app()
        # plt.show()

    @property
    def pose_history(self):
        return self._pose_history[:, :self._history_length]

    @property
    def temp_history(self):
        return self._temp_history[:, :self._history_length]

    def start(self):
        # rospy.loginfo("Connecting to turtle_shape...")
        # self._shape_client.wait_for_server()
        # rospy.loginfo("...turtle_shape connected.")
        self._app.run_server(host=Dashboard.APP_HOST,
                             port=Dashboard.APP_PORT,
                             debug=True)

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown (cannot use rospy now!)
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def _define_app(self):
        """
        Define the app layout and callbacks here
        """
        
        # Then the section that will display the status of the shape server
        server_status_layout = html.Div(
            dcc.Markdown(id='server-status', className='col'),
            className='row my-2'
        )
        self._app.layout = html.Div(
            id="app-container", children=[

                # Banner
                html.Div(
                    id="banner",
                    className="banner",
                    children=[
                        # html.Img(src=app.get_asset_url("plotly_logo.png")),
                        html.H2(f"Welcome to the {self._app.title}"), 
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
                        dcc.Graph(id="temp")  # , style={'display': 'inline-block'}),
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
                # The interval component to update the plots
                dcc.Interval(id='interval-component',
                             n_intervals=0,
                             interval=(Dashboard.POSE_UPDATE_INTERVAL * 1000)),
            ])


        self._app.callback(
            dash.dependencies.Output('temp', 'figure'),
            [dash.dependencies.Input('interval-component', 'n_intervals')]
        )(self._define_temp_history_callback())


        # Add the flask API endpoints
        self._flask_server.add_url_rule(
            Dashboard.APP_STATUS_URL,
            Dashboard.APP_STATUS_ENDPOINT,
            self._flask_status_endpoint
        )

    def _define_server_status_callback(self):
        """
        Define a callback to populate the server status display when the status
        refresh button (hidden) is pressed
        """
        def server_status_callback(n_clicks):
            status = GOAL_STATUS_TO_TXT.get(self._server_status)
            return Dashboard.SERVER_STATUS_OUTPUT_FORMAT.format(**locals())

        return server_status_callback


    def _define_temp_history_callback(self):
        """
        Define a callback that will be invoked on every update of the interval
        component. Keep in mind that we return a callback here; not a result
        """
        def temp_history_callback(n_intervals):
            # Get a view into the latest pose history
            temp_history = self.temp_history

            # Create the output graph
            data = [
                go.Scatter(
                    name=attr,
                    x=temp_history[0, :],
                    y=temp_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.TEMP_ATTRIBUTES)
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
                # margin=dict(
                #     autoexpand=True
                # ),
                # yaxis_range=[-111,111]
            )

            return {'data': data, 'layout': layout}

        return temp_history_callback

    def _flask_status_endpoint(self):
        return jsonify({
            'server_status': self._server_status,
        })

    def temp_callback(self, data):
        # with self.mutex:
        rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
        # rospy.loginfo('I heard Temp of %s', data.data)
        # self.beats.popleft()
        # self.beats.append(data.data)
        # print("CPU: {}".format(self.beats))

        # self.ax.cla()

        # self.ax.plot(self.beats)
        # self.ax.scatter(len(self.beats)-1, self.beats[-1])
        # self.ax.text(len(self.beats)-1, self.beats[-1]+2, "{}".format(int(self.beats[-1])))
        # self.ax.set_ylim(35,38)

        # plt.draw()

        if self._history_length == Dashboard.POSE_MAX_TIMESTEPS:
            self._temp_history[:, :-1] = self._temp_history[:, 1:]
        else:
            self._history_length += 1

        self._temp_history[:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            # rospy.Time.now().to_time(),

            data.data,
        ]
    def acc_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' I heard %s %s %s', data.x,data.y,data.z)


        # if self._history_length == Dashboard.POSE_MAX_TIMESTEPS:
        #     self._temp_history[:, :-1] = self._temp_history[:, 1:]
        # else:
        #     self._history_length += 1

        # self._temp_history[:, self._history_length-1] = [
        #     rospy.Time.now().to_time() % 1000,
        #     data.data,
        # ]
