#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import division, print_function

# import collections
# import json
import os
import pathlib
import signal
import sys
import time
# import traceback
from threading import Lock

# import actionlib
import dash
# import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go
import rospkg
import rospy
from actionlib_msgs.msg import GoalStatus
from dash import dcc, html
from flask import jsonify
from geometry_msgs.msg import Point
# from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32
# from turtle_actionlib.msg import ShapeAction, ShapeGoal
# from turtlesim.msg import Pose

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
    TEMP_UPDATE_INTERVAL = 1
    TEMP_MAX_TIMESTEPS = 400
    UPDATE_INTERVAL = {
        'acc': 1,
        'bvp': 1,
        'gsr': 1,
        'hr': 1,
        'ibi': 1,
        'temp': 1,
    }
    MAX_TIMESTEPS = {
        'acc': 400,
        'bvp': 400,
        'gsr': 400,
        'hr': 400,
        'ibi': 400,
        'temp': 400,
    }
    ATTRIBUTES = {
        'acc': ['X', 'Y', 'Z', ],
        'bvp': ['Blood_Volume_Pulse', ],
        'gsr': ['Galvanic_Skin_Response', ],
        'hr': ['Heart_Rate', ],
        'ibi': ['IBI', ],
        'temp': ['Temperature', ],
    }
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

        self._history = {}
        self._history_lock = {}
        for key in Dashboard.ATTRIBUTES:
            self._history[key] = np.ones(
                (1+len(Dashboard.ATTRIBUTES[key]), Dashboard.MAX_TIMESTEPS[key])) * np.nan
            self._history_lock[key] = Lock()
        self._history_length = 0

        # Setup the subscribers
        rospy.Subscriber('Accelerometer', Point, self.acc_callback)
        rospy.Subscriber('Blood_Volume_Pulse', Float32, self.bvp_callback)
        rospy.Subscriber('Galvanic_Skin_Response', Float32, self.gsr_callback)
        rospy.Subscriber('Heart_Rate', Float32, self.hr_callback)
        rospy.Subscriber('IBI', Float32, self.ibi_callback)
        rospy.Subscriber('Temperature', Float32, self.temp_callback)

        self._define_app()

    @property
    def acc_history(self):
        return self._history['acc'][:, :self._history_length]

    @property
    def bvp_history(self):
        return self._history['bvp'][:, :self._history_length]

    @property
    def gsr_history(self):
        return self._history['gsr'][:, :self._history_length]

    @property
    def hr_history(self):
        return self._history['hr'][:, :self._history_length]

    @property
    def ibi_history(self):
        return self._history['ibi'][:, :self._history_length]

    @property
    def temp_history(self):
        return self._history['temp'][:, :self._history_length]

    def start(self):
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
                        html.H2(f"Welcome to the {self._app.title}"),
                    ],
                ),
                html.Div(id='graph_card', children=[
                    html.Div(className="four columns", children=[
                        html.H3(f"Accelerometer"),
                        dcc.Graph(id="acc")
                    ]),
                    html.Div(className="four columns", children=[
                        html.H3(f"Blood_Volume_Pulse"),
                        dcc.Graph(id="bvp")
                    ]),
                    html.Div(className="four columns", children=[
                        html.H3(f"Galvanic_Skin_Response"),
                        dcc.Graph(id="gsr")
                    ]),
                ]),
                html.Div(id='graph_card', children=[
                    html.Div(className="four columns", children=[
                        html.H3(f"Heart_Rate"),
                        dcc.Graph(id="hr")
                    ]),
                    html.Div(className="four columns", children=[
                        html.H3(f"IBI"),
                        dcc.Graph(id="ibi")
                    ]),
                    html.Div(className="four columns", children=[
                        html.H3(f"Temperature"),
                        dcc.Graph(id="temp")
                    ]),
                ]),
                # The interval component to update the plots
                dcc.Interval(id='interval-component-acc',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['acc'] * 100)),
                dcc.Interval(id='interval-component-bvp',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['bvp'] * 100)),
                dcc.Interval(id='interval-component-gsr',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['gsr'] * 100)),
                dcc.Interval(id='interval-component-hr',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['hr'] * 100)),
                dcc.Interval(id='interval-component-ibi',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['ibi'] * 100)),
                dcc.Interval(id='interval-component-temp',
                             n_intervals=0,
                             interval=(Dashboard.UPDATE_INTERVAL['temp'] * 100)),
            ])

        self._app.callback(
            dash.dependencies.Output('acc', 'figure'),
            [dash.dependencies.Input('interval-component-acc', 'n_intervals')]
        )(self._define_acc_history_callback())

        self._app.callback(
            dash.dependencies.Output('bvp', 'figure'),
            [dash.dependencies.Input('interval-component-bvp', 'n_intervals')]
        )(self._define_bvp_history_callback())

        self._app.callback(
            dash.dependencies.Output('gsr', 'figure'),
            [dash.dependencies.Input('interval-component-gsr', 'n_intervals')]
        )(self._define_gsr_history_callback())

        self._app.callback(
            dash.dependencies.Output('hr', 'figure'),
            [dash.dependencies.Input('interval-component-hr', 'n_intervals')]
        )(self._define_hr_history_callback())

        self._app.callback(
            dash.dependencies.Output('ibi', 'figure'),
            [dash.dependencies.Input('interval-component-ibi', 'n_intervals')]
        )(self._define_ibi_history_callback())

        self._app.callback(
            dash.dependencies.Output('temp', 'figure'),
            [dash.dependencies.Input('interval-component-temp', 'n_intervals')]
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
    

    def _define_acc_history_callback(self):
        def acc_history_callback(n_intervals):
            acc_history = self.acc_history
            data = [
                go.Scatter(
                    name=attr,
                    x=acc_history[0, :],
                    y=acc_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['acc'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return acc_history_callback

    def _define_bvp_history_callback(self):
        def bvp_history_callback(n_intervals):
            bvp_history = self.bvp_history
            data = [
                go.Scatter(
                    name=attr,
                    x=bvp_history[0, :],
                    y=bvp_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['bvp'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return bvp_history_callback

    def _define_gsr_history_callback(self):
        def gsr_history_callback(n_intervals):
            gsr_history = self.gsr_history
            data = [
                go.Scatter(
                    name=attr,
                    x=gsr_history[0, :],
                    y=gsr_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['gsr'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return gsr_history_callback

    def _define_hr_history_callback(self):
        def hr_history_callback(n_intervals):
            hr_history = self.hr_history
            data = [
                go.Scatter(
                    name=attr,
                    x=hr_history[0, :],
                    y=hr_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['hr'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return hr_history_callback

    def _define_ibi_history_callback(self):
        def ibi_history_callback(n_intervals):
            ibi_history = self.ibi_history
            data = [
                go.Scatter(
                    name=attr,
                    x=ibi_history[0, :],
                    y=ibi_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['ibi'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return ibi_history_callback
    
    def _define_temp_history_callback(self):
        def temp_history_callback(n_intervals):
            temp_history = self.temp_history
            data = [
                go.Scatter(
                    name=attr,
                    x=temp_history[0, :],
                    y=temp_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.ATTRIBUTES['temp'])
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
            )
            return {'data': data, 'layout': layout}
        return temp_history_callback

    def _flask_status_endpoint(self):
        return jsonify({
            'server_status': self._server_status,
        })

    def temp_callback(self, data):
        if self._history_length == Dashboard.MAX_TIMESTEPS['temp']:
            self._history['temp'][:, :-1] = self._history['temp'][:, 1:]
        else:
            self._history_length += 1

        self._history['temp'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.data,
        ]

    def acc_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() +
                      ' I heard %s %s %s', data.x, data.y, data.z)

        if self._history_length == Dashboard.MAX_TIMESTEPS['acc']:
            self._history['acc'][:, :-1] = self._history['acc'][:, 1:]
        else:
            self._history_length += 1

        self._history['acc'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.x,
            data.y,
            data.z,
        ]

    def bvp_callback(self, data):

        if self._history_length == Dashboard.MAX_TIMESTEPS['bvp']:
            self._history['bvp'][:, :-1] = self._history['bvp'][:, 1:]
        else:
            self._history_length += 1

        self._history['bvp'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.data,
        ]

    def gsr_callback(self, data):
        if self._history_length == Dashboard.MAX_TIMESTEPS['gsr']:
            self._history['gsr'][:, :-1] = self._history['gsr'][:, 1:]
        else:
            self._history_length += 1

        self._history['gsr'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.data,
        ]

    def hr_callback(self, data):
        if self._history_length == Dashboard.MAX_TIMESTEPS['hr']:
            self._history['hr'][:, :-1] = self._history['hr'][:, 1:]
        else:
            self._history_length += 1

        self._history['hr'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.data,
        ]

    def ibi_callback(self, data):
        if self._history_length == Dashboard.MAX_TIMESTEPS['ibi']:
            self._history['ibi'][:, :-1] = self._history['ibi'][:, 1:]
        else:
            self._history_length += 1

        self._history['ibi'][:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            data.data,
        ]
