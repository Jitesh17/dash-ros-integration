import dash
from dash import dcc
from dash import html
from dash.dependencies import Input, Output, ClientsideFunction

import numpy as np
import pandas as pd
import datetime
from datetime import datetime as dt
import pathlib

app = dash.Dash(
    __name__,
    meta_tags=[{"name": "viewport",
                "content": "width=device-width, initial-scale=1"}],
)
app.title = "Clinical Analytics Dashboard"

server = app.server
app.config.suppress_callback_exceptions = True

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


if __name__ == "__main__":
    app.run_server(debug=True)
