#!/usr/bin/env python3.8
# ROS imports
import sys
import subprocess
from pydoc import classname
import rospy
from std_msgs.msg import Float32

# dash imports
import dash
from dash.dependencies import Output, Input
from dash import dcc
from dash import html
import dash_daq as daq
from dash.exceptions import PreventUpdate

import plotly
import random
import plotly.graph_objs as go

from time import sleep
from datetime import datetime
import math

enable_box = False
major_component_spacing_vw = 0.5
num_rows = 12
num_cols = 4
horizontal_dividers = ['1', '3', '8', str(num_rows + 1)]
vertical_dividers = ['1', '2', '3', str(num_cols + 1)]

# bird's eye graph dimension
min_x = -32
max_x = 32
x_dtick = 10
min_y = -40
max_y = 128
y_dtick = (max_y - min_y) / (max_x - min_x) * x_dtick

screen_refresh_time_ms = 500
fault_refresh_time_ms = screen_refresh_time_ms * 10
critical_temp_refresh_time_ms = screen_refresh_time_ms * 10
is_dark_mode = True


def create_gauge_fig(interval, max_safe_temp, temp_var_name, title,
                     number_format):
    low_mark = max_safe_temp * 0.8
    high_mark = max_safe_temp * 1.2
    bar_color = 'green' if globals()[temp_var_name] < low_mark else (
        'orange' if globals()[temp_var_name] < high_mark else 'red')

    fig = go.Figure(
        go.Indicator(domain={
            'x': [0.1, 0.85],
            'y': [0, 0.9]
        },
            value=globals()[temp_var_name],
            mode="gauge+number",
            title={
            'text': title,
            'font': {
                'color': 'white' if is_dark_mode else 'black',
                'size': 20
            }
        },
            number={
            'font': {
                'color': 'white' if is_dark_mode else 'black'
            },
            'valueformat': number_format
        },
            gauge={
            'axis': {
                'range': [None, high_mark],
                'tickfont': {
                    'color': 'white' if is_dark_mode else 'black',
                    'size': 13
                },
                'tickmode': 'array',
                'tickvals': [0, low_mark, max_safe_temp]
            },
            'steps': [{
                'range': [0, low_mark],
                'color': "lightgray" if is_dark_mode else 'gray'
            }, {
                'range': [low_mark, max_safe_temp],
                'color': "gray" if is_dark_mode else 'dimgray'
            }],
            'bar': {
                'color': bar_color
            },
            'bordercolor':
            '#AAA' if is_dark_mode else '#555',
            'threshold': {
                'line': {
                    'color': "red",
                    'width': 8
                },
                'thickness': 0.85,
                'value': max_safe_temp
            }
        }),
        go.Layout(
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)',
            margin=go.layout.Margin(
                l=0,  # left margin
                r=0,  # right margin
                b=0,  # bottom margin
                t=0,  # top margin
            ),
            transition={
                'duration': 500,
                'easing': 'linear'
            }))

    return fig


# global variables to store the data
# ACC mode
ACC_State = -999
ACC_Controller_Mode = -999
# lead vehicle position and presence
Lead_Vehicle_Bool = False
Lead_Vehicle_Long_Pos = -999
Lead_Vehicle_Lat_Pos = -999
Lead_Vehicle_TrackID = -999
Distance_Envelope_Max = -999
Distance_Envelope_Min = -999
# flow rate/pump speed/fan speed
P0_Pump_Flow = -999
P4_Pump_Flow = -999
ICE_Fan_Speed_PWM = -999
ICE_Coolant_Pump_PWM = -999
HV_Cooling_Fan = -999
# critical temps
HV_Bat_Temp_C = -999
P0_Motor_Temp_C = -999
P0_Inverter_Temp_C = -999
P4_Motor_Temp_C = -999
P4_Inverter_Temp_C = -999
ICE_Coolant_Temp_C = -999
Trans_Coolant_Temp_C = -999
# Battery and voltage
HV_Bat_SOC = -999
HV_Bat_Current = -999
HV_Bus_V = -999
a_12V_Bus_V = -999
# CAV faults
Camera_Fault = -999
Radar_Fault = -999
Tank_Fault = -999
# PCM faults
P0_Fault = -999
P4_Fault = -999
ICE_Fault = -999
HV_Fault = -999
a_12V_Fault = -999
PCM_CAV_Fault = -999
# V2X
V2X_Signal = 0
V2X_Remaining_Time = -1
# lanes
C2 = 0
C1 = 0
C0_R = 0
C0_L = 0

P0_Fault_dict = {
    1: "P0 CAN Health Fault; ",
    2: "HV CAN Health Fault; ",
    4: "Hardware Fault; ",
    8: "Voltage Mismatch; ",
    16: "Pump Fault; ",
    32: "Overspeed; ",
    64: "Belt Slip; ",
    128: "Over Temperature;  ",
    256: "Torque Mismatch; ",
    512: "Over Torque; ",
}
P4_Fault_dict = {
    1: "P4 CAN Health Fault; ",
    2: "Hardware Fault; ",
    4: "Voltage Mismatch; ",
    8: "Pump Fault; ",
    16: "Overspeed; ",
    32: "Undefined; ",
    64: "Over Temperature; ",
    128: "Torque Mismatch; ",
    256: "Over Torque; ",
}
ICE_Fault_dict = {
    1: "Engine CAN Health Fault; ",
    2: "Coolant Over Temperature; ",
    4: "Cat Over Temperature; ",
    8: "P0 Speed Mismatch; ",
    16: "Fuel Level Low; ",
    32: "Overspeed; ",
}
HV_Fault_dict = {
    1: "Contactor Fault; ",
    2: "Battery Over Temperature; ",
    4: "Battery Low SOC; ",
    8: "Battery High SOC; ",
    16: "Battery CAN Health Fault; ",
}
PCM_CAV_Fault_dict = {
    1: "Camera Health Fault; ",
    2: "Radar Health Fault; ",
    4: "CAV Health Fault; ",
    8: "Long Control Switch Off; ",
    16: "RC and PV Fault; ",
    32: "CAN Health Fault",
}

# update global variable when new message is received from ROS topic


def callback(data, var_name):
    globals()[var_name] = data.data
    return


# initialize the dash app
app = dash.Dash(__name__, update_title=None)

# specify layout using dash's HTML components
# Simple layout with a graph component
# animate = True handles scroll animation for the graph, which would look better than an abrupt change in the values after updating
# interval specifies the time elapsed between two updates
# data.n_interval referss to the number of intervals completed from the start of the server


def reload_layout():
    # fault icon style
    fault_icon_style = {
        'font-size': '2.5vw',
        'font-weight': '500',
        'margin': '1.0vw 0vw 0vw 0vw',
    }

    # header text style
    header_style = {
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.5vw',
        'color': '#ffffff' if is_dark_mode else '#000',
        'font-weight': '500',
        'margin': '0.5vw 0vw 0vw 0.7vw',
    }
    toggle_style = {
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.5vw',
        'color': '#ffffff' if is_dark_mode else '#000',
        'font-weight': '500',
        'text-align': 'center',
        'display': 'inline-block',
        'padding-top': '10px',
    }
    # component background style
    component_background_style = {
        'height': '100%',
        'width': '100%',
        'margin': '0px',
        'background-image': 'linear-gradient(to top, #303030, #3B3B3B, #3B3B3B)' if is_dark_mode else 'linear-gradient(to top, #CFCFCF, #C4C4C4, #C4C4C4)',
        'border-radius': '1.5vw',
        'overflow': 'hidden',
    }

    acctutorial_button_style = {
        'font-family': 'sans-serif',
        'font-size': '30px',
        'font-weight': '200',
        'text-align': 'center',
        'text-transform': 'uppercase',
        'color': '#2186f4',
        'border-style': 'groove',
        'border-width': '5px',
        'border-radius': '10px',
        'border-color': '#2186f4',
        'background-color': '#0e1012' if is_dark_mode else '#F1EFED',
        'padding': '20px',
        # 'float': 'right',
    }

    datalabel_style2 = {
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.0vw',
        'color': '#ffffff' if is_dark_mode else 'black',
        'font-weight': '500',
        'text-align': 'center',
        'margin': 'auto',
    }
    datalabel_style3 = {
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.0vw',
        'color': '#72D5CD' if is_dark_mode else '#1D8078',
        'font-weight': '500',
        'text-align': 'center',
        'margin': 'auto',
    }
    marquee_style = {
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.0vw',
        'color': 'red',
        'font-weight': '500',
        'padding-top': '10px',
    }

    body_style = {
        'font-family': 'Monaco, Georgia, sans-serif',
        'font-size': '2.0vw',
        'color': '#72D5CD' if is_dark_mode else '#1D8078',
        'font-weight': '500',
        'text-align': 'center',
        'margin': 'auto',
    }

    flowrate_container_style = {
        'border': '1px solid red' if enable_box else '0px',
        'padding-top': '30px',
        'padding-right': '20px',
    }

    # temperature gauge style
    gauge_graph_container_style = {
        'border': '1px solid red' if enable_box else '0px'
    }
    gauge_graph_style = {
        'height': '100%',
        'width': '100%',
        'margin': '0px',
    }

    # ACC controller mode and lead vehicle track ID
    ACC_CM_LVT_style = {
        'position': 'absolute',
        'top': '98%',
        'transform': 'translate(-50%, -100%)',
        'width': '30%',
        'font-family': 'Segoe UI, Helvetica, sans-serif',
        'font-size': '1.2vw',
        'color': '#FFF' if is_dark_mode else 'black',
        'font-weight': '500',
        'border': '1px solid red' if enable_box else '0px'
    }
    ACC_CM_x_pos = 28
    ACC_LVT_x_pos = 100 - ACC_CM_x_pos
    return html.Div(
        style={
            'position': 'fixed',
            'left': '0',
            'top': '0',
            'backgroundColor': 'black' if is_dark_mode else 'white',
            'color': 'white' if is_dark_mode else 'black',
            'margin': '0px',
            'height': '100vh',  # '1080px',
            'width': '100vw',  # '1920px',
            'overflow': 'hidden',
            'display': 'grid',
            # 6 rows, 4 cols
            'grid-template-columns': 'repeat(' + str(num_cols) + ', 1fr)',
            'grid-template-rows': 'repeat(' + str(num_rows) + ', 1fr)',
            'border': '1px solid red' if enable_box else '0px'
        },
        children=[
            # POPY icon
            html.Img(id = 'output-teamphoto', src = '/assets/teamphoto2022.jpg', style = {'display': 'none'}),
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[0],
                    'grid-column-end':
                    vertical_dividers[1],
                    'grid-row-start':
                    horizontal_dividers[0],
                    'grid-row-end':
                    horizontal_dividers[1],
                    'margin':
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=[
                    html.Div(children='P0P4', className='popy-white' if is_dark_mode else 'popy-black',), 
                    html.Button(id = 'teamphoto-button', style = {'background-color': 'transparent','border-color':'transparent', 'position': 'absolute', 'width':'300px', 'height':'150px', 'top':'1%', 'left':'1%', 'z-index': '5'}, n_clicks=0),
                ],
            ),
            # fault icons
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[1],
                    'grid-column-end':
                    '4',
                    'grid-row-start':
                    horizontal_dividers[0],
                    'grid-row-end':
                    horizontal_dividers[1],
                    'margin':
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=[html.Div(
                    style={
                        'display': 'grid',
                        'grid-template-columns': '1fr',
                        'grid-template-rows': '1fr 1fr',
                    },
                    children=[
                        html.Div(
                            style={
                                'grid-column-start': '1',
                                'grid-column-end': '2',
                                'grid-row-start': '1',
                                'grid-row-end': '2',
                                'display': 'grid',
                                'grid-template-columns': 'repeat(9, 1fr)',
                                'grid-template-rows': 'repeat(1, 1fr)',
                            },
                            children=[
                                # CAV Faults
                                # Camera
                                html.Div([
                                    dcc.Interval(
                                        id='Camera_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='Camera_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # Radar
                                html.Div([
                                    dcc.Interval(
                                        id='Radar_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='Radar_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # Tank
                                html.Div([
                                    dcc.Interval(
                                        id='Tank_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='Tank_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # PCM Faults
                                # a_12V
                                html.Div([
                                    dcc.Interval(
                                        id='a_12V_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='a_12V_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # P0
                                html.Div([
                                    dcc.Interval(
                                        id='P0_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='P0_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # P4
                                html.Div([
                                    dcc.Interval(
                                        id='P4_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='P4_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # ICE
                                html.Div([
                                    dcc.Interval(
                                        id='ICE_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='ICE_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                                # HV
                                html.Div([
                                    dcc.Interval(
                                        id='HV_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='HV_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),

                                # PCM_CAV
                                html.Div([
                                    dcc.Interval(
                                        id='PCM_CAV_Fault-update',
                                        interval=fault_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    html.P(
                                        id='PCM_CAV_Fault',
                                        style=fault_icon_style,
                                    )
                                ]),
                            ]),
                        html.Div(
                            style={
                                'grid-column-start': '1',
                                'grid-column-end': '2',
                                'grid-row-start': '2',
                                'grid-row-end': '3',
                            },
                            children=[
                                dcc.Interval(
                                    id='fault_marquee-update',
                                    interval=fault_refresh_time_ms,
                                    n_intervals=0,
                                ),
                                html.Div(
                                    id='fault_marquee',
                                    style=marquee_style,
                                )
                            ]
                        )
                    ],

                )]
            ),
            # ACCTutorial Button
            html.Div(
                style={
                    'grid-column-start':
                    '4',
                    'grid-column-end':
                    '5',
                    'grid-row-start':
                    horizontal_dividers[0],
                    'grid-row-end':
                    horizontal_dividers[1],
                    'margin':
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=html.Div(
                    style={
                        'display': 'grid',
                        # 'grid-template-rows': '1fr',
                        'grid-template-columns': '1fr 1fr 1fr',
                        'align-items': 'center',
                    },
                    children=[
                        # html.Div(children=html.Button(
                        #     'Dark' if is_dark_mode else 'Light', style=acctutorial_button_style, id='light-dark-theme-switch', n_clicks=n_clicks)),
                        html.Div(
                            style={
                                'grid-column-start': '1',
                                'grid-column-end': '2',
                                'border': '1px solid red' if enable_box else '0px'
                            },
                            children=[
                                html.Div(
                                    style={
                                        'display': 'grid',
                                        # 'grid-template-rows': '1fr',
                                        'grid-template-columns': '1fr 1fr 1fr',
                                    },
                                    children=[
                                        html.Div(style=toggle_style,
                                                 children='👼'),
                                        html.Div(style={'display': 'inline-block', 'margin': 'auto', 'padding-top': '10px'}, children=[
                                            daq.BooleanSwitch(
                                                id='light-dark-theme-switch', on=is_dark_mode
                                            ),

                                        ]
                                        ),
                                        html.Div(style=toggle_style,
                                                 children='😈'),
                                    ]
                                ),
                            ],
                        ),
                        html.Div(
                            style={
                                'grid-column-start': '2',
                                'grid-column-end': '4',
                                'padding-left': '10px',
                                'border': '1px solid red' if enable_box else '0px'
                            },
                            children=[

                                html.Button(
                                    'ACC Tutorial', id='acc_tutorial_button', style=acctutorial_button_style, n_clicks=0
                                ),
                                html.Div(id='output-container-button',
                                         style={'Display': 'None'})
                            ]
                        ),
                    ],
                )
            ),
            # battery
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[0],
                    'grid-column-end':
                    vertical_dividers[1],
                    'grid-row-start':
                    horizontal_dividers[1],
                    'grid-row-end':
                    horizontal_dividers[2],
                    'margin':
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=[
                    html.Div(
                        style=component_background_style,
                        children=[
                            #html.Div(style=header_style, children='Battery'),
                            html.Div(
                                style={
                                    'display': 'grid',
                                    'grid-template-rows': '1fr 1fr 1fr',
                                    'grid-template-columns': '1fr 1fr',
                                    'height': '70%',
                                    'width': '90%',
                                    'margin': 'auto',
                                    'padding-top': '60px',
                                    'padding-bottom': '10px',
                                    'padding-left': '10px',
                                    'padding-right': '10px',
                                    'border':
                                    '1px solid red' if enable_box else '0px',
                                },
                                children=[
                                    dcc.Interval(
                                        id='battery_soc-update',
                                        interval=screen_refresh_time_ms,
                                        n_intervals=0,
                                    ),
                                    # Battery Icon
                                    html.Div(
                                        style={
                                            'grid-column-start': '1',
                                            'grid-column-end': '2',
                                            'grid-row-start': '1',
                                            'grid-row-end': '4',
                                            'display': 'flex',
                                            'justify-content': 'space-around'
                                        },
                                        children=[
                                            html.Div(
                                                style={
                                                    'align-self': 'center',
                                                    'height': '90%',
                                                    'width': '65%',
                                                    'display': 'flex',
                                                    'flex-direction': 'column'
                                                },
                                                children=[
                                                    html.Div(
                                                        style={
                                                            'align-self': 'center',
                                                            'width': '20%',
                                                            'height': '5%',
                                                            'margin': '0px',
                                                            'background': 'rgba(0,0,0,1)',
                                                            'z-index': '1',
                                                            'box-shadow': '0px 0px 25px rgba(255,255,255,0.4)',
                                                            'border-radius': '10px 10px 0px 0px',
                                                        }
                                                    ),
                                                    html.Div(
                                                        style={
                                                            'align-self': 'center',
                                                            'width': '100%',
                                                            'height': '95%',
                                                            'margin': '0px',
                                                            'background': 'rgba(255,255,255,0.1)',
                                                            'z-index': '1',
                                                            'box-shadow': '0px 0px 25px rgba(255,255,255,0.4)',
                                                            'border': '5px solid rgba(0,0,0,1)',
                                                            'border-radius': '15px',
                                                            'padding': '3px',
                                                            'display': 'flex',
                                                            'align-items': 'flex-end'
                                                        },
                                                        children=[
                                                            html.Div(
                                                                id='battery-percentage',
                                                                style={
                                                                    'height': '30%',
                                                                    'width': '100%',
                                                                    'background': 'rgba(0,0,0,0)',
                                                                    'border-radius': '10px',
                                                                }
                                                            )
                                                        ]
                                                    )
                                                ]
                                            ),
                                        ]
                                    ),
                                    # Battery SOC
                                    html.Div(style={
                                        'grid-column-start':
                                        '2',
                                        'grid-column-end':
                                        '3',
                                        'grid-row-start':
                                        '1',
                                        'grid-row-end':
                                        '2',
                                        'border':
                                        '1px solid red' if enable_box else '0px',
                                    },
                                        children=[
                                        html.Div([
                                            # dcc.Interval(
                                            #     id='battery_soc-update',
                                            #     interval=screen_refresh_time_ms,
                                            #     n_intervals=0,
                                            # ),
                                            html.P(
                                                id='battery_soc',
                                                style=body_style,
                                            )
                                        ]),
                                        html.Div(style=datalabel_style2,
                                                 children="Charge")
                                    ]),
                                    # DC Current
                                    html.Div(
                                        style={
                                            'grid-column-start':
                                            '2',
                                            'grid-column-end':
                                            '3',
                                            'grid-row-start':
                                            '2',
                                            'grid-row-end':
                                            '3',
                                            'border':
                                            '1px solid red'
                                            if enable_box else '0px',
                                        },
                                        #"DC Current"
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='dc_current-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='dc_current',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(style=datalabel_style2,
                                                     children=[
                                                         'DC Current',
                                                     ])
                                        ]),
                                    #"HV Bus Voltage"
                                    html.Div(
                                        style={
                                            'grid-column-start':
                                            '2',
                                            'grid-column-end':
                                            '3',
                                            'grid-row-start':
                                            '3',
                                            'grid-row-end':
                                            '4',
                                            'border':
                                            '1px solid red'
                                            if enable_box else '0px',
                                        },
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='hv_bus_voltage-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='hv_bus_voltage',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(
                                                style=datalabel_style2,
                                                children='HV Bus Voltage',  # Topleft
                                            )
                                        ])
                                ])
                        ])
                ],
            ),
            # flow rates
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[1],
                    'grid-column-end':
                    vertical_dividers[2],
                    'grid-row-start':
                    horizontal_dividers[1],
                    'grid-row-end':
                    horizontal_dividers[2],
                    'margin':
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=[
                    html.Div(
                        style=component_background_style,
                        children=[
                            html.Div(
                                style={
                                    'display': 'grid',
                                    'grid-template-rows': '1fr 1fr',
                                    'grid-template-columns': '1fr 1fr 1fr',
                                    'height': '70%',
                                    'width': '90%',
                                    'margin': 'auto',
                                    'padding-top': '40px',
                                    'padding-bottom': '10px',
                                    'padding-left': '20px',
                                    'padding-right': '10px',
                                    'border':
                                    '1px solid red' if enable_box else '0px',
                                },
                                children=[
                                    html.Div(
                                        style={
                                            'font-family':
                                            'Segoe UI, Helvetica, sans-serif',
                                            'font-size': '1.5vw',
                                            'color': '#ffffff' if is_dark_mode else 'black',
                                            'font-weight': '500',
                                            'margin': '0.5vw 0vw 0vw 0.7vw',
                                            'align-self': 'center',
                                        },
                                        children='Flow Rates'),
                                    html.Div(
                                        style=flowrate_container_style,
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='P0_Pump_Flow-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='P0_Pump_Flow',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(
                                                style=datalabel_style2,
                                                children=[
                                                    'P0 Pump',
                                                    html.Br(), 'L/min'
                                                ]  # Topleft
                                            )
                                        ]),
                                    html.Div(
                                        style=flowrate_container_style,  # TopRight
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='ICE_Fan_Speed_PWM-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='ICE_Fan_Speed_PWM',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(
                                                style=datalabel_style2,
                                                children=[
                                                    'ICE Fan Speed',
                                                    #html.Br(), '%'
                                                ],
                                            ),
                                        ]),
                                        html.Div(
                                        style=flowrate_container_style,  # TopRight
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='HV_Cooling_Fan-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='HV_Cooling_Fan',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(
                                                style=datalabel_style2,
                                                children=[
                                                    'HV Cooling Fan',
                                                    #html.Br(), '%'
                                                ],
                                            ),
                                        ]),
                                    html.Div(
                                        style=flowrate_container_style,  # BottomLeft
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='P4_Pump_Flow-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='P4_Pump_Flow',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(style=datalabel_style2,
                                                     children=[
                                                         'P4 Pump',
                                                         html.Br(), 'L/min'
                                                     ]),
                                        ]),
                                    html.Div(
                                        style=flowrate_container_style,  # BottomRight
                                        children=[
                                            html.Div([
                                                dcc.Interval(
                                                    id='ICE_Coolant_Pump_PWM-update',
                                                    interval=screen_refresh_time_ms,
                                                    n_intervals=0,
                                                ),
                                                html.P(
                                                    id='ICE_Coolant_Pump_PWM',
                                                    style=body_style,
                                                )
                                            ]),
                                            html.Div(
                                                style=datalabel_style2,
                                                children=[
                                                    'ICE Coolant Pump',
                                                    #html.Br(), '%'
                                                ],
                                            ),
                                        ]),
                                ])
                        ])
                ],
            ),
            # Critical Component Temperatures
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[0],
                    'grid-column-end':
                    vertical_dividers[2],
                    'grid-row-start':
                    horizontal_dividers[2],
                    'grid-row-end':
                    horizontal_dividers[3],
                    'margin':
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ',
                    'border':
                    '1px solid red' if enable_box else '0px'
                },
                children=[
                    html.Div(
                        style=component_background_style,
                        children=[
                            html.Div(
                                style={
                                    'height': '100%',
                                    'width': '100%',
                                    'margin': '0px',
                                    'display': 'grid',
                                    'grid-template-columns': 'repeat(4, 1fr)',
                                    'grid-template-rows': 'repeat(2, 1fr)',
                                },
                                children=[
                                    html.Div(
                                        style={
                                            'font-family':
                                            'Segoe UI, Helvetica, sans-serif',
                                            'font-size': '1.5vw',
                                            'color': '#ffffff' if is_dark_mode else 'black',
                                            'font-weight': '500',
                                            'margin': '0.5vw 0vw 0vw 0.7vw',
                                            'align-self': 'center',
                                        },
                                        children='Critical Component Temperature (°C)'),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='p0-motor-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='p0-motor-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='p4-motor-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='p4-motor-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='ice-coolant-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='ice-coolant-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='bat-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='bat-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='p0-inverter-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='p0-inverter-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='p4-inverter-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='p4-inverter-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                    html.Div(
                                        style=gauge_graph_container_style,
                                        children=[
                                            dcc.Graph(
                                                style=gauge_graph_style,
                                                id='trans-coolant-temp-graph',
                                                config={'displayModeBar': False}),
                                            dcc.Interval(
                                                id='trans-coolant-temp-graph-update',
                                                interval=critical_temp_refresh_time_ms,
                                                n_intervals=0,
                                            ),
                                        ],
                                    ),
                                ])
                        ])
                ],
            ),
            # distance envelope + bird's eye view
            html.Div(
                style={
                    'grid-column-start':
                    vertical_dividers[2],
                    'grid-column-end':
                    vertical_dividers[3],
                    'grid-row-start':
                    horizontal_dividers[1],
                    'grid-row-end':
                    horizontal_dividers[3],
                    'margin':
                    str(major_component_spacing_vw) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw * 2) + 'vw ' +
                    str(major_component_spacing_vw) + 'vw ',
                    'display':
                    'grid',
                    'gap':
                    str(major_component_spacing_vw * 2) + 'vw',
                    'grid-template-rows':
                    'repeat(12, 1fr)',
                },
                children=[
                    # distance envelope
                    html.Div(
                        style={
                            'grid-row-start': '1',
                            'grid-row-end': '3',
                            'display': 'flex',
                            'flex-direction': 'column',
                            'justify-content': 'center',
                            'border': '1px solid red' if enable_box else '0px'
                        },
                        children=[
                            html.Div(
                                id='dist_envelope_title',
                                style=header_style,
                                children='Distance Envelope',
                            ),
                            dcc.Graph(
                                style={
                                    'height': '100%',
                                    'border':
                                    '1px solid red' if enable_box else '0px',
                                    'transform': 'scaleX(-1)',
                                },
                                id='dist-envelope-graph',
                                config={'displayModeBar': False},
                                figure={
                                    'layout':
                                    go.Layout(
                                        xaxis=dict(range=[-0.05, 1.05],
                                                   zeroline=False,
                                                   showticklabels=False,
                                                   showgrid=False,
                                                   fixedrange=True),
                                        yaxis=dict(range=[-1, 1],
                                                   zeroline=False,
                                                   showticklabels=False,
                                                   showgrid=False,
                                                   fixedrange=True),
                                        xaxis_visible=False,
                                        yaxis_visible=False,
                                        showlegend=False,
                                        paper_bgcolor='rgba(0,0,0,0)',
                                        plot_bgcolor='rgba(0,0,0,0)',
                                        margin=go.layout.Margin(
                                            l=0,  # left margin
                                            r=0,  # right margin
                                            b=0,  # bottom margin
                                            t=0,  # top margin
                                        ))
                                }),
                            dcc.Interval(
                                id='dist-envelope-graph-update',
                                interval=screen_refresh_time_ms,
                                n_intervals=0,
                            ),
                        ],
                    ),
                    # bird's eye view
                    html.Div(
                        style={
                            'grid-row-start': '3',
                            'grid-row-end': '13',
                            'display': 'flex',
                            'justify-content': 'center',
                            'position': 'relative',
                            'border': '1px solid red' if enable_box else '0px'
                        },
                        children=[

                            html.Div(
                                children=[
                                    dcc.Graph(
                                        style={
                                            'height': '100%',
                                        },
                                        id='birds-eye-graph',
                                        config={'displayModeBar': False},
                                        figure={
                                            'layout':
                                            go.Layout(
                                                xaxis=dict(range=[min_x, max_x],
                                                           zeroline=False,
                                                           tick0=0,
                                                           dtick=x_dtick,
                                                           showticklabels=False,
                                                           gridwidth=1,
                                                           gridcolor='#AAA' if is_dark_mode else '#555',
                                                           fixedrange=True),
                                                yaxis=dict(range=[min_y, max_y],
                                                           zeroline=False,
                                                           tick0=0,
                                                           dtick=y_dtick,
                                                           showticklabels=False,
                                                           gridwidth=1,
                                                           gridcolor='#AAA' if is_dark_mode else '#555',
                                                           fixedrange=True),
                                                xaxis_visible=False,
                                                yaxis_visible=False,
                                                showlegend=False,
                                                paper_bgcolor='rgba(0,0,0,0)',
                                                plot_bgcolor='rgba(0,0,0,0)',
                                                margin=go.layout.Margin(
                                                    l=0,  # left margin
                                                    r=0,  # right margin
                                                    b=0,  # bottom margin
                                                    t=0,  # top margin
                                                ),
                                            )
                                        }),
                                    dcc.Interval(
                                        id='birds-eye-graph-update', interval=screen_refresh_time_ms, n_intervals=0),
                                ], ),
                            html.Img(style={
                                'position': 'absolute',
                                'top': '99%',
                                'left': '50%',
                                'width':'73px',
                                'height':'160px',
                                'transform': 'translate(-50%, -100%)'
                            },
                                src='/assets/PopyIcon.png'),
                            html.Div(style={'display': 'none'},
                                     children=[html.Div(className='wifi')],
                                     id='birds-eye-acc-symbol'),
                            html.Div(
                                style={
                                    **ACC_CM_LVT_style, 'left':
                                    str(ACC_CM_x_pos) + '%'
                                },
                                children=[
                                    html.Div(
                                        style={
                                            #**component_background_style, 
                                            'padding':
                                            '1vw',
                                            'box-sizing': 'border-box'
                                        },
                                        children=[
                                            html.Div(
                                                style=datalabel_style2, children='ACC Controller Mode:'),
                                            html.Div(id='acc-controller-mode', style=datalabel_style3, children='N/A',
                                                     )
                                        ])
                                ],
                            ),
                            html.Div(
                                style={
                                    **ACC_CM_LVT_style, 'left':
                                    str(ACC_LVT_x_pos) + '%'
                                },
                                children=[
                                    html.Div(
                                        style={
                                            #**component_background_style, 
                                            'padding':
                                            '1vw',
                                            'box-sizing': 'border-box',
                                        },
                                        children=[
                                            html.Div(style=datalabel_style2,
                                                children='Lead Vehicle'),
                                            html.Div(
                                                style ={

                                                    'display':'grid',
                                                    'grid-template-columns': '1fr 1fr',
                                                    'grid-template-rows': '1fr 1fr',
                                                },
                                                children =[

                                                    html.Div(style=datalabel_style2,
                                                        children='Track ID:'),
                                                    html.Div(
                                                                style=datalabel_style3,
                                                                children='N/A',
                                                                id='acc-lead-vehicle-track-number'),
                                                    html.Div(
                                                        style=datalabel_style2, children='Distance:'),
                                                    html.Div(id='lead-vehicle-dist-text', style=datalabel_style3, children='N/A',
                                                    )
                                                ]
                                                        ),
                                        ])
                                ],
                            ),
                            html.Div(
                                style={
                                    'position': 'absolute',
                                    'top': '0%',
                                    'right': '5%',
                                    'transform': 'translateX(0%)',
                                    'display': 'flex',
                                    'flex-direction': 'column',
                                    'align-items': 'center'
                                },
                                children=[
                                    dcc.Interval(
                                        id='v2x-update',
                                        interval=100,
                                        n_intervals=0,
                                    ),
                                    html.Div(
                                        id='v2x-light',
                                        className='trf-lgt-container',
                                        children=[
                                            html.Div(className='red-off'),
                                            html.Div(className='yellow-off'),
                                            html.Div(className='green-off')
                                        ]
                                    ),
                                    html.Div(
                                        id='v2x-remaining-time',
                                        style={
                                            'font-family':
                                            'Segoe UI, Helvetica, sans-serif',
                                            'font-size': '1.5vw',
                                            'color': '#ffffff' if is_dark_mode else 'black',
                                            'font-weight': '500',
                                            'margin': '0.5vw 0vw 0vw 0vw',
                                        },
                                        children='N/A'
                                    )
                                ]
                            ),
                            # html.Div(
                            #     style={
                            #         **ACC_CM_LVT_style, 'left':
                            #         str(ACC_LVT_x_pos) + '%',
                            #         'position': 'absolute',
                            #         'top': '0%',
                            #         'left': '95%',
                            #         'transform': 'translateX(-100%)',
                            #         'width': '20%'
                            #     },
                            #     children=[
                            #         html.Div(
                            #             style={
                            #                 **component_background_style, 'padding':
                            #                 '1.5vw',
                            #                 'box-sizing': 'border-box',
                            #             },
                            #             children=[
                            #                 html.Div(id='lead-vehicle-dist-text', style=body_style, children='N/A',
                            #                          ),
                            #                 html.Div(
                            #                     style=datalabel_style2, children='Lead Vehicle Distance'),
                                            
                            #             ]),
                            #     ],
                            # ),


                        ],
                    ),
                    dcc.Interval(
                        id='birds-eye-acc-symbol-update',
                        interval=screen_refresh_time_ms,
                        n_intervals=0,
                    ),
                ],
            ),
        ])


app.layout = html.Div(children=reload_layout(), id='top')

# add wait time for the graph to stablize
sleep(2)


@app.callback(
    Output('top', 'children'),
    Input('light-dark-theme-switch', 'on'),
)
def update_output(on):
    global is_dark_mode
    is_dark_mode = True if on else False
    return reload_layout()

# v2x traffic light update


@app.callback(Output('v2x-light', 'children'),
              Output('v2x-remaining-time', 'children'),
              Output('v2x-light', 'style'), [
    Input('v2x-update', 'n_intervals'),
    Input('v2x-update', 'interval')
])
def update_birds_eye_acc_symbol(n_intervals, interval):
    children = []
    remaining_time_str = ''
    container_style = {}
    if(V2X_Signal == 3):
        children = [html.Div(className='red'), html.Div(
            className='yellow-off'), html.Div(className='green-off'), ]
        container_style = {
            'border': '2px solid #e83348',
            'box-shadow': '0 0 20px #e83348',
        }
    elif(V2X_Signal == 2):
        children = [html.Div(className='red-off'), html.Div(
            className='yellow'), html.Div(className='green-off'), ]
        container_style = {
            'border': '2px solid yellow',
            'box-shadow': '0 0 20px yellow',
        }
    elif(V2X_Signal == 1):
        children = [html.Div(className='red-off'), html.Div(
            className='yellow-off'), html.Div(className='green'), ]
        container_style = {
            'border': '2px solid rgb(0, 255, 0)',
            'box-shadow': '0 0 20px rgb(0, 255, 0)',
        }
    else:
        children = [html.Div(className='red-off'), html.Div(
            className='yellow-off'), html.Div(className='green-off'), ]
        container_style = {
            'border': '2px solid grey',
            'box-shadow': '0 0 20px grey',
        }

    if(V2X_Signal == 0 or V2X_Remaining_Time == -1):
        remaining_time_str = 'N/A'
    else:
        remaining_time_str = str(math.floor(V2X_Remaining_Time)) + 's'

    return children, remaining_time_str, container_style

# bird's eye ACC color/controller mode/lead vehicle tracking number


@app.callback(Output('birds-eye-acc-symbol', 'style'), [
    Input('birds-eye-acc-symbol-update', 'n_intervals'),
    Input('birds-eye-acc-symbol-update', 'interval')
])
def update_birds_eye_acc_symbol(n_intervals, interval):
    style = {
        'display':
        'none' if
        (ACC_State != 1 and ACC_State != 2 and ACC_State != 3) else 'block',
        'position':
        'absolute',
        'top':
        '75%',
        'left':
        '50%',
        'transform':
        'translate(-50%, -50%)',
        'color':
        '#BBB' if ACC_State == 1 else 'green' if ACC_State == 2 else 'orange',
        'opacity': '0.6',
        'border':
        '1px solid LightSkyBlue' if enable_box else '0px'
    }
    return style


@app.callback(Output('output-container-button', 'children'),
              [Input('acc_tutorial_button', 'n_clicks')])
def run_acctutorial(n_clicks):
    if not n_clicks:
        raise PreventUpdate
    result = subprocess.run(
        'cd ../../acc_tutorial/src; python3 acctutorialappV3_with_voiceovers.py', shell=True)
    return result

@app.callback(Output('output-teamphoto', 'style'),
                Output('output-teamphoto', 'src'),
              [Input('teamphoto-button', 'n_clicks')])
def show_teamphoto(n_clicks):
    if n_clicks%6 == 0:
        style = {'display':'none'}
    else:
        style = {'display': 'block', 'position': 'absolute', 'width':'70vw', 'height':'auto', 'transform': 'translate(-50%, -50%)', 'top': '50%', 'left': '50%', 'z-index':'5'}
    
    if n_clicks%6 == 5:
        src = '/assets/skittles.jpg'
    elif n_clicks%6 == 4:
        src = '/assets/teamtrivia.jpg'
    elif n_clicks%6 == 3:
        src = '/assets/teamphoto2019.jpg'
    elif n_clicks%6 == 2:
        src = '/assets/teamphoto2021.jpg'
    else:
        src = '/assets/teamphoto2022.jpg'
    return style, src


#A. Battery
# Icon


@app.callback(
    Output('battery-percentage', 'style'),
    Output('battery_soc', 'children'),
    [
        Input('battery_soc-update', 'n_intervals'),
        Input('battery_soc-update', 'interval')
    ]
)
def update_battery_icon(n_intervals, interval):
    color = 'linear-gradient(#4de8db, #59e64c)' if HV_Bat_SOC > 45 \
            else 'linear-gradient(#f2eb22, #f2a922)' if HV_Bat_SOC > 30 \
            else 'linear-gradient(#f05151, #f00505)'
    batt_str = str('{:.0f}'.format(HV_Bat_SOC)) + '%'

    style = {
        'height': batt_str,
        'width': '100%',
        'background': color,
        'border-radius': '10px',
    }

    return style, batt_str

# #SOC
# @app.callback(Output('battery_soc', 'children'), [
#     Input('battery_soc-update', 'n_intervals'),
#     Input('battery_soc-update', 'interval')
# ])
# def update_flowdata_hv_bus_voltage(n_intervals, interval):
#     return str('{:.0f}'.format(HV_Bat_SOC)) + '%'


# DCCurrent
@app.callback(Output('dc_current', 'children'), [
    Input('dc_current-update', 'n_intervals'),
    Input('dc_current-update', 'interval')
])
def update_dc_current(n_intervals, interval):
    return str('{:.0f}'.format(HV_Bat_Current)) + 'A'


# hvbusvoltage
@app.callback(Output('hv_bus_voltage', 'children'), [
    Input('hv_bus_voltage-update', 'n_intervals'),
    Input('hv_bus_voltage-update', 'interval')
])
def update_hv_bus_voltage(n_intervals, interval):
    return str('{:.0f}'.format(HV_Bus_V)) + 'V'


# B. Flow Rate
# ICE_Fan_Speed_PWM
@app.callback(Output('ICE_Fan_Speed_PWM', 'children'), [
    Input('ICE_Fan_Speed_PWM-update', 'n_intervals'),
    Input('ICE_Fan_Speed_PWM-update', 'interval')
])
def update_flowdata_ICE_Fan_Speed_PWM(n_intervals, interval):
    return str('{:.0f}'.format(ICE_Fan_Speed_PWM)) + '%'


# ICE_Coolant_Pump_PWM
@app.callback(Output('ICE_Coolant_Pump_PWM', 'children'), [
    Input('ICE_Coolant_Pump_PWM-update', 'n_intervals'),
    Input('ICE_Coolant_Pump_PWM-update', 'interval')
])
def update_flowdata_ICE_Coolant_Pump_PWM(n_intervals, interval):
    return str('{:.0f}'.format(ICE_Coolant_Pump_PWM)) + '%'


# P0_Pump_Flow
@app.callback(Output('P0_Pump_Flow', 'children'), [
    Input('P0_Pump_Flow-update', 'n_intervals'),
    Input('P0_Pump_Flow-update', 'interval')
])
def update_flowdata_P0_Pump_Flow(n_intervals, interval):
    return str('{:.0f}'.format(P0_Pump_Flow))


# P4_Pump_Flow
@app.callback(Output('P4_Pump_Flow', 'children'), [
    Input('P4_Pump_Flow-update', 'n_intervals'),
    Input('P4_Pump_Flow-update', 'interval')
])
def update_flowdata_P4_Pump_Flow(n_intervals, interval):
    return str('{:.0f}'.format(P4_Pump_Flow))

# HV_Cooling_Fan


@app.callback(Output('HV_Cooling_Fan', 'children'), [
    Input('HV_Cooling_Fan-update', 'n_intervals'),
    Input('HV_Cooling_Fan-update', 'interval')
])
def update_HV_Cooling_Fan(n_intervals, interval):
    if HV_Cooling_Fan == 1:
        fan_state = 'On'
    else:
        fan_state = 'Off'
    return fan_state


#D. Faults
# CAV
# Camera
@app.callback(Output('Camera_Fault', 'children'), [
    Input('Camera_Fault-update', 'n_intervals'),
    Input('Camera_Fault-update', 'interval')
])
def update_Camera_Fault(n_intervals, interval):
    # return '📸'
    if Camera_Fault == 0:
        return None
    # low sun
    elif Camera_Fault == 1:
        return '⛅'
    # blur
    elif Camera_Fault == 2:
        return '👓'
    # other
    elif Camera_Fault == 3:
        return '📸'


# Radar
@app.callback(Output('Radar_Fault', 'children'), [
    Input('Radar_Fault-update', 'n_intervals'),
    Input('Radar_Fault-update', 'interval')
])
def update_Radar_Fault(n_intervals, interval):
    # HWFail
    if Radar_Fault == 0:
        return None
    elif Radar_Fault == 1:
        return '📡'
    # SGUFail
    elif Radar_Fault == 2:
        return '📶'


# Tank
@app.callback(Output('Tank_Fault', 'children'), [
    Input('Tank_Fault-update', 'n_intervals'),
    Input('Tank_Fault-update', 'interval')
])
def update_Tank_Fault(n_intervals, interval):
    # no PCM CAN
    if Tank_Fault == 0:
        return None
    elif Tank_Fault == 1:
        return '🥫'
    # no V2X Connection
    elif Tank_Fault == 2:
        return '🚦'
    # other
    elif Tank_Fault == 3:
        return '🤖'
# P0_Fault


@app.callback(Output('a_12V_Fault', 'children'), [
    Input('a_12V_Fault-update', 'n_intervals'),
    Input('a_12V_Fault-update', 'interval')
])
def update_a_12V_Fault(n_intervals, interval):
    if a_12V_Fault == 0:
        return None
    elif a_12V_Fault == 1:
        return '⚡'

# PCM
# P0_Fault


@app.callback(Output('P0_Fault', 'children'), [
    Input('P0_Fault-update', 'n_intervals'),
    Input('P0_Fault-update', 'interval')
])
def update_P0_Fault(n_intervals, interval):
    if P0_Fault == 0:
        return None
    elif P0_Fault > 0:
        return '0️⃣'


# P4_Fault
@app.callback(Output('P4_Fault', 'children'), [
    Input('P4_Fault-update', 'n_intervals'),
    Input('P4_Fault-update', 'interval')
])
def update_P4_Fault(n_intervals, interval):
    if P4_Fault == 0:
        return None
    elif P4_Fault > 0:
        return '4️⃣'


# ICE_Fault
@app.callback(Output('ICE_Fault', 'children'), [
    Input('ICE_Fault-update', 'n_intervals'),
    Input('ICE_Fault-update', 'interval')
])
def update_ICE_Fault(n_intervals, interval):
    if ICE_Fault == 0:
        return None
    elif ICE_Fault > 0:
        return '🧊'


# HV_Fault
@app.callback(Output('HV_Fault', 'children'), [
    Input('HV_Fault-update', 'n_intervals'),
    Input('HV_Fault-update', 'interval')
])
def update_HV_Fault(n_intervals, interval):
    if HV_Fault == 0:
        return None
    elif HV_Fault > 0:
        return '🔋'

# PCM_CAV_Fault


@app.callback(Output('PCM_CAV_Fault', 'children'), [
    Input('PCM_CAV_Fault-update', 'n_intervals'),
    Input('PCM_CAV_Fault-update', 'interval')
])
def update_PCM_CAV_Fault(n_intervals, interval):
    if PCM_CAV_Fault > 0:
        return '💻'
    else:
        return None

# PCM Fault Marquee


@app.callback(Output('fault_marquee', 'children'), [
    Input('fault_marquee-update', 'n_intervals'),
    Input('fault_marquee-update', 'interval')
])
def update_fault_marquee(n_intervals, interval):
    faults_list = []
    # P0 enums
    if(P0_Fault > 0):
        faults_list.append('0️⃣ P0: ')
        bin_num = format(int(P0_Fault), 'b')
        bin_length = len(bin_num)
        for bit_index in range(bin_length):
            fault_enum = int(bin_num) & (1 << bit_index)
            if fault_enum > 0:
                faults_list.append(P0_Fault_dict[fault_enum])
    # #P4 enums
    if(P4_Fault > 0):
        faults_list.append('4️⃣ P4: ')
        bin_num = format(int(P4_Fault), 'b')
        bin_length = len(bin_num)
        for bit_index in range(bin_length):
            fault_enum = int(bin_num) & (1 << bit_index)
            if fault_enum > 0:
                faults_list.append(P4_Fault_dict[fault_enum])
    # #EngHealth_enum
    if(ICE_Fault > 0):
        faults_list.append('🧊 ICE: ')
        bin_num = format(int(ICE_Fault), 'b')
        bin_length = len(bin_num)
        for bit_index in range(bin_length):
            fault_enum = int(bin_num) & (1 << bit_index)
            if fault_enum > 0:
                faults_list.append(ICE_Fault_dict[fault_enum])
    # #BattHealth_enum
    if(HV_Fault > 0):
        faults_list.append('🔋HV: ')
        bin_num = format(int(HV_Fault), 'b')
        bin_length = len(bin_num)
        for bit_index in range(bin_length):
            fault_enum = int(bin_num) & (1 << bit_index)
            if fault_enum > 0:
                faults_list.append(HV_Fault_dict[fault_enum])
    # #ACCHealth_enum:
    if(PCM_CAV_Fault > 0):
        faults_list.append('💻 PCMCAV: ')
        bin_num = format(int(HV_Fault), 'b')
        bin_length = len(bin_num)
        for bit_index in range(bin_length):
            fault_enum = int(bin_num) & (1 << bit_index)
            if fault_enum > 0:
                faults_list.append(PCM_CAV_Fault_dict[fault_enum])
    return faults_list

# E. bird's eye ACC color/controller mode/lead vehicle tacking number


@app.callback(Output('lead-vehicle-dist-text', 'children'), [
    Input('birds-eye-acc-symbol-update', 'n_intervals'),
    Input('birds-eye-acc-symbol-update', 'interval')
])
def update_acc_controller_mode_text(n_intervals, interval):
    return '{:.1f}'.format(Lead_Vehicle_Long_Pos)+'m' if Lead_Vehicle_Bool else 'N/A'


@app.callback(Output('acc-controller-mode', 'children'), [
    Input('birds-eye-acc-symbol-update', 'n_intervals'),
    Input('birds-eye-acc-symbol-update', 'interval')
])
def update_acc_controller_mode_text(n_intervals, interval):
    ACC_Controller_Modes = [
        'Disabled', 'Velocity control', 'Distance control', 'Low speed',
        'Stop and hold', 'V2X velocity control', 'V2X distance control'
    ]
    ACC_Controller_Mode_int = int(ACC_Controller_Mode)
    if (ACC_Controller_Mode_int >= 0 and ACC_Controller_Mode_int <=
            (len(ACC_Controller_Modes) - 1)):
        return ACC_Controller_Modes[ACC_Controller_Mode_int]
    else:
        return 'N/A'


# E. bird's eye ACC color/controller mode/lead vehicle tacking number
@app.callback(Output('acc-lead-vehicle-track-number', 'children'), [
    Input('birds-eye-acc-symbol-update', 'n_intervals'),
    Input('birds-eye-acc-symbol-update', 'interval')
])
def update_lead_vehicle_track_ID_text(n_intervals, interval):
    return str(int(Lead_Vehicle_TrackID))


# temps
@app.callback(Output('bat-temp-graph', 'figure'), [
    Input('bat-temp-graph-update', 'n_intervals'),
    Input('bat-temp-graph-update', 'interval')
])
def update_bat_temp_graph(n_intervals, interval):
    max_safe_temp = 50
    return create_gauge_fig(interval, max_safe_temp, 'HV_Bat_Temp_C', 'HV Bat',
                            '.1f')


@app.callback(Output('p0-motor-temp-graph', 'figure'), [
    Input('p0-motor-temp-graph-update', 'n_intervals'),
    Input('p0-motor-temp-graph-update', 'interval')
])
def update_p0_motor_temp_graph(n_intervals, interval):
    max_safe_temp = 140
    return create_gauge_fig(interval, max_safe_temp, 'P0_Motor_Temp_C',
                            'P0 Motor', '.1f')


@app.callback(Output('p0-inverter-temp-graph', 'figure'), [
    Input('p0-inverter-temp-graph-update', 'n_intervals'),
    Input('p0-inverter-temp-graph-update', 'interval')
])
def update_p0_inverter_temp_graph(n_intervals, interval):
    max_safe_temp = 80
    return create_gauge_fig(interval, max_safe_temp, 'P0_Inverter_Temp_C',
                            'P0 Inverter', '.1f')


@app.callback(Output('p4-motor-temp-graph', 'figure'), [
    Input('p4-motor-temp-graph-update', 'n_intervals'),
    Input('p4-motor-temp-graph-update', 'interval')
])
def update_p4_motor_temp_graph(n_intervals, interval):
    max_safe_temp = 160
    return create_gauge_fig(interval, max_safe_temp, 'P4_Motor_Temp_C',
                            'P4 Motor', '.1f')


@app.callback(Output('p4-inverter-temp-graph', 'figure'), [
    Input('p4-inverter-temp-graph-update', 'n_intervals'),
    Input('p4-inverter-temp-graph-update', 'interval')
])
def update_p4_inverter_temp_graph(n_intervals, interval):
    max_safe_temp = 85
    return create_gauge_fig(interval, max_safe_temp, 'P4_Inverter_Temp_C',
                            'P4 Inverter', '.1f')


@app.callback(Output('ice-coolant-temp-graph', 'figure'), [
    Input('ice-coolant-temp-graph-update', 'n_intervals'),
    Input('ice-coolant-temp-graph-update', 'interval')
])
def update_ice_coolant_temp_graph(n_intervals, interval):
    max_safe_temp = 105
    return create_gauge_fig(interval, max_safe_temp, 'ICE_Coolant_Temp_C',
                            'ICE Coolant', '.1f')


@app.callback(Output('trans-coolant-temp-graph', 'figure'), [
    Input('trans-coolant-temp-graph-update', 'n_intervals'),
    Input('trans-coolant-temp-graph-update', 'interval')
])
def update_trans_coolant_temp_graph(n_intervals, interval):
    max_safe_temp = 95
    return create_gauge_fig(interval, max_safe_temp, 'Trans_Coolant_Temp_C',
                            'Trans Coolant', '.1f')


# Distance envelope
@app.callback(Output('dist_envelope_title', 'children'), [
    Input('dist-envelope-graph-update', 'n_intervals'),
    Input('dist-envelope-graph-update', 'interval')
])
def update_dist_envelope_title(n_intervals, interval):
    return 'Distance Envelope: '+'{:.1f}'.format(Distance_Envelope_Min)+'m - '+'{:.1f}'.format(Distance_Envelope_Max)+'m'


@app.callback(Output('dist-envelope-graph', 'figure'), [
    Input('dist-envelope-graph-update', 'n_intervals'),
    Input('dist-envelope-graph-update', 'interval')
])
def update_dist_envelope_graph(n_intervals, interval):
    right_most = 120
    car_dist_x = [1, max((right_most - Lead_Vehicle_Long_Pos) / right_most, 0)]
    line_y = [0, 0]

    car_dist = plotly.graph_objs.Scatter(
        x=car_dist_x,
        y=line_y,
        marker=dict(
            color='#FF0000'
            if Lead_Vehicle_Long_Pos > Distance_Envelope_Max else 'green'
            if Lead_Vehicle_Long_Pos > Distance_Envelope_Min else '#FFC000',
            size=15),
        mode='lines' if Lead_Vehicle_Bool else 'text',
        name='Ego Vehicle',
    )

    envelope = plotly.graph_objs.Scatter(
        x=[
            (right_most - Distance_Envelope_Min) / right_most,
            (right_most - Distance_Envelope_Max) / right_most
        ],
        y=line_y,
        marker=dict(color='#BBB' if is_dark_mode else '#444', size=20),
        name='Base Mark',
    )

    car_icon = plotly.graph_objs.Scatter(
        x=car_dist_x,
        y=line_y,
        mode='text',
        text='🏎️' if Lead_Vehicle_Bool else '',
        textfont={'size': 50, }
    )

    return {
        'data': [
            envelope,
            car_dist,
            car_icon,
        ],
        'layout':
        go.Layout(
            xaxis=dict(range=[-0.05, 1.05],
                       zeroline=False,
                       showticklabels=False,
                       showgrid=False,
                       fixedrange=True),
            yaxis=dict(range=[-1, 1],
                       zeroline=False,
                       showticklabels=False,
                       showgrid=False,
                       fixedrange=True),
            xaxis_visible=False,
            yaxis_visible=False,
            showlegend=False,
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)',
            margin=go.layout.Margin(
                l=0,  # left margin
                r=0,  # right margin
                b=0,  # bottom margin
                t=0,  # top margin
            ),
            transition={
                'duration': interval,
                'easing': 'linear'
            })
    }

# Bird's Eye View


@app.callback(Output('birds-eye-graph', 'figure'),
              [Input('birds-eye-graph-update', 'n_intervals'),
              Input('birds-eye-graph-update', 'interval')])
def update_birds_eye_graph(n_intervals, interval):
    lead = plotly.graph_objs.Scatter(
        x=[Lead_Vehicle_Lat_Pos],
        y=[Lead_Vehicle_Long_Pos],
        marker=dict(color='LightSkyBlue', size=30, opacity=Lead_Vehicle_Bool),
        name='Leading Vehicle',
        mode='text',
        # text=('<br>🚘<br>'+str(Lead_Vehicle_Long_Pos)) if Lead_Vehicle_Bool else '',
        text=('🚘') if Lead_Vehicle_Bool else '',
        textfont={'size': 50, 'color': 'white'}
    )

    ego = plotly.graph_objs.Scatter(x=[0],
                                    y=[0],
                                    marker=dict(color='#FFC000', size=20),
                                    name='Ego Vehicle',
                                    mode='markers',
                                    opacity=0)

    min_dist_envelope = plotly.graph_objs.Scatter(x=[-C2*Distance_Envelope_Min**2-C1*Distance_Envelope_Min-C0_L, -C2*Distance_Envelope_Min**2-C1*Distance_Envelope_Min-C0_R],
                                                  y=[Distance_Envelope_Min,
                                                     Distance_Envelope_Min],
                                                  marker=dict(
        color='#FFC000', size=1),
        name='Min Dist Envelope',
        line=dict(width=5))

    max_dist_envelope = plotly.graph_objs.Scatter(x=[-C2*Distance_Envelope_Max**2-C1*Distance_Envelope_Max-C0_L, -C2*Distance_Envelope_Max**2-C1*Distance_Envelope_Max-C0_R],
                                                  y=[Distance_Envelope_Max,
                                                     Distance_Envelope_Max],
                                                  marker=dict(
        color='#FF0000', size=1),
        name='Min Dist Envelope',
        line=dict(width=5))

    # generate lane mark
    right_lane_x = []
    right_lane_y = []
    left_lane_x = []
    left_lane_y = []
    for i in range(120):
        if(i%4==0):
            right_lane_y.append(i)
            right_lane_x.append(-C2*i**2-C1*i-C0_R)
            left_lane_y.append(i)
            left_lane_x.append(-C2*i**2-C1*i-C0_L)

    right_lane = plotly.graph_objs.Scatter(
        x=right_lane_x,
        y=right_lane_y,
        marker=dict(color='#BBB' if is_dark_mode else '#444', size=1),
        name='right lane',
        line=dict(width=5))

    left_lane = plotly.graph_objs.Scatter(
        x=left_lane_x,
        y=left_lane_y,
        name='left lane',
        marker=dict(color='#BBB' if is_dark_mode else '#444', size=1),
        line=dict(width=5))

    return {
        'data': [min_dist_envelope, max_dist_envelope, left_lane, right_lane, ego, lead, ],
        'layout':
        go.Layout(
            xaxis=dict(range=[min_x, max_x],
                       zeroline=False,
                       tick0=0,
                       dtick=x_dtick,
                       showticklabels=False,
                       gridwidth=1,
                       gridcolor='#AAA',
                       fixedrange=True),
            yaxis=dict(range=[min_y, max_y],
                       zeroline=False,
                       tick0=0,
                       dtick=y_dtick,
                       showticklabels=False,
                       gridwidth=1,
                       gridcolor='#AAA',
                       fixedrange=True),
            xaxis_visible=False,
            yaxis_visible=False,
            showlegend=False,
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)',
            margin=go.layout.Margin(
                l=0,  # left margin
                r=0,  # right margin
                b=0,  # bottom margin
                t=0,  # top margin
            ),
            transition={
                'duration': interval,
                'easing': 'linear'
            },)
    }


def listener():
    rospy.init_node('listener', anonymous=True)
    subs = [
        rospy.Subscriber('Lead_Vehicle_Bool', Float32, callback,
                         'Lead_Vehicle_Bool'),
        rospy.Subscriber('Lead_Vehicle_Long_Pos', Float32, callback,
                         'Lead_Vehicle_Long_Pos'),
        rospy.Subscriber('Lead_Vehicle_Lat_Pos', Float32, callback,
                         'Lead_Vehicle_Lat_Pos'),
        rospy.Subscriber('ACC_State', Float32, callback, 'ACC_State'),
        rospy.Subscriber('ACC_Controller_Mode', Float32, callback,
                         'ACC_Controller_Mode'),
        rospy.Subscriber('Lead_Vehicle_TrackID', Float32, callback,
                         'Lead_Vehicle_TrackID'),
        rospy.Subscriber('Distance_Envelope_Max', Float32, callback,
                         'Distance_Envelope_Max'),
        rospy.Subscriber('Distance_Envelope_Min', Float32, callback,
                         'Distance_Envelope_Min'),
        rospy.Subscriber('ICE_Fan_Speed_PWM', Float32, callback,
                         'ICE_Fan_Speed_PWM'),
        rospy.Subscriber('ICE_Coolant_Pump_PWM', Float32, callback,
                         'ICE_Coolant_Pump_PWM'),
        rospy.Subscriber('P0_Pump_Flow', Float32, callback, 'P0_Pump_Flow'),
        rospy.Subscriber('P4_Pump_Flow', Float32, callback, 'P4_Pump_Flow'),
        rospy.Subscriber('HV_Bat_Temp_C', Float32, callback, 'HV_Bat_Temp_C'),
        rospy.Subscriber('P0_Motor_Temp_C', Float32, callback,
                         'P0_Motor_Temp_C'),
        rospy.Subscriber('P0_Inverter_Temp_C', Float32, callback,
                         'P0_Inverter_Temp_C'),
        rospy.Subscriber('P4_Motor_Temp_C', Float32, callback,
                         'P4_Motor_Temp_C'),
        rospy.Subscriber('P4_Inverter_Temp_C', Float32, callback,
                         'P4_Inverter_Temp_C'),
        rospy.Subscriber('ICE_Coolant_Temp_C', Float32, callback,
                         'ICE_Coolant_Temp_C'),
        rospy.Subscriber('Trans_Coolant_Temp_C', Float32, callback,
                         'Trans_Coolant_Temp_C'),
        rospy.Subscriber('HV_Bat_SOC', Float32, callback, 'HV_Bat_SOC'),
        rospy.Subscriber('HV_Bat_Current', Float32, callback,
                         'HV_Bat_Current'),
        rospy.Subscriber('HV_Bus_V', Float32, callback, 'HV_Bus_V'),
        rospy.Subscriber('a_12V_Bus_V', Float32, callback, 'a_12V_Bus_V'),
        rospy.Subscriber('Camera_Fault', Float32, callback, 'Camera_Fault'),
        rospy.Subscriber('Radar_Fault', Float32, callback, 'Radar_Fault'),
        rospy.Subscriber('Tank_Fault', Float32, callback, 'Tank_Fault'),
        rospy.Subscriber('P0_Fault', Float32, callback, 'P0_Fault'),
        rospy.Subscriber('P4_Fault', Float32, callback, 'P4_Fault'),
        rospy.Subscriber('ICE_Fault', Float32, callback, 'ICE_Fault'),
        rospy.Subscriber('HV_Fault', Float32, callback, 'HV_Fault'),
        rospy.Subscriber('a_12V_Fault', Float32, callback, 'a_12V_Fault'),
        rospy.Subscriber('PCM_CAV_Fault', Float32, callback, 'PCM_CAV_Fault'),
        rospy.Subscriber('HV_Cooling_Fan', Float32,
                         callback, 'HV_Cooling_Fan'),
        rospy.Subscriber('V2X_Signal', Float32, callback, 'V2X_Signal'),
        rospy.Subscriber('V2X_Remaining_Time', Float32,
                         callback, 'V2X_Remaining_Time'),
        rospy.Subscriber('C2', Float32,
                         callback, 'C2'),
        rospy.Subscriber('C1', Float32,
                         callback, 'C1'),
        rospy.Subscriber('C0_R', Float32,
                         callback, 'C0_R'),
        rospy.Subscriber('C0_L', Float32,
                         callback, 'C0_L'),
    ]

    app.run_server()

    rospy.spin()


if __name__ == '__main__':
    listener()
