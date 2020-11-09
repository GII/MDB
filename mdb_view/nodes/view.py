#!/usr/bin/env python3
# coding=utf-8
# Implementacion de nx_listener pero con clases

import os
import dash
import json
import math
import rospy
import random
import sys
import yaml
import time
import yamlloader
import numpy as np
import pandas as pd
import networkx as nx
import plotly.graph_objects as go
import dash_core_components as dcc
import dash_html_components as html
#from beginner_tutorials.msg import Graph
# from mdb_common.msg import PNodeMsg
from collections import OrderedDict
from networkx.readwrite import json_graph
from dash.dependencies import Input, Output, State

class VIEW(object):
    def __init__(self):
        self.graph = nx.Graph()
        self.paused = True
        self.nodes_config = {'PNode': {'color': 'purple', 'history': 0},
                             'CNode': {'color': 'blue'  , 'history': 0},
                             'Goal':  {'color': 'green' , 'history': 0},
                             'ForwardModel': {'color': 'orange' , 'history': 0}}

        self.colors = ["gray", "purple", "green", "orange", "red", "blue"]
        self.sizes = [300,310,320,330,340,350]
        self.texts = ['Perception', 'PNode', 'Goal', 'ForwardModel', 'Policy', 'CNode']
        self.history = [0, 0, 0, 0, 0, 0]
        self.node_number = 0
        self.area = 1
        self.default_class = OrderedDict()
        self.default_ros_name_prefix = OrderedDict()
        self.topics = []
        self.messages = []
        # Trace used to graph the markers that represent the nodes in the figure
        self.node_trace = go.Scatter(
            x=[], y=[],
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=True,
                size=[],
                sizemode = 'area',
                line_width = 2,
                opacity = 1)
            )
        # Trace for the edges that connect CNodes and Pnodes
        self.edge_trace_1 =  go.Scatter(
            x=[], y=[],
            mode='lines',
            hoverinfo='text',
            line=dict(
                width = 1, 
                color = 'magenta',
                shape = 'spline',
                )
            )
        
        # Trace for the edges that connect CNode-Goal
        self.edge_trace_2 = go.Scatter(
            x=[], y=[],
            mode='lines',
            hoverinfo='text',
            line=dict(
                width = 1, 
                color = 'green',
                shape = 'spline',
                )
            )

        # Trace for the edges that connect CNode-ForwardModel
        self.edge_trace_3 = go.Scatter(
            x=[], y=[],
            mode='lines',
            hoverinfo='text',
            line=dict(
                width = 1, 
                color = 'blue',
                shape = 'spline',
                )
            )

        # Trace for the edges that connect CNode-Policy
        self.edge_trace_4 = go.Scatter(
            x=[], y=[],
            mode='lines',
            hoverinfo='text',
            line=dict(
                width = 1, 
                color = 'red',
                shape = 'spline',
                )
            )

        # Trace for the edges that don't contain a CNode
        self.edge_trace_5 = go.Scatter(
            x=[], y=[],
            mode='lines',
            hoverinfo='text',
            line=dict(
                width = 0.2, 
                color = 'black',
                shape = 'spline',
                )
            )

        self.pnode_trace1 = go.Scatterpolar(
            r = [], theta = [],
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=True,
                size = 10,
                sizemode = 'area',
                line_width = 2,
                opacity = 1)
            )

        self.pnode_trace2 = go.Scatterpolar(
            r = [], theta = [],
            mode='markers',
            hoverinfo='text',
            marker=dict(
                showscale=False,
                reversescale=True,
                size = 10,
                sizemode = 'area',
                line_width = 2,
                opacity = 1)
            )
    
    # Function to update the node figure
    def update_pnode_figure(self, trace, value):
        df = pd.read_csv(os.getcwd() + '/minmax_normalized_output/pnode' + value + '.csv')
        df['color'] = ['blue' if x == 1 else 'red' for x in df['activation']]
        trace.r = df['ball_dist']
        trace.theta = df['ball_ang']*180/math.pi
        trace.marker.color = df['color']

    # Functions to update the LTM figure
    def node_coordinate_update(self):
        self.node_trace.x = list(nx.get_node_attributes(self.graph,'x').values())
        self.node_trace.y = list(nx.get_node_attributes(self.graph,'y').values())

    def node_looks_update(self):
        height = max(self.node_trace.y) - min(self.node_trace.y)
        width = max(self.node_trace.x) - min(self.node_trace.x)
        
        if(height*width != 0): 
           self.area = height*width    
        
        self.node_trace.marker.color = list(nx.get_node_attributes(self.graph,'color').values())
        self.node_trace.marker.size = np.array(list(nx.get_node_attributes(self.graph,'size').values()))*100/self.area
        self.node_trace.marker.opacity = list(nx.get_node_attributes(self.graph,'opacity').values())
        self.node_trace.text = list(nx.get_node_attributes(self.graph, 'text').values())
    
    def edge_update(self):
        edge_traces = [self.edge_trace_1, self.edge_trace_2, self.edge_trace_3, self.edge_trace_4, self.edge_trace_5]
        etx = [[],[],[],[],[]]
        ety = [[],[],[],[],[]]

        for edge in self.graph.edges():
            x0, y0 = self.graph.nodes()[edge[0]]['x'], self.graph.nodes()[edge[0]]['y']
            x1, y1 = self.graph.nodes()[edge[1]]['x'], self.graph.nodes()[edge[1]]['y']
            
            # x, y = quad_bezier_curve(x0,y0,x1,y1,0.3)

            type0 = self.graph.nodes()[edge[0]]['text']
            type1 = self.graph.nodes()[edge[1]]['text']
            go_to = 0

            if('CNode' in type0 or 'CNode' in type1):
                if('PNode' in type0 or 'PNode' in type1):
                    go_to = 0
                elif('Goal' in type0 or 'Goal' in type1):
                    go_to = 1
                elif('ForwardModel' in type0 or 'ForwardModel' in type1):
                    go_to = 2
                elif('Policy' in type0 or 'Policy' in type1):
                    go_to = 3
            else:
                go_to = 4
            
            etx[go_to].append(x0)
            etx[go_to].append(x1)
            # etx[go_to] += x
            etx[go_to].append(None)
            ety[go_to].append(y0)
            ety[go_to].append(y1)
            # ety[go_to] += y
            ety[go_to].append(None)

        for edge_trace, ex, ey in zip(edge_traces, etx, ety):
            edge_trace.x = ex
            edge_trace.y = ey
        # print('Callback:', data, 'End')
        # self.graph = json_graph.adjacency_graph(json.loads(data.graph))
        # self.node_coordinate_update()
        # self.node_looks_update()
        # self.edge_update()

    # Function to display info on hover or click
    def display_info(self, data):
        display = ''
        title = 'Información del nodo'
        node_name = 'Nodo'
        try:
            point = data['points']
            info = point[0]
            node_name = info['text']
            display = 'Tipo de nodo: {}\nActivación: {}\nAparición: {}\n'.format(node_name, info['marker.opacity'], info['pointNumber'])
            adj_nodes = [n for n in self.graph.adj[info['pointNumber']].keys()]
            display += 'Conectado con: \n'
            for node in adj_nodes:
                display += ' '*5 + self.graph.nodes()[node]['text'] + '\n'
                display += ' '*10 + 'Activación: ' + str(self.graph.nodes()[node]['opacity']) + '\n'
            title = 'Información del ' + info['text']
        except:
            rospy.loginfo('No hay información disponible')
        
        return display, title

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        if sys.version_info < (3, 0):
            # node_module = __import__(module_string, fromlist=[bytes(class_string, "utf-8")])
            node_module = __import__(module_string, fromlist=[class_string])
        else:
            node_module = __import__(module_string, fromlist=[class_string])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class
    
    def add_node(self, data):
        ident = data.id
        name = data.class_name
        if(not(self.graph.has_node(ident))):
            idx = self.nodes_config[name]['history']
            if(name == "PNode"):
                posx = idx // 3
                posy = idx % 3
                posx = 1 + posx + posy / 3.0
            elif(name == "Goal"):
                posx, posy = 3*(idx + 1), 4
            elif(name == "ForwardModel"):
                posx, posy = 3 * (idx + 1), 6
            elif(name == "Policy"):
                posx, posy = 9, 3 - idx
            elif(name == "CNode"):
                posx = idx // 3
                posy = idx % 3
                posx, posy = 1 + posx + posy / 3.0, posy + 8
            elif(name == "Perception"):
                posx, posy = 0, idx + 1
            
            self.graph.add_node(ident,
                                color = self.nodes_config[name]['color'],
                                size = self.sizes[2],
                                text = name + ' ' + str(self.nodes_config[name]['history']),
                                opacity = 1,
                                x = posx,
                                y = posy
                                )
            self.nodes_config[name]['history'] += 1

        else:
            rospy.logwarn('Node {} already created'.format(ident))

    def add_node_callback(self, data):
            rospy.loginfo('Received node')
            self.add_node(data)
            self.node_coordinate_update()
            self.node_looks_update()
            self.edge_update()

    def setup_topics(self, connectors):
        for connector in connectors:
            self.default_class[connector["data"]] = connector.get("default_class")
            self.default_ros_name_prefix[connector["data"]] = connector.get("ros_name_prefix")
            if self.default_ros_name_prefix[connector["data"]] is not None:
                topic = rospy.get_param(connector["ros_name_prefix"] + "_topic")
                self.topics.append(topic)
                message = self.class_from_classname(rospy.get_param(connector["ros_name_prefix"] + "_msg"))
                self.messages .append(message)

    # Function to update the graph
    def listen_topics(self):
        for topic, message in zip(self.topics, self.messages):
            rospy.Subscriber(name = topic, data_class = message, callback = self.add_node_callback)
        # rospy.Subscriber(name = '/mdb/ltm/p_node', data_class = PNodeMsg, callback = self.add_node_callback)

    def setup(self, log_level, file_name):
        """Init LTM: read ROS parameters, init ROS subscribers and load initial nodes."""
        # t = time.localtime()
        # current_time = time.strftime("%H:%M:%S", t)
        # print(current_time)
        # rospy.loginfo(current_time)
        if file_name is None:
            rospy.logerr("No configuration file specified!")
        else:
            if not os.path.isfile(file_name):
                rospy.logerr(file_name + " does not exist!")
            else:
                # rospy.loginfo(time.time())
                rospy.loginfo("Loading configuration from %s...", file_name)
                configuration = yaml.load(open(file_name, "r"), Loader=yamlloader.ordereddict.CLoader)
                # configuration = yaml.load(open(file_name, "r", encoding="utf-8"), Loader=yamlloader.ordereddict.CLoader)
                self.setup_topics(configuration["LTM"]["Connectors"])
                rospy.on_shutdown(self.shutdown)
    
    def run(self, seed=None, log_level="INFO"):
        """Start the visualization part"""
        rospy.init_node("mdb_view", anonymous = True)
        try:
            self.setup(log_level, seed)
            rospy.loginfo("Running visualization...")
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")

    def shutdown(self):
        rospy.loginfo('Closing port 8050')
        os.system('kill -9 $( lsof -i:8050 -t )')

app = dash.Dash(__name__, suppress_callback_exceptions=True)

app.layout = html.Div([
    dcc.Location(id='url', refresh=False),
    html.Div(id='page-content')
])

index_page = html.Div(
    className = 'twelve columns',
    style = {'width': '100%', 'display': 'flex', 'align-items': 'center', 'justify-content': 'center', 'textAlign': 'center'},
    children = [
        html.Div(
            className = 'four columns',
            children = [
                html.H1('LTM'),
                html.A([
                    html.Img(
                        src= app.get_asset_url('ltm.jpg'),
                        style = {'margin' : '20px'}
                        )
                    ], href='/ltm'),
                ]
        ),
        html.Div(
            className = 'four columns',
            children = [
                html.H1('Nodes'),
                html.A([
                    html.Img(
                        src= app.get_asset_url('ltm.jpg'),
                        style = {'margin' : '20px'}
                        )
                    ], href='/nodes'),
                ]
        )
    ])

ltm_layout = html.Div([
    html.H1('LTM', style={'textAlign': 'center'}),
    html.Div(
        # className = 'row',
        style = {'width': '100%', 'display': 'flex', 'align-items': 'center', 'justify-content': 'center'},
        children = [
            html.Button(
                # className = 'col-sm-3',
                children = 'Pausa', 
                title = 'Inicio', 
                id = 'pause', 
                n_clicks = 0,
                style={'margin':'10px'}
            ),
            html.Button(
                # className = 'col-sm-3',
                children = 'Guardar', 
                title = 'Guardar', 
                id = 'save', 
                style={'margin':'10px'}
            )
        ]
    ),
    html.Div(
        children=[
            dcc.Graph(
                className="eight columns",
                id="graph-with-update"
                )
        ]
    ),
    html.Div(
        style = {'background-color': '#6f8bc0'},
        className="three columns",
        children=[
            html.Div(
                className='twelve columns',
                children=[
                    dcc.Markdown(children = "Información del nodo", id = 'hover_mk', style={'textAlign': 'center'}),
                    # html.Pre(id='hover-data', style=styles['pre'])
                    html.Pre(
                        id='hover-data')
                ],
                style={'height': '100%'}),
            html.Div(
                className='twelve columns',
                children=[
                    dcc.Markdown(children = "Información del nodo", id = 'click_mk', style={'textAlign': 'center'}),
                    # html.Pre(id='click-data', style=styles['pre'])
                    html.Pre(
                        id='click-data')
                ],
                style={'height': '100%'}),
            ]
    ),
    html.Div(
        className = 'twelve columns',
        children = [
            dcc.Link('Nodes', href='/nodes'),
        ]
    ),
    dcc.Interval(
        id = 'interval-ltm',
        interval = 1*1000,
        n_intervals = 0)
    ])


node_layout = html.Div([
    html.H1('Nodes', style={'textAlign': 'center'}),
    html.Div(
        className = 'five columns',
        style = {'width': '40%','display': 'inline-block', 'align-items': 'center', 'justify-content': 'center'},
        children = [
            dcc.Dropdown(
                id='nodes-dropdown1',
                options=[],
                value='', 
                style = {'width': '100%', 'color': 'black'}
                ),
            dcc.Graph(
                id="graph-with-nodes1"
                )
        ]
    ),
    html.Div(
        className = 'five columns',
        style = {'width': '40%','display': 'inline-block', 'align-items': 'center', 'justify-content': 'center'},
        children = [
            dcc.Dropdown(
                id='nodes-dropdown2',
                options=[],
                value='', 
                style = {'width': '100%', 'color': 'black'}
                ),
            dcc.Graph(
                id="graph-with-nodes2"
                )
        ]
    ),
    html.Div(
        className = 'twelve columns',
        children = [
            dcc.Link('LTM', href='/ltm'),
        ]
    ),
    dcc.Interval(
        id = 'interval-nodes',
        interval = 1*1000,
        n_intervals = 0
    )
])

# Definición de los callbacks de la aplicación
# Se encargan de la interactividad de la aplicación

@app.callback(
    Output('nodes-dropdown1', 'options'),
    Output('nodes-dropdown2', 'options'),
    Output('graph-with-nodes1', 'figure'),
    Output('graph-with-nodes2', 'figure'),
    [Input('interval-nodes', 'n_intervals'),
    Input('nodes-dropdown1', 'value'),
    Input('nodes-dropdown2', 'value')])
def update_nodes(n_intervals, value1, value2):
    options = [{'label': view.graph.nodes()[node]['text'], 'value': view.graph.nodes()[node]['text']} for node in view.graph.nodes() if 'PNode' in view.graph.nodes()[node]['text']]
    
    try:
        view.update_pnode_figure(view.pnode_trace1, value1[6::])
        view.update_pnode_figure(view.pnode_trace2, value2[6::])
    except:
        pass

    fig1 = go.Figure(
            data = [view.pnode_trace1],
            layout = go.Layout(
                xaxis_title="Iteracion 90",
                titlefont_size = 16,
                autosize = True,
                showlegend = False,
                hovermode ='closest',
                margin = dict(b = 0,l = 0,r = 0,t = 0),
                xaxis = dict(showgrid = False, zeroline = False, showticklabels = False),
                yaxis = dict(showgrid = False, zeroline = False, showticklabels = False))
            )

    fig2 = go.Figure(
            data = [view.pnode_trace2],
            layout=go.Layout(
                xaxis_title="Iteracion 90",
                titlefont_size = 16,
                autosize = True,
                showlegend = False,
                hovermode ='closest',
                margin = dict(b = 0,l = 0,r = 0,t = 0),
                xaxis = dict(showgrid = False, zeroline = False, showticklabels = False),
                yaxis = dict(showgrid = False, zeroline = False, showticklabels = False))
            )

    return options, options, fig1, fig2

@app.callback(
    Output('graph-with-update', 'figure'),
    [Input('interval-ltm', 'n_intervals')])
def update_figure(n_intervals):
    if(not(view.paused)):
        view.listen_topics()

    fig = go.Figure(
            data=[view.node_trace,  view.edge_trace_1, view.edge_trace_2, view.edge_trace_3, view.edge_trace_4, view.edge_trace_5],
            layout=go.Layout(
                xaxis_title="Iteracion 90",
                titlefont_size = 16,
                autosize = True,
                showlegend = False,
                hovermode = 'closest',
                margin = dict(b = 0,l = 0,r = 0,t = 0),
                xaxis = dict(showgrid = False, zeroline = False, showticklabels = False),
                yaxis = dict(showgrid = False, zeroline = False, showticklabels = False))
            )
    
    return fig

# Estos callbacks están bastante bien, parece que no deben tocarse 
# Callback de pausado
@app.callback(
    Output('pause', 'children'),
    Output('interval-ltm', 'interval'),
    [Input('pause', 'n_clicks')]
)
def update_pause(n_clicks):
    view.paused = not(view.paused)
    if(view.paused):
        rospy.loginfo('Se ha pausado el proceso')
        return 'Reanudar', 1*1000*60*60
    rospy.loginfo('Se ha reanudado el proceso')
    return 'Pausar', 1*1000

# Callback de guardado
@app.callback(
    Output('save', 'title'),
    [Input('save', 'n_clicks')])
def save_graph(n_clicks2):
    nx.write_edgelist(view.graph, 'graph_{}.txt'.format(n_clicks2))
    nx.write_gexf(view.graph, 'graph_{}.gexf'.format(n_clicks2))
    rospy.loginfo('Se ha guardado el archivo graph_{}.txt'.format(n_clicks2))

# Callback para mostrar info al hacer click sobre nodo
@app.callback(
    Output('click-data', 'children'),
    Output('click_mk', 'children'),
    [Input('graph-with-update', 'clickData')])
def display_click_data(clickData):
    return view.display_info(clickData)

# Callback para mostrar la info al pasar el mouse encima de nodo
@app.callback(
    Output('hover-data', 'children'),
    Output('hover_mk', 'children'),
    [Input('graph-with-update', 'hoverData')])
def display_hover_data(hoverData):
    return view.display_info(hoverData)

@app.callback(Output('page-content', 'children'),
              [Input('url', 'pathname')])
def display_page(pathname):
    if pathname == '/ltm':
        return ltm_layout
    elif pathname == '/nodes':
        return node_layout
    else:
        return index_page

view = VIEW()
    
