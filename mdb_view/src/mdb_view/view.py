#!/usr/bin/env python3
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
import pandas as pd
import networkx as nx
import dash_cytoscape as cyto
import plotly.graph_objects as go
import dash_core_components as dcc
import dash_html_components as html
from collections import OrderedDict
from networkx.readwrite import json_graph
from dash.dependencies import Input, Output, State

class VIEW(object):
    def __init__(self):
        self.iteration = 0
        self.graph = nx.Graph()
        self.paused = True
        self.pnode_temp_trace = go.Scatter()
        self.pnode_dict = {}
        self.nodes_config = {'pnode': {'history': 0},
                             'cnode': {'history': 0},
                             'goal':  {'history': 0},
                             'forward_model': {'history': 0},
                             'goal': {'history': 0},
                             'perception': {'history': 0},
                             'policy': {'history': 0}}

        self.default_class = OrderedDict()
        self.default_ros_node_prefix = OrderedDict()
        self.save_dir = ""

        self.app = dash.Dash(__name__, suppress_callback_exceptions=True)
        
        self.stylesheet=[
            {
                'selector': 'node',
                'style': {
                    'label': 'data(name)',
                    'position': 'data(position)',
                    'background-opacity' : 'data(activation)'
                }
            },
            {
                'selector': 'edge',
                'style': {
                    'width' : '1%',
                    'curve-style' : 'bezier',
                    'line-color' : 'data(color)'
                }
            },
            {
                'selector': '[node_type *= "pnode"]',
                'style': {
                    'background-color': 'purple'
                }
            },
            {
                'selector': '[node_type *= "cnode"]',
                'style': {
                    'background-color': 'blue'
                }
            },
            {
                'selector': '[node_type *= "policy"]',
                'style': {
                    'background-color': 'red'
                }
            },
            {
                'selector': '[node_type *= "perception"]',
                'style': {
                    'background-color': 'grey'
                }
            },
            {
                'selector': '[node_type *= "forward_model"]',
                'style': {
                    'background-color': 'orange'
                }
            },
            {
                'selector': '[node_type *= "goal"]',
                'style': {
                    'background-color': 'green'
                }
            },
            {
                'selector': '[type *= "PC"]',
                'style': {
                    'line-color': 'green'
                }
            }
        ]
        
        self.index_page = html.Div(
            className = 'twelve columns',
            style = {'width': '100%', 'display': 'flex', 'align-items': 'center', 'justify-content': 'center', 'textAlign': 'center'},
            children = [
                html.Div(
                    className = 'four columns',
                    children = [
                        html.H1('LTM'),
                        html.H6('Seleccione esta imagen para ver la red de nodos de la LTM'),
                        html.A([
                            html.Img(
                                src= self.app.get_asset_url('ltm.jpg'),
                                )
                            ], href='/ltm'),
                        ]
                ),
                html.Div(
                    className = 'four columns',
                    children = [
                        html.H1('Nodes'),
                        html.H6('Seleccione esta imagen para ver los puntos y antipuntos de los PNodes'),
                        html.A([
                            html.Img(
                                src= self.app.get_asset_url('ltm.jpg'),
                                )
                            ], href='/nodes'),
                        ]
                )
            ])
    
        self.ltm_layout = html.Div([
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
                className="eight columns",
                style = {'background-color' : 'white'},
                children = [
                    cyto.Cytoscape(
                        id='cytoscape-ltm',
                        layout={'name': 'preset'},
                        style={'width': '100%', 'height': '550px'},
                        elements = [],
                        stylesheet = self.stylesheet
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
        
        self.node_layout = html.Div([
            html.H1('Nodes', style={'textAlign': 'center'}),
            html.Div(
                className = 'five columns',
                style = {'width': '37%', 'display': 'inline-block', 'align-items': 'center', 'justify-content': 'center'},
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
                style = {'width': '37%', 'display': 'inline-block', 'align-items': 'center', 'justify-content': 'center'},
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
                className="two columns",
                style = {'align-items': 'center'},
                children=[
                    html.Div(
                        children=[
                            dcc.Markdown(
                                children = "Configuración", 
                                id = 'config', 
                                style={
                                    'textAlign': 'center',
                                    'background-color': '#6f8bc0'
                                    }
                            ),
                            html.Pre(
                                id='info_config',
                                children = ['Seleccione el tipo de gráfica \nque desea observar']
                            ),
                            dcc.RadioItems(
                                id = 'radio_type',
                                options=[
                                    {'label': 'Polar', 'value': 'pol'},
                                    {'label': 'Rectangular', 'value': 'rect'},
                                ],
                                value='rect',
                                labelStyle={'display': 'inline-block'}
                            ),   
                            html.Pre(
                                id='info_perc',
                                children = ['Seleccione las percepciones \npor graficar']
                            ),
                            dcc.Dropdown(
                                id = 'perceptions1',
                                options=[],
                                value='',
                                style = {'width': '100%', 'color': 'black'}
                            ),
                            dcc.Dropdown(
                                id = 'perceptions2',
                                options=[],
                                value='',
                                style = {'width': '100%', 'color': 'black'}
                            )
                        ],
                        style={'height': '100%'}),
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

    ########CALLBACK FUNCTIONS##########
    '''Functions used to add interactivity to the web app. Here
    are included the web app callbacks. They include the
    functions to update the PNodes graphs, the LTM network graph, 
    functions to pause the simulation, to save the graphs, display 
    info when hovering or clicking over a node and to update the
    web page layout'''

    # Functions to update the LTM figure
    def update_fig_ltm(self, n_intervals):
        elements = cyto_data = nx.readwrite.json_graph.cytoscape.cytoscape_data(self.graph)
        nodes = cyto_data['elements']['nodes']
        for node in nodes:
            node['position'] = node.get('data').get('position')
            del node['data']['position']
        edges = cyto_data['elements']['edges']
        elements = nodes + edges
        return elements
    # Function to update the node figure
    def update_pnode_figure(self, value, xaxis, yaxis, type_plot):
        df = self.pnode_dict[value]
        if(type_plot == 'pol'):
            print(value, xaxis, yaxis, type_plot)
            trace = go.Scatterpolar(
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
            trace.r = df[xaxis]
            trace.theta = df[yaxis]*180/math.pi
            trace.marker.color = df['color']
        else:
            trace = go.Scatter(
            x=[], y=[],
            mode='markers+text',
            hoverinfo='text',
            textposition='bottom center',
            marker=dict(
                showscale=False,
                reversescale=True,
                size= 10,
                sizemode = 'area',
                line_width = 2,
                opacity = 1)
            )
            trace.x = df[xaxis]
            trace.y = df[yaxis]
            trace.marker.color = df['color']
        return trace
    
    def update_fig_nodes(self, n_intervals, value1, value2, value3, value4, value5):        
        options = [{'label': self.graph.nodes()[node]['name'], 'value': self.graph.nodes()[node]['name']} for node in self.graph.nodes() if self.graph.nodes()[node]['node_type'] == 'pnode']
        options2 = [{'label': self.graph.nodes()[node]['name'], 'value': self.graph.nodes()[node]['name']} for node in self.graph.nodes() if self.graph.nodes()[node]['node_type'] == 'perception']
        try:
            trace1 = self.update_pnode_figure(value1, value3, value4, value5)
            trace2 = self.update_pnode_figure(value2, value3, value4, value5)
        except:
            trace1 = self.pnode_temp_trace
            trace2 = self.pnode_temp_trace
        fig1 = go.Figure(
                data = [trace1],
                layout = go.Layout(
                    title = value1 + ' en la iteración ' + str(self.iteration),
                    xaxis_title = value3,
                    yaxis_title = value4,
                    titlefont_size = 16,
                    autosize = True,
                    showlegend = False,
                    hovermode ='closest',
                    margin = dict(b = 0,l = 0,r = 0,t = 30),
                    xaxis = dict(showgrid = False, zeroline = False, showticklabels = True),
                    yaxis = dict(showgrid = False, zeroline = False, showticklabels = True))
                )

        fig2 = go.Figure(
                data = [trace2],
                layout=go.Layout(
                    title = value2 + ' en la iteración ' + str(self.iteration), 
                    xaxis_title = value3,
                    yaxis_title = value4,
                    autosize = True,
                    showlegend = False,
                    hovermode ='closest',
                    margin = dict(b = 0,l = 0,r = 0,t = 30),
                    xaxis = dict(showgrid = False, zeroline = False, showticklabels = True),
                    yaxis = dict(showgrid = False, zeroline = False, showticklabels = True))
                )

        return options, options, fig1, fig2, options2, options2
    # Function to display info on hover or click
    def display_info(self, data):
        display = ''
        title = 'Información del nodo'
        try:
            node_name = 'Nodo'
            node = data['id']
            node_name = data['name']
            display = 'Tipo de nodo: {}\n'.format(data['node_type'])
            display += 'Nombre del nodo: {}\n'.format(data['id'])
            display += 'Activación: {}\n'.format(data['activation'])
            adj_nodes = [n for n in self.graph.adj[node].keys()]
            display += 'Conectado con: \n'
            for node in adj_nodes:
                display += ' '*5 + self.graph.nodes()[node]['name'] + '\n'
                display += ' '*7 + 'Tipo de nodo: {}\n'.format(self.graph.nodes()[node]['node_type']) + '\n'
                display += ' '*7 + 'Activación: {}\n'.format(self.graph.nodes()[node]['activation']) + '\n'
            title = 'Información de ' + node_name
        except:
            pass
        return display, title

    def define_app(self):
        self.app.layout = html.Div([
            dcc.Location(id='url', refresh=False),
            html.Div(id='page-content')
        ])

        self.app.callback(
            Output('nodes-dropdown1', 'options'),
            Output('nodes-dropdown2', 'options'),
            Output('graph-with-nodes1', 'figure'),
            Output('graph-with-nodes2', 'figure'),
            Output('perceptions1', 'options'),
            Output('perceptions2', 'options'),
            [Input('interval-nodes', 'n_intervals'),
            Input('nodes-dropdown1', 'value'),
            Input('nodes-dropdown2', 'value'), 
            Input('perceptions1', 'value'),
            Input('perceptions2', 'value'),
            Input('radio_type', 'value')]
            )(self.update_fig_nodes)
        
        self.app.callback(
            Output('cytoscape-ltm', 'elements'),
            [Input('interval-ltm', 'n_intervals')]
            )(self.update_fig_ltm)

        self.app.callback(
            Output('pause', 'children'),
            Output('interval-ltm', 'interval'),
            [Input('pause', 'n_clicks')]
            )(self.update_pause)
        self.app.callback(
            Output("cytoscape-ltm", "generateImage"),
            [Input('save', 'n_clicks')]
            )(self.save_graph)
    
        self.app.callback(
            Output('click-data', 'children'),
            Output('click_mk', 'children'),
            [Input('cytoscape-ltm', 'tapNodeData')]
        )(self.display_info)
        
        self.app.callback(
            Output('hover-data', 'children'),
            Output('hover_mk', 'children'),
            [Input('cytoscape-ltm', 'mouseoverNodeData')]
        )(self.display_info)

        self.app.callback(
            Output('page-content', 'children'),
            [Input('url', 'pathname')]
        )(self.display_page)
    
    def update_pause(self, n_clicks):
        self.paused = not(self.paused)
        if(self.paused):
            rospy.loginfo('Se ha pausado el proceso')
            return 'Reanudar', 1*1000*60*60
        rospy.loginfo('Se ha reanudado el proceso')
        return 'Pausar', 1000

    def save_graph(self, n_clicks2):
        ftype = 'png'
        filename = 'ltm_' + str(n_clicks2)
        action = 'download'

        nx.write_edgelist(self.graph, self.save_dir + 'graph_{}.txt'.format(n_clicks2))
        nx.write_gexf(self.graph, self.save_dir + 'graph_{}.gexf'.format(n_clicks2))
        rospy.loginfo('Se ha guardado el archivo graph_{}.txt'.format(n_clicks2))
        if(n_clicks2 != None):
            return {
                    'type': ftype,
                    'action' : action,
                    'filename': filename
                }
        else:
            return {}
    
    def display_page(self, pathname):
        if pathname == '/ltm':
            return self.ltm_layout
        elif pathname == '/nodes':
            return self.node_layout
        else:
            return self.index_page

    ######ROS NODE CONFIGURATION FUCNTIONS#####
    '''These functions are responsible of setting up the ROS node
    for visualization'''
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
    
    def setup_control_channel(self, control_channel):
        """Load simulator / robot configuration channel."""
        topic = rospy.get_param(control_channel["control_prefix"] + "_topic")
        self.control_message = self.class_from_classname(rospy.get_param(control_channel["control_prefix"] + "_msg"))
        self.control_publisher = rospy.Publisher(topic, self.control_message, latch=True, queue_size=0)
        rospy.logdebug("Subscribing to %s...", topic)
    
    def setup_topics(self, connectors):
        """Load topic configuration for adding nodes."""
        for connector in connectors:
            self.default_class[connector["data"]] = connector.get("default_class")
            self.default_ros_node_prefix[connector["data"]] = connector.get("ros_node_prefix")
            if self.default_ros_node_prefix[connector["data"]] is not None:
                topic = rospy.get_param(connector["ros_node_prefix"] + "_topic")
                message = self.class_from_classname(rospy.get_param(connector["ros_node_prefix"] + "_msg"))
                callback = getattr(self, connector["callback"])
                rospy.logdebug("Subscribing to %s...", topic)
                rospy.Subscriber(topic, message, callback=callback)
    
    def setup(self, log_level, file_name):
        """Init VIEW: read ROS parameters, init ROS subscribers and load initial nodes."""
        if file_name is None:
            rospy.logerr("No configuration file specified!")
        else:
            if not os.path.isfile(file_name):
                rospy.logerr(file_name + " does not exist!")
            else:
                rospy.init_node("mdb_view", log_level = getattr(rospy, log_level))
                rospy.loginfo("Loading configuration from %s...", file_name)
                configuration = yaml.load(open(file_name, "r"), Loader=yamlloader.ordereddict.CLoader)
                # configuration =s yaml.load(open(file_name, "r", encoding="utf-8"), Loader=yamlloader.ordereddict.CLoader)
                self.setup_topics(configuration["LTM"]["Connectors"])
                self.setup_control_channel(configuration["Control"])
                rospy.on_shutdown(self.shutdown)
                # rospy.loginfo('setup function gets here 3')
                # rospy.on_shutdown(self.shutdown)
                # rospy.loginfo('setup function gets here 3')
    
    def run(self, seed = None, log_level = "INFO", save_dir = None):
        """Start the visualization part"""
        try:
            self.define_app()
            self.setup(log_level, seed)
            self.save_dir = save_dir
            self.ask_for_ltm()
            self.app.run_server()
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")

    def shutdown(self):
        rospy.loginfo('Closing port 8050')
        os.system('kill -9 $( lsof -i:8050 -t )')

    ######ROS NODE CALLBACK FUCNTIONS#####
    '''These functions are executed when a new message is 
    published in a topic to which the node is subscribed 
    and for sending commands to the control topic'''

    def add_node_callback(self, data):
        rospy.loginfo('Received node')
        ident = data.id
        name = data.class_name
        name = name.split('/')[2]
        if(not(self.graph.has_node(ident))):
            idx = self.nodes_config[name]['history']
            if(name == "pnode"):
                posx = idx // 3
                posy = idx % 3
                posx = 1 + posx + posy / 3.0
            elif(name == "goal"):
                posx, posy = 3*idx, 4
            elif(name == "forward_model"):
                posx, posy = 3 * idx, 6
            elif(name == "policy"):
                posx, posy = 13, idx + 3
            elif(name == "cnode"):
                posx = idx // 3
                posy = idx % 3
                posx, posy = 1 + posx + posy / 3.0, posy + 8
            elif(name == "perception"):
                posx, posy = -3, idx + 3
            
            self.graph.add_node(ident,
                                id = ident,
                                position =  {'x': posx*75, 'y': posy*75},
                                node_type = name,
                                name = ident,
                                activation = 0.5
                                )
            self.nodes_config[name]['history'] += 1
        else:
            rospy.logwarn('Node {} already created'.format(ident))

    def ask_for_ltm(self):
        rospy.logdebug('Asking for current nodes in LTM')
        self.control_publisher.publish(command = "publish_ltm", world = "not", reward = False)


