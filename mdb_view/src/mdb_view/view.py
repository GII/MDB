"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

standard_library.install_aliases()
from builtins import dict, list, object, open, zip

# Standard imports
import os
import math

# Library imports
import dash
import rospy
import yaml
import yamlloader
import pandas as pd
import networkx as nx
import dash_cytoscape as cyto
import plotly.graph_objects as go
import dash_core_components as dcc
import dash_html_components as html
from collections import OrderedDict
from dash.dependencies import Input, Output


class View(object):
    def __init__(self):
        # INFO VARIABLES
        self.iteration = 0
        self.current_world = ""
        self.current_policy = ""
        self.current_reward = 0
        self.current_goal = ""
        # GRAPH CONSTRUCTION VARIABLES
        self.graph = nx.Graph()
        self.pnode_dict = {}
        self.nodes_config = {
            "Perception": 0,
            "PNode": 0,
            "CNode": 0,
            "Goal": 0,
            "ForwardModel": 0,
            "Policy": 0,
        }
        self.init_graph()
        # CONTROL VARIABLES
        self.paused = True
        self.save_dir = ""
        self.default_class = OrderedDict()
        self.default_ros_node_prefix = OrderedDict()
        self.default_ros_data_prefix = OrderedDict()
        # ROS communication
        self.control_message = None
        self.control_publisher = None
        # VISUALIZATION CONTAINERS AND STYLES
        self.pnode_temp_trace = go.Scatter()
        self.app = dash.Dash(__name__, suppress_callback_exceptions=True)
        self.sens_names = []
        self.stylesheet = [
            {
                "selector": "node",
                "style": {
                    "label": "data(id)",
                    "size": "20%",
                    "font-size": "18",
                    "background-opacity": "data(activation)",
                    "background-color": "data(color)",
                },
            },
            {
                "selector": "edge",
                "style": {"width": "data(width)", "curve-style": "bezier", "line-color": "data(color)"},
            },
            {"selector": '[node_type *= "title"]', "style": {"size": "1%", "text-wrap": "wrap", "font-weight": "bold"}},
        ]
        self.index_page = html.Div(
            className="twelve columns",
            style={
                "width": "100%",
                "display": "flex",
                "align-items": "center",
                "justify-content": "center",
                "textAlign": "center",
            },
            children=[
                html.Div(
                    className="four columns",
                    children=[
                        html.H1("LTM"),
                        html.H6("Seleccione esta imagen para ver la red de nodos de la LTM"),
                        html.A(
                            [
                                html.Img(
                                    src=self.app.get_asset_url("ltm.jpg"),
                                )
                            ],
                            href="/ltm",
                        ),
                    ],
                ),
                html.Div(
                    className="four columns",
                    children=[
                        html.H1("Nodes"),
                        html.H6("Seleccione esta imagen para ver los puntos y antipuntos de los PNodes"),
                        html.A(
                            [
                                html.Img(
                                    src=self.app.get_asset_url("ltm.jpg"),
                                )
                            ],
                            href="/nodes",
                        ),
                    ],
                ),
            ],
        )
        self.ltm_layout = html.Div(
            [
                html.H1("LTM", style={"textAlign": "center"}),
                html.Div(
                    style={"width": "100%", "display": "flex", "align-items": "center", "justify-content": "center"},
                    children=[
                        html.Button(children="Pausa", title="Inicio", id="pause", n_clicks=0, style={"margin": "10px"}),
                        html.Button(children="Guardar", title="Guardar", id="save", style={"margin": "10px"}),
                    ],
                ),
                html.Div(
                    className="eight columns",
                    style={"background-color": "white"},
                    children=[
                        cyto.Cytoscape(
                            id="cytoscape-ltm",
                            layout={"name": "preset"},
                            style={"width": "100%", "height": "600px"},
                            elements=[],
                            stylesheet=self.stylesheet,
                        )
                    ],
                ),
                html.Div(
                    style={"background-color": "#6f8bc0", "height": "100%"},
                    className="three columns",
                    children=[
                        html.Div(
                            className="twelve columns",
                            children=[
                                dcc.Markdown(
                                    children="Información del nodo", id="hover_mk", style={"textAlign": "center"}
                                ),
                                html.Pre(id="hover-data"),
                                dcc.Markdown(
                                    children="Información del nodo", id="click_mk", style={"textAlign": "center"}
                                ),
                                html.Pre(id="click-data"),
                            ],
                        )
                    ],
                ),
                html.Div(
                    className="twelve columns",
                    children=[
                        dcc.Link("Nodes", href="/nodes"),
                        dcc.Interval(id="interval-ltm", interval=1 * 1000, n_intervals=0),
                    ],
                ),
            ]
        )
        self.node_layout = html.Div(
            [
                html.H1("Nodes", style={"textAlign": "center"}),
                html.Div(
                    className="five columns",
                    style={
                        "width": "37%",
                        "display": "inline-block",
                        "align-items": "center",
                        "justify-content": "center",
                    },
                    children=[
                        dcc.Dropdown(
                            id="nodes-dropdown1", options=[], value="", style={"width": "100%", "color": "black"}
                        ),
                        dcc.Graph(id="graph-with-nodes1"),
                    ],
                ),
                html.Div(
                    className="five columns",
                    style={
                        "width": "37%",
                        "display": "inline-block",
                        "align-items": "center",
                        "justify-content": "center",
                    },
                    children=[
                        dcc.Dropdown(
                            id="nodes-dropdown2", options=[], value="", style={"width": "100%", "color": "black"}
                        ),
                        dcc.Graph(id="graph-with-nodes2"),
                    ],
                ),
                html.Div(
                    className="two columns",
                    style={"align-items": "center", "height": "100%"},
                    children=[
                        dcc.Markdown(
                            children="Configuración",
                            id="config",
                            style={"textAlign": "center", "background-color": "#6f8bc0"},
                        ),
                        html.Pre(id="info_config", children=["Seleccione el tipo de gráfica \nque desea observar"]),
                        dcc.RadioItems(
                            id="radio_type",
                            options=[
                                {"label": "Polar", "value": "pol"},
                                {"label": "Rectangular", "value": "rect"},
                            ],
                            value="rect",
                            labelStyle={"display": "inline-block"},
                        ),
                        html.Pre(id="info_perc", children=["Seleccione las percepciones \npor graficar"]),
                        dcc.Dropdown(
                            id="perceptions1", options=[], value="", style={"width": "100%", "color": "black"}
                        ),
                        dcc.Dropdown(
                            id="perceptions2", options=[], value="", style={"width": "100%", "color": "black"}
                        ),
                    ],
                ),
                html.Div(
                    className="twelve columns",
                    children=[
                        dcc.Link("LTM", href="/ltm"),
                        dcc.Interval(id="interval-nodes", interval=1 * 1000, n_intervals=0),
                    ],
                ),
            ]
        )

    ########CALLBACK FUNCTIONS##########
    # Functions used to add interactivity to the web app. Here
    # are included the web app callbacks. They include the
    # functions to update the PNodes graphs, the LTM network graph,
    # functions to pause the simulation, to save the graphs, display
    # info when hovering or clicking over a node and to update the
    # web page layout

    def init_graph(self):
        self.graph.add_node("title", node_type="title", id="", activation=0, position={"x": 5 * 75, "y": -0.5 * 75})
        self.set_title()

    def set_title(self):
        self.graph.nodes()["title"]["id"] = "GOAL: {} WORLD: {} \nITERATION: {} REWARD: {}\nPOLICY: {}".format(
            self.current_goal, self.current_world, self.iteration, self.current_reward, self.current_policy
        )

    def update_fig_ltm(self):
        """Update the LTM figure."""
        elements = cyto_data = nx.readwrite.json_graph.cytoscape.cytoscape_data(self.graph)
        nodes = cyto_data["elements"]["nodes"]
        for node in nodes:
            node["position"] = node.get("data").get("position")
        edges = cyto_data["elements"]["edges"]
        elements = nodes + edges
        return elements

    def update_pnode_figure(self, value, xaxis, yaxis, type_plot):
        """Update the node figure."""
        df = self.pnode_dict[value]
        if type_plot == "pol":
            print(value, xaxis, yaxis, type_plot)
            trace = go.Scatterpolar(
                r=[],
                theta=[],
                mode="markers",
                hoverinfo="text",
                marker=dict(showscale=False, reversescale=True, size=10, sizemode="area", line_width=2, opacity=1),
            )
            trace.r = df[xaxis]
            trace.theta = df[yaxis] * 180 / math.pi
        else:
            trace = go.Scatter(
                x=[],
                y=[],
                mode="markers+text",
                hoverinfo="text",
                textposition="bottom center",
                marker=dict(showscale=False, reversescale=True, size=10, sizemode="area", line_width=2, opacity=1),
            )
            trace.x = df[xaxis]
            trace.y = df[yaxis]
        trace.marker.color = ["blue" if confidence == 1 else "red" for confidence in df["confidence"]]
        return trace

    def update_fig_nodes(self, value1, value2, value3, value4, value5):
        options = [
            {"label": self.graph.nodes()[node]["id"], "value": self.graph.nodes()[node]["id"]}
            for node in self.graph.nodes()
            if self.graph.nodes()[node]["node_type"] == "PNode"
        ]
        options2 = [{"label": column, "value": column} for column in self.sens_names]
        try:
            trace1 = self.update_pnode_figure(value1, value3, value4, value5)
            trace2 = self.update_pnode_figure(value2, value3, value4, value5)
        except:
            trace1 = self.pnode_temp_trace
            trace2 = self.pnode_temp_trace
        fig1 = go.Figure(
            data=[trace1],
            layout=go.Layout(
                title=value1 + " en la iteración " + str(self.iteration),
                xaxis_title=value3,
                yaxis_title=value4,
                titlefont_size=16,
                autosize=True,
                showlegend=False,
                hovermode="closest",
                margin=dict(b=0, l=0, r=0, t=30),
                xaxis=dict(showgrid=False, zeroline=False, showticklabels=True),
                yaxis=dict(showgrid=False, zeroline=False, showticklabels=True),
            ),
        )
        fig2 = go.Figure(
            data=[trace2],
            layout=go.Layout(
                title=value2 + " en la iteración " + str(self.iteration),
                xaxis_title=value3,
                yaxis_title=value4,
                autosize=True,
                showlegend=False,
                hovermode="closest",
                margin=dict(b=0, l=0, r=0, t=30),
                xaxis=dict(showgrid=False, zeroline=False, showticklabels=True),
                yaxis=dict(showgrid=False, zeroline=False, showticklabels=True),
            ),
        )
        return options, options, fig1, fig2, options2, options2

    def display_info(self, data):
        """Display info on hover or click."""
        display = ""
        title = "Información del nodo"
        try:
            node_name = "Nodo"
            node = data["id"]
            node_name = data["name"]
            display = "Tipo de nodo: {}\n".format(data["node_type"])
            display += "Nombre del nodo: {}\n".format(data["id"])
            display += "Activación: {}\n".format(data["activation"])
            adj_nodes = [n for n in self.graph.adj[node].keys()]
            display += "Conectado con: \n"
            for node in adj_nodes:
                display += " " * 5 + self.graph.nodes()[node]["id"] + "\n"
                display += " " * 7 + "Tipo de nodo: {}\n".format(self.graph.nodes()[node]["node_type"])
                display += " " * 7 + "Activación: {}\n".format(self.graph.nodes()[node]["activation"])
            title = "Información de " + node_name
        except:
            pass
        return display, title

    def define_app(self):
        self.app.layout = html.Div([dcc.Location(id="url", refresh=False), html.Div(id="page-content")])
        self.app.callback(
            Output("nodes-dropdown1", "options"),
            Output("nodes-dropdown2", "options"),
            Output("graph-with-nodes1", "figure"),
            Output("graph-with-nodes2", "figure"),
            Output("perceptions1", "options"),
            Output("perceptions2", "options"),
            [
                Input("interval-nodes", "n_intervals"),
                Input("nodes-dropdown1", "value"),
                Input("nodes-dropdown2", "value"),
                Input("perceptions1", "value"),
                Input("perceptions2", "value"),
                Input("radio_type", "value"),
            ],
        )(self.update_fig_nodes)
        self.app.callback(Output("cytoscape-ltm", "elements"), [Input("interval-ltm", "n_intervals")])(
            self.update_fig_ltm
        )
        self.app.callback(
            Output("pause", "children"), Output("interval-ltm", "interval"), [Input("pause", "n_clicks")]
        )(self.update_pause)
        self.app.callback(Output("cytoscape-ltm", "generateImage"), [Input("save", "n_clicks")])(self.save_graph)
        self.app.callback(
            Output("click-data", "children"), Output("click_mk", "children"), [Input("cytoscape-ltm", "tapNodeData")]
        )(self.display_info)
        self.app.callback(
            Output("hover-data", "children"),
            Output("hover_mk", "children"),
            [Input("cytoscape-ltm", "mouseoverNodeData")],
        )(self.display_info)
        self.app.callback(Output("page-content", "children"), [Input("url", "pathname")])(self.display_page)

    def update_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.control_publisher.publish(command="pause", world="", reward=False)
            rospy.loginfo("Se ha pausado el proceso")
            return "Reanudar", 1 * 1000 * 60 * 60
        rospy.loginfo("Se ha reanudado el proceso")
        self.control_publisher.publish(command="continue", world="", reward=False)
        return "Pausar", 1000

    def save_graph(self, n_clicks2):
        ftype = "png"
        filename = "ltm_" + str(n_clicks2)
        action = "download"
        nx.write_edgelist(self.graph, self.save_dir + "graph_{}.txt".format(n_clicks2))
        nx.write_gexf(self.graph, self.save_dir + "graph_{}.gexf".format(n_clicks2))
        rospy.loginfo("Se ha guardado el archivo graph_{}.txt".format(n_clicks2))
        if n_clicks2 is not None:
            return {"type": ftype, "action": action, "filename": filename}
        return {}

    def display_page(self, pathname):
        if pathname == "/ltm":
            return self.ltm_layout
        if pathname == "/nodes":
            return self.node_layout
        return self.index_page

    ######ROS NODE CONFIGURATION FUCNTIONS#####
    # These functions are responsible of setting up the ROS node
    # for visualization

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        node_module = __import__(module_string, fromlist=[class_string])
        node_class = getattr(node_module, class_string)
        return node_class

    def setup_control_channel(self, control_channel):
        """Load simulator / robot configuration channel."""
        topic = rospy.get_param(control_channel["control_prefix"] + "_topic")
        self.control_message = self.class_from_classname(rospy.get_param(control_channel["control_prefix"] + "_msg"))
        self.control_publisher = rospy.Publisher(topic, self.control_message, latch=True, queue_size=0)
        topic = rospy.get_param(control_channel["info_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control_channel["info_prefix"] + "_msg"))
        rospy.Subscriber(topic, message, callback=self.info_callback)

    def setup_topics(self, connectors):
        """Load topic configuration for adding nodes."""
        for connector in connectors:
            self.default_class[connector["data"]] = connector.get("default_class")
            self.default_ros_node_prefix[connector["data"]] = connector.get("ros_node_prefix")
            self.default_ros_data_prefix[connector["data"]] = connector.get("ros_data_prefix")
            if self.default_ros_node_prefix[connector["data"]] is not None:
                topic = rospy.get_param(connector["ros_node_prefix"] + "_topic")
                message = self.class_from_classname(rospy.get_param(connector["ros_node_prefix"] + "_msg"))
                callback = getattr(self, connector["callback"])
                callback_args = connector["data"]
                rospy.logdebug("Subscribing to %s...", topic)
                rospy.Subscriber(topic, message, callback=callback, callback_args=callback_args)
        topic = "/mdb/ltm/p_node_update"
        message = self.class_from_classname(rospy.get_param("/mdb/p_node_update_msg"))
        callback = self.add_point_callback
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
                rospy.init_node("mdb_view", log_level=getattr(rospy, log_level))
                rospy.loginfo("Loading configuration from %s...", file_name)
                configuration = yaml.load(open(file_name, "r"), Loader=yamlloader.ordereddict.CLoader)
                self.setup_topics(configuration["LTM"]["Connectors"])
                self.setup_control_channel(configuration["Control"])
                rospy.on_shutdown(self.shutdown)

    def run(self, seed=None, log_level="INFO", save_dir=None):
        """Start the visualization part."""
        try:
            self.define_app()
            self.setup(log_level, seed)
            self.save_dir = save_dir
            self.ask_for_ltm()
            self.app.run_server(use_reloader=False, debug=False)
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")

    def shutdown(self):
        rospy.loginfo("Closing port 8050")
        os.system("kill -9 $( lsof -i:8050 -t )")

    ######ROS NODE CALLBACK FUCNTIONS#####
    """These functions are executed when a new message is
    published in a topic to which the node is subscribed 
    and for sending commands to the control topic"""

    def graph_set_edge_properties(self, node, node_type, neighbor_ids, neighbor_types):
        for neighbor, neighbor_node_type in zip(neighbor_ids, neighbor_types):
            if not self.graph.has_edge(node, neighbor):
                if neighbor_node_type == "CNode" or node_type == "CNode":
                    width = 1
                    if neighbor_node_type == "PNode" or node_type == "PNode":
                        color = "purple"
                    elif neighbor_node_type == "Goal" or node_type == "Goal":
                        color = "green"
                    elif neighbor_node_type == "ForwardModel" or node_type == "ForwardModel":
                        color = "blue"
                    elif neighbor_node_type == "Policy" or node_type == "Policy":
                        color = "red"
                else:
                    color = "black"
                    width = 0.2
                self.graph.add_edge(node, neighbor, color=color, width=width)
                # rospy.loginfo('Adding connection between {} and {}'.format(node, neighbor))

    def add_node_callback(self, data, node_type):
        ident = data.id
        if data.command == "new":
            idx = self.nodes_config[node_type]
            if node_type == "PNode":
                posx = idx // 3
                posy = idx % 3
                posx = 1 + posx + posy / 3.0
                color = "purple"
            elif node_type == "Goal":
                posx, posy = 3 * idx, 4
                color = "green"
            elif node_type == "ForwardModel":
                posx, posy = 3 * idx, 6
                color = "orange"
            elif node_type == "Policy":
                posx, posy = 13, idx + 3
                color = "red"
            elif node_type == "CNode":
                posx = idx // 3
                posy = idx % 3
                posx, posy = 1 + posx + posy / 3.0, posy + 8
                color = "blue"
            elif node_type == "Perception":
                posx, posy = -3, idx + 3
                color = "grey"
            self.graph.add_node(
                ident,
                id=ident,
                position={"x": posx * 75, "y": posy * 75},
                node_type=node_type,
                color=color,
                activation=data.activation,
            )
            self.nodes_config[node_type] += 1
            self.graph_set_edge_properties(ident, node_type, data.neighbor_ids, data.neighbor_types)
            rospy.logdebug("MENSAJE: Creating node {}".format(ident))
        elif data.command == "update":
            if node_type == "PNode":
                columns = data.names
                columns.append("confidence")
                self.sens_names = columns
                if not ident in self.pnode_dict:
                    self.pnode_dict[ident] = pd.DataFrame(columns=columns)
                else:
                    self.pnode_dict[ident].columns = columns
            self.graph.nodes()[ident]["activation"] = data.activation
            self.graph_set_edge_properties(ident, node_type, data.neighbor_ids, data.neighbor_types)
            rospy.logdebug("MENSAJE: " + node_type + " activation for " + ident + " = " + str(data.activation))

    def add_point_callback(self, data):
        new_data = list(data.point)
        new_data.append(data.confidence)
        if not data.id in self.pnode_dict:
            self.pnode_dict[data.id] = pd.DataFrame(columns=[""] * len(new_data))
        rospy.loginfo(new_data)
        self.pnode_dict[data.id].loc[len(self.pnode_dict[data.id])] = new_data
        rospy.logdebug("MENSAJE: Updating points of " + data.id)

    def info_callback(self, data):
        # rospy.loginfo('Updating information from LTM')
        self.iteration = data.iteration
        self.current_world = data.current_world
        self.current_policy = data.current_policy
        self.current_reward = data.current_reward
        self.current_goal = data.current_goal
        rospy.logdebug("MENSAJE: Updated info of iteration: {}".format(self.iteration))
        self.set_title()
        rospy.loginfo("*** ITERATION: " + str(self.iteration) + " ***")

    def ask_for_ltm(self):
        rospy.logdebug("Asking for current nodes in LTM")
        self.control_publisher.publish(command="publish_ltm", world="", reward=False)
