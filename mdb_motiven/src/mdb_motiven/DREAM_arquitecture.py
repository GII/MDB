"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Library imports
import numpy as np
from matplotlib import pyplot as plt
import pickle


class EVOLUTION(object):
    def calc_r(v):
        r = 0
        for i in range(len(v)):
            r = r + v[i]
        return r

    v = [0.5, 0.5, 0.5, 0.5]
    r = calc_r(v)
    for i in range(500):
        v2 = v[:]
        indice = np.random.choice([0, 1, 2, 3])
        v2[indice] = np.random.uniform(low=0.0, high=1.01)
        r2 = calc_r(v2)
        if r2 < r:
            r = r2
            v = v2[:]


class DRIVE(object):
    def __init__(self, name, coefficient):
        self.name = name
        self.value = 0  # Percentage of the need that is satisfied
        self.c_drive = coefficient
        # self.c_energy = 1.0
        # self.c_task = 0.85
        # self.c_human = 0.95

    def update_value(self, sensor_value):
        if self.name == "energy":
            light = sensor_value  # Sensor light_value
            # self.value = light / 10
            # self.value = 100 - (light / 10)
            # if self.value > 80:
            #     self.value = 100
            # self.value = 100*0.8**(light/10.0)  PAPER ICAE
            self.value = self.c_drive * (100 * 0.8 ** (light / 10.0))
        elif self.name == "human_feedback":
            human_presence = sensor_value[0]  # sensores human presence y iterations_without_human
            # it_without_human = sensor_value[1]
            satisfied_human = sensor_value[1]
            # self.value = max(human_presence * 100, 100 - 2.5 * it_without_human)
            # self.value = min(100 - human_presence * 100, 2.5 * it_without_human)
            if human_presence:
                # self.value = 0.95*(100 - satisfied_human*100) PAPER ICAE
                self.value = self.c_drive * (100 - satisfied_human * 100)
            else:
                self.value = 0
        # elif self.name == 'human_close':
        #     human_presence = sensor_value  # sensores human presence y iterations_without_human
        #     self.value = self.c_drive*(100 - human_presence*100)
        elif self.name == "pleasure":
            # new_piece_in_area = sensor_value[0] # Sensor assembly task step
            # piece1_assembled = sensor_value[1]
            # piece2_assembled = sensor_value[2]
            new_assembly_piece = 1 if sensor_value == 6 else 0  # Sensor assembly task step
            # piece1_assembled = 1 if sensor_value >= 3 else 0
            # piece2_assembled = 1 if sensor_value >= 5 else 0
            # self.value = min(100, new_piece_in_area * 100 + 25 * piece1_assembled + 25 * piece2_assembled)
            # basket_full = sensor_value[1]
            # self.value = (100 - (new_piece_in_area or basket_full) * 100)  # Montaje o # Fill basket
            # self.value = 0.85*(100 - (new_assembly_piece * 99.9)) PAPER ICAE
            self.value = self.c_drive * (100 - (new_assembly_piece * 99.9))
        elif self.name == "novelty":
            self.value = 0.9 * (100 - sensor_value * 100)

    def goal_achieved(self):
        self.value = 100


class SIMULATOR(object):
    def __init__(self):
        # Light
        # self.light_state = 0 # 1-ON, 0-OFF
        self.light_value = 1000  # 0-1000 lux
        self.iterations_light_on = 0
        self.switch_close = True  # PAra saber si el robot puede pulsar el boton o debe pedir ayuda a otro humano

        # Human
        self.human = False  # if True the human is in the scenario
        self.human_id = 0
        self.iterations_without_human = 0
        self.iterations_for_human_to_appear = np.random.randint(
            75, 125
        )  # np.random.randint(100, 150)np.random.randint(25, 75)
        self.human_basket_full = False
        self.pieces_in_human_basket = 0
        self.max_pieces_in_human_basket = 4

        # Assembly task
        self.n_assembled_pieces = 0  # Number of assembled pieces
        self.assembly_task_step = 1  # 1-Pick_obj1 2-Place obj1 in mounting area
        # 3-Pick_obj2 4-Place obj2 in mounting area (on top of object 1)
        # 5-Pick assembled obj 6-Place assembled obj in assembled objects area
        # No usado: serviria para limitarle el numero de piezas que puede montar
        self.n_pieces_to_assembly = True  # Para saber si hay piezas disponibles para montar o no

        # Si es falso, lo que hago es que el robot llama al humano y viene y le deja nuevas piezas para montar. Puedo contar que cada vez que viene el humano con la caja a recoger que siempre le deja piezas pero que puede que se quede sin ellas.

        # Verify task
        self.verified_pieces = 0
        self.total_verified_pieces = 0
        self.finish_verify_task = False

        self.reward = [False, False, False, False, False, False, False, False]  # To indicate when a goal is achieved
        # [Light_switch, Assembly_task, light_human, basket_full,  , verify, novel_state_achieved, effectance]

    def turn_on_light(self):
        self.light_value = 1000
        self.iterations_light_on = 0
        self.reward[0] = True

    def turn_on_light_by_human(self):
        self.light_value = 1000
        self.iterations_light_on = 0
        self.reward[2] = True

    def update_light_state(self):
        self.iterations_light_on += 1
        # if self.iterations_light_on > 20:
        self.light_value = max(
            0, 1000 - 20.0 * (self.iterations_light_on)
        )  # max(0, self.light_value - 0.95 * self.iterations_light_on) #
        # else:
        #     self.light_value = 1000

    def update_human_state(self):
        if not self.human:
            if (
                self.iterations_for_human_to_appear == self.iterations_without_human
            ):  # Asi el humano puede aparecer tambien sin ser llamado
                self.human = True
                self.human_id = np.random.choice([1, 2], p=[0.0, 1.0])  # p=[0.5, 0.5])
                self.iterations_without_human = 0
                self.iterations_for_human_to_appear = np.random.randint(
                    50, 100
                )  # np.random.randint(100, 150)np.random.randint(25, 80)
            else:
                self.iterations_without_human += 1
        else:
            # Condicion para que se marche
            if self.human_basket_full:  # Si la cesta esta llena o si no hay mas piezas que meter en la cesta
                self.human = False
                self.pieces_in_human_basket = 0  # Para la proxima vez volver a empezar de cero
                self.human_basket_full = False
            if self.finish_verify_task:
                self.human = False
                self.verified_pieces = 0
                self.finish_verify_task = False

    def update_human_basket(self):
        # if self.pieces_in_human_basket == self.max_pieces_in_human_basket or self.n_assembled_pieces == 0:  # No quedan mas piezas que meter en la cesta
        #     self.human_basket_full = True
        #     self.reward[1] = True
        #     # self.pieces_in_human_basket = 0  # Para la proxima vez volver a empezar de cero
        # else:
        #     # Cada pieza que meto en la cesta es una pieza menos para recoger
        #     self.pieces_in_human_basket += 1
        #     self.n_assembled_pieces -= 1
        #     self.reward[1] = False

        # Cada pieza que meto en la cesta es una pieza menos para recoger
        self.pieces_in_human_basket += 1
        self.n_assembled_pieces -= 1
        if (
            self.pieces_in_human_basket == self.max_pieces_in_human_basket
        ):  # or self.n_assembled_pieces == 0:  # No quedan mas piezas que meter en la cesta
            self.human_basket_full = True
            self.reward[3] = True
            # self.pieces_in_human_basket = 0  # Para la proxima vez volver a empezar de cero
        else:
            self.reward[3] = False

    def update_verify_task(self):
        # if self.n_assembled_pieces > 0:
        if self.n_assembled_pieces - self.total_verified_pieces > 0:  # Hai piezas disponibles para verificar
            self.verified_pieces += 1
            self.total_verified_pieces += 1
            # if self.verified_pieces == self.n_assembled_pieces:
            # if self.verified_pieces == self.n_assembled_pieces-self.total_verified_pieces:
            if self.n_assembled_pieces - self.total_verified_pieces == 0:
                self.finish_verify_task = True
                self.reward[5] = True
            else:
                self.reward[5] = False
        else:
            self.reward[5] = False

    def update_assembly_task(self):
        if self.assembly_task_step == 5:  # 6:
            self.assembly_task_step += 1  # Indico que la tarea no esta en proceso
            self.n_assembled_pieces += 1  # tengo una pieza nueva montada
            self.reward[1] = True
        else:
            if self.assembly_task_step == 6:  # 7:
                self.assembly_task_step = 1
            self.assembly_task_step += 1  # Paso al siguiente paso de montaje
            self.reward[1] = False

    def get_perception(self):
        return (
            self.light_value,
            self.human,
            self.human_id,
            self.human_basket_full,
            self.assembly_task_step,
            self.n_pieces_to_assembly,
        )

    def get_reward(self):
        return self.reward

    def world_rules(self, executed_policy, active_goal):
        # Presencia humano
        if (
            active_goal == "fill_basket" and executed_policy == "put_object_in" and self.light_value > 20
        ):  # Estamos poniendo piezas en cesta humano
            self.update_human_basket()
        else:
            self.reward[3] = False

        if (
            active_goal == "verify_piece" and executed_policy == "show_object" and self.light_value > 20
        ):  # Estamos ensenandole las piezas
            self.update_verify_task()
        else:
            self.reward[5] = False

        # if active_goal == 'call_human' and executed_policy == 'press_object':
        #     self.human = True
        #     self.iterations_without_human = 0
        #     self.reward[2] = True
        # else:
        #     self.reward[2] = False
        if active_goal == "call_human" and executed_policy == "ask_nicely":
            self.turn_on_light_by_human()
        else:
            self.reward[2] = False
        self.update_human_state()

        # Piezas para montaje/jugar
        if (
            active_goal == "assemble_piece"
            and (
                executed_policy == "grasp_object"
                or executed_policy == "put_object_in"
                or executed_policy == "sweep_object"
            )
            and self.light_value > 20
        ):
            self.update_assembly_task()
        else:
            if self.assembly_task_step == 6:
                self.assembly_task_step = 1
            self.reward[1] = False

        # Tiempo luz encendida
        if active_goal == "turn_on_light" and executed_policy == "press_object":
            self.turn_on_light()
        else:
            self.reward[0] = False
        self.update_light_state()

        # Novelty
        if active_goal == "novel_state_achieved" and executed_policy == "ask_nicely":
            self.n_pieces_to_assembly = True
            self.reward[6] = True
        else:
            self.reward[6] = False


class DREAM(object):
    def __init__(self, c_energy, c_task, c_human_feedback):  # , c_human_close):

        # Import seed
        # f = open('seed.pckl', 'rb')
        # seed = pickle.load(f)
        # f.close()
        # np.random.set_state(seed)

        self.drives = {
            "Novelty": DRIVE("novelty", 1.0),
            "Effectance": DRIVE("effectance", 1.0),
            "Energy": DRIVE("energy", c_energy),
            "Human Feedback": DRIVE("human_feedback", c_human_feedback),
            # 'Human Close': DRIVE('human_close', c_human_close),
            "Pleasure": DRIVE("pleasure", c_task),  # Task dependant
        }
        self.goals = [
            "turn_on_light",
            "assemble_piece",  # tiene subgoals asociados: agarrar_obj1, montar_obj1, agarrar_obj2, montar_obj2,
            #'play_with_pieces',  # Juega con las piezas mediante sweep porque asi hacen ruido y le mola
            "fill_basket",
            "verify_piece",
            "call_human",
            "novel_state_achieved",
            "unmodelled_state_achieved",
        ]
        self.policies = [
            "grasp_object",  # Use a gripper to grasp an object.
            "ask_nicely",  # Ask experimenter to bring something to within reach.
            "put_object_in",  # Place an object in a receptacle.
            "sweep_object",  # Sweep an object to the central line of the table.
            "show_object",  # Show the object to a human # Grab the object and show it
            # 'look_for_human', # Look for the presence of a human to know his position
            "press_object",  # Use a gripper to press an object/ a clickable object # Otro nombre: 'press_button'
        ]
        self.sensors = {
            "light": 0,
            "human": False,  # detects the presence of a human
            "human_id": 0,  # when there are several humans it indicates which one it is
            # 'assembled_piece_in_box': False,
            # 'part1_assembled': False, # No necesario porque consiste en ponerlo justo delante del robot?
            # 'part2_assembled': False,
            "object_left_hand": False,
            "object_right_hand": False,
            # Sensores que indiquen distancia y angulo a piezas y area de montaje y recepcion.
            # Debe existir un mecanismo de atencion para solo centrarse en una pieza
            # posicion objetivo, ya sea objeto, humano, zona... para agarrar, soltar, etc
            "objective_distance": 0.0,
            "objective_angle": 0.0,
        }

        self.iterations = 0
        self.stop_condition = 500  # 445  # Iterations to finish
        self.simulator = SIMULATOR()
        self.perception = None
        self.active_goal = None
        self.active_policy = None
        self.graph = []
        self.file_name = "CognitiveDrives.txt"

    def run(self):

        # # Import seed
        # f = open('seed.pckl', 'rb')
        # seed = pickle.load(f)
        # f.close()
        # np.random.set_state(seed)

        # Save seed
        # seed = np.random.get_state()
        # f = open('seed.pckl', 'wb')
        # pickle.dump(seed, f)
        # f.close()

        iterations_policy = -1
        while self.iterations <= self.stop_condition:
            self.iterations += 1
            self.perception = self.update_perception()
            self.update_drive_values(self.perception)
            ### Movemos el pulsador a fuera del alcance
            # if self.iterations > 250:
            #     self.simulator.switch_close = False
            ### Dejamos al robot sin piezas para montar
            # if self.iterations > 380 and self.iterations < 385:
            #     self.simulator.n_pieces_to_assembly = False
            if iterations_policy < 0:
                self.active_goal = self.choose_active_goal()
                self.active_policy, iterations_policy = self.choose_action()
            if (
                iterations_policy > 0
            ):  # Mientras se esta ejecutando una policy solo se actualizan los valores del simulador
                self.simulator.update_human_state()
                self.simulator.update_light_state()
                iterations_policy -= 1
            else:
                self.apply_action()
                iterations_policy -= 1

            self.save_data()

        # self.plot_graph()
        # self.save_graph_matrix_to_txt(self.graph, self.file_name)

    def update_perception(self):
        return self.simulator.get_perception()

    def update_drive_values(self, perception):
        # Perception: (self.light, self.human, self.iterations_without_human, self.human_basket_full, self.assembly_task_step)
        for drive in self.drives:
            if drive == "Energy":
                self.drives[drive].update_value(perception[0])
            elif drive == "Human Feedback":
                self.drives[drive].update_value((perception[1], perception[3]))
            # elif drive == "Human Close":
            #     self.drives[drive].update_value(perception[1])
            elif drive == "Pleasure":
                self.drives[drive].update_value(perception[4])  # self.assembly_task_step
            elif drive == "Novelty":
                self.drives[drive].update_value(
                    perception[5]
                )  # Ad hoc: Le digo que no hay piezas disponibles, por lo que se busca un estado nuevo

    def choose_active_goal(self):
        drive_less_satisfied = "Human Feedback"
        for drive in self.drives:
            if self.drives[drive].value > self.drives[drive_less_satisfied].value:
                if (drive != "Human Feedback") or (
                    drive == "Human Feedback" and self.perception[1]
                ):  # Hay humano en escena
                    drive_less_satisfied = drive

        # if self.perception[1]:  # Human
        #     active_goal = 'fill_basket'
        # elif self.drives["Energy"].value > 75:
        #     active_goal = 'turn_on_light'
        # else:
        #     active_goal = 'assemble_piece'
        # if drive_less_satisfied == "Energy":
        #     active_goal = 'turn_on_light'
        # elif drive_less_satisfied == "Human Feedback":
        #     active_goal = 'call_human'
        # elif drive_less_satisfied == "Pleasure":
        #     if self.perception[1] and self.simulator.n_assembled_pieces > 0:  # Human y hay piezas
        #         active_goal = 'fill_basket'
        #     else:  # Si no hay humano o hay humano pero no hay piezas
        #         active_goal = 'assemble_piece'

        if drive_less_satisfied == "Human Feedback":  # Human in scenario
            if self.perception[2] == 1:  # Human_id = 1
                active_goal = "fill_basket"
            elif self.perception[2] == 2:  # Human_id = 2
                active_goal = "verify_piece"
        # elif drive_less_satisfied == "Human Close":
        #     active_goal = 'call_human'
        elif drive_less_satisfied == "Energy":
            if self.simulator.switch_close:
                active_goal = "turn_on_light"
            else:
                active_goal = "call_human"
        elif drive_less_satisfied == "Pleasure":
            active_goal = "assemble_piece"
        elif drive_less_satisfied == "Novelty":
            active_goal = "novel_state_achieved"

        return active_goal

    def choose_action(self):
        if self.active_goal == "fill_basket":
            if self.active_policy == "grasp_object":  # Si lo tenia cogido antes
                policy = "put_object_in"
                iterations_to_execute_policy = 2
            elif self.simulator.n_assembled_pieces > 0:
                policy = "grasp_object"
                iterations_to_execute_policy = 2
            else:
                policy = "ask_nicely"
                iterations_to_execute_policy = 1
        elif self.active_goal == "turn_on_light":
            policy = "press_object"
            iterations_to_execute_policy = 1
        elif self.active_goal == "call_human":
            policy = "ask_nicely"
            iterations_to_execute_policy = np.random.randint(2, 5)  # Puede que no haya humanos cerca y que tarden mucho
        elif self.active_goal == "assemble_piece":
            assembly_step = self.perception[4]
            if (
                assembly_step == 1 or assembly_step == 3 or assembly_step == 6
            ):  # or assembly_step == 5 or assembly_step == 7:  # self.assembly_task_step
                policy = "grasp_object"
                iterations_to_execute_policy = np.random.randint(2, 4)
            elif assembly_step == 2 or assembly_step == 4:  # or assembly_step == 6:
                policy = "put_object_in"
                iterations_to_execute_policy = np.random.randint(2, 4)
            elif assembly_step == 5:  # or assembly_step == 6:
                policy = "sweep_object"
                iterations_to_execute_policy = 2
        elif self.active_goal == "novel_state_achieved":
            if self.active_policy == "grasp_object":
                policy = "ask_nicely"
                iterations_to_execute_policy = np.random.randint(
                    4, 6
                )  # Lo que tarda el operario en ir a por las piezas y traerlas
            else:
                policy = "grasp_object"
                iterations_to_execute_policy = 2
        # elif self.active_goal == 'call_human':
        #     policy = 'press_object'
        elif self.active_goal == "verify_piece":
            if self.active_policy == "grasp_object":
                policy = "show_object"
                iterations_to_execute_policy = 3  # 1
            else:
                policy = "grasp_object"
                iterations_to_execute_policy = 2
        return policy, iterations_to_execute_policy  # 2

    def apply_action(self):
        self.simulator.world_rules(self.active_policy, self.active_goal)

    def save_data(self):

        drive_values = []
        for drive in self.drives:
            drive_values.append(self.drives[drive].value)
        rewards = []
        for i in range(len(self.simulator.get_reward())):
            rewards.append(self.simulator.get_reward()[i])
        self.graph.append(
            (self.iterations, self.active_goal, self.active_policy, self.perception, rewards, drive_values)
        )

    def save_graph_matrix_to_txt(self, matrix, file_name):
        f = open(file_name, "w")
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                f.write(str(matrix[i][j]) + " ")
            f.write("\n")
        f.close()

    # def read_graph_matrix_from_txt(self, file_name):
    #     graph=[]
    #     f = open(file_name, 'r')
    #     x=f.readlines()
    #     for i in range(len(x)):
    #         graph.append(map(float,x[i].strip().split(' ')))
    #     return graph

    def plot_graph(self):
        # Graph 1: Drives - Mecanismo Cognitivo
        energy = []
        human = []
        pleasure = []
        novelty = []
        effectance = []
        performance = (
            []
        )  # Drive que mide las piezas que hace el robot. 1 si pieza montada. 0 en otro caso. Es antagonico al de pleasure
        for i in range(300):
            # for i in range(len(self.graph)):
            energy.append(self.graph[i][5][1])
            human.append(self.graph[i][5][3])
            pleasure.append(self.graph[i][5][4])
            # novelty.append(self.graph[i][5][2])
            novelty.append(30)
            effectance.append(self.graph[i][5][0])
            if self.graph[i][4][1]:
                if not self.graph[i - 1][4][1]:  # 'assemble_piece'
                    performance.append(0)
                else:
                    performance.append(100)
            else:
                performance.append(100)
        plt.figure()
        plt.plot(list(range(1, len(energy) + 1)), energy, color="m", label="Energy")
        plt.plot(list(range(1, len(human) + 1)), human, color="orange", label="Human")
        plt.plot(list(range(1, len(pleasure) + 1)), pleasure, color="r", label="Task")
        plt.plot(list(range(1, len(performance) + 1)), performance, color="g", linestyle="--", label="Performance")
        plt.plot(list(range(1, len(novelty) + 1)), novelty, linestyle=":", label="Novelty")
        plt.plot(list(range(1, len(effectance) + 1)), effectance, color="cyan", linestyle=":", label="Effectance")
        indices_humano1 = []
        indices_humano2 = []
        for i in range(300):
            # for i in range(len(self.graph)):
            if (self.graph[i][3][1] > self.graph[i - 1][3][1]) or (
                self.graph[i][3][1] < self.graph[i - 1][3][1]
            ):  # MArco cuando llega el humano y cuando se va
                # Pintar las lineas de diferente color en funcion del humano que sea
                if self.graph[i][3][2] == 1:
                    plt.axvline(x=i, linewidth=1.5, color="grey", linestyle="--")
                    indices_humano1.append(i)
                else:
                    plt.axvline(x=i, linewidth=1.5, color="purple", linestyle="--")
                    # plt.axvline(x=i, linewidth=1.5, color='grey', linestyle='--')
                    indices_humano2.append(i)
        for i in range(0, len(indices_humano1), 2):
            plt.fill_betweenx([0, 100], indices_humano1[i], indices_humano1[i + 1], alpha=0.2, color="grey")
        for i in range(0, len(indices_humano2), 2):
            plt.fill_betweenx([0, 100], indices_humano2[i], indices_humano2[i + 1], alpha=0.2, color="yellow")
            # plt.fill_betweenx([0, 100], indices_humano2[i], indices_humano2[i+1], alpha=0.2, color='grey')
        # Marco ejecucion de polcies aleatoriasw
        # plt.fill_betweenx([0, 100], 381, 388, alpha=0.4, color='magenta')
        # plt.axvline(x=250, linewidth=2.0, color='black', linestyle='-')  # Momento en el que cambio el pulsador de lugar
        plt.axes().set_xlabel("time", size=11.0)
        plt.axes().set_ylabel("Drive value (norm.)", size=11.0)
        plt.yticks([0, 20, 40, 60, 80, 100], [0, 0.2, 0.4, 0.6, 0.8, 1.0], size=11.0)
        plt.xticks(size=11.0)
        # plt.legend(loc='upper left')
        plt.grid()

        # Genero la leyenda de la figura anterior
        plt.figure()
        plt.plot(1, energy[0], color="m", label="Energy")
        plt.plot(1, human[0], color="orange", label="Human")
        plt.plot(1, pleasure[0], color="r", label="Task")
        plt.plot(1, performance[0], linestyle="--", color="g", label="Performance")
        plt.plot(1, novelty[0], linestyle=":", label="Novelty")
        plt.plot(1, effectance[0], linestyle=":", color="cyan", label="Effectance")
        plt.legend(loc="upper left")

        # # Graph 2: Environment
        # plt.figure()
        # for i in range(1, len(self.graph)):
        #     if self.graph[i][4][6] and not self.graph[i-1][4][6]:  # novelty
        #         plt.plot(self.graph[i][0], 7, 'yx', markersize=5.0, label='novel_state_achieved')
        #     elif self.graph[i][4][7] and not self.graph[i - 1][4][7]:  # effectance
        #         plt.plot(self.graph[i][0], 6, 'kx', markersize=5.0, label='unmodelled_state_achieved')
        #     elif self.graph[i][4][2] and not self.graph[i-1][4][2]:  # 'play_with_pieces'
        #         plt.plot(self.graph[i][0], 5, 'cx', markersize=5.0, label='human_present')
        #     elif self.graph[i][4][0] and not self.graph[i - 1][4][0]:  # 'turn_on_light'
        #         plt.plot(self.graph[i][0], 4, 'bx', markersize=5.0, label='light_switch_pushed')
        #     # elif self.graph[i][4][3] and not self.graph[i-1][4][3]:  # 'fill_basket'
        #     #     plt.plot(self.graph[i][0], 3, 'gx', markersize=6.0, label='fill_basket')  # Deberia ser el de pieza construida en cesta. Una vez con cada pieza.
        #     # Marco goals intermedios human 1
        #     elif self.graph[i][2] != 'put_object_in' and  self.graph[i-1][2] == 'put_object_in' and self.graph[i-1][3][1] and self.graph[i-1][3][2]==1: # Hay human 1 presente
        #         plt.plot(self.graph[i-1][0], 3, 'gx', markersize=5.0, label='fill_basket')
        #     # elif self.graph[i][4][5] and not self.graph[i - 1][4][5]:  # 'verify_piece'
        #     #     plt.plot(self.graph[i][0], 2, 'mx', markersize=6.0, label='verify_pieces')  # Deberia ser el de Gripper_holding_piece_towards_human. Una vez con cada pieza mostrada
        #     # Marco goals intermedios human 2
        #     elif self.graph[i][2] != 'show_object' and  self.graph[i-1][2] == 'show_object':
        #         plt.plot(self.graph[i-1][0], 2, 'mx', markersize=5.0, label='verify_pieces')
        #     elif self.graph[i][4][1] and not self.graph[i - 1][4][1]:  # 'assemble_piece'
        #         plt.plot(self.graph[i][0], 1, 'rx', markersize=5.0, label='assembled_piece_in_storage_area')
        #
        #
        #
        # for i in range(len(indices_humano1)):
        #     plt.axvline(x=indices_humano1[i], linewidth=1.5, color='grey', linestyle='--')
        # for i in range(len(indices_humano2)):
        #     plt.axvline(x=indices_humano2[i], linewidth=1.5, color='purple', linestyle='--')
        # for i in range(0, len(indices_humano1), 2):
        #     plt.fill_betweenx([0, 7.5], indices_humano1[i], indices_humano1[i+1], alpha=0.2, color='grey')
        # for i in range(0, len(indices_humano2), 2):
        #     plt.fill_betweenx([0, 7.5], indices_humano2[i], indices_humano2[i+1], alpha=0.2, color='yellow')
        # # Marco ejecucion de polcies aleatoriasw
        # plt.fill_betweenx([0, 7.5], 381, 388, alpha=0.4, color='magenta')
        # plt.axvline(x=250, linewidth=2.0, color='black', linestyle='-')  # Momento en el que cambio el pulsador de lugar
        # plt.axes().set_xlabel('time', size=11.0)
        # plt.axes().set_ylabel('Goal achieved', size=11.0)
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7, 8], [' ', 'G7', 'G6', 'G5', 'G4', 'G3', 'G2', 'G1', ' '], size=11.0)
        # plt.xticks(size=11.0)
        # # plt.legend()
        # plt.grid()
        #
        # # Genero la leyenda de la figura anterior
        # plt.figure()
        # plt.plot(self.graph[i][0], 7, 'yx', markersize=6.0, label='novel_state_achieved')
        # plt.plot(self.graph[i][0], 6, 'kx', markersize=6.0, label='unmodelled_state_achieved')
        # plt.plot(self.graph[i][0], 5, 'cx', markersize=6.0, label='human_present')
        # plt.plot(self.graph[i][0], 4, 'bx', markersize=6.0, label='light_switch_pushed')
        # plt.plot(self.graph[i][0], 3, 'gx', markersize=6.0, label='assembled_piece_in_basket')
        # plt.plot(self.graph[i][0], 2, 'mx', markersize=6.0, label='gripper_holding_piece_towards_human')
        # plt.plot(self.graph[i][0], 1, 'rx', markersize=6.0, label='assembled_piece_in_storage_area')
        # plt.axes().set_xlabel('time')
        # plt.axes().set_ylabel('Goal achieved')
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7, 8], [' ', 'G7', 'G6', 'G5', 'G4', 'G3', 'G2', 'G1', ' '])
        # plt.legend()
        # # plt.grid()
        #
        # # Graph 3: Policies
        # policies_iterations = []
        # plt.figure()
        # for i in range(len(self.graph)):
        #     if self.graph[i][2] == 'grasp_object':
        #         plt.plot(self.graph[i][0], 1, 'b.', markersize=7.0, label='grasp_object')
        #         policies_iterations.append(1)
        #     elif self.graph[i][2] == 'ask_nicely':
        #         plt.plot(self.graph[i][0], 2, 'r.', markersize=7.0, label='ask_nicely')
        #         policies_iterations.append(2)
        #     elif self.graph[i][2] == 'put_object_in':
        #         plt.plot(self.graph[i][0], 3, 'c.', markersize=7.0, label='put_object_in')
        #         policies_iterations.append(3)
        #     elif self.graph[i][2] == 'sweep_object':  # 'fill_basket'
        #         plt.plot(self.graph[i][0], 4, 'g.', markersize=7.0, label='push_object')
        #         policies_iterations.append(4)
        #     elif self.graph[i][2] == 'show_object':  # 'verify_piece'
        #         plt.plot(self.graph[i][0], 5, 'y.', markersize=7.0, label='show_object')
        #         policies_iterations.append(5)
        #     elif self.graph[i][2] == 'press_object':
        #         plt.plot(self.graph[i][0], 6, 'm.', markersize=7.0, label='press_object')
        #         policies_iterations.append(6)
        #
        # for i in range(len(indices_humano1)):
        #     plt.axvline(x=indices_humano1[i], linewidth=1.5, color='grey', linestyle='--')
        # for i in range(len(indices_humano2)):
        #     plt.axvline(x=indices_humano2[i], linewidth=1.5, color='purple', linestyle='--')
        # for i in range(0, len(indices_humano1), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano1[i], indices_humano1[i+1], alpha=0.2, color='grey')
        # for i in range(0, len(indices_humano2), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano2[i], indices_humano2[i+1], alpha=0.2, color='yellow')
        # plt.axvline(x=250, linewidth=2.0, color='black', linestyle='-')  # Momento en el que cambio el pulsador de lugar
        # plt.axes().set_xlabel('time', size=110)
        # plt.axes().set_ylabel('Policy used', size=11.0)
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7], [' ', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', ' '], size=11.0)
        # plt.xticks(size=12.0)
        # # plt.legend()
        # plt.grid()
        #
        # plt.figure()
        # plt.plot(range(1, len(policies_iterations) + 1), policies_iterations, zorder=2, linewidth=0.5)
        # plt.scatter(range(1, len(policies_iterations) + 1), policies_iterations, zorder=1, marker='.')
        # for i in range(len(indices_humano1)):
        #     plt.axvline(x=indices_humano1[i], linewidth=1.5, color='grey', linestyle='--')
        # for i in range(len(indices_humano2)):
        #     plt.axvline(x=indices_humano2[i], linewidth=1.5, color='purple', linestyle='--')
        # for i in range(0, len(indices_humano1), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano1[i], indices_humano1[i+1], alpha=0.2, color='grey')
        # for i in range(0, len(indices_humano2), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano2[i], indices_humano2[i+1], alpha=0.2, color='yellow')
        # plt.axvline(x=250, linewidth=2.0, color='black', linestyle='-')  # Momento en el que cambio el pulsador de lugar
        # plt.axes().set_xlabel('time', size=11.0)
        # plt.axes().set_ylabel('Policy used', size=11.0)
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7], [' ', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', ' '], size=11.0)
        # plt.xticks(size=12.0)
        # plt.grid()
        #
        # # Combinacion de las dos anteriores en una misma figura
        # plt.figure()
        # plt.plot(range(1, len(policies_iterations) + 1), policies_iterations, zorder=2, linewidth=0.5, color='grey')
        # for i in range(len(self.graph)):
        #     if self.graph[i][2] == 'grasp_object':
        #         plt.plot(self.graph[i][0], 1, 'b.', markersize=5.0, label='grasp_object')
        #     elif self.graph[i][2] == 'ask_nicely':
        #         plt.plot(self.graph[i][0], 2, 'r.', markersize=5.0, label='ask_nicely')
        #     elif self.graph[i][2] == 'put_object_in':
        #         plt.plot(self.graph[i][0], 3, 'c.', markersize=5.0, label='put_object_in')
        #     elif self.graph[i][2] == 'sweep_object':  # 'fill_basket'
        #         plt.plot(self.graph[i][0], 4, 'g.', markersize=5.0, label='push_object')
        #     elif self.graph[i][2] == 'show_object':  # 'verify_piece'
        #         plt.plot(self.graph[i][0], 5, 'y.', markersize=5.0, label='show_object')
        #     elif self.graph[i][2] == 'press_object':
        #         plt.plot(self.graph[i][0], 6, 'm.', markersize=5.0, label='press_object')
        # # plt.plot(range(1, 400 + 1), policies_iterations, linewidth=0.5)
        # # plt.scatter(range(1, len(policies_iterations) + 1), policies_iterations, zorder=1, marker='.')
        # for i in range(len(indices_humano1)):
        #     plt.axvline(x=indices_humano1[i], linewidth=1.5, color='grey', linestyle='--')
        # for i in range(len(indices_humano2)):
        #     plt.axvline(x=indices_humano2[i], linewidth=1.5, color='purple', linestyle='--')
        # for i in range(0, len(indices_humano1), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano1[i], indices_humano1[i+1], alpha=0.2, color='grey')
        # for i in range(0, len(indices_humano2), 2):
        #     plt.fill_betweenx([0, 6.5], indices_humano2[i], indices_humano2[i+1], alpha=0.2, color='yellow')
        # # Marco ejecucion de polcies aleatoriasw
        # plt.fill_betweenx([0, 6.5], 381, 388, alpha=0.4, color='magenta')
        # plt.axvline(x=250, linewidth=2.0, color='black', linestyle='-')  # Momento en el que cambio el pulsador de lugar
        # plt.axes().set_xlabel('time', size=11.0)
        # plt.axes().set_ylabel('Policy used', size=11.0)
        # # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7], [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '], size=11.0)
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7], [' ', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', ' '], size=11.0)
        # plt.xticks(size=12.0)
        # plt.grid()
        #
        #
        # # Genero la leyenda de la figura anterior
        # plt.figure()
        # plt.plot(self.graph[i][0], 1, 'b.', markersize=7.0, label='P1: grasp_object')
        # plt.plot(self.graph[i][0], 2, 'r.', markersize=7.0, label='P2: ask_nicely')
        # plt.plot(self.graph[i][0], 3, 'c.', markersize=7.0, label='P3: put_object_in')
        # plt.plot(self.graph[i][0], 4, 'g.', markersize=7.0, label='P4: push_object')
        # plt.plot(self.graph[i][0], 5, 'y.', markersize=7.0, label='P5: show_object')
        # plt.plot(self.graph[i][0], 6, 'm.', markersize=7.0, label='P6: press_object')
        #
        # plt.axes().set_xlabel('Iterations')
        # plt.axes().set_ylabel('Policy used')
        # plt.yticks([0, 1, 2, 3, 4, 5, 6, 7], [' ', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', ' '])
        # plt.legend()
        # # plt.grid()


def main():
    # Import seed
    # Graficas coeficientes hechas con seed4
    f = open("seed4.pckl", "rb")
    seed = pickle.load(f)
    f.close()
    np.random.set_state(seed)

    # #Save seed
    # # seed = np.random.get_state()
    # # f = open('seed30.pckl', 'wb')
    # # pickle.dump(seed, f)
    # # f.close()
    #
    #
    # instance = DREAM()
    # instance.run()
    coef_1 = []
    coef_2 = []
    coef_3 = []
    coef_4 = []
    rewards = []
    coef_1p = []
    coef_2p = []
    coef_3p = []
    coef_4p = []
    rewardsp = []
    # v = [0.55, 0.5, 0.95]#, 0.0]
    v = [0.5, 0.5, 0.5]
    instance = DREAM(c_task=v[0], c_human_feedback=v[1], c_energy=v[2])  # , c_human_close=v[3])
    instance.run()
    r = instance.simulator.n_assembled_pieces
    # r = instance.simulator.total_verified_pieces
    coef_1.append(v[0])
    coef_2.append(v[1])
    coef_3.append(v[2])
    # coef_4.append(v[3])
    rewards.append(r)
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        v2[0] = np.random.uniform(low=0.0, high=1.01)
        v2[1] = np.random.uniform(low=0.0, high=1.01)
        v2[2] = np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        r2 = instance2.simulator.n_assembled_pieces
        # r2 = instance2.simulator.total_verified_pieces
        if r2 > r:
            r = r2
            v = v2[:]
        coef_1.append(v[0])
        coef_2.append(v[1])
        coef_3.append(v[2])
        # coef_4.append(v[3])
        rewards.append(r)
        coef_1p.append(v2[0])
        coef_2p.append(v2[1])
        coef_3p.append(v2[2])
        # coef_4p.append(v2[3])
        rewardsp.append(r2)
    # Cambio el objetivo:
    r = instance.simulator.total_verified_pieces
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        v2[0] = np.random.uniform(low=0.0, high=1.01)
        v2[1] = np.random.uniform(low=0.0, high=1.01)
        v2[2] = np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        # r2 = instance2.simulator.n_assembled_pieces
        r2 = instance2.simulator.total_verified_pieces
        if r2 > r:
            r = r2
            v = v2[:]
        coef_1.append(v[0])
        coef_2.append(v[1])
        coef_3.append(v[2])
        # coef_4.append(v[3])
        rewards.append(r)
        coef_1p.append(v2[0])
        coef_2p.append(v2[1])
        coef_3p.append(v2[2])
        # coef_4p.append(v2[3])
        rewardsp.append(r2)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(list(range(1, len(coef_1) + 1)), coef_1, "yo", markersize=4.0, label="c_task")  # Puntos
    ax.plot(list(range(1, len(coef_1) + 1)), coef_1, "y", markersize=0.5)  # Linea para unirlos
    ax.plot(list(range(1, len(coef_2) + 1)), coef_2, "mo", markersize=4.0, label="c_human_feedback")
    ax.plot(list(range(1, len(coef_2) + 1)), coef_2, "m", markersize=0.5)
    ax.plot(list(range(1, len(coef_3) + 1)), coef_3, "co", markersize=4.0, label="c_energy")
    ax.plot(list(range(1, len(coef_3) + 1)), coef_3, "c", markersize=0.5)
    # ax.plot(range(1, len(coef_4) + 1), coef_4, 'bp', markersize=6.0, label='c_human_close')
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    # ax.legend()
    ax.grid()
    # ax.set_title("Valores finales")
    ax2 = ax.twinx()
    ax2.plot(list(range(1, len(rewards) + 1)), rewards, "k", linewidth=2.5, label="valid pieces")
    ax2.set_ylabel("Number of valid pieces", size=11.0)
    # ax2.legend()
    plt.axvline(x=20, linewidth=2.0, color="grey", linestyle="--")  # Momento en el que cambio el objetivo

    # Genero leyenda figura anterior
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(1, coef_1[0], "yo", markersize=4.0, label="$c_{task}$")  # Puntos
    ax.plot(1, coef_2[0], "mo", markersize=4.0, label="$c_{human}$")
    ax.plot(1, coef_3[0], "co", markersize=4.0, label="$c_{energy}$")
    ax.plot(1, rewards[0], "r", linewidth=2.0, label="Valid pieces")
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    ax.legend(prop={"size": 11})

    # Coeficientes estaticos
    coef_1b = []
    coef_2b = []
    coef_3b = []
    coef_4b = []
    rewardsb = []
    coef_1pb = []
    coef_2pb = []
    coef_3pb = []
    coef_4pb = []
    rewardspb = []
    # v = [0.55, 0.5, 0.95]#, 0.0]
    v = [0.38, 0.55, 0.8]
    instance = DREAM(c_task=v[0], c_human_feedback=v[1], c_energy=v[2])  # , c_human_close=v[3])
    instance.run()
    r = instance.simulator.n_assembled_pieces
    # r = instance.simulator.total_verified_pieces
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        # v2[0] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[1] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[2] = 0.5#np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        r2 = instance2.simulator.n_assembled_pieces
        # r2 = instance2.simulator.total_verified_pieces
        if r2 > r:
            r = r2
            v = v2[:]
        coef_1b.append(v[0])
        coef_2b.append(v[1])
        coef_3b.append(v[2])
        # coef_4b.append(v[3])
        rewardsb.append(r)
        coef_1pb.append(v2[0])
        coef_2pb.append(v2[1])
        coef_3pb.append(v2[2])
        # coef_4pb.append(v2[3])
        rewardspb.append(r2)
    # Cambio el objetivo:
    r = instance.simulator.total_verified_pieces
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        # v2[0] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[1] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[2] = 0.5#np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        # r2 = instance2.simulator.n_assembled_pieces
        r2 = instance2.simulator.total_verified_pieces
        if r2 > r:
            r = r2
            v = v2[:]
        coef_1b.append(v[0])
        coef_2b.append(v[1])
        coef_3b.append(v[2])
        # coef_4b.append(v[3])
        rewardsb.append(r)
        coef_1pb.append(v2[0])
        coef_2pb.append(v2[1])
        coef_3pb.append(v2[2])
        # coef_4pb.append(v2[3])
        rewardspb.append(r2)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(list(range(1, len(coef_1b) + 1)), coef_1b, "yo", markersize=4.0, label="c_task")  # Puntos
    ax.plot(list(range(1, len(coef_1b) + 1)), coef_1b, "y", markersize=0.5)  # Linea para unirlos
    ax.plot(list(range(1, len(coef_2b) + 1)), coef_2b, "mo", markersize=4.0, label="c_human_feedback")
    ax.plot(list(range(1, len(coef_2b) + 1)), coef_2b, "m", markersize=0.5)
    ax.plot(list(range(1, len(coef_3b) + 1)), coef_3b, "co", markersize=4.0, label="c_energy")
    ax.plot(list(range(1, len(coef_3b) + 1)), coef_3b, "c", markersize=0.5)
    # ax.plot(range(1, len(coef_4) + 1), coef_4, 'bp', markersize=6.0, label='c_human_close')
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    # ax.legend()
    ax.grid()
    # ax.set_title("Valores finales")
    ax2 = ax.twinx()
    ax2.plot(list(range(1, len(rewardsb) + 1)), rewardsb, "k", linewidth=2.5, label="valid pieces")
    ax2.set_ylabel("Number of valid pieces", size=11.0)
    # ax2.legend()
    plt.axvline(x=20, linewidth=2.0, color="grey", linestyle="--")  # Momento en el que cambio el objetivo

    # Coeficientes estaticos 2
    coef_1c = []
    coef_2c = []
    coef_3c = []
    coef_4c = []
    rewardsc = []
    coef_1pc = []
    coef_2pc = []
    coef_3pc = []
    coef_4pc = []
    rewardspc = []
    # v = [0.55, 0.5, 0.95]#, 0.0]
    v = [0.5, 0.5, 0.5]
    instance = DREAM(c_task=v[0], c_human_feedback=v[1], c_energy=v[2])  # , c_human_close=v[3])
    instance.run()
    r = instance.simulator.n_assembled_pieces
    # r = instance.simulator.total_verified_pieces
    coef_1c.append(v[0])
    coef_2c.append(v[1])
    coef_3c.append(v[2])
    # coef_4c.append(v[3])
    rewardsc.append(r)
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        # v2[0] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[1] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[2] = 0.5#np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        r2 = instance2.simulator.n_assembled_pieces
        # r2 = instance2.simulator.total_verified_pieces
        # if r2 > r:
        #     r = r2
        #     v = v2[:]
        coef_1c.append(v[0])
        coef_2c.append(v[1])
        coef_3c.append(v[2])
        # coef_4c.append(v[3])
        rewardsc.append(r)
        coef_1pc.append(v2[0])
        coef_2pc.append(v2[1])
        coef_3pc.append(v2[2])
        # coef_4pc.append(v2[3])
        rewardspc.append(r2)
    # Cambio el objetivo:
    r = instance.simulator.total_verified_pieces
    for i in range(20):
        v2 = v[:]
        # indice = np.random.choice([0, 1, 2])#, 3])
        # v2[indice] = np.random.uniform(low=0.0, high=1.01)
        # v2[0] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[1] = 0.5#np.random.uniform(low=0.0, high=1.01)
        # v2[2] = 0.5#np.random.uniform(low=0.0, high=1.01)
        instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
        instance2.run()
        # r2 = instance2.simulator.n_assembled_pieces
        r2 = instance2.simulator.total_verified_pieces
        if r2 > r:
            r = r2
            v = v2[:]
        coef_1c.append(v[0])
        coef_2c.append(v[1])
        coef_3c.append(v[2])
        # coef_4c.append(v[3])
        rewardsc.append(r)
        coef_1pc.append(v2[0])
        coef_2pc.append(v2[1])
        coef_3pc.append(v2[2])
        # coef_4pc.append(v2[3])
        rewardspc.append(r2)

    rewardsc_mean = []
    rewardspc_mean = []
    for i in range(len(coef_1c) // 2 + 1):
        rewardsc_mean.append(np.array(rewardsc[: len(rewardsc) // 2 + 1]).mean())
        rewardspc_mean.append(np.array(rewardspc[: len(rewardspc) // 2 + 1]).mean())
    for i in range(len(coef_1c) // 2 + 1, len(coef_1c)):
        rewardsc_mean.append(np.array(rewardsc[len(rewardsc) // 2 + 1 :]).mean())
        rewardspc_mean.append(np.array(rewardspc[len(rewardspc) // 2 + 1 :]).mean())

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(list(range(1, len(coef_1c) + 1)), coef_1c, "yo", markersize=4.0, label="c_task")  # Puntos
    ax.plot(list(range(1, len(coef_1c) + 1)), coef_1c, "y", markersize=0.5)  # Linea para unirlos
    ax.plot(list(range(1, len(coef_2c) + 1)), coef_2c, "mo", markersize=4.0, label="c_human_feedback")
    ax.plot(list(range(1, len(coef_2c) + 1)), coef_2c, "m", markersize=0.5)
    ax.plot(list(range(1, len(coef_3c) + 1)), coef_3c, "co", markersize=4.0, label="c_energy")
    ax.plot(list(range(1, len(coef_3c) + 1)), coef_3c, "c", markersize=0.5)
    # ax.plot(range(1, len(coef_4) + 1), coef_4, 'bp', markersize=6.0, label='c_human_close')
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    # ax.legend()
    ax.grid()
    # ax.set_title("Valores finales")
    ax2 = ax.twinx()
    ax2.plot(list(range(1, len(rewardsc) + 1)), rewardsc, "k", linewidth=2.5, label="valid pieces")
    ax2.plot(list(range(1, len(rewardspc) + 1)), rewardspc, "red", linewidth=2.5, label="valid pieces")
    ax2.plot(
        list(range(1, len(rewardsc_mean) + 1)), rewardsc_mean, "k", linestyle=":", linewidth=2.5, label="valid pieces"
    )
    ax2.plot(
        list(range(1, len(rewardspc_mean) + 1)),
        rewardspc_mean,
        "red",
        linestyle=":",
        linewidth=2.5,
        label="valid pieces",
    )
    ax2.set_ylabel("Number of valid pieces", size=11.0)
    # ax2.legend()
    plt.axvline(x=20, linewidth=2.0, color="grey", linestyle="--")  # Momento en el que cambio el objetivo

    ## FINAL
    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(list(range(0, len(coef_1))), coef_1, "yo", markersize=4.0, label="c_task")  # Puntos
    ax.plot(list(range(0, len(coef_1))), coef_1, "y", linewidth=1.0)  # Linea para unirlos
    ax.plot(list(range(0, len(coef_2))), coef_2, "mo", markersize=4.0, label="c_human_feedback")
    ax.plot(list(range(0, len(coef_2))), coef_2, "m", linewidth=1.0)
    ax.plot(list(range(0, len(coef_3))), coef_3, "co", markersize=4.0, label="c_energy")
    ax.plot(list(range(0, len(coef_3))), coef_3, "c", linewidth=1.0)
    # ax.plot(range(1, len(coef_4) + 1), coef_4, 'bp', markersize=6.0, label='c_human_close')
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    # ax.legend()
    ax.grid(alpha=0.2)
    # ax.set_title("Valores finales")
    ax2 = ax.twinx()
    ax2.plot(list(range(0, len(rewards))), rewards, "k", linewidth=2.5, label="valid pieces")
    ax2.plot(list(range(0, len(rewardspc_mean))), rewardspc_mean, "yellowgreen", linewidth=2.5, label="valid pieces")
    ax2.set_ylabel("Number of valid pieces", size=11.0)
    # ax2.legend()
    # plt.axvline(x=20, linewidth=2.0, color='grey', linestyle='--')  # Momento en el que cambio el objetivo
    plt.axvline(x=20, ymin=0.0, ymax=0.02, linewidth=2.0, color="k")
    plt.axvline(x=20, ymin=0.98, ymax=1.1, linewidth=2.0, color="k")

    an2 = ax.annotate(
        "Task change",
        xy=(0.5, 1.0),
        xycoords=ax,
        xytext=(0.5, 1.08),
        textcoords=(ax, "axes fraction"),
        va="bottom",
        ha="center",
        bbox=dict(boxstyle="round", fc="w"),
        arrowprops=dict(arrowstyle="->"),
    )

    plt.savefig("coefs_40executions_dpi_new.png", dpi=200)

    # Genero leyenda figura anterior
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(1, coef_1[0], "yo", markersize=4.0, label="$c_{task}$")  # Puntos
    ax.plot(1, coef_2[0], "mo", markersize=4.0, label="$c_{human}$")
    ax.plot(1, coef_3[0], "co", markersize=4.0, label="$c_{energy}$")
    ax.plot(1, rewards[0], "k", linewidth=2.0, label="Valid pieces")
    ax.plot(1, rewards[0], "yellowgreen", linewidth=2.0, label="Valid pieces baseline")
    ax.set_xlabel("Execution", size=11.0)
    ax.set_ylabel("Coefficient value", size=11.0)
    ax.legend(prop={"size": 11})
    plt.savefig("legend_40execution_dpi_new.png", dpi=200)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(range(1, len(coef_1p) + 1), coef_1p, 'yo', markersize=6.0, label='c_task')
    # ax.plot(range(1, len(coef_2p) + 1), coef_2p, 'kx', markersize=6.0, label='c_human')
    # ax.plot(range(1, len(coef_3p) + 1), coef_3p, 'c+', markersize=6.0, label='c_energy')
    # # ax.plot(range(1, len(coef_4p) + 1), coef_4p, 'bp', markersize=6.0, label='c_human_close')
    # ax.set_xlabel('iteration')
    # ax.set_ylabel('coefficient value')
    # ax.legend()
    # ax.grid()
    # ax.set_title("Valores probados")
    # ax2 = ax.twinx()
    # ax2.plot(range(1, len(rewardsp) + 1), rewardsp, 'r', markersize=6.0, label='valid pieces')
    # ax2.set_ylabel('number of verified pieces')
    # plt.axvline(x=10, linewidth=2.0, color='grey', linestyle='--')  # Momento en el que cambio el objetivo

    # 30 ejecuciones
    # datos = open('datos.txt', 'w')
    # for i in range(1, 31):
    #     f = open('seed'+str(i)+'.pckl', 'rb')
    #     seed = pickle.load(f)
    #     f.close()
    #     np.random.set_state(seed)
    #     coef_1 = []
    #     coef_2 = []
    #     coef_3 = []
    #     rewards = []
    #     v = [0.5, 0.5, 0.5]
    #     instance = DREAM(c_task=v[0], c_human_feedback=v[1], c_energy=v[2])  # , c_human_close=v[3])
    #     instance.run()
    #     r = instance.simulator.n_assembled_pieces
    #     # r = instance.simulator.total_verified_pieces
    #     for i in range(20):
    #         v2 = v[:]
    #         v2[0] = np.random.uniform(low=0.0, high=1.01)
    #         v2[1] = np.random.uniform(low=0.0, high=1.01)
    #         v2[2] = np.random.uniform(low=0.0, high=1.01)
    #         instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
    #         instance2.run()
    #         r2 = instance2.simulator.n_assembled_pieces
    #         if r2 > r:
    #             r = r2
    #             v = v2[:]
    #         coef_1.append(v[0])
    #         coef_2.append(v[1])
    #         coef_3.append(v[2])
    #         rewards.append(r)
    #     # Cambio el objetivo:
    #     r = instance.simulator.total_verified_pieces
    #     for i in range(20):
    #         v2 = v[:]
    #         v2[0] = np.random.uniform(low=0.0, high=1.01)
    #         v2[1] = np.random.uniform(low=0.0, high=1.01)
    #         v2[2] = np.random.uniform(low=0.0, high=1.01)
    #         instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
    #         instance2.run()
    #         r2 = instance2.simulator.total_verified_pieces
    #         if r2 > r:
    #             r = r2
    #             v = v2[:]
    #         coef_1.append(v[0])
    #         coef_2.append(v[1])
    #         coef_3.append(v[2])
    #         rewards.append(r)
    #     datos.write(("%f %f %f %f %f %f %f %f \n" % (coef_1[19], coef_2[19], coef_3[19], rewards[19], coef_1[39], coef_2[39], coef_3[39], rewards[39])))

    # Plot datos 30 ejecuciones
    archivo = open("datos.txt", "r")
    datos = archivo.readlines()
    coef1_m = []
    coef2_m = []
    coef3_m = []
    rewards_m = []
    coef1_f = []
    coef2_f = []
    coef3_f = []
    rewards_f = []
    for i in range(len(datos)):
        aux = datos[i].strip().split(" ")
        coef1_m.append(float(aux[0]))
        coef2_m.append(float(aux[1]))
        coef3_m.append(float(aux[2]))
        rewards_m.append(float(aux[3]))
        coef1_f.append(float(aux[4]))
        coef2_f.append(float(aux[5]))
        coef3_f.append(float(aux[6]))
        rewards_f.append(float(aux[7]))

    coef1_m_mean = []
    coef2_m_mean = []
    coef3_m_mean = []
    rewards_m_mean = []
    coef1_f_mean = []
    coef2_f_mean = []
    coef3_f_mean = []
    rewards_f_mean = []
    for i in range(len(coef1_m)):
        coef1_m_mean.append(np.array(coef1_m).mean())
        coef2_m_mean.append(np.array(coef2_m).mean())
        coef3_m_mean.append(np.array(coef3_m).mean())
        rewards_m_mean.append(np.array(rewards_m).mean())
        coef1_f_mean.append(np.array(coef1_f).mean())
        coef2_f_mean.append(np.array(coef2_f).mean())
        coef3_f_mean.append(np.array(coef3_f).mean())
        rewards_f_mean.append(np.array(rewards_f).mean())

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(range(1, len(coef1_m)+1), coef1_m, 'yo', markersize=4.0, label='c_task')  # Puntos
    # ax.plot(range(1, len(coef1_m) + 1), coef1_m, 'y', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef1_m_mean) + 1), coef1_m_mean, 'y', linestyle=':', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef2_m)+1), coef2_m, 'mo', markersize=4.0, label='c_human_feedback')
    # ax.plot(range(1, len(coef2_m) + 1), coef2_m, 'm', markersize=1.0)
    # ax.plot(range(1, len(coef2_m_mean) + 1), coef2_m_mean, 'm', linestyle=':', markersize=1.0)
    # ax.plot(range(1, len(coef3_m)+1), coef3_m, 'co', markersize=4.0, label='c_energy')
    # ax.plot(range(1, len(coef3_m) + 1), coef3_m, 'c', markersize=1.0)
    # ax.plot(range(1, len(coef3_m_mean) + 1), coef3_m_mean, 'c', linestyle=':', markersize=1.0)
    # ax.set_xlabel('Execution', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.set_title("Assembled pieces", size=11.0)
    # ax.grid()
    # ax2 = ax.twinx()
    # ax2.plot(range(1, len(rewards_m) + 1), rewards_m, 'r', linewidth=1.0, label='valid pieces')
    # ax2.plot(range(1, len(rewards_m_mean) + 1), rewards_m_mean, 'r', linewidth=2.0, linestyle=':', label='valid pieces')
    # ax2.set_ylabel('Number of valid pieces', size=11.0)
    # # plt.axvline(x=20, linewidth=2.0, color='k', linestyle='--')  # Momento en el que cambio el objetivo
    #
    # # Genero leyenda figura anterior
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(1, coef1_m[0], 'yo', markersize=4.0, label='$c_{task}$')  # Puntos
    # ax.plot(1, coef2_m[0], 'mo', markersize=4.0, label='$c_{human}$')
    # ax.plot(1, coef3_m[0], 'co', markersize=4.0, label='$c_{energy}$')
    # ax.plot(1, rewards_m[0], 'k', linewidth=2.0, label='Valid pieces')
    # ax.set_xlabel('Execution', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.legend(prop={'size': 11})
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(range(1, len(coef1_m) + 1), coef1_m, 'yo', markersize=4.0, label='c_task')  # Puntos
    # ax.plot(range(1, len(coef1_m) + 1), coef1_m, 'y', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef1_m_mean) + 1), coef1_m_mean, 'y', linestyle=':', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef2_m) + 1), coef2_m, 'mo', markersize=4.0, label='c_human_feedback')
    # ax.plot(range(1, len(coef2_m) + 1), coef2_m, 'm', markersize=1.0)
    # ax.plot(range(1, len(coef2_m_mean) + 1), coef2_m_mean, 'm', linestyle=':', markersize=1.0)
    # ax.plot(range(1, len(coef3_m) + 1), coef3_m, 'co', markersize=4.0, label='c_energy')
    # ax.plot(range(1, len(coef3_m) + 1), coef3_m, 'c', markersize=1.0)
    # ax.plot(range(1, len(coef3_m_mean) + 1), coef3_m_mean, 'c', linestyle=':', markersize=1.0)
    # ax.set_xlabel('Execution', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.set_title("Assembled pieces", size=11.0)
    # # ax.grid()
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(range(1, len(coef1_f) + 1), coef1_f, 'yo', markersize=4.0, label='c_task')  # Puntos
    # ax.plot(range(1, len(coef1_f) + 1), coef1_f, 'y', markersize=0.5)  # Linea para unirlos
    # ax.plot(range(1, len(coef1_f_mean) + 1), coef1_f_mean, 'y', linestyle=':', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef2_f) + 1), coef2_f, 'mo', markersize=4.0, label='c_human_feedback')
    # ax.plot(range(1, len(coef2_f) + 1), coef2_f, 'm', markersize=0.5)
    # ax.plot(range(1, len(coef2_f_mean) + 1), coef2_f_mean, 'm', linestyle=':', markersize=1.0)
    # ax.plot(range(1, len(coef3_f) + 1), coef3_f, 'co', markersize=4.0, label='c_energy')
    # ax.plot(range(1, len(coef3_f) + 1), coef3_f, 'c', markersize=0.5)
    # ax.plot(range(1, len(coef3_f_mean) + 1), coef3_f_mean, 'c', linestyle=':', markersize=1.0)
    # ax.set_xlabel('Experiment', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.set_title("Verified pieces", size=11.0)
    # # ax.grid()
    # ax2 = ax.twinx()
    # ax2.plot(range(1, len(rewards_f) + 1), rewards_f, 'k', linewidth=3.0, label='valid pieces')
    # ax2.plot(range(1, len(rewards_f_mean) + 1), rewards_f_mean, 'k', linewidth=2.0, linestyle=':', label='valid pieces')
    # ax2.set_ylabel('Number of valid pieces', size=11.0)
    # ax.tick_params(labelsize=12.0)
    # ax2.tick_params(labelsize=12.0)
    # # plt.axvline(x=20, linewidth=2.0, color='k', linestyle='--')  # Momento en el que cambio el objetivo
    #
    # # Genero leyenda figura anterior
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(1, coef1_f[0], 'yo', markersize=4.0, label='$c_{task}$')  # Puntos
    # ax.plot(1, coef2_f[0], 'mo', markersize=4.0, label='$c_{human}$')
    # ax.plot(1, coef3_f[0], 'co', markersize=4.0, label='$c_{energy}$')
    # ax.plot(1, rewards_f[0], 'r', linewidth=2.0, label='Valid pieces')
    # ax.set_xlabel('Execution', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.legend(prop={'size': 11})
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(range(1, len(coef1_f) + 1), coef1_f, 'yo', markersize=4.0, label='c_task')  # Puntos
    # ax.plot(range(1, len(coef1_f) + 1), coef1_f, 'y', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef1_f_mean) + 1), coef1_f_mean, 'y', linestyle=':', markersize=1.0)  # Linea para unirlos
    # ax.plot(range(1, len(coef2_f) + 1), coef2_f, 'mo', markersize=4.0, label='c_human_feedback')
    # ax.plot(range(1, len(coef2_f) + 1), coef2_f, 'm', markersize=1.0)
    # ax.plot(range(1, len(coef2_f_mean) + 1), coef2_f_mean, 'm', linestyle=':', markersize=1.0)
    # ax.plot(range(1, len(coef3_f) + 1), coef3_f, 'co', markersize=4.0, label='c_energy')
    # ax.plot(range(1, len(coef3_f) + 1), coef3_f, 'c', markersize=1.0)
    # ax.plot(range(1, len(coef3_f_mean) + 1), coef3_f_mean, 'c', linestyle=':', markersize=1.0)
    # ax.set_xlabel('Execution', size=11.0)
    # ax.set_ylabel('Coefficient value', size=11.0)
    # ax.set_title("Verified pieces", size=11.0)
    # ax.grid()
    #
    #
    # # Box plots
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # data_boxplot = [np.array(coef1_m), np.array(coef2_m), np.array(coef3_m), np.array(coef1_f),np.array(coef2_f), np.array(coef3_f)]
    # medianprops = dict(linestyle='-', linewidth=2.5, color='k')
    # bp = ax.boxplot(data_boxplot, patch_artist=True, medianprops=medianprops,labels=['$c_{task}$', '$c_{human}$', '$c_{energy}$','$c_{task}$', '$c_{human}$', '$c_{energy}$'])
    # colors = ['y', 'm', 'c', 'y', 'm', 'c']
    # for patch, color in zip(bp['boxes'], colors):
    #     patch.set_facecolor(color)
    #     patch.set_linewidth(0)
    # colors2 = ['y','y', 'm', 'm', 'c', 'c', 'y', 'y', 'm', 'm', 'c', 'c'] # We have two whiskers
    # for patch, color in zip(bp['whiskers'], colors2):
    #     patch.set_color(color)
    #     patch.set_linewidth(1.5)
    #     patch.set_linestyle('--')
    # for patch, color in zip(bp['caps'], colors2):
    #     patch.set_color(color)
    #     patch.set_linewidth(1.5)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # data_boxplot = [np.array(coef1_m), np.array(coef2_m), np.array(coef3_m), np.array(coef1_f),np.array(coef2_f), np.array(coef3_f)]
    # medianprops = dict(linestyle='-', linewidth=2.5, color='k')
    # bp = ax.boxplot(data_boxplot, patch_artist=True, medianprops=medianprops,labels=['$c_{task}$', '$c_{human}$', '$c_{energy}$','$c_{task}$', '$c_{human}$', '$c_{energy}$'])
    # colors = ['y', 'm', 'c', 'y', 'm', 'c']
    # for patch, color in zip(bp['boxes'], colors):
    #     patch.set_facecolor(color)
    #     patch.set_linewidth(0)

    # Boxplot para demostrar que aprende
    fig, ax = plt.subplots(ncols=2, sharey=True)
    fig.subplots_adjust(wspace=0)
    data1_boxplot = [np.array(coef1_m), np.array(coef2_m), np.array(coef3_m)]
    data2_boxplot = [np.array(coef1_f), np.array(coef2_f), np.array(coef3_f)]
    medianprops = dict(linestyle="-", linewidth=2.5, color="k")
    bp1 = ax[0].boxplot(
        data1_boxplot, patch_artist=True, medianprops=medianprops, labels=["$c_{task}$", "$c_{human}$", "$c_{energy}$"]
    )
    bp2 = ax[1].boxplot(
        data2_boxplot, patch_artist=True, medianprops=medianprops, labels=["$c_{task}$", "$c_{human}$", "$c_{energy}$"]
    )
    colors = ["y", "m", "c"]
    for patch, color in zip(bp1["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_linewidth(0)
    for patch, color in zip(bp2["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_linewidth(0)
    colors2 = ["y", "y", "m", "m", "c", "c"]  # We have two whiskers
    for patch, color in zip(bp1["whiskers"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
        patch.set_linestyle("--")
    for patch, color in zip(bp2["whiskers"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
        patch.set_linestyle("--")
    for patch, color in zip(bp1["caps"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
    for patch, color in zip(bp2["caps"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
    ax[0].set_title("Original task")  # ax[0].set_title('Assembled pieces')
    ax[1].set_title("New task")  # ax[1].set_title('Verified pieces')
    ax[0].tick_params(labelsize=12.0)
    ax[1].tick_params(labelsize=12.0)
    ax[0].set_ylabel("Coefficient value", size=11.0)

    plt.savefig("boxplots_coefficient_values_title_dpi.png", dpi=200)

    # Boxplot para demostrar que el aprendizaje es util y relevante para la tarea
    archivo = open("datos_coef_fijos_mismo_valor.txt", "r")
    datos = archivo.readlines()
    rewards_m_valor = []
    rewards_f_valor = []
    for i in range(len(datos)):
        aux = datos[i].strip().split(" ")
        rewards_m_valor.append(float(aux[3]))
        rewards_f_valor.append(float(aux[7]))
    archivo = open("datos_coef_fijos_no_humano.txt", "r")
    datos = archivo.readlines()
    rewards_m_no_humano = []
    rewards_f_no_humano = []
    for i in range(len(datos)):
        aux = datos[i].strip().split(" ")
        rewards_m_no_humano.append(float(aux[3]))
        rewards_f_no_humano.append(float(aux[7]))
    archivo = open("datos_coef_fijos_humano.txt", "r")
    datos = archivo.readlines()
    rewards_m_humano = []
    rewards_f_humano = []
    for i in range(len(datos)):
        aux = datos[i].strip().split(" ")
        rewards_m_humano.append(float(aux[3]))
        rewards_f_humano.append(float(aux[7]))

    fig, ax = plt.subplots(ncols=2, sharey=True)
    fig.subplots_adjust(wspace=0)
    data1_boxplot = [
        np.array(rewards_m),
        np.array(rewards_m_valor),
        np.array(rewards_m_no_humano),
        np.array(rewards_m_humano),
    ]
    data2_boxplot = [
        np.array(rewards_f),
        np.array(rewards_f_valor),
        np.array(rewards_f_no_humano),
        np.array(rewards_f_humano),
    ]
    medianprops = dict(linestyle="-", linewidth=2.5, color="k")
    bp1 = ax[0].boxplot(data1_boxplot, patch_artist=True, medianprops=medianprops, labels=["$a$", "$b$", "$c$", "$d$"])
    bp2 = ax[1].boxplot(data2_boxplot, patch_artist=True, medianprops=medianprops, labels=["$a$", "$b$", "$c$", "$d$"])
    colors = ["y", "c", "m", "r"]
    for patch, color in zip(bp1["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
        patch.set_linewidth(0)
    for patch, color in zip(bp2["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
        patch.set_linewidth(0)
    colors2 = ["y", "y", "c", "c", "m", "m", "r", "r"]  # We have two whiskers
    for patch, color in zip(bp1["whiskers"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
        patch.set_linestyle("--")
    for patch, color in zip(bp2["whiskers"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
        patch.set_linestyle("--")
    for patch, color in zip(bp1["caps"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
    for patch, color in zip(bp2["caps"], colors2):
        patch.set_color(color)
        patch.set_linewidth(1.5)
    for patch, color in zip(bp1["medians"], ["goldenrod", "darkcyan", "purple", "darkred"]):
        patch.set_color(color)
        # patch.set_linewidth(1.5)
    for patch, color in zip(bp2["medians"], ["goldenrod", "darkcyan", "purple", "darkred"]):
        patch.set_color(color)
        # patch.set_linewidth(1.5)
    colors3 = ["goldenrod", "darkcyan", "purple", "darkred"]  # We have four fliers
    for patch, color in zip(bp1["fliers"], colors3):
        patch.set_markeredgecolor(color)
        # patch.set_linewidth(1.5)
    for patch, color in zip(bp2["fliers"], colors3):
        patch.set_markeredgecolor(color)
        # patch.set_linewidth(1.5)
    ax[0].set_title("Original task")  # ax[0].set_title('Assembled pieces')
    ax[1].set_title("New task")  # ax[1].set_title('Verified pieces')
    ax[0].tick_params(labelsize=12.0)
    ax[1].tick_params(labelsize=12.0)
    ax[0].set_ylabel("Number of valid pieces", size=11.0)
    legend_boxes = [bp2["boxes"][0], bp2["boxes"][1], bp2["boxes"][2], bp2["boxes"][3]]
    labels_legend = [
        "$a$: $Autonomous$ $balance$",
        "$b$: $c_{task}=c_{human}=c_{energy}$",
        "$c$: $c_{energy}>c_{task}>c_{human}$",
        "$d$: $c_{energy}>c_{human}>c_{energy}$",
    ]
    ax[1].legend(legend_boxes, labels_legend, loc="upper right", fontsize="small")

    # an2 = ax.annotate("Task change", xy=(0.5, 1.), xycoords=ax,
    #                   xytext=(0.5, 1.08), textcoords=(ax, "axes fraction"),
    #                   va="bottom", ha="center",
    #                   bbox=dict(boxstyle="round", fc="w"),
    #                   arrowprops=dict(arrowstyle="->"))

    plt.savefig("boxplots_valid_pieces_title_dpi_new.png", dpi=200)

    # Datos para Boxplot para demostrar que el aprendizaje es util y relevante para la tarea
    # 30 ejecuciones
    # datos = open('datos_coef_fijos_humano.txt', 'w')
    # for i in range(1, 31):
    #     f = open('seed'+str(i)+'.pckl', 'rb')
    #     seed = pickle.load(f)
    #     f.close()
    #     np.random.set_state(seed)
    #     coef_1 = []
    #     coef_2 = []
    #     coef_3 = []
    #     rewards = []
    #     v = [0.1, 0.45, 0.8]
    #     instance = DREAM(c_task=v[0], c_human_feedback=v[1], c_energy=v[2])  # , c_human_close=v[3])
    #     instance.run()
    #     r = instance.simulator.n_assembled_pieces
    #     # r = instance.simulator.total_verified_pieces
    #     for i in range(20):
    #         v2 = v[:]
    #         # v2[0] = np.random.uniform(low=0.0, high=1.01)
    #         # v2[1] = np.random.uniform(low=0.0, high=1.01)
    #         # v2[2] = np.random.uniform(low=0.0, high=1.01)
    #         instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
    #         instance2.run()
    #         r2 = instance2.simulator.n_assembled_pieces
    #         # if r2 > r:
    #         #     r = r2
    #         #     v = v2[:]
    #         coef_1.append(v[0])
    #         coef_2.append(v[1])
    #         coef_3.append(v[2])
    #         rewards.append(r2)
    #     # Cambio el objetivo:
    #     r = instance.simulator.total_verified_pieces
    #     for i in range(20):
    #         v2 = v[:]
    #         # v2[0] = np.random.uniform(low=0.0, high=1.01)
    #         # v2[1] = np.random.uniform(low=0.0, high=1.01)
    #         # v2[2] = np.random.uniform(low=0.0, high=1.01)
    #         instance2 = DREAM(c_task=v2[0], c_human_feedback=v2[1], c_energy=v2[2])  # , c_human_close=v[3])
    #         instance2.run()
    #         r2 = instance2.simulator.total_verified_pieces
    #         # if r2 > r:
    #         #     r = r2
    #         #     v = v2[:]
    #         coef_1.append(v[0])
    #         coef_2.append(v[1])
    #         coef_3.append(v[2])
    #         rewards.append(r2)
    #     datos.write(("%f %f %f %f %f %f %f %f \n" % (coef_1[19], coef_2[19], coef_3[19], rewards[19], coef_1[39], coef_2[39], coef_3[39], rewards[39])))

    # Genero la leyenda de la figura anterior
    plt.figure()
    plt.plot(1, 2, color="m", label="Energy $(_{op}D_{e})$")
    plt.plot(1, 2, color="orange", label="Human $(_{op}D_{h})$")
    plt.plot(1, 2, color="r", label="Task $(_{op}D_{t})$")
    plt.plot(1, 2, linestyle="--", color="g", label="Performance $(_{op}D_{p})$")
    plt.plot(1, 2, linestyle=":", label="Novelty $(_{cg}D_{nov})$")
    plt.plot(1, 2, linestyle=":", color="cyan", label="Effectance $(_{cg}D_{eff})$")
    plt.legend(loc="upper left")
    plt.savefig("legend_drives.png", dpi=200)

    plt.figure()
    plt.plot(1, 2, color="m", label="$_{op}D_{e}$")
    plt.plot(1, 2, color="orange", label="$_{op}D_{h}$")
    plt.plot(1, 2, color="r", label="$_{op}D_{t}$")
    plt.plot(1, 2, linestyle="--", color="g", label="$_{op}D_{p}$")
    plt.plot(1, 2, linestyle=":", label="$_{cg}D_{nov}$")
    plt.plot(1, 2, linestyle=":", color="cyan", label="$_{cg}D_{eff}$")
    plt.legend(loc="upper left")
    plt.savefig("legend_drives2.png", dpi=200)

    print(2)


if __name__ == "__main__":
    main()
