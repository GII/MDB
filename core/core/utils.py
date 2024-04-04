import importlib
from cognitive_node_interfaces.msg import Perception, PerceptionParameters

def class_from_classname(class_name):
    """Return a class object from a class name."""
    module_string, _, class_string = class_name.rpartition(".")
    module = importlib.import_module(module_string)
    class_object = getattr(module, class_string)
    return class_object

def perception_dict_to_msg(perception_dict):
    """
    Transform a perception dictionary into a ROS message

    :param perception_dict: Dictionary that contais the perceptions
    :type perception_dict: dict
    :return: The ROS message with the perception
    :rtype: cognitve_node_interfaces.msg.Perception
    """
    msg = Perception()
    if perception_dict:
        msg.layout.data_offset = 0
        msg.layout.dim = []
        len_float = 8 #bytes
        for sensor, data in perception_dict.items():
            for values in enumerate(data):
                dimension = PerceptionParameters()
                dimension.size_stride_units = 'bytes'
                dimension.sensor = sensor + str(values[0])
                dimension.labels = list(values[1].keys())
                dimension.size = len(values[1])*len_float #bytes
                dimension.stride = len_float #bytes
                msg.layout.dim.append(dimension)
                for value in values[1].values():
                    msg.data.append(value)
    else:
        msg.data = []
    return msg

def perception_msg_to_dict(msg):
    """
    Transform a ROS message that contains a perception into a dictionary 

    :param msg: The ROS message with the perception
    :type msg: cognitve_node_interfaces.msg.Perception
    :return: The dictionary with the perceptions
    :rtype: dict
    """
    perception_dict = {}
    first_value = 0 
    for dim in msg.layout.dim:
        sensor = dim.sensor[:-1]
        labels = dim.labels
        size = dim.size
        stride = dim.stride
        num_elements = size//stride
        final_value = num_elements + first_value
        values = msg.data[first_value:final_value]
        values_dict = {labels[i]: values[i] for i in range(len(labels))}

        if not sensor in perception_dict.keys():
            perception_dict[sensor] = [values_dict]
        else:
            perception_dict[sensor].append(values_dict)
            
        first_value += num_elements

    return perception_dict

def separate_perceptions(perception):
    """
    Separate a dicionary with several percepcions in several ones with one perception

    :param perception: The dictionary with all perceptions
    :type perception: dict
    :return: A list with the several dictionaries
    :rtype: list
    """
    perceptions = []
    for i in range(max([len(sensor) for sensor in perception.values()])):
            perception_line = {} 
            for sensor, value in perception.items():
                sid = i % len(value)
                perception_line[sensor + str(sid)] = value[sid]
            perceptions.append(perception_line)

    return perceptions
