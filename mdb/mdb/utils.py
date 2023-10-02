import importlib

def class_from_classname(class_name):
    """Return a class object from a class name."""
    module_string, _, class_string = class_name.rpartition(".")
    node_module = importlib.import_module(module_string)
    node_class = getattr(node_module, class_string)
    return node_class