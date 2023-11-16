import importlib

def class_from_classname(class_name):
    """Return a class object from a class name."""
    print("class_name: " + str(class_name))
    module_string, _, class_string = class_name.rpartition(".")
    module = importlib.import_module(module_string)
    class_object = getattr(module, class_string)
    return class_object