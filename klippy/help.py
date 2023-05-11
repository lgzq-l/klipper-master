import ast


def get_class_functions(file_path):
    with open(file_path, 'r') as f:
        tree = ast.parse(f.read())
    
    functions = []
    classes = []
    class_funcs = {}
    
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            functions.append(node)
        elif isinstance(node, ast.ClassDef):
            classes.append(node.name)
            
    for f in functions:
        for child in f.body:
            if isinstance(child, ast.Attribute):
                class_name = child.value.id
                if class_name not in class_funcs:
                    class_funcs[class_name] = []
                class_funcs[class_name].append(f.name)
                
    return class_funcs

class_functions = get_class_functions('some_file.py')
print(class_functions)
