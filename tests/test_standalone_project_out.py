import tests.test_header
import sys
import os

# -----

import ast
import inspect
import types
import sys

def extract_function_info(filename):
    with open(filename, "r") as f:
        source = f.read()
        tree = ast.parse(source, filename=filename)
        
    function_info = []
    
    # Use a dictionary to track functions
    module_globals = {}

    # Add the source to the locals (useful if you use local functions)
    exec(source, module_globals)

    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            docstring = ast.get_docstring(node) or ""
            
            # Attempt to get the callable object from the globals
            try:
              if node.name in module_globals:
                func_obj = module_globals[node.name]
                signature = inspect.signature(func_obj)
                function_info.append({
                    "name": node.name,
                    "signature": str(signature),
                    "docstring": docstring
                })
              else:
                 function_info.append({
                    "name": node.name,
                    "signature": "Could not get signature",
                    "docstring": docstring
                })
            except TypeError as e:
              print(f"Could not get function signature for {node.name} in {filename}: {e}", file=sys.stderr)
              function_info.append({
                "name": node.name,
                "signature": "Could not get signature",
                "docstring": docstring
              })

    class_info = []
    for node in ast.walk(tree):
      if isinstance(node, ast.ClassDef):
            docstring = ast.get_docstring(node) or ""
            methods = []
            for method in node.body:
                if isinstance(method, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    method_docstring = ast.get_docstring(method) or ""
                    try:
                      if node.name in module_globals:
                         class_obj = module_globals[node.name]
                         method_obj = getattr(class_obj, method.name)
                         signature = inspect.signature(method_obj)
                         methods.append({
                            "name": method.name,
                            "signature": str(signature),
                            "docstring": method_docstring
                        })
                      else:
                        methods.append({
                            "name": method.name,
                            "signature": "Could not get signature",
                            "docstring": method_docstring
                        })
                    except AttributeError as e:
                      print(f"Could not get method signature for {node.name}.{method.name} in {filename}: {e}", file=sys.stderr)
                      methods.append({
                          "name": method.name,
                          "signature": "Could not get signature",
                          "docstring": method_docstring
                        })
                    except TypeError as e:
                      print(f"Could not get method signature for {node.name}.{method.name} in {filename}: {e}", file=sys.stderr)
                      methods.append({
                          "name": method.name,
                          "signature": "Could not get signature",
                          "docstring": method_docstring
                        })
            class_info.append({
                "name": node.name,
                "docstring": docstring,
                "methods": methods
            })
    
    return {
      "function_info": function_info,
      "class_info": class_info
    }

# Usage:
file_path = "./dimos/agents/memory/base.py"
extracted_info = extract_function_info(file_path)
print(extracted_info)

file_path = "./dimos/agents/memory/chroma_impl.py"
extracted_info = extract_function_info(file_path)
print(extracted_info)

file_path = "./dimos/agents/agent.py"
extracted_info = extract_function_info(file_path)
print(extracted_info)