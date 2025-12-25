#!/usr/bin/env python3
"""
DH Parameters Forward Kinematics Calculator
Loads conf.json and computes symbolic transformation matrices
"""

import json
import sympy as sp
from functools import reduce

def str_to_sympy_angle(angle_str):
    """Convert angle strings to SymPy symbols"""
    angle_str = angle_str.strip().lower()
    
    # Variable names stay symbolic
    if angle_str in ['alpha', 'beta'] or angle_str.startswith('theta'):
        return sp.symbols(angle_str)
    
    # Common constants
    if angle_str == "0":
        return sp.Integer(0)
    elif angle_str == "pi":
        return sp.pi
    elif angle_str == "pi/2":
        return sp.pi / 2
    elif angle_str == "pi/4":
        return sp.pi / 4
    
    # Try SymPy parsing
    try:
        return sp.sympify(angle_str, locals={'pi': sp.pi})
    except:
        return sp.symbols(angle_str)

def dh_transform(theta, alpha, a, d):
    """Standard DH transformation matrix"""
    ct, st = sp.cos(theta), sp.sin(theta)
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    
    return sp.Matrix([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,     sa,     ca,    d],
        [0,     0,      0,    1]
    ])

def main():
    # Load configuration
    try:
        with open('conf.json', 'r') as file:
            data = json.load(file)
    except FileNotFoundError:
        print("Error: conf.json not found!")
        return
    except json.JSONDecodeError:
        print("Error: Invalid JSON format!")
        return
    
    matrix_count = len(data)
    symbols = {}
    R_matrices = []
    
    # Parse DH parameters and build transformation matrices
    print(f"Computing {matrix_count} DH transformations...\n")
    
    for i in range(1, matrix_count + 1):
        if str(i) not in data:
            continue
            
        params = str(data[str(i)]).split(', ')
        if len(params) != 4:
            print(f"Warning: Invalid params for joint {i}")
            continue
        
        # Parse parameters
        theta_i = str_to_sympy_angle(params[0])
        d_i = str_to_sympy_angle(params[1])
        a_i = str_to_sympy_angle(params[2])
        alpha_i = str_to_sympy_angle(params[3])
        
        # Store symbols
        symbols[f"θ{i}"] = theta_i
        symbols[f"d{i}"] = d_i
        symbols[f"a{i}"] = a_i
        symbols[f"α{i}"] = alpha_i
        
        # Build DH matrix
        R_i = dh_transform(theta_i, alpha_i, a_i, d_i)
        R_matrices.append(R_i)
        
        # Display individual matrix
        print(f"R{i}:")
        sp.pprint(R_i, use_unicode=True)
        print()
    
    # Chain all transformations
    print("=" * 60)
    print("FINAL FORWARD KINEMATICS (T_total):")
    print("=" * 60)
    
    T_total = sp.eye(4)
    for R in R_matrices:
        T_total = T_total * R
    
    # Simplify and display final result
    T_simplified = T_total.applyfunc(sp.trigsimp)
    sp.pprint(T_simplified, use_unicode=True)
    
    # Extract and display position
    print("\n" + "=" * 60)
    print("END-EFFECTOR POSITION:")
    print("=" * 60)
    pos = T_simplified[:3, 3]
    sp.pprint(sp.Matrix(pos), use_unicode=True)
    
    print(f"\n✅ Computation complete! {matrix_count} joints processed.")

if __name__ == "__main__":
    main()
