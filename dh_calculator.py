#!/usr/bin/env python3
"""
DH Parameters Forward Kinematics Calculator
Loads conf.json and computes symbolic transformation matrices
"""

import json
import os
import sys

import sympy as sp


def str_to_sympy_angle(angle_str):
    """Convert angle strings to SymPy symbols"""
    angle_str = str(angle_str).strip().lower()

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
    except (sp.SympifyError, TypeError, ValueError, AttributeError):
        return sp.symbols(angle_str)


def parse_dh_entry(entry):
    """Return the 4 DH params (theta, d, a, alpha) from a conf.json entry.

    Accepts either a JSON array ["theta1", "l1", "0", "pi/2"] or the legacy
    comma-separated string "theta1, l1, 0, pi/2".
    """
    if isinstance(entry, str):
        parts = entry.split(',')
    elif isinstance(entry, (list, tuple)):
        parts = [str(p) for p in entry]
    else:
        return None

    parts = [p.strip() for p in parts]
    return parts if len(parts) == 4 else None


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


def build_dh_matrices(data):
    """Build the list of DH transformation matrices from parsed conf data.

    Only integer-numbered keys are considered. Returns (matrices, warnings).
    """
    matrices = []
    warnings = []

    numeric_keys = sorted(
        (k for k in data if str(k).strip().lstrip('-').isdigit()),
        key=lambda k: int(k),
    )

    for key in numeric_keys:
        params = parse_dh_entry(data[key])
        if params is None:
            warnings.append(f"Warning: Invalid params for joint {key}")
            continue

        theta_i = str_to_sympy_angle(params[0])
        d_i = str_to_sympy_angle(params[1])
        a_i = str_to_sympy_angle(params[2])
        alpha_i = str_to_sympy_angle(params[3])

        matrices.append((key, dh_transform(theta_i, alpha_i, a_i, d_i)))

    return matrices, warnings


def forward_kinematics(matrices):
    """Chain DH matrices into the total transform T_total."""
    T_total = sp.eye(4)
    for _, R in matrices:
        T_total = T_total * R
    return T_total


def main():
    # Unicode box-drawing output needs a UTF-8 capable console
    # (Windows' default cp1252 would crash sp.pprint).
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, OSError):
        pass

    # Load configuration
    conf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'conf.json')
    if not os.path.exists(conf_path):
        conf_path = 'conf.json'
    try:
        with open(conf_path, encoding='utf-8') as file:
            data = json.load(file)
    except FileNotFoundError:
        print("Error: conf.json not found!")
        return
    except json.JSONDecodeError:
        print("Error: Invalid JSON format!")
        return

    matrices, warnings = build_dh_matrices(data)

    print(f"Computing {len(matrices)} DH transformations...\n")

    for w in warnings:
        print(w)

    for key, R_i in matrices:
        print(f"R{key}:")
        sp.pprint(R_i, use_unicode=True)
        print()

    if not matrices:
        print("No valid DH joints found in conf.json.")
        return

    # Chain all transformations
    print("=" * 60)
    print("FINAL FORWARD KINEMATICS (T_total):")
    print("=" * 60)

    T_simplified = forward_kinematics(matrices).applyfunc(sp.trigsimp)
    sp.pprint(T_simplified, use_unicode=True)

    # Extract and display position
    print("\n" + "=" * 60)
    print("END-EFFECTOR POSITION:")
    print("=" * 60)
    pos = T_simplified[:3, 3]
    sp.pprint(sp.Matrix(pos), use_unicode=True)

    print(f"\nComputation complete! {len(matrices)} joints processed.")


if __name__ == "__main__":
    main()
