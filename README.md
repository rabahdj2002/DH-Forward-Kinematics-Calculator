# DH Forward Kinematics Calculator

[![Python](https://img.shields.io/badge/Python-3.8+-blue)](https://www.python.org/)
[![SymPy](https://img.shields.io/badge/SymPy-1.12+-orange)](https://www.sympy.org/)

Symbolic **Denavit-Hartenberg (DH)** calculator for robot arms, developed by [**Djebbes Rabah**](https://rabahdj2002.github.io/).

Click my name to know more about me :)

## ✨ Features

- Fully symbolic mathematics
- Automatic trig simplification
- Chain forward kinematics `T₀ⁿ`
- End-effector position extraction
- Unicode pretty-printing

## 🚀 Quick Start

1. **Install**: `pip install -r requirements.txt`
2. **Create `conf.json`** with DH parameters
3. **Run**: `python dh_calculator.py`

Tests: `pip install -r requirements-dev.txt && pytest -q`

## 📐 Input Format Example (`conf.json`)
```json
{
"1": ["theta1", "0", "a1", "pi/2"],
"2": ["theta2", "0", "a2", "0"],
...
}
```

**Order**: `[θ, d, a, α]`


## 📐 Supported Angles

| Input           | Result               |
|-----------------|----------------------|
| `theta1, 2 ...` | θ₁ or θ₂ ... symbol  |
| `alpha1, 2 ...` | α₁ or α₂ ... symbol  |
| `pi/2`          | π/2                  |
| `0`             | 0                    |

## 🎓 Perfect For

- Robotics thesis work
- Forward kinematics verification
- LaTeX matrix export
- Teaching DH parameters

## 📄 License
MIT
