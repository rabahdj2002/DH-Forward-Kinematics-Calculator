import sympy as sp

from dh_calculator import (
    build_dh_matrices,
    dh_transform,
    forward_kinematics,
    parse_dh_entry,
    str_to_sympy_angle,
)


class TestParseDhEntry:
    def test_json_array(self):
        assert parse_dh_entry(["theta1", " l1", "0", "pi/2"]) == [
            "theta1", "l1", "0", "pi/2",
        ]

    def test_legacy_string(self):
        assert parse_dh_entry("theta1, l1, 0, pi/2") == [
            "theta1", "l1", "0", "pi/2",
        ]

    def test_string_without_spaces(self):
        # README example values pasted with plain commas must still parse
        assert parse_dh_entry("theta1,l1,0,pi/2") == [
            "theta1", "l1", "0", "pi/2",
        ]

    def test_wrong_param_count(self):
        assert parse_dh_entry(["theta1", "l1"]) is None
        assert parse_dh_entry("theta1,l1,0,pi/2,extra") is None

    def test_invalid_type(self):
        assert parse_dh_entry(42) is None
        assert parse_dh_entry({"theta": "theta1"}) is None


class TestStrToSympyAngle:
    def test_constants(self):
        assert str_to_sympy_angle("0") == 0
        assert str_to_sympy_angle("pi") == sp.pi
        assert str_to_sympy_angle("pi/2") == sp.pi / 2
        assert str_to_sympy_angle("-pi/2") == -sp.pi / 2

    def test_symbols(self):
        assert str_to_sympy_angle("theta1") == sp.Symbol("theta1")
        assert str_to_sympy_angle("l1") == sp.Symbol("l1")
        assert str_to_sympy_angle("alpha") == sp.Symbol("alpha")


class TestDhTransform:
    def test_shape_and_homogeneous_row(self):
        m = dh_transform(sp.Symbol("t"), sp.Symbol("a"), sp.Symbol("l"), sp.Symbol("d"))
        assert m.shape == (4, 4)
        assert list(m.row(3)) == [0, 0, 0, 1]

    def test_identity_for_zero_params(self):
        m = dh_transform(0, 0, 0, 0)
        assert m == sp.eye(4)

    def test_rotation_is_orthogonal(self):
        t, a = sp.symbols("t a")
        R = dh_transform(t, a, 0, 0)[:3, :3]
        assert (R.T * R).applyfunc(sp.simplify) == sp.eye(3)


class TestBuildAndChain:
    def test_two_link_planar_arm(self):
        # 2-link planar arm: a1=a2=1, all d=alpha=0
        data = {
            "1": ["theta1", "0", "1", "0"],
            "2": ["theta2", "0", "1", "0"],
        }
        matrices, warnings = build_dh_matrices(data)
        assert warnings == []
        assert len(matrices) == 2

        T = forward_kinematics(matrices)
        # x = cos(t1) + cos(t1+t2), y = sin(t1) + sin(t1+t2)
        t1, t2 = sp.symbols("theta1 theta2")
        x = sp.trigsimp(T[0, 3])
        y = sp.trigsimp(T[1, 3])
        assert sp.simplify(x - (sp.cos(t1) + sp.cos(t1 + t2))) == 0
        assert sp.simplify(y - (sp.sin(t1) + sp.sin(t1 + t2))) == 0

    def test_numeric_end_position(self):
        data = {
            "1": ["theta1", "0", "1", "0"],
            "2": ["theta2", "0", "1", "0"],
        }
        matrices, _ = build_dh_matrices(data)
        T = forward_kinematics(matrices)
        t1, t2 = sp.symbols("theta1 theta2")
        Tn = T.subs({t1: 0, t2: 0})
        # arm fully stretched along x: position (2, 0, 0)
        assert Tn[0, 3] == 2
        assert Tn[1, 3] == 0
        assert Tn[2, 3] == 0

    def test_ignores_non_joint_keys(self):
        data = {
            "1": ["theta1", "0", "1", "0"],
            "comment": "not a joint",
        }
        matrices, warnings = build_dh_matrices(data)
        assert len(matrices) == 1
        assert warnings == []

    def test_warns_on_bad_entry(self):
        data = {"1": ["theta1", "0"], "2": ["theta2", "0", "1", "0"]}
        matrices, warnings = build_dh_matrices(data)
        assert len(matrices) == 1
        assert len(warnings) == 1

    def test_repo_conf_parses(self):
        import json
        import os

        conf = os.path.join(os.path.dirname(__file__), "..", "conf.json")
        with open(conf, encoding="utf-8") as f:
            data = json.load(f)
        matrices, warnings = build_dh_matrices(data)
        assert warnings == []
        assert len(matrices) == 3
