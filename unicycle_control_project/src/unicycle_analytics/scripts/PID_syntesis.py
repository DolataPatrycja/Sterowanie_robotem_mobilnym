#!/usr/bin/env python3

import sympy as sp


def derive_pid_characteristic():
    s = sp.symbols('s')

    Kp, Ki, Kd = sp.symbols('Kp Ki Kd')

    G = 1 / s**2
    C = Kp + Ki / s + Kd * s

    closed_loop_den = sp.expand(1 + C * G)
    characteristic = sp.expand(s**3 * closed_loop_den)

    return {
        "s": s,
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        "G": G,
        "C": C,
        "characteristic": characteristic
    }


def symbolic_pid_from_poles():
    p1, p2, p3 = sp.symbols('p1 p2 p3')
    s = sp.symbols('s')

    Kp, Ki, Kd = sp.symbols('Kp Ki Kd')

    desired = sp.expand((s - p1) * (s - p2) * (s - p3))
    actual = s**3 + Kd * s**2 + Kp * s + Ki

    desired_poly = sp.Poly(desired, s)
    actual_poly = sp.Poly(actual, s)

    equations = [
        sp.Eq(actual_poly.coeff_monomial(s**2), desired_poly.coeff_monomial(s**2)),
        sp.Eq(actual_poly.coeff_monomial(s), desired_poly.coeff_monomial(s)),
        sp.Eq(actual_poly.coeff_monomial(1), desired_poly.coeff_monomial(1)),
    ]

    solution = sp.solve(equations, (Kd, Kp, Ki), dict=True)[0]

    return solution


def numeric_example():
    solution = symbolic_pid_from_poles()

    values = {
        sp.Symbol('p1'): -2,
        sp.Symbol('p2'): -3,
        sp.Symbol('p3'): -4
    }

    numeric = {
        key: sp.simplify(val.subs(values))
        for key, val in solution.items()
    }

    return numeric, values


def main():
    model = derive_pid_characteristic()

    print("Transmitancja obiektu:")
    print(model["G"])
    print()

    print("Transmitancja PID:")
    print(model["C"])
    print()

    print("Wielomian charakterystyczny:")
    print(model["characteristic"])
    print()

    symbolic = symbolic_pid_from_poles()

    print("Symboliczne nastawy PID:")
    for key, value in symbolic.items():
        print(f"{key} = {sp.simplify(value)}")

    print()

    numeric, poles = numeric_example()

    print(f"Przykład numeryczny dla biegunów ({poles[sp.Symbol('p1')]}, {poles[sp.Symbol('p2')]}, {poles[sp.Symbol('p3')]}):")
    for key, value in numeric.items():
        print(f"{key} = {value}")


if __name__ == "__main__":
    main()