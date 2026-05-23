#!/usr/bin/env python3

import sympy as sp
import math


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


def damping_ratio_from_overshoot(percent_overshoot):
    """
    Wyznacza współczynnik tłumienia na podstawie przeregulowania.

    Args:
        percent_overshoot (float): Dopuszczalne przeregulowanie w procentach.

    Returns:
        float: Współczynnik tłumienia.
    """
    if percent_overshoot <= 0:
        raise ValueError("Przeregulowanie musi być większe od 0%.")
    if percent_overshoot >= 100:
        raise ValueError("Przeregulowanie musi być mniejsze od 100%.")

    mp = percent_overshoot / 100.0
    log_mp = math.log(mp)

    damping_ratio = -log_mp / math.sqrt(math.pi**2 + log_mp**2)

    return damping_ratio


def natural_pulsation_from_settling_time(
    settling_time,
    damping_ratio,
    settling_criterion_percent=2.0
):
    """
    Wyznacza pulsację własną na podstawie czasu regulacji.

    Args:
        settling_time (float): Wymagany czas regulacji.
        damping_ratio (float): Współczynnik tłumienia.
        settling_criterion_percent (float): Kryterium czasu regulacji w procentach.

    Returns:
        float: Pulsacja własna w rad/s.
    """
    if settling_time <= 0:
        raise ValueError("Czas regulacji musi być większy od 0.")
    if damping_ratio <= 0 or damping_ratio >= 1:
        raise ValueError("Współczynnik tłumienia musi być z zakresu (0, 1).")
    if settling_criterion_percent <= 0 or settling_criterion_percent >= 100:
        raise ValueError("Kryterium regulacji musi być z zakresu (0, 100).")

    settling_criterion = settling_criterion_percent / 100.0

    coefficient = -math.log(
        settling_criterion * math.sqrt(1.0 - damping_ratio**2)
    )

    omega_n = coefficient / (damping_ratio * settling_time)

    return omega_n


def dominant_poles_from_requirements(
    settling_time=2.0,
    percent_overshoot=5.0,
    settling_criterion_percent=2.0,
    third_pole_factor=5.0
):
    """
    Wyznacza bieguny układu na podstawie wymagań projektowych.

    Args:
        settling_time (float): Wymagany czas regulacji.
        percent_overshoot (float): Dopuszczalne przeregulowanie w procentach.
        settling_criterion_percent (float): Kryterium czasu regulacji w procentach.
        third_pole_factor (float): Współczynnik przesunięcia trzeciego bieguna.

    Returns:
        dict: Parametry dynamiczne oraz wyznaczone bieguny.
    """
    damping_ratio = damping_ratio_from_overshoot(percent_overshoot)

    omega_n = natural_pulsation_from_settling_time(
        settling_time,
        damping_ratio,
        settling_criterion_percent
    )

    real_part = -damping_ratio * omega_n
    imaginary_part = omega_n * math.sqrt(1.0 - damping_ratio**2)

    p1 = complex(real_part, imaginary_part)
    p2 = complex(real_part, -imaginary_part)
    p3 = third_pole_factor * real_part

    return {
        "damping_ratio": damping_ratio,
        "omega_n": omega_n,
        "poles": [p1, p2, p3]
    }


def numeric_example():
    solution = symbolic_pid_from_poles()

    requirements = dominant_poles_from_requirements(
        settling_time=30.0,
        percent_overshoot=30.0,
        settling_criterion_percent=10.0,
        third_pole_factor=2.0
    )

    p1, p2, p3 = requirements["poles"]

    values = {
        sp.Symbol('p1'): p1,
        sp.Symbol('p2'): p2,
        sp.Symbol('p3'): p3
    }

    numeric = {
        key: sp.N(sp.simplify(val.subs(values)))
        for key, val in solution.items()
    }

    return numeric, values, requirements


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

    numeric, poles, requirements = numeric_example()

    print("Wymagania projektowe:")
    print("Ts <= 2.0 s")
    print("Mp <= 5.0 %")
    print("Kryterium regulacji: 2.0 %")
    print("Trzeci biegun: 5 razy szybszy od części rzeczywistej biegunów dominujących")
    print()

    print("Wyznaczone parametry dynamiczne:")
    print(f"współczynnik tłumienia = {requirements['damping_ratio']:.4f}")
    print(f"pulsacja własna omega_n = {requirements['omega_n']:.4f} rad/s")
    print()

    print("Bieguny wyznaczone z wymagań:")
    print(f"p1 = {poles[sp.Symbol('p1')]}")
    print(f"p2 = {poles[sp.Symbol('p2')]}")
    print(f"p3 = {poles[sp.Symbol('p3')]}")
    print()

    print("Nastawy PID wyznaczone z biegunów:")
    for key, value in numeric.items():
        print(f"{key} = {value}")


if __name__ == "__main__":
    main()