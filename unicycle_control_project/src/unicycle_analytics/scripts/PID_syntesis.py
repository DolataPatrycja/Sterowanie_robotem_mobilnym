#!/usr/bin/env python3

import math
import sympy as sp


def damping_ratio_from_overshoot(percent_overshoot):
    """
    Wyznacza współczynnik tłumienia zeta na podstawie
    dopuszczalnego przeregulowania Mp wyrażonego w procentach.

    Zależność:
        Mp = exp(-zeta*pi / sqrt(1 - zeta^2))

    Po przekształceniu:
        zeta = -ln(Mp) / sqrt(pi^2 + ln(Mp)^2)
    """
    if percent_overshoot <= 0:
        raise ValueError("Przeregulowanie musi być większe od 0%.")
    if percent_overshoot >= 100:
        raise ValueError("Przeregulowanie musi być mniejsze od 100%.")

    mp = percent_overshoot / 100.0
    log_mp = math.log(mp)

    zeta = -log_mp / math.sqrt(math.pi**2 + log_mp**2)

    return zeta


def natural_frequency_from_settling_time(
    settling_time,
    damping_ratio,
    settling_criterion_percent=5.0
):
    """
    Wyznacza pulsację własną omega_n na podstawie czasu regulacji.

    Przyjęto zależność dla układu drugiego rzędu:

        Ts = -ln(epsilon * sqrt(1 - zeta^2)) / (zeta * omega_n)

    gdzie:
        Ts      - czas regulacji,
        epsilon - kryterium regulacji, np. 0.05 dla 5%,
        zeta    - współczynnik tłumienia,
        omega_n - pulsacja własna.
    """
    if settling_time <= 0:
        raise ValueError("Czas regulacji musi być większy od 0.")
    if damping_ratio <= 0 or damping_ratio >= 1:
        raise ValueError("Współczynnik tłumienia musi być z zakresu (0, 1).")
    if settling_criterion_percent <= 0 or settling_criterion_percent >= 100:
        raise ValueError("Kryterium regulacji musi być z zakresu (0, 100).")

    criterion = settling_criterion_percent / 100.0

    omega_n = -math.log(
        criterion * math.sqrt(1.0 - damping_ratio**2)
    ) / (damping_ratio * settling_time)

    return omega_n


def dominant_poles(zeta, omega_n):
    """
    Wyznacza dominującą parę biegunów zespolonych:

        s1,2 = -zeta*omega_n ± j*omega_n*sqrt(1 - zeta^2)
    """
    real_part = -zeta * omega_n
    imaginary_part = omega_n * math.sqrt(1.0 - zeta**2)

    s1 = complex(real_part, imaginary_part)
    s2 = complex(real_part, -imaginary_part)

    return s1, s2


def pid_settings_for_double_integrator(
    settling_time,
    percent_overshoot,
    settling_criterion_percent,
    third_pole_factor
):
    """
    Dobór nastaw PID metodą lokowania biegunów dla obiektu:

        G(s) = 1/s^2

    Regulator PID:

        C(s) = Kp + Ki/s + Kd*s

    Równanie charakterystyczne układu zamkniętego:

        1 + C(s)G(s) = 0

    Po podstawieniu:

        1 + (Kp + Ki/s + Kd*s) * 1/s^2 = 0

    Po pomnożeniu przez s^3:

        s^3 + Kd*s^2 + Kp*s + Ki = 0

    Para biegunów dominujących dobierana jest na podstawie:
        - czasu regulacji Ts,
        - dopuszczalnego przeregulowania Mp.

    Trzeci biegun dobierany jest jako biegun niedominujący:

        s3 = -p3

    gdzie:

        p3 = alpha * zeta * omega_n

    alpha to parametr third_pole_factor.

    Wielomian wzorcowy:

        (s^2 + 2*zeta*omega_n*s + omega_n^2)(s + p3)

    Po wymnożeniu:

        s^3
        + (2*zeta*omega_n + p3) s^2
        + (omega_n^2 + 2*zeta*omega_n*p3) s
        + omega_n^2*p3

    Porównując z:

        s^3 + Kd*s^2 + Kp*s + Ki

    otrzymuje się:

        Kd = 2*zeta*omega_n + p3
        Kp = omega_n^2 + 2*zeta*omega_n*p3
        Ki = omega_n^2*p3
    """
    if third_pole_factor <= 1.0:
        raise ValueError("third_pole_factor powinien być większy od 1.")

    zeta = damping_ratio_from_overshoot(percent_overshoot)

    omega_n = natural_frequency_from_settling_time(
        settling_time,
        zeta,
        settling_criterion_percent
    )

    s1, s2 = dominant_poles(zeta, omega_n)

    p3 = third_pole_factor * zeta * omega_n
    s3 = complex(-p3, 0.0)

    kd = 2.0 * zeta * omega_n + p3
    kp = omega_n**2 + 2.0 * zeta * omega_n * p3
    ki = omega_n**2 * p3

    return {
        "zeta": zeta,
        "omega_n": omega_n,
        "s1": s1,
        "s2": s2,
        "s3": s3,
        "p3": p3,
        "third_pole_factor": third_pole_factor,
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "settling_time": settling_time,
        "percent_overshoot": percent_overshoot,
        "settling_criterion_percent": settling_criterion_percent
    }


def derive_pid_for_double_integrator():
    """
    Symboliczne wyprowadzenie wielomianu charakterystycznego
    dla obiektu G(s) = 1/s^2 oraz regulatora PID.
    """
    s = sp.symbols("s")
    Kp, Ki, Kd = sp.symbols("Kp Ki Kd")
    zeta, omega_n, p3 = sp.symbols("zeta omega_n p3")

    G = 1 / s**2
    C = Kp + Ki / s + Kd * s

    characteristic_equation = sp.expand(1 + C * G)
    characteristic_polynomial = sp.expand(s**3 * characteristic_equation)

    desired_polynomial = sp.expand(
        (s**2 + 2 * zeta * omega_n * s + omega_n**2) * (s + p3)
    )

    return {
        "G": G,
        "C": C,
        "characteristic_equation": characteristic_equation,
        "characteristic_polynomial": characteristic_polynomial,
        "desired_polynomial": desired_polynomial
    }


def format_complex_pole(pole):
    """
    Formatowanie liczby zespolonej do czytelnego wypisywania biegunów.
    """
    real = pole.real
    imag = pole.imag

    if abs(imag) < 1e-10:
        return f"{real:.4f}"

    if imag >= 0:
        return f"{real:.4f} + {imag:.4f}j"

    return f"{real:.4f} - {abs(imag):.4f}j"


def print_controller_settings(name, settings):
    """
    Wypisuje nastawy regulatora PID dla danego toru regulacji.
    """
    print(f"NASTAWY TORU {name.upper()}:")
    print(f"Ts = {settings['settling_time']} s")
    print(f"Mp = {settings['percent_overshoot']} %")
    print(f"kryterium regulacji = {settings['settling_criterion_percent']} %")
    print(f"zeta = {settings['zeta']:.4f}")
    print(f"omega_n = {settings['omega_n']:.4f} rad/s")
    print(f"third_pole_factor = {settings['third_pole_factor']:.4f}")
    print(f"p3 = {settings['p3']:.4f}")
    print(f"biegun dominujący s1 = {format_complex_pole(settings['s1'])}")
    print(f"biegun dominujący s2 = {format_complex_pole(settings['s2'])}")
    print(f"trzeci biegun s3 = {format_complex_pole(settings['s3'])}")
    print(f"Kp_{name} = {settings['kp']:.4f}")
    print(f"Ki_{name} = {settings['ki']:.4f}")
    print(f"Kd_{name} = {settings['kd']:.4f}")
    print()


def main():
    symbolic = derive_pid_for_double_integrator()

    print("=" * 80)
    print("SYNTEZA REGULATORÓW PID METODĄ LOKOWANIA BIEGUNÓW")
    print("=" * 80)
    print()

    print("MODEL ZASTĘPCZY TORU REGULACJI:")
    print("G(s) =")
    sp.pprint(symbolic["G"])
    print()

    print("REGULATOR PID:")
    print("C(s) =")
    sp.pprint(symbolic["C"])
    print()

    print("RÓWNANIE CHARAKTERYSTYCZNE UKŁADU ZAMKNIĘTEGO:")
    print("1 + C(s)G(s) = 0")
    print("czyli:")
    sp.pprint(symbolic["characteristic_equation"])
    print()

    print("WIELOMIAN CHARAKTERYSTYCZNY PO POMNOŻENIU PRZEZ s^3:")
    sp.pprint(symbolic["characteristic_polynomial"])
    print()

    print("WIELOMIAN WZORCOWY Z TRZECIM BIEGUNEM:")
    print("(s^2 + 2*zeta*omega_n*s + omega_n^2)(s + p3)")
    print("czyli:")
    sp.pprint(symbolic["desired_polynomial"])
    print()

    print("Z porównania współczynników:")
    print("Kd = 2*zeta*omega_n + p3")
    print("Kp = omega_n^2 + 2*zeta*omega_n*p3")
    print("Ki = omega_n^2*p3")
    print()

    linear = pid_settings_for_double_integrator(
        settling_time=5.0,
        percent_overshoot=20.0,
        settling_criterion_percent=5.0,
        third_pole_factor=2.0
    )

    angular = pid_settings_for_double_integrator(
        settling_time=5.0,
        percent_overshoot=20.0,
        settling_criterion_percent=5.0,
        third_pole_factor=2.0
    )

    print("=" * 80)
    print("WYNIKI SYNTEZY")
    print("=" * 80)
    print()

    print_controller_settings("linear", linear)
    print_controller_settings("angular", angular)

if __name__ == "__main__":
    main()