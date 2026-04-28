import sympy as sp


def main():

    x, y, theta = sp.symbols('x y theta')
    v, omega = sp.symbols('v omega')

    state = sp.Matrix([
        x,
        y,
        theta
    ])

    control = sp.Matrix([
        v,
        omega
    ])

    f = sp.Matrix([
        v * sp.cos(theta),
        v * sp.sin(theta),
        omega
    ])

    A = f.jacobian(state)
    B = f.jacobian(control)

    print("\nMODEL NIELINIOWY f(x,u):")
    sp.pprint(f)

    print("\nMACIERZ A = df/dx:")
    sp.pprint(A)

    print("\nMACIERZ B = df/du:")
    sp.pprint(B)

    print("\nLINEARYZACJA W PUNKCIE PRACY [0,1,0]:")

    theta0 = 0
    v0 = 1
    omega0 = 0

    A0 = A.subs({
        theta: theta0,
        v: v0
    })

    B0 = B.subs({
        theta: theta0,
        v: v0
    })

    print("\nA0 =")
    sp.pprint(A0)

    print("\nB0 =")
    sp.pprint(B0)

    zeta_linear = 0.7
    wn_linear = 1.2
    kp_linear = 2 * zeta_linear * wn_linear
    ki_linear = wn_linear * wn_linear

    print(f"OBLICZONE NASTAWY LINIOWEGO PID:\n Kp:{kp_linear}\n Ki:{ki_linear}")

    zeta_angular = 0.8
    wn_angular = 2.5
    kp_angular = 2 * zeta_angular * wn_angular
    ki_angular = wn_angular * wn_angular

    print(f"OBLICZONE NASTAWY KĄTOWEGO PID:\n Kp:{kp_angular}\n Ki:{ki_angular}")

if __name__ == "__main__":
    main()