import numpy as np


def s_array(q):
    rho, phi, theta = q
    return np.array(
        [[np.cos(phi - theta), 0], [-1 / rho * np.sin(phi - theta), 0], [0, 1]]
    )


def q_dot(q, z):
    return s_array(q) @ z


def control_law(q_c, q_r, z_c, z_r, gamma, p, q):
    """Robust control law for the trajectory tracking problem.

    Parameters
    ----------
    q_c : np.ndarray
        Robot state vector.
    q_r : np.ndarray
        Reference state vector.
    z_c : np.ndarray
        Internal state vector.
    z_r : np.ndarray
        Reference internal state vector.
    gamma : np.ndarray
        Sliding mode parameters (gamma >= 0).
    p : np.ndarray
        Sliding mode parameters (p >= f_m).
    q : np.ndarray
        Sliding mode parameters (q >= 0).

    Returns
    -------
    _type_
        _description_
    """
    # Based on the paper doi:10.1109/70.768190
    # https://ieeexplore.ieee.org/document/768190
    rho_c, phi_c, theta_c = q_c
    rho_r, phi_r, theta_r = q_r
    v_c, omega_c = z_c
    v_r, omega_r = z_r
    gamma_1, gamma_2 = gamma
    p1, p2 = p
    q1, q2 = q

    v_r_dot = 0  # TODO: Implement this

    # Parameters
    R = v_c ** 2 / rho_c * np.sin(phi_c - theta_c) ** 2 + v_c * omega_c * np.sin(
        phi_c - theta_c
    )
    R_r = v_r ** 2 / rho_r * np.sin(phi_r - theta_r) ** 2 + v_r * omega_r * np.sin(
        phi_r - theta_r
    )

    # Robot posture error
    rho_e = rho_c - rho_r
    theta_e = theta_c - theta_r
    phi_e = phi_c - phi_r

    rho_e_dot, phi_e_dot, theta_e_dot = q_dot(q_r, z_c) - q_dot(q_r, z_r)

    # Vector of sliding surfaces
    s1 = rho_e_dot + gamma_1 * rho_e
    s2 = theta_e_dot + gamma_2 * theta_e + np.sign(theta_e) * abs(phi_e)

    # Robust control law
    u1 = (
        1
        / np.cos(phi_c - theta_c)
        * (
            -q1 * s1
            - p1 * np.sign(s1)
            - gamma_1 * rho_e_dot
            + v_r * np.cos(phi_r - theta_r)
            + R_r
            - R
        )
        - v_r_dot
    )
    u2 = (
        -q2 * s2
        - p2 * np.sign(s2)
        - gamma_2 * theta_e_dot
        - np.sign(theta_e) * abs(phi_e_dot)  # TODO: Check this, it's not in the paper
    )
    u = np.array([u1, u2])
    return u


def integration(u):
    """Trajectory tracking integration.

    Parameters
    ----------
    u : np.ndarray
        Control law.

    Returns
    -------
    np.ndarray
        Torque vector.
    """
    u  # avoid unused variable warning
    tau = np.array([0, 0])
    return tau


def sliding_mode_control(q_c, q_r, z_c, z_r):
    # Parameters
    gamma = np.array([0.1, 0.1])
    p = np.array([0.1, 0.1])
    q = np.array([0.1, 0.1])

    # Control law
    u = control_law(q_c, q_r, z_c, z_r, gamma, p, q)

    # Integration
    tau = integration(u)
    return tau
