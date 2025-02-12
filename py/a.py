import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# Define the Lorenz system (Master/Drive System)
def drive_system(t, state, sigma, rho, beta):
    x1, x2, x3 = state
    dx1 = sigma * (x2 - x1)
    dx2 = x1 * (rho - x3) - x2
    dx3 = x1 * x2 - beta * x3
    return np.array([dx1, dx2, dx3])


# Define the Response (Slave) System with Control Inputs
def response_system(y, x, u, sigma, rho, beta):
    y1, y2, y3 = y
    u1, u2, u3 = u
    dy1 = sigma * (y2 - y1) + u1
    dy2 = y1 * (rho - y3) - y2 + u2
    dy3 = y1 * y2 - beta * y3 + u3
    return np.array([dy1, dy2, dy3])


# Backstepping Controller
def backstepping_control(x, y, sigma, rho, beta, k):
    e1, e2, e3 = y - x  # Synchronization errors
    u1 = -sigma * (y[1] - y[0] - x[1] + x[0]) + e2
    u2 = -rho * (y[0] - x[0]) + (y[1] - x[1]) + (y[0] * y[2]) - (x[0] * x[2]) + e3
    u3 = (
        (-y[0] * y[1])
        + (x[0] * x[1])
        + (beta * (y[2] - x[2]))
        - ((3 + (2 * k[0])) * e1)
        - ((5 + (2 * k[0])) * e2)
        - ((3 + k[0]) * e3)
    )
    return np.array([u1, u2, u3])


# Parameters
sigma, rho, beta = 10.0, 28.0, 8 / 3
k = np.array([5.0, 5.0, 5.0])  # Controller gains

# Time span and fixed evaluation points
t_start, t_end, dt = 0.0, 100.0, 0.01
t_eval = np.arange(t_start, t_end, dt)

# Initial conditions
x0 = np.array([1.0, 1.0, 1.0])
y0 = np.array([0.5, -0.65, 0.0])

# Solve the Drive System (Master System)
sol_master = solve_ivp(
    drive_system,
    [t_start, t_end],
    x0,
    t_eval=t_eval,
    args=(sigma, rho, beta),
    method="RK45",
)

# Precompute dx_values for each timestep
dx_values = np.array(
    [drive_system(t, x, sigma, rho, beta) for t, x in zip(sol_master.t, sol_master.y.T)]
)


# Define the coupled dynamics function for solving the response system
def coupled_system(t, u, sigma, rho, beta, k, t_eval, dx_values):
    x, y = u[:3], u[3:]  # Extract master and response states

    # Find the index of the closest precomputed dx
    idx = np.searchsorted(t_eval, t) - 1
    idx = np.clip(idx, 0, len(dx_values) - 1)  # Ensure index is within bounds
    dx = dx_values[idx]

    # Compute control input
    u_control = backstepping_control(x, y, sigma, rho, beta, k)

    # Compute response system dynamics
    dy = response_system(y, x, u_control, sigma, rho, beta)

    return np.concatenate([dx, dy])


# Solve the coupled system using precomputed dx values
u0 = np.concatenate([x0, y0])  # Initial condition (x0 + y0)
sol_coupled = solve_ivp(
    coupled_system,
    [t_start, t_end],
    u0,
    t_eval=t_eval,
    args=(sigma, rho, beta, k, t_eval, dx_values),
    method="RK45",
)

# Extract solutions
x_vals = sol_master.y.T  # Shape (N, 3) for (x1, x2, x3)
y_vals = sol_coupled.y.T[:, 3:]  # Shape (N, 3) for (y1, y2, y3)
print(sol_master)
print(sol_coupled)
# Plot each state separately
state_labels = ["x₁", "x₂", "x₃"]
response_labels = ["y₁", "y₂", "y₃"]

for i in range(3):
    plt.figure(figsize=(8, 4))
    plt.plot(t_eval, x_vals[:, i], label=f"{state_labels[i]} (Master)", lw=2)
    plt.xlabel("Time")
    plt.ylabel("State")
    plt.title(f"Master System {state_labels[i]}")
    plt.legend()
    plt.savefig(f"master_system_{state_labels[i]}.png")
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(
        t_eval, y_vals[:, i], label=f"{response_labels[i]} (Response)", lw=2, color="r"
    )
    plt.xlabel("Time")
    plt.ylabel("State")
    plt.title(f"Response System {response_labels[i]}")
    plt.legend()
    plt.savefig(f"response_system_{response_labels[i]}.png")
    plt.close()
