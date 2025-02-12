import numpy as np
import socket
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# Define the Response System with Control
def response_system(t, y, x, u, sigma, rho, beta):
    y1, y2, y3 = y
    u1, u2, u3 = u
    dy1 = sigma * (y2 - y1) + u1
    dy2 = y1 * (rho - y3) - y2 + u2
    dy3 = y1 * y2 - beta * y3 + u3
    return np.array([dy1, dy2, dy3])


# Backstepping Control
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
k = np.array([5.0, 5.0, 5.0])  # Control gains
dt = 0.01  # Fixed time step

# Initial conditions
y = np.array([5.0, 5.0, 5.0])

# UDP Setup
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Storage for plotting
t_vals = []
x_vals = []
y_vals = []


# Function to integrate response system
def ode_solver(y, x, sigma, rho, beta, k, dt):
    u_control = backstepping_control(x, y, sigma, rho, beta, k)  # Compute control input
    sol = solve_ivp(
        response_system,
        [0, dt],
        y,
        args=(x, u_control, sigma, rho, beta),
        method="RK45",
        t_eval=[dt],  # RK4-style one-step integration
    )
    return sol.y[:, -1]  # Return updated state after one step


# Receive data, update the response system, and store results
print("Waiting for data...")
while True:
    data, addr = sock.recvfrom(1024)  # Receive data
    t = len(t_vals) * dt  # Current time
    x_latest = np.array(list(map(float, data.decode().split(","))))  # Convert to array
    y = ode_solver(y, x_latest, sigma, rho, beta, k, dt)  # Integrate using RK4

    # Store values
    t_vals.append(t)
    x_vals.append(x_latest)
    y_vals.append(y)

    # print(f"Time: {t:.2f}, Received x: {x_latest}, Computed y: {y}")

    # Stop after a certain number of steps
    if t >= 9.9:
        break

# Convert lists to arrays
t_vals = np.array(t_vals)
x_vals = np.array(x_vals)
y_vals = np.array(y_vals)

# Compute synchronization error
error_vals = x_vals - y_vals

# Save data
# np.save("master_states.npy", x_vals)
# np.save("response_states.npy", y_vals)
# np.save("time_values.npy", t_vals)
# np.save("error_values.npy", error_vals)

print("Data saved. Plotting results...")

# Plot Master & Response States
state_labels = ["x₁", "x₂", "x₃"]
response_labels = ["y₁", "y₂", "y₃"]

for i in range(3):
    plt.figure(figsize=(8, 4))
    plt.plot(t_vals, x_vals[:, i], label=f"{state_labels[i]} (Master)", lw=2)
    plt.plot(
        t_vals,
        y_vals[:, i],
        label=f"{response_labels[i]} (Response)",
        lw=2,
        linestyle="dashed",
        color="r",
    )
    plt.xlabel("Time")
    plt.ylabel("State")
    plt.title(f"Synchronization: {state_labels[i]} vs. {response_labels[i]}")
    plt.legend()
    plt.savefig(f"synchronization_{state_labels[i]}.png")
    plt.close()

# Plot Synchronization Error
for i in range(3):
    plt.figure(figsize=(8, 4))
    plt.plot(
        t_vals, error_vals[:, i], label=f"Error in {state_labels[i]}", lw=2, color="m"
    )
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.title(f"Synchronization Error: {state_labels[i]}")
    plt.legend()
    plt.savefig(f"error_{state_labels[i]}.png")
    plt.close()

print("Plots saved.")
import pandas as pd

df = pd.DataFrame({"time": t_vals, "x1": x_vals[:, 0], "x2": x_vals[:, 1], "x3": x_vals[:, 2], "y1": y_vals[:, 0], "y2": y_vals[:, 1], "y3": y_vals[:, 2], "e1": error_vals[:, 0], "e2": error_vals[:, 1], "e3": error_vals[:, 2]})
df.to_csv("data.csv", index=False)