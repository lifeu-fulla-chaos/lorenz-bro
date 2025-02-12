import numpy as np
import socket
import time
from scipy.integrate import solve_ivp


# Define the Lorenz system (Master)
def drive_system(t, state, sigma, rho, beta):
    x1, x2, x3 = state
    dx1 = sigma * (x2 - x1)
    dx2 = x1 * (rho - x3) - x2
    dx3 = x1 * x2 - beta * x3
    return np.array([dx1, dx2, dx3])


# Parameters
sigma, rho, beta = 10.0, 28.0, 8 / 3
dt = 0.01  # Fixed time step
t_start, t_end = 0.0, 10.0
t_eval = np.arange(t_start, t_end, dt)  # Fixed evaluation times

# Initial condition
x0 = np.array([1.0, 1.0, 1.0])

# Solve the system using a fixed-step solver (RK4)
sol_master = solve_ivp(
    drive_system,
    [t_start, t_end],
    x0,
    t_eval=t_eval,
    args=(sigma, rho, beta),
    method="RK45",
)

# UDP Setup
UDP_IP = "127.0.0.1"  # Change to the IP of the slave device
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Send the states in real-time
for i in range(len(sol_master.t)):
    state = sol_master.y[:, i]
    message = ",".join(map(str, state))  # Convert state to comma-separated string
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    time.sleep(dt)  # Simulate real-time sending

print("Master finished sending data.")
