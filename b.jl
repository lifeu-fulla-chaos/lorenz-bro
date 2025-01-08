# Import necessary libraries
using LinearAlgebra

# Define the Lorenz system for the drive (master) system
function lorenz_master(x, p)
    σ, ρ, β = p
    dx = zeros(3)
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
    return dx
end

# Define the Lorenz system for the response (slave) system with control input u
function lorenz_slave(y, x, p, u)
    σ, ρ, β = p
    dy = zeros(3)
    dy[1] = σ * (y[2] - y[1]) + u[1]
    dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
    dy[3] = y[1] * y[2] - β * y[3] + u[3]
    return dy
end

# Define the backstepping control law
function backstepping_control(x, y, p, k)
    # Errors
    e1 = y[1] - x[1]
    e2 = y[2] - x[2]
    e3 = y[3] - x[3]

    # Lyapunov-based backstepping control
    u1 = -k[1] * e1
    u2 = -k[2] * e2 + σ * (y[2] - y[1]) - σ * (x[2] - x[1])
    u3 = -k[3] * e3 + y[1] * (ρ - y[3]) - y[2] - (x[1] * (ρ - x[3]) - x[2])
    
    return SVector(u1, u2, u3)
end

# Simulate the systems
dt = 0.01  # Time step
T = 10.0   # Total simulation time
n_steps = Int(T / dt)

# Parameters of the Lorenz system
p = (10.0, 28.0, 8 / 3)

# Initial conditions
x = [1.0, 1.0, 1.0]  # Initial state of master system
y = [5.0, 5.0, 5.0]  # Initial state of slave system

# Gain for the backstepping controller
k = [10.0, 10.0, 10.0]

# Storage for trajectories
x_traj = zeros(3, n_steps)
y_traj = zeros(3, n_steps)
e_traj = zeros(3, n_steps)

# Simulation loop
for i in 1:n_steps
    # Store current states
    x_traj[:, i] = x
    y_traj[:, i] = y
    e_traj[:, i] = y - x

    # Compute control input
    u = backstepping_control(x, y, k)

    # Update the master and slave systems using Euler integration
    global x += lorenz_master(x, p) * dt
    global y += lorenz_slave(y, x, p, u) * dt
end

# Plot results
using Plots

time = 0:dt:(n_steps - 1) * dt

# Plot synchronization error
plot(time, e_traj', label=["e1" "e2" "e3"], xlabel="Time", ylabel="Error", title="Synchronization Error")
savefig("synchronization_error.png")
