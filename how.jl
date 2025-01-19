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
function backstepping_control(x, y, k)
    # Synchronization error
    e = y - x

    # Control input using backstepping
    u = -k .* e
    return u
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

# Plot master and slave system trajectories separately for x, y, and z components
plot(time, x_traj[1, :], label="Master x1", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (x)")
plot!(time, y_traj[1, :], label="Slave y1")
savefig("master_slave_system_trajectories_x.png")

plot(time, x_traj[2, :], label="Master x2", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (y)")
plot!(time, y_traj[2, :], label="Slave y2")
savefig("master_slave_system_trajectories_y.png")

plot(time, x_traj[3, :], label="Master x3", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (z)")
plot!(time, y_traj[3, :], label="Slave y3")
savefig("master_slave_system_trajectories_z.png")