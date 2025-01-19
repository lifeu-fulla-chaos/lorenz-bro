using LinearAlgebra
using DifferentialEquations
using Plots

# Define the Lorenz system for the drive (master) system
function lorenz_master!(dx, x, p, t)
    σ, ρ, β = p
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
end

# Define the Lorenz system for the response (slave) system with control input u
function lorenz_slave!(dy, y, p, t, x, k)
    σ, ρ, β = p
    e = y - x
    u = -k .* e
    dy[1] = σ * (y[2] - y[1]) + u[1]
    dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
    dy[3] = y[1] * y[2] - β * y[3] + u[3]
end

# Parameters of the Lorenz system
p = (10.0, 28.0, 8 / 3)

# Initial conditions
x0 = [1.0, 1.0, 1.0]  # Initial state of master system
y0 = [5.0, 5.0, 5.0]  # Initial state of slave system

# Gain for the backstepping controller
k = [10.0, 10.0, 10.0]

# Time span
tspan = (0.0, 10.0)

# Define the problem for the master system
master_prob = ODEProblem(lorenz_master!, x0, tspan, p)

# Define the problem for the slave system
slave_prob = ODEProblem((dy, y, p, t) -> lorenz_slave!(dy, y, p, t, x0, k), y0, tspan, p)

# Solve the problems using the Tsit5 solver
master_sol = solve(master_prob, Tsit5())
slave_sol = solve(slave_prob, Tsit5())

# Interpolate the solutions to a common time grid
common_time = range(tspan[1], tspan[2], length=length(master_sol.t))  # Common time grid

# Define interpolation functions
master_interp = t -> master_sol(t)
slave_interp = t -> slave_sol(t)

# Evaluate trajectories at the common time points
x_traj = [master_interp(t) for t in common_time]
y_traj = [slave_interp(t) for t in common_time]

# Compute synchronization error
e_traj = [y - x for (x, y) in zip(x_traj, y_traj)]

# Convert trajectory arrays to matrices for plotting
x_matrix = hcat(x_traj...)
y_matrix = hcat(y_traj...)
e_matrix = hcat(e_traj...)

# Plot synchronization error
plot(common_time, e_matrix', label=["e1" "e2" "e3"], xlabel="Time", ylabel="Error", title="Synchronization Error")
savefig("synchronization_error.png")

# Plot master and slave system trajectories separately for x, y, and z components
plot(common_time, x_matrix[1, :], label="Master x1", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (x)")
plot!(common_time, y_matrix[1, :], label="Slave y1")
savefig("master_slave_system_trajectories_x.png")

plot(common_time, x_matrix[2, :], label="Master x2", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (y)")
plot!(common_time, y_matrix[2, :], label="Slave y2")
savefig("master_slave_system_trajectories_y.png")

plot(common_time, x_matrix[3, :], label="Master x3", xlabel="Time", ylabel="State", title="Master and Slave System Trajectories (z)")
plot!(common_time, y_matrix[3, :], label="Slave y3")
savefig("master_slave_system_trajectories_z.png")
