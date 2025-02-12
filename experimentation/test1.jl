using LinearAlgebra
using StaticArrays
using CSV
using DataFrames
using DifferentialEquations

# Define the master (drive) chaotic system
function drive_system(x, p, t)
    # Example: Lorenz system or another chaotic system
    σ, ρ, β = p
    x1, x2, x3 = x
    dx1 = σ * (x2 - x1)
    dx2 = x1 * (ρ - x3) - x2
    dx3 = x1 * x2 - β * x3
    return SVector(dx1, dx2, dx3)
end

# Define the slave (response) system with control inputs
function response_system(y, p, t)
    # Synchronize with the drive system
    σ, ρ, β, x, u = p
    y1, y2, y3 = y
    u1, u2, u3 = u
    dy1 = σ * (y2 - y1) + u1
    dy2 = y1 * (ρ - y3) - y2 + u2
    dy3 = y1 * y2 - β * y3 + u3
    return [dy1, dy2, dy3]
end

# Backstepping controller
function backstepping_control(x, y, p, k)
    # Errors
    e1 = y[1] - x[1]
    e2 = y[2] - x[2]
    e3 = y[3] - x[3]

    u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
    u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
    u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 +( 2 * k[1])) * e2) - ((3 + k[1]) * e3)
    
    return SVector(u1, u2, u3), SVector(e1, e2, e3)
end

# Example usage
σ, ρ, β = 10.0, 28.0, 8/3
p = (σ, ρ, β)
k = (5.0, 5.0, 5.0) # Gains for the controller
tspan = (0.0, 7.0)
# Initial conditions
x0 = SVector(1.0, 1.0, 1.0)
y0 = [0.5, -0.65, 0.0]
prob = ODEProblem(drive_system, x0, tspan, p)
sol_master = solve(prob, Tsit5())
dx_values = []
xdata = reduce(hcat, sol_master.u) |> transpose
println(size(xdata))
dt = 0.01
dx_values = reduce(hcat, [drive_system(xdata[i, :], p, 1) for i in 1:size(xdata, 1)])'
n_steps = sol_master.t
y_traj = zeros(length(n_steps), 3)
x_traj = zeros(length(n_steps), 3)
e_traj = zeros(length(n_steps), 3)
println("herer")
for (i, t) in enumerate(n_steps)
    global y0
    x_t = xdata[i, :]  # Master state at time step i
            # Compute control input
    u_t, e_t = backstepping_control(x_t, y0, p, k)

    p_slave = (p..., x_t, u_t)
    prob_slave = ODEProblem(response_system, y0, (t, t+dt), p_slave)
    sol_slave = solve(prob_slave, Tsit5())
    y_traj[i, :] = sol_slave.u[end]
    x_traj[i, :] = x_t
    e_traj[i, :] = y_traj[i, :] - x_t
    y0 = y_traj[i, :]  # Update the response system state
end

println("done")
# Plot the synchronization error
using Plots

# Time values
println(size(y_traj))
# Labels for states
state_labels = ["x₁", "x₂", "x₃"]
response_labels = ["y₁", "y₂", "y₃"]
println(y_traj[1, :])
t_vals = sol_master.t  # Time values from the master system

for i in 1:3
    # Plot x_i
    plt_x = plot(t_vals, x_traj[:, i], label=state_labels[i], xlabel="Time", ylabel="State", title="Master System $(state_labels[i])", lw=2)
    savefig(plt_x, "master_system_$(state_labels[i]).png")

    # Plot y_i
    plt_y = plot(t_vals, y_traj[:, i], label=response_labels[i], xlabel="Time", ylabel="State", title="Response System $(response_labels[i])", lw=2)
    savefig(plt_y, "response_system_$(response_labels[i]).png")

    # Plot e_i
    plt_e = plot(t_vals, e_traj[:, i], label="e$(i)", xlabel="Time", ylabel="Error", title="Synchronization Error $(i)", lw=2)
    savefig(plt_e, "synchronization_error_$(i).png")
end

