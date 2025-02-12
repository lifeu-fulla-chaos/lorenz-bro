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
function response_system(y, x, u, p)
    # Synchronize with the drive system
    σ, ρ, β = p
    y1, y2, y3 = y
    u1, u2, u3 = u
    dy1 = σ * (y2 - y1) + u1
    dy2 = y1 * (ρ - y3) - y2 + u2
    dy3 = y1 * y2 - β * y3 + u3
    return SVector(dy1, dy2, dy3)
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
    
    return SVector(u1, u2, u3)
end

# Example usage
σ, ρ, β = 10.0, 28.0, 8/3
p = (σ, ρ, β)
k = (5.0, 5.0, 5.0) # Gains for the controller
tspan = (0.0, 100.0)
# Initial conditions
x0 = SVector(1.0, 1.0, 1.0)
y0 = SVector(0.5, -0.65, 0.0)
prob = ODEProblem(drive_system, x0, tspan, p)
sol_master = solve(prob, RK4(), dt=0.01)
dx_values = []
xdata = sol_master.u
dx_values = reduce(hcat, [drive_system(xdata[i], p, 1) for i in 1:length(xdata)])'



# Define coupled systems with backstepping control
function dynamics!(du, u, p, t)
    x, y, dx1 = u[1:3], u[4:6], u[7:9]  # Split state into drive and response systems
    σ, ρ, β = p

    # Compute control
    u_control = backstepping_control(x, y, p, k)

    # Drive system dynamics
    dx = dx1
    # Response system dynamics
    dy = response_system(y, x, u_control, p)

    # Combine derivatives
    du[1:3] = dx
    du[4:6] = dy
end
# Initial conditions and state
u0 = vcat(x0, y0, dx_values[1, :])  # Combine master and slave initial states
# Solve the coupled system
prob = ODEProblem(dynamics!, u0, tspan, p)
sol = solve(prob, RK4(), dt=0.01)


# Plot the synchronization error
using Plots

# Extract state values properly
x_vals = reduce(hcat, sol_master.u) |> transpose
println(size(x_vals))
y_vals = hcat(sol.u...)'[:, 1:3] # Response system states (y1, y2, y3)
println(size(y_vals))
# Time values
t_vals = sol.t
t_vals1 = sol_master.t
# Labels for states
state_labels = ["x₁", "x₂", "x₃"]
response_labels = ["y₁", "y₂", "y₃"]

# Plot each state separately
for i in 1:3
    # Plot x_i
    plt_x = plot(t_vals1, x_vals[:, i], label=state_labels[i], xlabel="Time", ylabel="State", title="Master System $(state_labels[i])", lw=2)
    savefig(plt_x, "master_system_$(state_labels[i]).png")

    # Plot y_i
    plt_y = plot(t_vals, y_vals[:, i], label=response_labels[i], xlabel="Time", ylabel="State", title="Response System $(response_labels[i])", lw=2)
    savefig(plt_y, "response_system_$(response_labels[i]).png")
end
