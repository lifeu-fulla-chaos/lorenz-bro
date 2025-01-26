using LinearAlgebra
using StaticArrays
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
function response_system(y, x, u, p, t)
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
k = (10.0, 10.0, 10.0) # Gains for the controller

# Initial conditions
x0 = [1.0, 0.0, 0.0]  # Drive system initial condition
y0 = [0.5, -0.5, 0.0] # Response system initial condition

# Time span
tspan = (0.0, 100.0)

# Synchronization with DifferentialEquations.jl
using DifferentialEquations

# Define coupled systems with backstepping control
function dynamics!(du, u, p, t)
    x, y = u[1:3], u[4:6]  # Split state into drive and response systems
    σ, ρ, β = p

    # Compute control
    u_control = backstepping_control(x, y, p, k)

    # Drive system dynamics
    dx = drive_system(x, p, t)

    # Response system dynamics
    dy = response_system(y, x, u_control, p, t)

    # Combine derivatives
    du[1:3] = dx
    du[4:6] = dy
end

# Initial conditions and state
u0 = vcat(x0, y0)  # Combine master and slave initial states

# Solve the coupled system
prob = ODEProblem(dynamics!, u0, tspan, p)
sol = solve(prob, Tsit5())

# Plot synchronization
using Plots
# Extract time and variables
time = sol.t
x1 = sol[1, :]
y1 = sol[4, :]
x2 = sol[2, :]
y2 = sol[5, :]
x3 = sol[3, :]
y3 = sol[6, :]

# Create overlaid plots
p1 = plot(time, x1, label="x1", title="Synchronization: x1 vs y1", xlabel="Time", ylabel="State")
plot!(p1, time, y1, label="y1")

p2 = plot(time, x2, label="x2", title="Synchronization: x2 vs y2", xlabel="Time", ylabel="State")
plot!(p2, time, y2, label="y2")

p3 = plot(time, x3, label="x3", title="Synchronization: x3 vs y3", xlabel="Time", ylabel="State")
plot!(p3, time, y3, label="y3")

# Save plots
savefig(p1, "synchronization_x1_y1_reg.png")
savefig(p2, "synchronization_x2_y2_reg.png")
savefig(p3, "synchronization_x3_y3_reg.png")

# Extract time and compute errors
time = sol.t
e1 = sol[4, :] .- sol[1, :]  # y1 - x1
e2 = sol[5, :] .- sol[2, :]  # y2 - x2
e3 = sol[6, :] .- sol[3, :]  # y3 - x3
print(sum(e1[end-10:end]), sum(e2[end-10:end]), sum(e3[end-10:end]))
# Plot errors
p_error = plot(time, e1, label="e1 = y1 - x1", title="Synchronization Errors", xlabel="Time", ylabel="Error")
plot!(p_error, time, e2, label="e2 = y2 - x2")
plot!(p_error, time, e3, label="e3 = y3 - x3")

# Save plot
savefig(p_error, "synchronization_errors_reg.png")

