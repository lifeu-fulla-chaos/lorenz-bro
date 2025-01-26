using LinearAlgebra
using StaticArrays
using CSV
using DataFrames
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
k = (5.0, 5.0, 5.0) # Gains for the controller

# Initial conditions
x0 = [1.0, 0.0, 0.0]  # Drive system initial condition
y0 = [0.5, -0.65, 0.0] # Response system initial condition

# Time span
tspan = (0.0, 100.0)
xdata, ydata, controldata = [], [], []
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
    append!(xdata, [x])
    append!(ydata, [y])
    append!(controldata, [u_control])
end

# Initial conditions and state
u0 = vcat(x0, y0)  # Combine master and slave initial states

# Solve the coupled system
prob = ODEProblem(dynamics!, u0, tspan, p)
sol = solve(prob, Tsit5())
print(length(controldata))
# Save the data
df = DataFrame(x1=[x[1] for x in xdata], x2=[x[2] for x in xdata], x3=[x[3] for x in xdata],
    y1=[y[1] for y in ydata], y2=[y[2] for y in ydata], y3=[y[3] for y in ydata],
    u1=[u[1] for u in controldata], u2=[u[2] for u in controldata], u3=[u[3] for u in controldata])
CSV.write("data1.csv", df)
