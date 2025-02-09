using LinearAlgebra
using StaticArrays
#using CSV
using DataFrames
using DifferentialEquations
using Plots

# Define the master (drive) chaotic system
function drive_system(x, p, t)
    σ, ρ, β = p
    x1, x2, x3 = x
    dx1 = σ * (x2 - x1)
    dx2 = x1 * (ρ - x3) - x2
    dx3 = x1 * x2 - β * x3
    return SVector(dx1, dx2, dx3)
end

# Define the slave (response) system with control inputs
function response_system(y, x, u, p, t)
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
    e1 = y[1] - x[1]
    e2 = y[2] - x[2]
    e3 = y[3] - x[3]

    u1 = -p[1] * (y[2] - y[1] - x[2] + x[1]) + e2
    u2 = -p[2] * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
    u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (p[3] * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)
    
    return SVector(u1, u2, u3)
end

# Parameters
σ, ρ, β = 10.0, 28.0, 8/3
p = (σ, ρ, β)
k = (5.0, 5.0, 5.0)

# Initial conditions
x0 = [1.0, 0.0, 0.0]
y0 = [0.5, -0.65, 0.0]

# Time step settings
tspan_step = (0.0, 0.1)
num_steps = 100

df = DataFrame()
xdata_all, ydata_all = [], []

# Loop over time steps
for step in 1:num_steps
    global x0, y0
    xdata, ydata, controldata = [], [], []
    
    function dynamics!(du, u, p, t)
        x, y = u[1:3], u[4:6]
        u_control = backstepping_control(x, y, p, k)
        dx = drive_system(x, p, t)
        dy = response_system(y, x, u_control, p, t)
        du[1:3] = dx
        du[4:6] = dy
        append!(xdata, [x])
        append!(ydata, [y])
        append!(controldata, [u_control])
    end
    
    # Solve the system
    u0 = vcat(x0, y0)
    prob = ODEProblem(dynamics!, u0, tspan_step, p)
    sol = solve(prob, Tsit5())
    
    # Extract final state as next initial condition
    x0 = sol.u[end][1:3]
    y0 = sol.u[end][4:6]
    
    append!(xdata_all, [x0])  # Only store final state per step
    append!(ydata_all, [y0])  # Only store final state per step

    # Store results in DataFrame
    temp_df = DataFrame(x1=[x[1] for x in xdata], x2=[x[2] for x in xdata], x3=[x[3] for x in xdata],
                         y1=[y[1] for y in ydata], y2=[y[2] for y in ydata], y3=[y[3] for y in ydata],
                         u1=[u[1] for u in controldata], u2=[u[2] for u in controldata], u3=[u[3] for u in controldata])
    append!(df, temp_df)
end

# Save data
#CSV.write("data1.csv", df)

# Plot results
plot(1:length(xdata_all), [x[1] for x in xdata_all], label="x1 (Master)")
plot!(1:length(ydata_all), [y[1] for y in ydata_all], label="y1 (Slave)", linestyle=:dash)
title!("Chaotic System Synchronization")
xlabel!("Time Step")
ylabel!("State Values")
savefig("synchronization_plot.png")
