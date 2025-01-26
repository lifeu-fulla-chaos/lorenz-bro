using LinearAlgebra
using StaticArrays
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
    σ, ρ, β = p
    e1 = y[1] - x[1]
    e2 = y[2] - x[2]
    e3 = y[3] - x[3]

    u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
    u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
    u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)

    return SVector(u1, u2, u3)
end

# Custom Tsit5 Solver Implementation
function tsit5_solver(dynamics!, u0, tspan, p; dt=0.01)
    t0, tf = tspan
    u = copy(u0)
    t = t0
    history = [(t, u)]  # Save initial state

    # Butcher tableau coefficients for Tsit5
    c = [0.0, 0.161, 0.327, 0.924, 0.25, 1.0]
    b = [0.174760, 0.551480, 0.198653, 0.0, 0.0, 0.0, 0.075107]
    b_hat = [0.157000, 0.0, 0.379471, 0.158220, 0.174325, 0.0, 0.0]

    a = [
        [],
        [0.161],
        [0.123, 0.204],
        [0.0, 0.25, 0.674],
        [0.161, 0.008, 0.065, -0.005],
        [0.0, 0.25, 0.5, 0.159, 0.011]
    ]

    while t < tf
        k = Vector{MVector{6, Float64}}(undef, 7)

        for i in 1:6
            k[i] = similar(MVector(u))
            temp = MVector(u)
            for j in 1:i-1
                temp += dt * a[i][j] * k[j]
            end
            dynamics!(k[i], temp, p, t + c[i] * dt)
        end

        u_next = MVector(u)
        for i in 1:6
            u_next += dt * b[i] * k[i]
        end

        u = SVector(u_next)  
        t += dt
        push!(history, (t, u))
    end

    return history
end

# Parameters and initial conditions
σ, ρ, β = 10.0, 28.0, 8/3
p = (σ, ρ, β)
k = (10.0, 10.0, 10.0)  # Gains for the controller
x0 = SVector(1.0, 0.0, 0.0)  # Drive system initial condition
y0 = SVector(0.5, -0.5, 0.0)  # Response system initial condition
u0 = vcat(x0, y0)
tspan = (0.0, 100.0)

# Define the coupled system dynamics
function dynamics!(du, u, p, t)
    x, y = u[1:3], u[4:6]
    u_control = backstepping_control(x, y, p, k)
    dx = drive_system(x, p, t)
    dy = response_system(y, x, u_control, p, t)
    du[1:3] .= dx
    du[4:6] .= dy
end

# Solve using the custom Tsit5 solver
dt = 0.01
solution = tsit5_solver(dynamics!, u0, tspan, p, dt=dt)

# Extract results
time = [t for (t, u) in solution]
states = [u for (t, u) in solution]
println(length(states))
println(length(states[1]))
# Extract individual components for x1, x2, x3 (drive system states)
x1 = [u[1] for u in states]  # Extract first component of each state (x1)
x2 = [u[2] for u in states]  # Extract second component of each state (x2)
x3 = [u[3] for u in states]  # Extract third component of each state (x3)

# Extract individual components for y1, y2, y3 (response system states)
y1 = [u[4] for u in states]  # Extract fourth component of each state (y1)
y2 = [u[5] for u in states]  # Extract fifth component of each state (y2)
y3 = [u[6] for u in states]  # Extract sixth component of each state (y3)

# Compute errors
e1 = y1 .- x1
e2 = y2 .- x2
e3 = y3 .- x3
total_error = sqrt.(e1.^2 .+ e2.^2 .+ e3.^2)
println(sum(total_error))
# Synchronization threshold
threshold = 1e-6
sync_index = findfirst(total_error .< threshold)

if sync_index !== nothing
    sync_time = time[sync_index]
    println("Synchronization achieved at t = $sync_time")
else
    println("Synchronization not achieved within the simulation time.")
end

# Plot results
p1 = plot(time, x1, label="x1", title="Synchronization: x1 vs y1", xlabel="Time", ylabel="State")
plot!(p1, time, y1, label="y1")
savefig(p1, "synchronization_x1_y1.png")
p2 = plot(time, x2, label="x2", title="Synchronization: x2 vs y2", xlabel="Time", ylabel="State")
plot!(p2, time, y2, label="y2")
savefig(p2, "synchronization_x2_y2.png")
p3 = plot(time, x3, label="x3", title="Synchronization: x3 vs y3", xlabel="Time", ylabel="State")
plot!(p3, time, y3, label="y3")
savefig(p3, "synchronization_x3_y3.png")
# Plot errors
p_error = plot(time, e1, label="e1 = y1 - x1", title="Synchronization Errors", xlabel="Time", ylabel="Error")
plot!(p_error, time, e2, label="e2 = y2 - x2")
plot!(p_error, time, e3, label="e3 = y3 - x3")
hline!([threshold], label="Threshold", linestyle=:dash)
savefig(p_error, "synchronization_errors.png")