using DifferentialEquations, LinearAlgebra, QuadGK, Plots

# Lorenz system parameters
const σ = 10.0
const β = 8/3
const ρ = 28.0

# Master Lorenz system
function master_lorenz!(du, u, p, t)
    du[1] = σ * (u[2] - u[1])
    du[2] = u[1] * (ρ - u[3]) - u[2]
    du[3] = u[1] * u[2] - β * u[3]
end

# Slave Lorenz system with Adaptive control
function slave_lorenz_adaptive!(du, u, p, t)
    x1, x2, x3 = u[1:3]  # Slave states
    xm1, xm2, xm3 = u[4:6]  # Master states
    u_control = p[1](u, t)  # Adaptive control input
    
    du[1] = σ * (x2 - x1) + u_control[1]
    du[2] = x1 * (ρ - x3) - x2 + u_control[2]
    du[3] = x1 * x2 - β * x3 + u_control[3]
    
    du[4] = σ * (xm2 - xm1)
    du[5] = xm1 * (ρ - xm3) - xm2
    du[6] = xm1 * xm2 - β * xm3
end

# Adaptive control design
function adaptive_control(lambda, gamma)
    # Adaptive control law (proportional feedback)
    return (u, t) -> begin
        e = u[1:3] - u[4:6]  # Synchronization error
        # Adaptive control law based on the error
        u_control = -lambda .* e .- gamma .* sign.(e)  # Basic adaptive control (could be modified)
        return u_control
    end
end

# Active Sliding Mode Control (ASMC)
function asmc_control(lambda, k)
    return (u, t) -> begin
        s = u[1:3] - u[4:6]  # Synchronization error
        return -lambda .* s - k .* sign.(s)  # Sliding mode + active term
    end
end

# Energy consumption calculation
function energy_consumption(sol)
    u_control_squared = [sum(sol[i, :] .^ 2) for i in 1:3]
    return quadgk(t -> sum([interp(sol.t, u_control_squared[j], t) for j in 1:3]), sol.t[1], sol.t[end])[1]
end

# Initial conditions
u0 = [1.0, 1.0, 1.0, 1.1, 1.1, 1.1]

# Time span
tspan = (0.0, 10.0)

# Adaptive Control setup
lambda_adaptive = [10, 10, 10]  # Adaptation gain
gamma_adaptive = [5, 5, 5]      # Sliding mode gain for adaptive control
adaptive_ctrl = adaptive_control(lambda_adaptive, gamma_adaptive)
prob_adaptive = ODEProblem(slave_lorenz_adaptive!, u0, tspan, [adaptive_ctrl])
sol_adaptive = solve(prob_adaptive, Tsit5())
energy_adaptive = energy_consumption(sol_adaptive)

# ASMC setup
asmc_ctrl = asmc_control([10, 10, 10], [5, 5, 5])
prob_asmc = ODEProblem(slave_lorenz_asmc!, u0, tspan, [asmc_ctrl])
sol_asmc = solve(prob_asmc, Tsit5())
energy_asmc = energy_consumption(sol_asmc)

# Print results
println("Adaptive Control Total Time: ", sol_adaptive.t[end], " Energy Consumption: ", energy_adaptive)
println("ASMC Total Time: ", sol_asmc.t[end], " Energy Consumption: ", energy_asmc)

# Plot results
plot(sol_adaptive, vars=(1, 4), label=["Slave x1 (Adaptive)" "Master x1"], title="Lorenz Synchronization Adaptive Control")
plot(sol_asmc, vars=(1, 4), label=["Slave x1 (ASMC)" "Master x1"], title="Lorenz Synchronization ASMC")
