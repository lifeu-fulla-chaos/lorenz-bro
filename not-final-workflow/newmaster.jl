using Sockets
using DifferentialEquations
using StaticArrays
using Plots

function lorenz_master!(dx, x, p, t)
    σ, ρ, β = p
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
    return dx
end

# Simulation parameters
dt = 0.01
T = 100.0
tspan = (0.0, T)
p = (10.0, 28.0, 8 / 3)

# Initial state of the master system
x0 = [1.0, 1.0, 1.0]

# Solve the Lorenz system
prob = ODEProblem(lorenz_master!, x0, tspan, p)
sol_master = solve(prob, Tsit5(), dtmax=dt)

# Extract state values
time = sol_master.t
x_values = hcat(sol_master.u...)'  # Convert solution vectors to a matrix
dx_values = []
for i in 1:size(x_values, 1)
    dx = lorenz_master!(similar(x_values[i, :]), x_values[i, :], p, time[i])
    push!(dx_values, [dx])
end
dx_values = hcat(dx_values...)'

function run_master()
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")

    while true
        client = accept(server)
        println("Master: Slave connected.")

        for i in 1:length(time)
            x = x_values[i, :]
            append!(x, dx_values[i])
            write(client, join(x, ",") * "\n")  # Convert to CSV format
            flush(client)
        end

        println("Master: Finished sending state data to slave.")
        close(client)
        break
    end

    close(server)
end

run_master()