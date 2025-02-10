using Sockets
using DifferentialEquations
using StaticArrays
using Plots

function lorenz_master!(dx, x, p)
    σ, ρ, β = p
    dx[1] = σ * (x[2] - x[1])
    dx[2] = x[1] * (ρ - x[3]) - x[2]
    dx[3] = x[1] * x[2] - β * x[3]
    return SVector(dx...)  # Returns a static vector
end

# Simulation parameters
dt = 0.01
T = 7.0
p = (10.0, 28.0, 8 / 3)

# Initial state of the master system
x0 = [1.0, 1.0, 1.0]
x = [x0]

for i in 1:81
    new_state = lorenz_master!(zeros(3), x[end], p)
    push!(x, collect(new_state))  # Convert SVector to standard Vector{Float64}
end

# Convert `x` to a matrix for plotting
x_mat = hcat(x...)'
time = 1:size(x_mat, 1)  # Time indices
println(x)
# Create separate plots
p1 = plot(time, x_mat[:, 1], label="x", xlabel="Time", ylabel="State", title="Lorenz System - X")
p2 = plot(time, x_mat[:, 2], label="y", xlabel="Time", ylabel="State", title="Lorenz System - Y")
p3 = plot(time, x_mat[:, 3], label="z", xlabel="Time", ylabel="State", title="Lorenz System - Z")

# Save plots
savefig(p1, "lorenz_x.png")
savefig(p2, "lorenz_y.png")
savefig(p3, "lorenz_z.png")

# Display plots together
# plot(p1, p2, p3, layout=(3,1), size=(600,800))

#     # Start server
#     server = listen(2000)
#     println("Master: Server started. Waiting for connections...")



#     while true
#         # Accept a connection from the slave
#         client = accept(server)
#         println("Master: Slave connected.")

#         for i in 1:length(sol_master.t)  # Loop through actual time steps
#             # Get the state of the master system at the i-th time step
#             x = sol_master.u[i]  # sol_master.u[i] contains the state vector at time sol_master.t[i]

#             # Send master state to slave
#             write(client, join(x, ",") * "\n")
#             flush(client)
#         end

#         println("Master: Finished sending data to slave.")
#         close(client)
#     end

#     close(server)
# end

# run_master()
