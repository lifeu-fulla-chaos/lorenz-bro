# using Sockets
# using LinearAlgebra
# using Plots

# function run_slave()
#     # Define the Lorenz system for the slave with control input
#     function lorenz_slave(y, x, p, u)
#         σ, ρ, β = p
#         dy = zeros(3)
#         dy[1] = σ * (y[2] - y[1]) + u[1]
#         dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
#         dy[3] = y[1] * y[2] - β * y[3] + u[3]
#         return dy
#     end

#     # Define the backstepping control law
#     function backstepping_control(x, y, k)
#         e = y - x  # Synchronization error
#         u = -k .* e
#         return u
#     end

#     # Simulation parameters
#     dt = 0.01  # Time step
#     T = 10.0   # Total simulation time
#     n_steps = Int(T / dt)

#     # Lorenz system parameters
#     p = (10.0, 28.0, 8 / 3)

#     # Initial state of the slave system
#     y = [5.0, 5.0, 5.0]

#     # Gain for the backstepping controller
#     k = [10.0, 10.0, 10.0]

#     # Storage for trajectories
#     y_traj = zeros(3, n_steps)
#     x_traj = zeros(3, n_steps)
#     e_traj = zeros(3, n_steps)

#     # Connect to the master system
#     println("Slave: Connecting to master...")
#     client = connect("127.0.0.1", 2000)
#     println("Slave: Connected to master.")

#     # Real-time plotting setup
#     time = 0:dt:(n_steps - 1) * dt
#     plt1 = plot(title="Synchronization Error", xlabel="Time", ylabel="Error", legend=:bottomright)
#     plt2 = plot(title="State Trajectories", xlabel="Time", ylabel="States", legend=:bottomright)

#     for i in 1:n_steps
#         # Read master state
#         line = try
#             readline(client)
#         catch
#             ""
#         end

#         if isempty(line)
#             println("Slave: Received empty data.")
#             continue
#         end

#         x = try
#             parse.(Float64, split(line, ","))
#         catch
#             println("Slave: Error parsing data.")
#             continue
#         end

#         # Compute control input
#         u = backstepping_control(x, y, k)

#         # Update the slave system
#         y += lorenz_slave(y, x, p, u) * dt

#         # Store trajectories
#         x_traj[:, i] = x
#         y_traj[:, i] = y
#         e_traj[:, i] = y - x

#         # Update real-time plots every 100 steps
#         if i % 100 == 0
#             plot!(plt1, time[1:i], e_traj[:, 1:i]', label=["e1" "e2" "e3"], color=:auto)
#             plot!(plt2, time[1:i], x_traj[:, 1:i]', label=["Master x1" "Master x2" "Master x3"], color=:auto, linestyle=:solid)
#             plot!(plt2, time[1:i], y_traj[:, 1:i]', label=["Slave y1" "Slave y2" "Slave y3"], color=:auto, linestyle=:dash)
#         end
#     end

#     # Final plot save
#     savefig("synchronization_error.png")
#     savefig("state_trajectories.png")
#     println("Slave: Plots saved as 'synchronization_error.png' and 'state_trajectories.png'.")

#     close(client)
# end

# run_slave()


using Sockets
using LinearAlgebra
using Plots

function run_slave()
    # Define the Lorenz system for the slave with control input
    function lorenz_slave(y, x, p, u)
        σ, ρ, β = p
        dy = zeros(3)
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
        return dy
    end

    # Define the backstepping control law
    function backstepping_control(x, y, k)
        e = y - x  # Synchronization error
        u = -k .* e
        return u
    end

    # Simulation parameters
    dt = 0.01  # Time step
    T = 10.0   # Total simulation time
    n_steps = Int(T / dt)

    # Lorenz system parameters
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the slave system
    y = [5.0, 5.0, 5.0]

    # Gain for the backstepping controller
    k = [10.0, 10.0, 10.0]

    # Storage for trajectories
    y_traj = zeros(3, n_steps)
    x_traj = zeros(3, n_steps)
    e_traj = zeros(3, n_steps)

    # Connect to the master system
    println("Slave: Connecting to master...")
    client = connect("127.0.0.1", 2000)
    println("Slave: Connected to master.")

    # Time vector
    time = 0:dt:(n_steps - 1) * dt

    for i in 1:n_steps
        # Read master state
        line = try
            readline(client)
        catch
            ""
        end

        if isempty(line)
            println("Slave: Received empty data.")
            continue
        end

        x = try
            parse.(Float64, split(line, ","))
        catch
            println("Slave: Error parsing data.")
            continue
        end

        # Compute control input
        u = backstepping_control(x, y, k)

        # Update the slave system
        y += lorenz_slave(y, x, p, u) * dt

        # Store trajectories
        x_traj[:, i] = x
        y_traj[:, i] = y
        e_traj[:, i] = y - x
    end

    # Generate separate plots for x, y, z synchronization and errors
    for j in 1:3
        # Synchronization plot
        sync_plot = plot(time, x_traj[j, :], label="Master $(["x", "y", "z"][j])",
                         xlabel="Time", ylabel="$(["x", "y", "z"][j])",
                         title="Synchronization for $(["x", "y", "z"][j])")
        plot!(sync_plot, time, y_traj[j, :], label="Slave $(["x", "y", "z"][j])", linestyle=:dash)
        savefig(sync_plot, "synchronization_$(["x", "y", "z"][j]).png")

        # Error plot
        error_plot = plot(time, e_traj[j, :], label="Error $(["x", "y", "z"][j])",
                          xlabel="Time", ylabel="Error",
                          title="Synchronization Error for $(["x", "y", "z"][j])")
        savefig(error_plot, "error_$(["x", "y", "z"][j]).png")
    end

    println("Slave: Plots saved for synchronization and errors for x, y, z.")

    close(client)
end

run_slave()
