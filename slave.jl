using Sockets
using LinearAlgebra
using Plots
using StaticArrays

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
    function backstepping_control(x, y, p, k)
        # Errors
        e1 = y[1] - x[1]
        e2 = y[2] - x[2]
        e3 = y[3] - x[3]

        # Lyapunov-based backstepping control
        u1 = -k[1] * e1
        u2 = -k[2] * e2 + p[1] * (y[2] - y[1]) - p[1] * (x[2] - x[1])
        u3 = -k[3] * e3 + y[1] * (p[2] - y[3]) - y[2] - (x[1] * (p[2] - x[3]) - x[2])

        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)
    k = (5.0, 5.0, 5.0)  # Gains for the controller

    # Initial state of the slave system
    y = [5.0, 5.0, 5.0]

    # Arrays to store trajectories for plotting
    y_traj = zeros(3, n_steps)
    x_traj = zeros(3, n_steps)
    e_traj = zeros(3, n_steps)
    e = 100
    # Connect to the master system
    println("Slave: Connecting to master...")
    while true
        client = try
            connect("127.0.0.1", 2000)
        catch
            println("Slave: Connection failed. Retrying...")
            sleep(2)  # Wait before retrying
            continue
        end
        println("Slave: Connected to master.")
        i = 1
        while norm(e) > 1e-3
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

            # Compute control input and error
            u_control, e = backstepping_control(x, y, p, k)

            # Update the slave system
            y += lorenz_slave(y, x, p, u_control) * dt

            # Store trajectories and errors
            x_traj[:, i] = x
            y_traj[:, i] = y
            e_traj[:, i] = e
            i += 1
            write(client, "not")
            flush(client)
        end
        write(client, "done")
        flush(client)
        close(client)  # Close connection after the simulation
        break
    end
    print(e_traj)
    # Generate separate plots for synchronization and errors for x, y, z
    time = 0:dt:(n_steps - 1) * dt
    # for j in 1:3
    #     # Synchronization plot
    #     sync_plot = plot(time, x_traj[j, :], label="Master $(["x", "y", "z"][j])",
    #                     xlabel="Time", ylabel="$(["x", "y", "z"][j])",
    #                     title="Synchronization for $(["x", "y", "z"][j])")
    #     plot!(sync_plot, time, y_traj[j, :], label="Slave $(["x", "y", "z"][j])", linestyle=:dash)
    #     savefig(sync_plot, "synchronization_$(["x", "y", "z"][j]).png")

    #     # Error plot
    #     error_plot = plot(time, e_traj[j, :], label="Error $(["x", "y", "z"][j])",
    #                     xlabel="Time", ylabel="Error",
    #                     title="Synchronization Error for $(["x", "y", "z"][j])")
    #     savefig(error_plot, "error_$(["x", "y", "z"][j]).png")
    # end
end

run_slave()