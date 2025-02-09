using Sockets
using DifferentialEquations
using StaticArrays
using Plots

function run_slave()
    # Define the Lorenz system for the slave with control input
    function lorenz_slave!(dy, y, p, t)
        σ, ρ, β, x, u = p  # Extract parameters and control input from p
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
    end

    # Define the backstepping control law
    function backstepping_control(x, y, p, k)
        σ, ρ, β = p
        e1 = y[1] - x[1]
        e2 = y[2] - x[2]
        e3 = y[3] - x[3]

        u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
        u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 +( 2 * k[1])) * e2) - ((3 + k[1]) * e3)
        
        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    # Simulation parameters
    dt = 0.01
    T = 7.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)  # Parameters for the Lorenz system
    k = (5.0, 5.0, 5.0)  # Gains for the controller

    # Initial state of the slave system
    y0 = [5.0, 5.0, 5.0]

    # Arrays to store trajectories for plotting
    y_traj = zeros(3, n_steps)
    x_traj = zeros(3, n_steps)
    e_traj = zeros(3, n_steps)

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

        # ODEProblem setup for the slave system
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

            # Compute control input and error
            u_control = backstepping_control(x, y0, p, k)

            # Bundle x, u, and parameters into a tuple
            p_slave = (p..., x, u_control)  # Combine parameters and inputs

            # Define the ODE problem with the new parameter structure
            prob_slave = ODEProblem(lorenz_slave!, y0, (0.0, T), p_slave)

            # Solve the ODE system for the slave
            sol_slave = solve(prob_slave, Tsit5())

            y_traj[:, i] = sol_slave[1:3, i]
            x_traj[:, i] = x
            e_traj[:, i] = sol_slave[1:3, i] .- x
        end

        close(client)  # Close connection after the simulation
        break
    end

    # Generate separate plots for synchronization and errors for x, y, z
    time = 0:dt:(n_steps - 1) * dt
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
end

run_slave()
