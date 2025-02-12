using Sockets
using DifferentialEquations
using StaticArrays
using Plots
using CSV
using DataFrames

function run_slave()
    # Lorenz system for the slave with control input
    function lorenz_slave!(dy, y, p, t)
        σ, ρ, β, x_t, u_t = p  # Extract parameters and inputs
        dy[1] = σ * (y[2] - y[1]) + u_t[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u_t[2]
        dy[3] = y[1] * y[2] - β * y[3] + u_t[3]
    end

    # Define the backstepping control law
    function backstepping_control(x, y, p, k)
        σ, ρ, β = p
        e1, e2, e3 = y .- x  # Error between slave and master states

        u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
        u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)

        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    tspan = (0.0, dt)  # Solve step-by-step
    p = (10.0, 28.0, 8 / 3)  # Lorenz system parameters
    k = (5.0, 5.0, 5.0)  # Controller gains

    # Initial state of the slave system
    y0 = [5.0, 5.0, 5.0]

    println("Slave: Connecting to master...")
    
    while true
        client = try
            connect("127.0.0.1", 2000)
        catch
            println("Slave: Connection failed. Retrying...")
            sleep(2)
            continue
        end
        println("Slave: Connected to master.")

        # Receive master states (not dx/dt) from the master
        master_x = []
        while true
            line = try
                readline(client)
            catch
                ""
            end
            if isempty(line)
                break
            end

            x = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end
            push!(master_x, x)
        end
        close(client)  # Close connection after receiving all data

        if isempty(master_x)
            println("Slave: No data received. Exiting.")
            return
        end

        master_x = hcat(master_x...)'  # Convert to matrix
        n_steps = size(master_x, 1)

        # Arrays to store trajectories for plotting
        y_traj = zeros(3, n_steps)
        x_traj = zeros(3, n_steps)
        e_traj = zeros(3, n_steps)

        # Synchronization Loop
        for i in 1:n_steps
            x_t = master_x[i, :]  # Master state at time step i

            # Compute control input
            u_t, e_t = backstepping_control(x_t, y0, p, k)

            # Solve ODE for the next step
            p_slave = (p..., x_t, u_t)
            prob_slave = ODEProblem(lorenz_slave!, y0, tspan, p_slave)
            sol_slave = solve(prob_slave, Tsit5(), dtmax=dt)
            # Store values
            y_traj[:, i] = sol_slave.u[end]
            x_traj[:, i] = x_t
            e_traj[:, i] = y_traj[:, i] .- x_traj[:, i]
            y0 = y_traj[:, i]
        end
        # Plot synchronization and error
       
        time = 0:dt:(n_steps - 1) * dt
        for j in 1:3
            sync_plot = plot(time, x_traj[j, :], label="Master $(["x", "y", "z"][j])",
                            xlabel="Time", ylabel="$(["x", "y", "z"][j])",
                            title="Synchronization for $(["x", "y", "z"][j])")
            plot!(sync_plot, time, y_traj[j, :], label="Slave $(["x", "y", "z"][j])", linestyle=:dash)
            savefig(sync_plot, "synchronization_$(["x", "y", "z"][j]).png")

            error_plot = plot(time, e_traj[j, :], label="Error $(["x", "y", "z"][j])",
                            xlabel="Time", ylabel="Error",
                            title="Synchronization Error for $(["x", "y", "z"][j])")
            savefig(error_plot, "error_$(["x", "y", "z"][j]).png")
        end

        println("Slave: Synchronization complete. Exiting.")


        # Create a time column
        time = collect(0:dt:(n_steps - 1) * dt)

        # Combine all trajectories into a single DataFrame
        df = DataFrame(
            time = time,
            x_master = x_traj[1, :],  y_master = x_traj[2, :],  z_master = x_traj[3, :],
            x_slave  = y_traj[1, :],  y_slave  = y_traj[2, :],  z_slave  = y_traj[3, :],
            error_x  = e_traj[1, :],  error_y  = e_traj[2, :],  error_z  = e_traj[3, :]
        )

        # Save to a single CSV file
        CSV.write("synchronization_data.csv", df)

        println("Slave: Data saved to synchronization_data.csv")

        break
    end
end

run_slave()