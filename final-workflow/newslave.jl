using Sockets
using DifferentialEquations
using StaticArrays
using Plots
using CSV
using DataFrames

function run_slave()
    # Lorenz system for the slave with control input
    function lorenz_slave(y, p, t)
        σ, ρ, β, u_t = p  # Extract parameters and inputs
        dy1 = σ * (y[2] - y[1]) + u_t[1]
        dy2 = y[1] * (ρ - y[3]) - y[2] + u_t[2]
        dy3 = y[1] * y[2] - β * y[3] + u_t[3]
        return SVector(dy1, dy2, dy3)
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

    function dynamics!(du, u, p, t)
        x, y, dx = u[1:3], u[4:6], u[7:9]  # Split state into drive and response systems
        σ, ρ, β = p

        # Compute control input
        u_t, e_t = backstepping_control(x, y, p, k)

        # Drive system dynamics
        p = (σ, ρ, β, u_t)
        # Response system dynamics
        dy = lorenz_slave(y, p, t)

        # Combine the results
        du[1:3] .= dx
        du[4:6] .= dy
    end

    # Simulation parameters
    dt = 0.01
    T = 7.0
    tspan = (0.0, T)  # Solve step-by-step
    p = (10.0, 28.0, 8 / 3)  # Lorenz system parameters
    k = (10.0, 10.0, 10.0)  # Controller gains

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
        master_dx = []
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
            push!(master_x, x[1:3])
            push!(master_dx, x[4:6])
        end
        close(client)  # Close connection after receiving all data

        if isempty(master_x)
            println("Slave: No data received. Exiting.")
            return
        end

        master_x = hcat(master_x...)'  # Convert to matrix
        master_dx = hcat(master_dx...)'  # Convert to matrix
        n_steps = size(master_x, 1)

        # Arrays to store trajectories for plotting

        u0 = vcat(master_x[1, :], y0, master_dx[1, :])  # Initial state for the slave system
        prob = ODEProblem(dynamics!, u0, tspan, p)
        sol = solve(prob, Tsit5())
        # Plot synchronization and error
        y_traj = hcat(sol.u...)'[:, 4:6]
        println(size(y_traj))
        x_traj = master_x
        print(size(x_traj))
        e_traj = y_traj .- x_traj

        time = 0:dt:(n_steps - 1) * dt
        for j in 1:3
            sync_plot = plot(time, x_traj[:, j], label="Master $(["x", "y", "z"][j])",
                            xlabel="Time", ylabel="$(["x", "y", "z"][j])",
                            title="Synchronization for $(["x", "y", "z"][j])")
            plot!(sync_plot, time, y_traj[:, j], label="Slave $(["x", "y", "z"][j])", linestyle=:dash)
            savefig(sync_plot, "synchronization_$(["x", "y", "z"][j]).png")

            error_plot = plot(time, e_traj[:, j], label="Error $(["x", "y", "z"][j])",
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