using Sockets
using DifferentialEquations
using StaticArrays
using Plots
using CSV
using DataFrames

function run_slave()
    # Lorenz system for the slave with control input
    function lorenz_slave!(dy, y, p, u_control)
        σ, ρ, β = p  
        dy[1] = σ * (y[2] - y[1]) + u_control[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u_control[2]
        dy[3] = y[1] * y[2] - β * y[3] + u_control[3]
    end

    # Backstepping control law
    function backstepping_control(x, y, p, k)
        σ, ρ, β = p
        e1, e2, e3 = y .- x  # Synchronization error

        u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
        u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)

        return SVector(u1, u2, u3)
    end

    # Slave dynamics using master data
    function dynamics!(du, u, p, t)
        x, dx, y = u[1:3], u[4:6], u[7:9]
        u_control = backstepping_control(x, y, p, k)

        dx_master = dx  # Master system derivatives are provided
        dy_slave = zeros(3)
        lorenz_slave!(dy_slave, y, p, u_control)

        du[1:3] .= dx_master
        du[4:6] .= dy_slave
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    tspan = (0.0, dt)  # Step-by-step integration
    p = (10.0, 28.0, 8 / 3)  # Lorenz parameters
    k = (5.0, 5.0, 5.0)  # Control gains

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

        # Receive master states and derivatives
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

            data = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end
            push!(master_x, data[1:3])  # First three are x
            push!(master_dx, data[4:6])  # Last three are dx
        end
        close(client)  # Close connection after receiving all data

        if isempty(master_x) || isempty(master_dx)
            println("Slave: No data received. Exiting.")
            return
        end

        master_x = hcat(master_x...)'  # Convert to matrix
        master_dx = hcat(master_dx...)'
        n_steps = size(master_x, 1)

        # Arrays to store trajectories
        y_traj = zeros(3, n_steps)
        x_traj = zeros(3, n_steps)
        dx_traj = zeros(3, n_steps)
        e_traj = zeros(3, n_steps)

        # Synchronization Loop
        for i in 1:n_steps
            x_t = master_x[i, :]
            dx_t = master_dx[i, :]

            # Solve ODE for one step
            u0 = vcat(x_t, dx_t, y0)  # Initial conditions: x, dx, y
            prob = ODEProblem(dynamics!, u0, tspan, p)
            sol = solve(prob, Tsit5(), dtmax=dt)

            # Store values
            y_traj[:, i] = sol.u[end][4:6]
            x_traj[:, i] = x_t
            dx_traj[:, i] = dx_t  # Fix dx storage
            e_traj[:, i] = y_traj[:, i] - x_traj[:, i]  # Error correction

            # Update initial state for next iteration
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

        # Save to CSV
        df = DataFrame(
            time = time,
            x_master = x_traj[1, :],  y_master = x_traj[2, :],  z_master = x_traj[3, :],
            dx_master = dx_traj[1, :], dy_master = dx_traj[2, :], dz_master = dx_traj[3, :],
            x_slave  = y_traj[1, :],  y_slave  = y_traj[2, :],  z_slave  = y_traj[3, :],
            error_x  = e_traj[1, :],  error_y  = e_traj[2, :],  error_z  = e_traj[3, :]
        )

        CSV.write("synchronization_data.csv", df)
        println("Slave: Data saved to synchronization_data.csv")

        break
    end
end

run_slave()
