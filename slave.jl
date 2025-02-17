using Sockets
using DifferentialEquations
using StaticArrays
using Plots
using CSV
using DataFrames

function run_slave()
    function lorenz_slave!(dy, y, p, t)
        σ, ρ, β, x_t, u_t = p  
        dy[1] = σ * (y[2] - y[1]) + u_t[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u_t[2]
        dy[3] = y[1] * y[2] - β * y[3] + u_t[3]
    end

    function backstepping_control(x, y, p, k)
        σ, ρ, β = p
        e1, e2, e3 = y .- x  

        u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
        u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)

        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    dt = 0.0001
    p = (10.0, 28.0, 8 / 3)  
    k = (5.0, 5.0, 5.0)  

    y0 = [5.0, 5.0, 5.0]

    while true
        client = try
            connect("127.0.0.1", 2000)
        catch
            sleep(2)
            continue
        end

        y_traj = []
        x_traj = []
        e_traj = []
        time = []

        while true
            line = try
                readline(client)
            catch
                ""
            end
            if isempty(line)
                break
            end

            x_t = try
                parse.(Float64, split(line, ","))
            catch
                continue
            end

            push!(time, length(time) * dt)

            u_t, e_t = backstepping_control(x_t, y0, p, k)

            p_slave = (p..., x_t, u_t)
            prob_slave = ODEProblem(lorenz_slave!, y0, (0.0, dt), p_slave)
            sol_slave = solve(prob_slave, Tsit5(), dtmax=dt)

            push!(y_traj, sol_slave.u[end])
            push!(x_traj, x_t)
            push!(e_traj, y_traj[end] .- x_traj[end])

            y0 = sol_slave.u[end]

            if length(time) % 80 == 0
                # Find the index of the minimum synchronization error
                err_magnitudes = sum(abs.(e_traj), dims=2)[:, 1]
                min_index = argmin(err_magnitudes)

                # Use the best matching state as the new initial condition
                y0 = y_traj[min_index]

                # Clear old trajectory data for the next segment
                if length(time) > 130  # Retain only the last 50 steps
                    y_traj = y_traj[(min_index - 50 + 1):end]
                    x_traj = x_traj[(min_index - 50 + 1):end]
                    e_traj = e_traj[(min_index - 50 + 1):end]
                    time = time[(min_index - 50 + 1):end]
                end
            end
        end

        close(client)  

        y_traj = hcat(y_traj...)'
        x_traj = hcat(x_traj...)'
        e_traj = hcat(e_traj...)'

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

        df = DataFrame(
            time = time,
            x_master = x_traj[:, 1], y_master = x_traj[:, 2], z_master = x_traj[:, 3],
            x_slave  = y_traj[:, 1], y_slave  = y_traj[:, 2], z_slave  = y_traj[:, 3],
            error_x  = e_traj[:, 1], error_y  = e_traj[:, 2], error_z  = e_traj[:, 3]
        )

        CSV.write("synchronization_data.csv", df)

        break
    end
end

run_slave()
