using Sockets
using DifferentialEquations
using StaticArrays
using Plots

using LinearAlgebra


function run_synchronization()
    # Lorenz system for slave with backstepping control
    function lorenz_slave!(dy, y, p, t)
        σ, ρ, β, x, u = p  # p contains master state and control input
        
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
    end

    # Backstepping control law
    function backstepping_control(x, y, p)
        σ, ρ, β = p
        e = y .- x  # Error vector
        
        u1 = -σ * (e[2] - e[1]) + e[2]
        u2 = -ρ * e[1] + e[2] + (y[1] * y[3]) - (x[1] * x[3]) + e[3]
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + β * e[3] - (3 + 2k) * e[1] - (5 + 2k) * e[2] - (3 + k) * e[3]
        
        return SVector(u1, u2, u3), e
    end
    k = 5.0  # Backstepping gain
    p = (10.0, 28.0, 8 / 3)
    # Initial conditions
    y0 = [5.0, 5.0, 5.0]  # Slave initial state (different from master)
    dt = 0.001
    T = 7.0
    t_eval = 0:dt:T
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
        master_x = hcat(master_x...)'
        n_steps = length(t_eval)
        y_traj = zeros(3, n_steps)
        e_traj = zeros(3, n_steps)
        y_current = copy(y0)
        lowest = Inf
        minarg = -1

        for (i, t) in enumerate(t_eval)
            x = master_x[i, :] 
            u, e = backstepping_control(x, y_current, p)
            
            prob_slave = ODEProblem(lorenz_slave!, y_current, (t, t+dt), (p..., x, u))
            sol_slave = solve(prob_slave, Tsit5(), dtmax=dt)
            
            y_current = sol_slave.u[end]
            y_traj[:, i] = y_current
            e_traj[:, i] = e
            if norm(e) < lowest
                lowest = norm(e)
                minarg = i
            end
        end
        
        println("lowest", e_traj[:, minarg], norm(e_traj[:, minarg]))
        for j in 1:3
            # State trajectories
            p1 = plot(t_eval, [master_x[:, j] y_traj[j, :]], 
                 label=["Master $("xyz"[j])" "Slave $("xyz"[j])"],
                 xlabel="Time", ylabel="State", 
                 title="Master-Slave Trajectories: Component $("xyz"[j])")
            savefig(p1, "trajectory$("xyz"[j]).png")
            
            # Error plots
            p2 = plot(t_eval, e_traj[j, :], 
                 label="Error $("xyz"[j])", 
                 xlabel="Time", ylabel="Error",
                 title="Synchronization Error: Component $("xyz"[j])")
            savefig(p2, "error$("xyz"[j]).png")
            
        end

        println("All plots have been saved.")
        break
    end
end 
run_synchronization()
