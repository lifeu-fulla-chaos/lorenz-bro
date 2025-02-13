using DifferentialEquations
using Plots
using StaticArrays

# System parameters - global constants
const σ = 10.0
const ρ = 28.0
const β = 8/3
const k = 5.0  # Backstepping gain

# Simulation parameters - global constants
const dt = 0.0001
const T = 10.0
const tspan = (0.0, T)
const t_eval = 0:dt:T

function run_synchronization()
    # Lorenz system for master
    function lorenz_master!(dx, x, p, t)
        dx[1] = σ * (x[2] - x[1])
        dx[2] = x[1] * (ρ - x[3]) - x[2]
        dx[3] = x[1] * x[2] - β * x[3]
    end

    # Lorenz system for slave with backstepping control
    function lorenz_slave!(dy, y, p, t)
        x, u = p  # p contains master state and control input
        
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
    end

    # Backstepping control law
    function backstepping_control(x, y)
        e = y .- x  # Error vector
        
        u1 = -σ * (e[2] - e[1]) + e[2]
        u2 = -ρ * e[1] + e[2] + (y[1] * y[3]) - (x[1] * x[3]) + e[3]
        u3 = (-y[1] * y[2]) + (x[1] * x[2]) + β * e[3] - (3 + 2k) * e[1] - (5 + 2k) * e[2] - (3 + k) * e[3]
        
        return SVector(u1, u2, u3), e
    end

    # Initial conditions
    x0 = [1.0, 1.0, 1.0]  # Master initial state
    y0 = [5.0, 5.0, 5.0]  # Slave initial state (different from master)

    # Solve master system
    prob_master = ODEProblem(lorenz_master!, x0, tspan)
    sol_master = solve(prob_master, Tsit5(), saveat=dt)

    # Arrays to store trajectories and errors
    n_steps = length(t_eval)
    y_traj = zeros(3, n_steps)
    e_traj = zeros(3, n_steps)
    y_current = copy(y0)

    # Simulate slave system with backstepping control
    for (i, t) in enumerate(t_eval)
        x = sol_master(t)  # Get master state at current time
        
        # Calculate control input
        u, e = backstepping_control(x, y_current)
        
        # Solve slave system for one time step
        prob_slave = ODEProblem(lorenz_slave!, y_current, (t, t+dt), (x, u))
        sol_slave = solve(prob_slave, Tsit5())
        
        # Store results
        y_current = sol_slave.u[end]
        y_traj[:, i] = y_current
        e_traj[:, i] = e
    end

    println("Generating plots...")
    
    # Plotting trajectories and errors
    for j in 1:3
        # State trajectories
        p1 = plot(t_eval, [sol_master[j, :] y_traj[j, :]], 
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

    # 3D phase space plot
    p3 = plot3d(
        1, 
        xlabel="x", ylabel="y", zlabel="z",
        title="Master-Slave Phase Space"
    )

    # Add master trajectory
    plot3d!(p3, 
        sol_master[1, :], sol_master[2, :], sol_master[3, :],
        label="Master",
        color=:blue
    )

    # Add slave trajectory
    plot3d!(p3,
        y_traj[1, :], y_traj[2, :], y_traj[3, :],
        label="Slave",
        color=:red
    )

    savefig(p3, "phase_space.png")
    println("All plots have been saved.")
end

# Run the synchronization
run_synchronization()