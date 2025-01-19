using Sockets
using DifferentialEquations

function run_master()
    # Define the Lorenz system for the master as an ODE
    function lorenz_master!(dx, x, p, t)
        σ, ρ, β = p
        dx[1] = σ * (x[2] - x[1])
        dx[2] = x[1] * (ρ - x[3]) - x[2]
        dx[3] = x[1] * x[2] - β * x[3]
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the master system
    x0 = [1.0, 1.0, 1.0]

    # Start server
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")

    # ODEProblem setup for the master system
    prob_master = ODEProblem(lorenz_master!, x0, (0.0, T), p)
    sol_master = solve(prob_master, Tsit5())

    while true
        # Accept a connection from the slave
        client = accept(server)
        println("Master: Slave connected.")

        for i in 1:length(sol_master.t)  # Loop through actual time steps
            # Get the state of the master system at the i-th time step
            x = sol_master.u[i]  # sol_master.u[i] contains the state vector at time sol_master.t[i]

            # Send master state to slave
            write(client, join(x, ",") * "\n")
            flush(client)
        end

        println("Master: Finished sending data to slave.")
        close(client)
    end

    close(server)
end

run_master()
