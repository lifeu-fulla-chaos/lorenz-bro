using Sockets

function run_master()
    # Define the Lorenz system for the master
    function lorenz_master(x, p)
        σ, ρ, β = p
        dx = zeros(3)
        dx[1] = σ * (x[2] - x[1])
        dx[2] = x[1] * (ρ - x[3]) - x[2]
        dx[3] = x[1] * x[2] - β * x[3]
        return dx
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the master system
    x = [1.0, 1.0, 1.0]

    # Start server
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")

    while true
        # Accept a connection from the slave
        client = accept(server)
        println("Master: Slave connected.")

        for i in 1:n_steps
            # Update master system
            x += lorenz_master(x, p) * dt

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



