using Sockets
using LinearAlgebra

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
    dt = 0.01  # Time step
    T = 10.0   # Total simulation time
    n_steps = Int(T / dt)

    # Lorenz system parameters
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the master system
    x = [1.0, 1.0, 1.0]

    # Open a TCP server to send master state
    server = listen(2000)

    println("Master: Waiting for connection...")
    client = accept(server)
    println("Master: Client connected.")

    for _ in 1:n_steps
        # Update the master system
        x += lorenz_master(x, p) * dt

        # Send current state to the slave system
        write(client, join(x, ",") * "\n")
        sleep(dt)
    end

    close(client)
    close(server)
end

run_master()

