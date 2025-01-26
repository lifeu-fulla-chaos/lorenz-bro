using Sockets

function run_master()
    function lorenz(x, p)
        σ, ρ, β = p
        dx = zeros(3)
        dx[1] = σ * (x[2] - x[1])
        dx[2] = x[1] * (ρ - x[3]) - x[2]
        dx[3] = x[1] * x[2] - β * x[3]
        return dx
    end

    # Parameters
    dt = 0.01
    p = (10.0, 28.0, 8 / 3)
    x_master = [1.0, 1.0, 1.0]

    # Start server
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")
    client = accept(server)
    println("Master: Slave connected.")

    # Synchronization phase
    for _ in 1:10
        x_master += lorenz(x_master, p) * dt
        write(client, join(x_master, ",") * "\n")
        print("Master state sent : $x_master\n")
        sleep(dt)
        sleep(1)
    end
    println("Master: Synchronization complete.")

    # Continuous loop for communication
    while true
        # Evolve Lorenz system
        x_master += lorenz(x_master, p) * dt

        # Encrypt and send a message
        message = "Hello from Master"
        mask = round(Int, sum(x_master))
        println("Mask :",mask)
        encrypted_message = join([Int(c) ⊻ mask for c in message], ",")
        write(client, encrypted_message * "\n")   # Send encrypted message
        println("Master: Sent encrypted message. $encrypted_message")
        break
        sleep(1)  # Adjust as needed
    end

    close(client)
    close(server)
end

run_master()

