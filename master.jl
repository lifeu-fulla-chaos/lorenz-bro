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

    # XOR encode message with chaotic signal
    function xor_message(message, chaotic_signal)
        return [UInt8(c % 256) ⊻ UInt8(m) for (m, c) in zip(codeunits(message), chaotic_signal)]
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)

    # Initial state of the master system
    x = [1.0, 1.0, 1.0]
    message = "Hello, Secure World!"

    # Start server
    server = listen(2000)
    println("Master: Server started. Waiting for connections...")

    # Accept a connection from the slave
    client = accept(server)
    println("Master: Slave connected.")

    try
        while true
            # Update master system
            x += lorenz_master(x, p) * dt

            # Send master state to slave
            write(client, join(x, ",") * "\n")
            flush(client)

            # Read acknowledgment from the slave
            ack = try
                readline(client)
            catch
                ""
            end

            if ack == "done"
                println("Master: Received termination signal from slave.")

                # Generate chaotic signal and encode message
                chaotic_signal = round.(Int, x .* 100)  # Scale state for XOR
                encoded_message = xor_message(message, chaotic_signal)
                println("Master: Sending encoded message to slave.")
                println("Master: Encoded message: ", String(encoded_message))
                # Send encoded message to slave
                write(client, String(encoded_message) * "\n")
                flush(client)
                break
            elseif isempty(ack)
                println("Master: No acknowledgment received. Assuming slave is done.")
                break
            end
        end
    finally
        println("Master: Closing connections.")
        close(client)
        close(server)
    end
end

run_master()
