using Sockets
using LinearAlgebra
using StaticArrays

function run_slave()
    # Define the Lorenz system for the slave with control input
    function lorenz_slave(y, x, p, u)
        σ, ρ, β = p
        dy = zeros(3)
        dy[1] = σ * (y[2] - y[1]) + u[1]
        dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
        dy[3] = y[1] * y[2] - β * y[3] + u[3]
        return dy
    end

    # Define the backstepping control law
    function backstepping_control(x, y, p, k)
        e1, e2, e3 = y .- x
        u1 = -k[1] * e1
        u2 = -k[2] * e2 + p[1] * (y[2] - y[1]) - p[1] * (x[2] - x[1])
        u3 = -k[3] * e3 + y[1] * (p[2] - y[3]) - y[2] - (x[1] * (p[2] - x[3]) - x[2])
        return SVector(u1, u2, u3), SVector(e1, e2, e3)
    end

    # XOR decode message with chaotic signal
    function xor_message(encoded_message, chaotic_signal)
        return String([UInt8(c) ⊻ UInt8(m % 256) for (c, m) in zip(encoded_message, chaotic_signal)])
    end

    # Simulation parameters
    dt = 0.01
    T = 10.0
    n_steps = Int(T / dt)
    p = (10.0, 28.0, 8 / 3)
    k = (5.0, 5.0, 5.0)

    # Initial state of the slave system
    y = [5.0, 5.0, 5.0]
    e = SVector(100.0, 100.0, 100.0)
    x = nothing  # Initialize x to keep track of the last received state

    # Connect to the master system
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

        while norm(e) > 1e-3
            line = try
                readline(client)
            catch
                ""
            end

            if isempty(line)
                println("Slave: Received empty data.")
                continue
            end

            x = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end

            u_control, e = backstepping_control(x, y, p, k)
            y += lorenz_slave(y, x, p, u_control) * dt
            write(client, "ack\n")
            flush(client)
        end
        write(client, "done\n")
        flush(client)
        # Read encoded message from the master
        encoded_message = try
            readline(client)
        catch
            ""
        end

        if !isempty(encoded_message)
            println("Slave: Received encoded message from master.")
            println("Slave: Encoded message: ", encoded_message)
            if x !== nothing
                # Decode the message using the chaotic signal
                chaotic_signal = round.(Int, x .* 100)
                decoded_message = xor_message(codeunits(encoded_message), chaotic_signal)
                println("Slave: Decoded message: ", decoded_message)
            else
                println("Slave: Error: Master state (x) is undefined.")
            end
        end

        break
    end
end

run_slave()