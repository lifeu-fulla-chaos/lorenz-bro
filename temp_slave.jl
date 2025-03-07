using Sockets
using LinearAlgebra
using StaticArrays

# Lorenz system evolution for the slave
function lorenz_slave(y, x, p, u)
    σ, ρ, β = p
    dy = zeros(3)
    dy[1] = σ * (y[2] - y[1]) + u[1]
    dy[2] = y[1] * (ρ - y[3]) - y[2] + u[2]
    dy[3] = y[1] * y[2] - β * y[3] + u[3]
    return dy
end

# Backstepping control function
function backstepping_control(x, y, p, k)
    σ, ρ, β = p
    e1 = y[1] - x[1]
    e2 = y[2] - x[2]
    e3 = y[3] - x[3]

    u1 = -σ * (y[2] - y[1] - x[2] + x[1]) + e2
    u2 = -ρ * (y[1] - x[1]) + (y[2] - x[2]) + (y[1] * y[3]) - (x[1] * x[3]) + e3
    u3 = (-y[1] * y[2]) + (x[1] * x[2]) + (β * (y[3] - x[3])) - ((3 + (2 * k[1])) * e1) - ((5 + (2 * k[1])) * e2) - ((3 + k[1]) * e3)

    return SVector(u1, u2, u3), SVector(e1, e2, e3)
end

# Slave function
function run_slave()
    dt = 0.01
    p = (10.0, 28.0, 8 / 3)  # Lorenz system parameters
    k = (5.0, 5.0, 5.0)      # Backstepping control gains
    y = SVector(5.0, 5.0, 5.0)  # Initial slave state

    println("Slave: Connecting to master...")
    client = nothing
    while true
        try
            client = connect("127.0.0.1", 2000)
            break
        catch
            println("Slave: Connection failed. Retrying...")
            sleep(2)
        end
    end
    println("Slave: Connected to master.")

    try
        # Synchronization Phase
        println("Slave: Synchronizing with master...")
        for _ in 1:10
            line = try
                readline(client)
            catch
                ""
            end

            if isempty(line)
                println("Slave: Received empty data.")
                println("Slave: Synchronization complete.")
                break
            end

            x_master = try
                parse.(Float64, split(line, ","))
            catch
                println("Slave: Error parsing data.")
                continue
            end

            # Apply backstepping control
            u_control, error = backstepping_control(SVector(x_master...), y, p, k)
            y += lorenz_slave(y, SVector(x_master...), p, u_control) * dt
            println("Slave state (sync): ", y)
        end

        # Message-Handling Phase
        println("Slave: Waiting for messages...")
        while true
            encrypted_message = try
                readline(client)
            catch
                ""
            end

            if isempty(encrypted_message)
                println("Slave: No more messages. Exiting.")
                break
            end

            x_master = y  # Assume x_master's state matches slave's state here.
            y += lorenz_slave(y, x_master, p, zeros(3)) * dt  # No control input, just evolution

            # Compute mask using the slave's current state
            mask = round(Int, sum(y))

            # Decrypt the message
            encrypted_values = try
                parse.(Int, split(encrypted_message, ","))
            catch
                println("Slave: Error parsing encrypted message.")
                continue
            end
            println("Maks :$mask")
            decrypted_message = join(Char(c ⊻ mask) for c in encrypted_values)
            println("Slave: Decrypted message: $decrypted_message\n")
        end
    catch e
        println("Slave: Connection error - $e")
    finally
        close(client)
    end
end

run_slave()



