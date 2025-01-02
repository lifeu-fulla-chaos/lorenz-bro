# using DynamicalSystems, PyPlot, Interpolations

# u0 = [10.0, 10, 10]
# lo = Systems.lorenz(u0)

# for i in 1:3
#     u = u0 .+ i * 1e-3
#     tr, times = trajectory(lo, 15, u)  # Get trajectory and times
#     interp_x = LinearInterpolation(times, tr[:, 1])  # Interpolate x
#     fixed_times = 0:0.01:15                          # Fixed time vector
#     plot(fixed_times, interp_x.(fixed_times), label="Trajectory $i")
# end

# xlabel("Time")
# ylabel("x(t)")
# legend()
# savefig("lorenz_plot.png")


using DynamicalSystems
using Plots

# Lorenz system rule
function lorenz_rule(u, p, t)
    theta, rho, beta = p
    x, y, z = u
    dx = theta * (y - x)
    dy = x * (rho - z) - y
    dz = x * y - beta * z
    return SVector(dx, dy, dz)
end

# Parameters, initial conditions, and system setup
p = [10.0, 28.0, 8/3]  # Parameters: theta, rho, beta
u0 = [0.0, 10.0, 0.0]  # Initial condition
lorenz = ContinuousDynamicalSystem(lorenz_rule, u0, p)

# Simulation settings
T = 100.0  # Total time
Δt = 0.01  # Sampling time

# Generate trajectory
A, _ = trajectory(lorenz, T; Δt = Δt)

# Extract x, y, z components of the trajectory
x = A[:, 1]
y = A[:, 2]
z = A[:, 3]

# Plot trajectory in 3D
plot(x, y, z, lw=0.5, title="Lorenz Attractor", legend=false)
xlabel!("x")
ylabel!("y")
zlabel!("z")

# Save the figure
savefig("lorenz_attractor.png")
println("Figure saved as 'lorenz_attractor.png'")
