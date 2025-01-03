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

# Parameters and initial conditions
p = [10.0, 28.0, 8/3]  # Parameters: theta, rho, beta
u0 = [0.0, 10.0, 0.0]  # Initial condition
p1 = [10.0000001, 28.0, 8/3]
T = 100.0  # Total time
Δt = 0.01  # Sampling time

# Generate trajectories
lorenz1 = ContinuousDynamicalSystem(lorenz_rule, u0, p)
lorenz2 = ContinuousDynamicalSystem(lorenz_rule, u0, p1)

A, _ = trajectory(lorenz1, T; Δt)
B, _ = trajectory(lorenz2, T; Δt)

# Extract x, y, z components of the trajectories
xA, yA, zA = A[:, 1], A[:, 2], A[:, 3]
xB, yB, zB = B[:, 1], B[:, 2], B[:, 3]

# Plot trajectories for x, y, and z together
p1 = plot(0:Δt:T, xA, label="Trajectory A", title="x(t)", xlabel="Time", ylabel="x")
plot!(p1, 0:Δt:T, xB, label="Trajectory B")

p2 = plot(0:Δt:T, yA, label="Trajectory A", title="y(t)", xlabel="Time", ylabel="y")
plot!(p2, 0:Δt:T, yB, label="Trajectory B")

p3 = plot(0:Δt:T, zA, label="Trajectory A", title="z(t)", xlabel="Time", ylabel="z")
plot!(p3, 0:Δt:T, zB, label="Trajectory B")

# Combine into a single layout
combined_plot = plot(p1, p2, p3, layout=(3, 1))
savefig(combined_plot, "lorenz_trajectory_comparison.png")

println("Figure saved as 'lorenz_trajectory_comparison.png'")