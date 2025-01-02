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


using DynamicalSystems # load the library

function lorenz_rule(u, p, t) 
    theta, rho, beta = p
    x, y, z = u
    dx = theta*(y - x)
    dy = x*(rho - z) - y
    dz = x*y - beta*z
    return SVector(dx, dy, dz) # Static Vector
end
p = [10.0, 28.0, 8/3] # parameters: theta, rho, beta
u0 = [0.0, 10.0, 0.0]
lorenz = ContinuousDynamicalSystem(lorenz_rule, u0, p)
T = 100.0 # total time
Δt = 0.01 # sampling time
A = trajectory(lorenz, T; Δt) # Use positional argument for dt
println(A)