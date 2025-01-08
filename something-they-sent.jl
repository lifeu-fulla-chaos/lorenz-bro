using DynamicalSystems, OrdinaryDiffEq, Attractors, ChaosTools, Plots

# Define the Lorenz84 system
function lorenz84(u, p, t)
    F, G, a, b = p
    x, y, z = u
    dx = -y^2 - z^2 - a * x + a * F
    dy = x * y - y - b * x * z + G
    dz = b * x * y + x * z - z
    return SVector(dx, dy, dz)
end

# Parameters and initial conditions
F, G, a, b = 6.886, 1.347, 0.255, 4.0
p = [F, G, a, b]
iu0 = rand(3) # initial condition doesn’t matter
lo = ContinuousDynamicalSystem(lorenz84, iu0, p)

# Grid setup
xg = range(-1, 3; length = 20) # Increase resolution
yg = range(-2, 3; length = 20) # Increase resolution
zg = range(-2, 3; length = 20) # Increase resolution
grid = (xg, yg, zg)

# Differential equation solver options
diffeq = (alg = Vern9(), reltol = 1e-9, abstol = 1e-9)

# Calculate basins of attraction
mapper_lorentz = AttractorsViaRecurrences(lo, grid; sparse = false, consecutive_lost_steps = 2000, consecutive_recurrences = 1000)
basins, attractors = basins_of_attraction(mapper_lorentz; show_progress = true)
# element corresponds to a grid point.
# The value represents the attractor key to which that point belongs.
# Further use output for e.g., Lyapunov exponents or basin fractions
# After calculating basins and attractors
fracs = basins_fractions(basins)

# Plotting
colors = ["red", "green", "blue"] # Adjust colors as needed
fig = plot(title="Basins of Attraction", legend=false)

# Plot basins of attraction
for (i, x) in enumerate(xg)
    println(i)
    for (j, y) in enumerate(yg)
        for (k, z) in enumerate(zg)
            basin = basins[i, j, k]
            scatter!([x], [y], [z], color=colors[basin], markerstrokecolor=:auto, markerstrokewidth=0.5, markersize=0.5, alpha=0.5)
        end
    end
end
println(length(attractors))
# Then use fracs in your loop
for (key, att) in attractors
    u0 = att[1] # First found point of attractor
    ls = lyapunovspectrum(lo, 10000; u0)
    println("Attractor $(key) has spectrum: $(ls) and fraction: $(fracs[key])")
    t, _ = trajectory(lo, 10000, u0) # Get the trajectory
    plot3d!(t[:, 1], t[:, 2], t[:, 3], label="Attractor $key", linewidth=2, color=colors[key])
end
savefig(fig, "a.png")