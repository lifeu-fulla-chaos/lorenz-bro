using DifferentialEquations, Plots, Dates
using ComponentArrays

# System parameters struct for easy modification
struct SystemParams
    a::Float64
    b::Float64
    c::Float64
    d::Float64
    p::Float64
    m::Float64
    n::Float64
    I::Float64
    k:: Float64
    w11::Float64
    w12::Float64
    w21::Float64
    w31::Float64
end

# Default parameters
function default_params()
    SystemParams(1.2, 2.3, 0.13, 1.2, 0.02, 1.0, 0.01, 0.1, 5.0, 0.4, 1.5, 2.5, 5)
end

# System equations
function neuron_memristor_system!(dx, x, params, t)
    dx[1] = -x[1] + params.w11*tanh(x[1]) + params.w12*tanh(x[2]) - params.p*(params.m + params.n*x[5]^2)*x[1]
    dx[2] = -x[2] - params.w21*tanh(x[2]) + params.k*(params.a + params.b*cos(x[4]))*tanh(x[3]) + params.I
    dx[3] = -x[3] - params.w31*tanh(x[1]) + tanh(x[3])
    dx[4] = -params.c*cos(x[4]) + params.d*tanh(x[3])
    dx[5] = x[1]
end

# Function to find local maxima
function find_local_maxima(ts)
    maxima = Float64[]
    for i in 2:length(ts)-1
        if ts[i] > ts[i-1] && ts[i] > ts[i+1]
            push!(maxima, ts[i])
        end
    end
    return maxima
end

# Function to perform bifurcation analysis
function perform_bifurcation(param_range, param_name::Symbol; 
                           tspan=(0.0, 1000.0), 
                           initial_conditions=[0.1, 0.1, 0.1, 0.1, 0.1],
                           transient_time=0.7)
    
    bifurcation_data = []
    
    for param_value in param_range
        # Create parameter set with current bifurcation parameter
        current_params = SystemParams([
            getfield(default_params(), field) == getfield(default_params(), param_name) ? 
            param_value : getfield(default_params(), field)
            for field in fieldnames(SystemParams)]...)
        
        # Create and solve ODE problem
        prob = ODEProblem(neuron_memristor_system!, initial_conditions, tspan, current_params)
        sol = solve(prob, Tsit5(), saveat=tspan[1]:0.1:tspan[2])
        
        # Remove transients
        start_idx = Int(floor(length(sol.t) * transient_time))
        t = sol.t[start_idx:end]
        solution = Array(sol)[1:5, start_idx:end]
        
        # Find maxima for each variable
        maxima = [find_local_maxima(solution[i, :]) for i in 1:5]
        
        push!(bifurcation_data, (param_value, maxima))
    end
    
    return bifurcation_data
end

# Function to plot bifurcation diagram
#= function plot_bifurcation(bifurcation_data, param_name::String)
    plots = []
    
    for var_idx in 1:5
        p = scatter([], [], 
                   title="Variable x$var_idx",
                   xlabel="$param_name",
                   ylabel="Local maxima",
                   markersize=1,
                   legend=false)
        
        for (param_value, maxima) in bifurcation_data
            scatter!(p, fill(param_value, length(maxima[var_idx])), 
                    maxima[var_idx],
                    markersize=1,
                    color=:black)
        end
        
        push!(plots, p)
    end
    
    final_plot = plot(plots..., 
                     layout=(5,1), 
                     size=(800, 2000),
                     plot_title="Bifurcation Analysis for $param_name")
    
    return final_plot
end =#
# Function to plot bifurcation diagram with improved formatting
# Function to plot bifurcation diagram for a single variable
function plot_bifurcation_single(bifurcation_data, param_name::String, var_idx::Int=1)
    p = scatter(
        [], [], 
        title="Variable x$var_idx",
        xlabel="$param_name",
        ylabel="Local maxima",
        markersize=1,
        legend=false,
        tickfont=font(14),      # Larger tick labels
        guidefont=font(16),     # Larger axis labels
        titlefont=font(18),     # Larger title
        margin=15Plots.mm,      # Add some margin
        size=(800, 800),        # Square plot
        dpi=300,                # Higher resolution
        fontfamily="Computer Modern"
    )
    
    for (param_value, maxima) in bifurcation_data
        scatter!(p, 
            fill(param_value, length(maxima[var_idx])), 
            maxima[var_idx],
            markersize=1,
            color=:black
        )
    end
    
    return p
end

# Main execution - for example, using w31
w31_range = range(0.0, 10.0, length=500)
bifurcation_data = perform_bifurcation(w31_range, :w31)
p1 = plot_bifurcation_single(bifurcation_data, "w31", 1)  # Plot only variable x1

# Display and save
datetime_str = Dates.format(Dates.now(), "ddmmyyyy_HHMMSS")
display(p1)
savefig(p1, "neuron_memristor_bifurcation_$datetime_str.svg")
# Main execution

# -------------------a------------------------------
#= a_range = range(0.0, 2.0, length=200)
bifurcation_data = perform_bifurcation(a_range, :a)
p1 = plot_bifurcation(bifurcation_data, "a") =#

# --------------------k------------------------------
#= k_range = range(0.0, 10.0, length=300)
bifurcation_data = perform_bifurcation(k_range, :k) =#
#= p1 = plot_bifurcation(bifurcation_data, "k") =#

#= c_range = range(0.0, 2.0, length=300)
bifurcation_data = perform_bifurcation(c_range, :c) 
p1 = plot_bifurcation(bifurcation_data, "c") =#

#= d_range = range(0.0, 2.0, length=300)
bifurcation_data = perform_bifurcation(d_range, :d) 
p1 = plot_bifurcation(bifurcation_data, "d") =#

#= p_range = range(0.0, 1.0, length=300)
bifurcation_data = perform_bifurcation(p_range, :p) 
p1 = plot_bifurcation(bifurcation_data, "p") =#

#= I_range = range(0.0, 1.0, length=300)
bifurcation_data = perform_bifurcation(I_range, :I) 
p1 = plot_bifurcation(bifurcation_data, "I") =#

#= w11_range = range(0.0, 2.0, length=300)
bifurcation_data = perform_bifurcation(w11_range, :w11) 
p1 = plot_bifurcation(bifurcation_data, "w11") =#

#= w12_range = range(0.0, 2.0, length=300)
bifurcation_data = perform_bifurcation(w12_range, :w12) 
p1 = plot_bifurcation(bifurcation_data, "w12") =#

#= w21_range = range(2.0, 4.0, length=500)
bifurcation_data = perform_bifurcation(w21_range, :w21) 
p1 = plot_bifurcation(bifurcation_data, "w21") =#

#= w31_range = range(0.0, 10.0, length=500)
bifurcation_data = perform_bifurcation(w31_range, :w31) 
p1 = plot_bifurcation(bifurcation_data, "w31")

datetime_str = Dates.format(Dates.now(), "ddmmyyyy_HHMMSS")
display(p1)
savefig(p1, "neuron_memristor_bifurcation_$datetime_str.svg") =#