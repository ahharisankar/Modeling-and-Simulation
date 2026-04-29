# ============================================================
# Newton's Cannonball — Stage 1: Simulation
# ============================================================
# Run this first to generate all trajectory solutions.
# Then run stage2_animation.jl to produce the video.
# ============================================================

using ModelingToolkit, OrdinaryDiffEqTsit5
using ModelingToolkit: t_nounits as t, D_nounits as D

# === Constants ===
μ = 3.986e14       # GM for Earth (m³/s²)
R_earth = 6.371e6  # Earth radius (m)
h_mountain = 300e3  # Launch altitude: 300 km above surface
R_launch = R_earth + h_mountain

# Orbital and escape velocities at launch altitude
v_orbital = sqrt(μ / R_launch)   # ≈ 7730 m/s
v_escape  = sqrt(2μ / R_launch)  # ≈ 10932 m/s

println("Launch altitude: $(h_mountain/1000) km")
println("v_orbital = $(round(v_orbital, digits=0)) m/s")
println("v_escape  = $(round(v_escape, digits=0)) m/s")

# === ODE System ===
# 4 states: x, y, vx, vy
# 1 observed: r (distance from Earth center)
# Gravity: F = -μm/r², decomposed into x,y components
@variables y(t) vy(t) x(t) vx(t) r(t)

eqs = [
    r ~ sqrt(x^2 + y^2),
    D(x)  ~ vx,
    D(y)  ~ vy,
    D(vx) ~ -μ * x / r^3,
    D(vy) ~ -μ * y / r^3
]

@mtkbuild cannon_sys = ODESystem(eqs, t)
println("State ordering: ", unknowns(cannon_sys))

# === Callback: ground impact ===
# Find state indices (MTK reorders states)
state_names = string.(unknowns(cannon_sys))
idx_x = findfirst(s -> s == "x(t)", state_names)
idx_y = findfirst(s -> s == "y(t)", state_names)
println("idx_x = $idx_x, idx_y = $idx_y")

function condition_impact(u, t, integrator)
    sqrt(u[idx_x]^2 + u[idx_y]^2) - R_earth
end

function affect_impact!(integrator)
    terminate!(integrator)
end

# Downcrossing only: condition goes from + (above surface) to - (below)
# We start at +300km altitude, so condition starts at +300,000
cb_impact = ContinuousCallback(condition_impact, nothing, affect_impact!)

# === Solve for multiple launch velocities ===
function solve_cannon(v_launch; t_end=50000.0)
    u0 = [x => 0, y => R_launch, vx => v_launch, vy => 0]
    prob = ODEProblem(cannon_sys, u0, (0.0, t_end))
    solve(prob, Tsit5(), callback=cb_impact, saveat=5.0, reltol=1e-10, abstol=1e-12)
end

T_orbital = 2π * R_launch / v_orbital  # orbital period ≈ 5420s

velocities = [3000.0, 5000.0, 6500.0, v_orbital, 9000.0, v_escape]
labels = [
    "3.0 km/s (sub-orbital)",
    "5.0 km/s (sub-orbital)",
    "6.5 km/s (sub-orbital)",
    "$(round(v_orbital/1000, digits=1)) km/s (circular orbit)",
    "9.0 km/s (elliptical orbit)",
    "$(round(v_escape/1000, digits=1)) km/s (escape)"
]
t_ends = [600.0, 600.0, 800.0, T_orbital * 1.05, T_orbital * 2.5, 8000.0]

solutions = []
for (v, label, te) in zip(velocities, labels, t_ends)
    sol = solve_cannon(v, t_end=te)
    r_vals = sqrt.(sol[x].^2 + sol[y].^2)
    min_alt = round((minimum(r_vals) - R_earth) / 1000, digits=1)
    max_alt = round((maximum(r_vals) - R_earth) / 1000, digits=1)
    println("$label: $(sol.retcode), t=$(round(sol.t[end]/60, digits=1))min, alt: $(min_alt)km to $(max_alt)km")
    push!(solutions, sol)
end

println("\n✓ All trajectories computed. Run stage2_animation.jl next.")
