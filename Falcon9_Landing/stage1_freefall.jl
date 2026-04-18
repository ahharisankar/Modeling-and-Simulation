# ============================================================
# Stage 1: Free Fall — gravity only, no thrust
# ============================================================
# A rocket falling under gravity with initial tilt and spin.
# No engine, no control. Just Newton's laws.
#
# What to observe:
# - CG follows a perfect parabola regardless of tilt
# - Constant angular velocity (no torque → no angular acceleration)
# - Crashes in ~5 seconds

using ModelingToolkit, OrdinaryDiffEqTsit5, GLMakie
using ModelingToolkit: t_nounits as t, D_nounits as D

@parameters g_ff
@variables x_ff(t) y_ff(t) θ_ff(t) vx_ff(t) vy_ff(t) ω_ff(t)

eqs_ff = [
    D(x_ff)  ~ vx_ff,
    D(y_ff)  ~ vy_ff,
    D(θ_ff)  ~ ω_ff,
    D(vx_ff) ~ 0.0,           # no horizontal force
    D(vy_ff) ~ -g_ff,         # gravity pulls down
    D(ω_ff)  ~ 0.0,           # no torque → constant spin rate
]

@mtkbuild freefall_sys = ODESystem(eqs_ff, t)

# --- Initial Conditions ---
u0_ff = [
    x_ff  => 50.0,             # 50m to the right of the pad
    y_ff  => 500.0,            # 500m altitude
    θ_ff  => deg2rad(10.0),    # tilted 10° from vertical
    vx_ff => 10.0,             # drifting right at 10 m/s
    vy_ff => -80.0,            # falling at 80 m/s
    ω_ff  => deg2rad(3.0),     # tumbling at 3°/s
]

p_ff = [g_ff => 9.81]

# Stop when rocket hits the ground
# NOTE: MTK reorders states — find index dynamically
y_ff_idx = findfirst(isequal(y_ff), unknowns(freefall_sys))
ground_hit_ff(u, t, integrator) = u[y_ff_idx]
cb_ff = ContinuousCallback(ground_hit_ff, terminate!)

# Solve (new MTK API: merge u0 and p into one dict)
prob_ff = ODEProblem(freefall_sys, merge(Dict(u0_ff), Dict(p_ff)), (0.0, 30.0))
sol_ff = solve(prob_ff, Tsit5(); callback=cb_ff, dtmax=0.05)

println("--- Stage 1: Free Fall Results ---")
println("Impact time: ", round(sol_ff.t[end], digits=2), " s")
println("Impact velocity: ", round(sol_ff[vy_ff][end], digits=1), " m/s")
println("Final tilt: ", round(rad2deg(sol_ff[θ_ff][end]), digits=1), "°")
println("Lateral drift: ", round(sol_ff[x_ff][end], digits=1), " m")

# --- Validate against hand calculations ---
# y(t) = 500 - 80t - 0.5*9.81*t²  →  t_impact ≈ 4.82 s
# vy(t) = -80 - 9.81*t             →  vy_impact ≈ -127.3 m/s
# θ(t) = 10° + 3°/s * t            →  θ_impact ≈ 24.5°

# --- Plot ---
fig1 = Figure(size=(1000, 700))

ax1a = Axis(fig1[1, 1], xlabel="Time (s)", ylabel="Altitude (m)",
            title="Altitude — Parabolic Fall")
lines!(ax1a, sol_ff.t, sol_ff[y_ff], color=:dodgerblue, linewidth=2)

ax1b = Axis(fig1[1, 2], xlabel="Time (s)", ylabel="Vertical Velocity (m/s)",
            title="Vertical Velocity — Linear Increase")
lines!(ax1b, sol_ff.t, sol_ff[vy_ff], color=:red, linewidth=2)

ax1c = Axis(fig1[2, 1], xlabel="Time (s)", ylabel="Tilt Angle (°)",
            title="Tilt — Constant Spin (No Torque)")
lines!(ax1c, sol_ff.t, rad2deg.(sol_ff[θ_ff]), color=:orange, linewidth=2)

ax1d = Axis(fig1[2, 2], xlabel="X Position (m)", ylabel="Y Position (m)",
            title="Trajectory — CG Parabola")
lines!(ax1d, sol_ff[x_ff], sol_ff[y_ff], color=:green, linewidth=2)
scatter!(ax1d, [sol_ff[x_ff][end]], [sol_ff[y_ff][end]], color=:red, markersize=15,
         label="Impact")
axislegend(ax1d)

display(fig1)
