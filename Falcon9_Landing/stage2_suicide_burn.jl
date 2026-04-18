# ============================================================
# Stage 2: Suicide Burn — Constant max thrust + PD attitude control
# ============================================================
# The engine fires at full power. A PD controller gimbals the engine
# to keep the rocket vertical. We compute the ignition altitude from
# kinematics: v² = 2·a·d where a = T/m - g.
#
# What to observe:
# - Vertical velocity decelerates from -80 m/s toward 0
# - Attitude controller corrects the initial 5° tilt in ~2s
# - Gimbal stays within ±5° limits
# - Mass decreases linearly (constant thrust → constant fuel flow)

using ModelingToolkit, OrdinaryDiffEqTsit5, GLMakie
using ModelingToolkit: t_nounits as t, D_nounits as D

# --- Falcon 9 First Stage Parameters ---
const GRAVITY    = 9.81       # m/s²
const T_MAX      = 845_000.0  # N, single Merlin 1D max thrust
const ISP        = 282.0      # s, specific impulse at sea level
const G0         = 9.81       # m/s², for mass flow
const L_ENGINE   = 15.0       # m, CG to engine distance
const M_DRY      = 22_200.0   # kg, dry mass
const M_FUEL     = 3_000.0    # kg, landing fuel budget
const M0         = M_DRY + M_FUEL
const I_BODY     = (1/12) * M0 * 42.0^2  # kg·m², rod approximation
const DELTA_MAX  = deg2rad(5.0)

# Compute ignition altitude
a_net = T_MAX / M0 - GRAVITY
vy0_burn = -80.0
h_ignite = vy0_burn^2 / (2 * a_net)

println("Net deceleration: ", round(a_net, digits=1), " m/s²")
println("Ignition altitude (constant mass): ", round(h_ignite, digits=0), " m")
println("Burn time budget: ", round(M_FUEL / (T_MAX / (G0 * ISP)), digits=1), " s")

# --- Build ODE System ---
@parameters g_s2 T_s2 L_s2 I_s2 Isp_s2 g0_s2 Kp_att Kd_att δ_lim
@variables x_s2(t) y_s2(t) θ_s2(t) vx_s2(t) vy_s2(t) ω_s2(t) m_s2(t)
@variables δ_s2(t)

eqs_s2 = [
    D(x_s2)  ~ vx_s2,
    D(y_s2)  ~ vy_s2,
    D(θ_s2)  ~ ω_s2,
    D(vx_s2) ~ -(T_s2 / m_s2) * sin(θ_s2 + δ_s2),
    D(vy_s2) ~  (T_s2 / m_s2) * cos(θ_s2 + δ_s2) - g_s2,
    D(ω_s2)  ~ -(T_s2 * L_s2 / I_s2) * sin(δ_s2),
    D(m_s2)  ~ -T_s2 / (g0_s2 * Isp_s2),
    # PD attitude control with smooth gimbal saturation
    δ_s2 ~ δ_lim * tanh((Kp_att * θ_s2 + Kd_att * ω_s2) / δ_lim),
]

@mtkbuild powered_sys = ODESystem(eqs_s2, t)

y_s2_idx = findfirst(isequal(y_s2), unknowns(powered_sys))

# --- Solve ---
# Start at 0.98 × h_ignite: accounts for mass depletion increasing deceleration
u0_s2 = [
    x_s2  => 0.0,
    y_s2  => h_ignite * 0.98,
    θ_s2  => deg2rad(5.0),
    vx_s2 => 0.0,
    vy_s2 => vy0_burn,
    ω_s2  => deg2rad(1.0),
    m_s2  => M0,
]

p_s2 = [
    g_s2   => GRAVITY,
    T_s2   => T_MAX,
    L_s2   => L_ENGINE,
    I_s2   => I_BODY,
    Isp_s2 => ISP,
    g0_s2  => G0,
    Kp_att => 1.5,
    Kd_att => 1.5,
    δ_lim  => DELTA_MAX,
]

ground_hit_s2(u, t, integrator) = u[y_s2_idx]
cb_s2 = ContinuousCallback(ground_hit_s2, terminate!)

prob_s2 = ODEProblem(powered_sys, merge(Dict(u0_s2), Dict(p_s2)), (0.0, 30.0))
sol_s2 = solve(prob_s2, Tsit5(); callback=cb_s2, dtmax=0.01)

println("\n--- Suicide Burn Results ---")
println("Burn time: ", round(sol_s2.t[end], digits=2), " s")
println("Landing velocity: ", round(sol_s2[vy_s2][end], digits=2), " m/s")
println("Final tilt: ", round(rad2deg(sol_s2[θ_s2][end]), digits=2), "°")
println("Fuel used: ", round(M0 - sol_s2[m_s2][end], digits=0), " kg")

# --- Plot ---
fig2 = Figure(size=(1200, 800))

ax2a = Axis(fig2[1, 1], xlabel="Time (s)", ylabel="Altitude (m)", title="Altitude")
lines!(ax2a, sol_s2.t, sol_s2[y_s2], color=:dodgerblue, linewidth=2)
hlines!(ax2a, [0.0], color=:black, linestyle=:dash, label="Ground")
axislegend(ax2a)

ax2b = Axis(fig2[1, 2], xlabel="Time (s)", ylabel="Velocity (m/s)", title="Vertical Velocity")
lines!(ax2b, sol_s2.t, sol_s2[vy_s2], color=:red, linewidth=2)
hlines!(ax2b, [0.0], color=:black, linestyle=:dash, label="Zero")
axislegend(ax2b)

ax2c = Axis(fig2[1, 3], xlabel="Time (s)", ylabel="Tilt (°)", title="Attitude Control")
lines!(ax2c, sol_s2.t, rad2deg.(sol_s2[θ_s2]), color=:orange, linewidth=2)
hlines!(ax2c, [0.0], color=:black, linestyle=:dash)

ax2d = Axis(fig2[2, 1], xlabel="Time (s)", ylabel="Gimbal (°)", title="Gimbal Command")
lines!(ax2d, sol_s2.t, rad2deg.(sol_s2[δ_s2]), color=:purple, linewidth=2)
hlines!(ax2d, [rad2deg(DELTA_MAX), -rad2deg(DELTA_MAX)], color=:red, linestyle=:dash,
        label="±5° limit")
axislegend(ax2d)

ax2e = Axis(fig2[2, 2], xlabel="Time (s)", ylabel="Mass (kg)", title="Mass Depletion")
lines!(ax2e, sol_s2.t, sol_s2[m_s2], color=:green, linewidth=2)
hlines!(ax2e, [M_DRY], color=:red, linestyle=:dash, label="Dry mass")
axislegend(ax2e)

ax2f = Axis(fig2[2, 3], xlabel="X (m)", ylabel="Y (m)", title="Trajectory")
lines!(ax2f, sol_s2[x_s2], sol_s2[y_s2], color=:black, linewidth=2)
scatter!(ax2f, [0.0], [0.0], color=:red, markersize=15, marker=:star5, label="Pad")
axislegend(ax2f)

display(fig2)
