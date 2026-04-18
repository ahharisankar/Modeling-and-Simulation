# ============================================================
# Stage 3: Guided Landing — Variable thrust + lateral steering
# ============================================================
# Instead of constant thrust, we define a desired velocity profile:
#   vy_desired(y) = -C·√y  (constant deceleration profile)
#
# Thrust controller: feedforward + feedback
#   T = m·(g + C²/2 + Kv·(vy_des - vy))
#
# Lateral guidance: command a tilt angle to steer toward the pad
#   θ_cmd = f(Kp_x·x + Kd_x·vx)
#
# Thrust saturation uses softplus (smooth ReLU) for clean [0, T_max] clamping.

using ModelingToolkit, OrdinaryDiffEqTsit5, GLMakie
using ModelingToolkit: t_nounits as t, D_nounits as D

# --- Falcon 9 Parameters ---
const GRAVITY    = 9.81
const T_MAX      = 845_000.0
const ISP        = 282.0
const G0         = 9.81
const L_ENGINE   = 15.0
const M_DRY      = 22_200.0
const M_FUEL     = 3_000.0
const M0         = M_DRY + M_FUEL
const I_BODY     = (1/12) * M0 * 42.0^2
const DELTA_MAX  = deg2rad(5.0)

# --- Build ODE System ---
@parameters begin
    g_s3; T_max_s3; L_s3; I_s3; Isp_s3; g0_s3
    Kp_att_s3; Kd_att_s3; δ_lim_s3
    C_guide; Kv_guide; Kp_lat; Kd_lat; θ_cmd_lim
end

@variables begin
    x_s3(t); y_s3(t); θ_s3(t); vx_s3(t); vy_s3(t); ω_s3(t); m_s3(t)
    vy_des_s3(t); T_raw_s3(t); T_pos_s3(t); T_s3(t); θ_cmd_s3(t); δ_s3(t)
end

# Softplus sharpness for thrust clamp
k_sat = 20.0

eqs_s3 = [
    # Kinematics
    D(x_s3)  ~ vx_s3,
    D(y_s3)  ~ vy_s3,
    D(θ_s3)  ~ ω_s3,

    # Translational dynamics
    D(vx_s3) ~ -(T_s3 / m_s3) * sin(θ_s3 + δ_s3),
    D(vy_s3) ~  (T_s3 / m_s3) * cos(θ_s3 + δ_s3) - g_s3,

    # Rotational dynamics
    D(ω_s3)  ~ -(T_s3 * L_s3 / I_s3) * sin(δ_s3),

    # Mass depletion
    D(m_s3)  ~ -T_s3 / (g0_s3 * Isp_s3),

    # --- Guidance ---
    # Desired descent velocity: vy_des = -C·√y
    vy_des_s3 ~ -C_guide * sqrt(max(y_s3, 0.01)),

    # Thrust command: feedforward (C²/2) + feedback (Kv·error)
    T_raw_s3 ~ m_s3 * (g_s3 + C_guide^2 / 2.0 + Kv_guide * (vy_des_s3 - vy_s3)),

    # Smooth clamp [0, T_max] using softplus:
    # Step 1: max(0, T_raw) — softplus lower bound
    T_pos_s3 ~ (T_max_s3 / k_sat) * log(1.0 + exp(k_sat * T_raw_s3 / T_max_s3)),
    # Step 2: min(T_max, T_pos) — softplus upper bound
    T_s3 ~ T_max_s3 - (T_max_s3 / k_sat) * log(1.0 + exp(k_sat * (T_max_s3 - T_pos_s3) / T_max_s3)),

    # --- Lateral guidance ---
    # Positive x → positive θ_cmd → tilt left → thrust pushes left → x decreases
    θ_cmd_s3 ~ θ_cmd_lim * tanh((Kp_lat * x_s3 + Kd_lat * vx_s3) / θ_cmd_lim),

    # --- Attitude control: PD on (θ - θ_cmd) ---
    δ_s3 ~ δ_lim_s3 * tanh((Kp_att_s3 * (θ_s3 - θ_cmd_s3) + Kd_att_s3 * ω_s3) / δ_lim_s3),
]

@mtkbuild guided_sys = ODESystem(eqs_s3, t)

y_s3_idx = findfirst(isequal(y_s3), unknowns(guided_sys))

# --- Initial Conditions ---
u0_s3 = [
    x_s3  => 15.0,              # 15m offset from pad
    y_s3  => 500.0,             # 500m altitude
    θ_s3  => deg2rad(3.0),      # slight initial tilt
    vx_s3 => 3.0,               # small lateral drift
    vy_s3 => -50.0,             # falling at 50 m/s
    ω_s3  => 0.0,
    m_s3  => M0,
]

p_s3 = [
    g_s3       => GRAVITY,
    T_max_s3   => T_MAX,
    L_s3       => L_ENGINE,
    I_s3       => I_BODY,
    Isp_s3     => ISP,
    g0_s3      => G0,
    Kp_att_s3  => 1.5,
    Kd_att_s3  => 1.5,
    δ_lim_s3   => DELTA_MAX,
    C_guide    => 2.5,           # velocity profile steepness
    Kv_guide   => 2.0,           # thrust feedback gain
    Kp_lat     => 0.006,         # lateral position gain
    Kd_lat     => 0.04,          # lateral velocity gain
    θ_cmd_lim  => deg2rad(15.0), # max lateral tilt command
]

ground_hit_s3(u, t, integrator) = u[y_s3_idx]
cb_s3 = ContinuousCallback(ground_hit_s3, terminate!)

prob_s3 = ODEProblem(guided_sys, merge(Dict(u0_s3), Dict(p_s3)), (0.0, 60.0))
sol_s3 = solve(prob_s3, Tsit5(); callback=cb_s3, dtmax=0.01)

println("--- Guided Landing Results ---")
println("Total time: ", round(sol_s3.t[end], digits=2), " s")
println("Landing velocity: ", round(sol_s3[vy_s3][end], digits=2), " m/s")
println("Lateral position: ", round(sol_s3[x_s3][end], digits=1), " m from pad")
println("Lateral velocity: ", round(sol_s3[vx_s3][end], digits=2), " m/s")
println("Final tilt: ", round(rad2deg(sol_s3[θ_s3][end]), digits=2), "°")
println("Fuel used: ", round(M0 - sol_s3[m_s3][end], digits=0), " kg of $M_FUEL available")

# --- Plot ---
fig3 = Figure(size=(1400, 900))

ax3a = Axis(fig3[1, 1], xlabel="Time (s)", ylabel="Altitude (m)", title="Altitude")
lines!(ax3a, sol_s3.t, sol_s3[y_s3], color=:dodgerblue, linewidth=2)

ax3b = Axis(fig3[1, 2], xlabel="Time (s)", ylabel="Velocity (m/s)",
            title="Vertical Velocity vs Desired")
lines!(ax3b, sol_s3.t, sol_s3[vy_s3], color=:red, linewidth=2, label="Actual vy")
lines!(ax3b, sol_s3.t, sol_s3[vy_des_s3], color=:blue, linewidth=2,
       linestyle=:dash, label="Desired vy")
hlines!(ax3b, [0.0], color=:black, linestyle=:dot)
axislegend(ax3b)

ax3c = Axis(fig3[1, 3], xlabel="Time (s)", ylabel="Thrust (kN)", title="Thrust Profile")
lines!(ax3c, sol_s3.t, sol_s3[T_s3] ./ 1000, color=:orange, linewidth=2)
hlines!(ax3c, [T_MAX/1000, 0.4*T_MAX/1000], color=:red, linestyle=:dash,
        label="Max / 40% min")
axislegend(ax3c)

ax3d = Axis(fig3[2, 1], xlabel="Time (s)", ylabel="Tilt (°)",
            title="Tilt & Lateral Command")
lines!(ax3d, sol_s3.t, rad2deg.(sol_s3[θ_s3]), color=:orange, linewidth=2,
       label="θ actual")
lines!(ax3d, sol_s3.t, rad2deg.(sol_s3[θ_cmd_s3]), color=:blue, linewidth=1.5,
       linestyle=:dash, label="θ commanded")
axislegend(ax3d)

ax3e = Axis(fig3[2, 2], xlabel="Time (s)", ylabel="Lateral Position (m)",
            title="Lateral Convergence to Pad")
lines!(ax3e, sol_s3.t, sol_s3[x_s3], color=:green, linewidth=2)
hlines!(ax3e, [0.0], color=:red, linestyle=:dash, label="Pad center")
axislegend(ax3e)

ax3f = Axis(fig3[2, 3], xlabel="X (m)", ylabel="Y (m)", title="Full Trajectory")
lines!(ax3f, sol_s3[x_s3], sol_s3[y_s3], color=:black, linewidth=2)
scatter!(ax3f, [0.0], [0.0], color=:red, markersize=20, marker=:star5, label="Pad")
scatter!(ax3f, [sol_s3[x_s3][1]], [sol_s3[y_s3][1]], color=:blue, markersize=12,
         label="Start")
axislegend(ax3f)

display(fig3)
