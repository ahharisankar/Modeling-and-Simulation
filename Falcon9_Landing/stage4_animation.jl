# ============================================================
# Stage 4: Animated Falcon 9 Landing
# ============================================================
# Run stage3_guided_landing.jl first to get sol_s3, guided_sys,
# and all the Stage 3 variables (x_s3, y_s3, T_s3, etc.).

# --- Resample solution at uniform time steps ---
N_sim = 500
t_sim = range(sol_s3.t[1], sol_s3.t[end], length=N_sim)

x_idx  = findfirst(isequal(x_s3), unknowns(guided_sys))
y_idx  = findfirst(isequal(y_s3), unknowns(guided_sys))
θ_idx  = findfirst(isequal(θ_s3), unknowns(guided_sys))

x_data  = [sol_s3(ti)[x_idx] for ti in t_sim]
y_data  = [sol_s3(ti)[y_idx] for ti in t_sim]
θ_data  = [sol_s3(ti)[θ_idx] for ti in t_sim]
vy_data = [sol_s3(ti, idxs=vy_s3) for ti in t_sim]
T_data  = [sol_s3(ti, idxs=T_s3) for ti in t_sim]

# Add hold frames (2 seconds at 30fps = 60 frames)
N_hold = 60
x_all  = vcat(x_data, fill(x_data[end], N_hold))
y_all  = vcat(y_data, fill(max(y_data[end], 0.0), N_hold))
θ_all  = vcat(θ_data, fill(0.0, N_hold))
vy_all = vcat(vy_data, fill(vy_data[end], N_hold))
T_all  = vcat(T_data, fill(0.0, N_hold))
t_all  = vcat(collect(t_sim), fill(t_sim[end], N_hold))
N_total = length(x_all)

println("Frames: $N_sim simulation + $N_hold hold = $N_total total")

# --- Figure: 2-column layout ---
# Left: trajectory view with tracking camera
# Right: live telemetry (altitude, velocity, thrust)
fig = Figure(size=(1400, 800))

# ===== LEFT: Trajectory with tracking camera =====
ax_traj = Axis(fig[1:3, 1],
    xlabel="Downrange (m)", ylabel="Altitude (m)",
    title="Falcon 9 Landing Burn")

# Pad marker (always visible)
hlines!(ax_traj, [0.0], color=:grey50, linewidth=2)
scatter!(ax_traj, [0.0], [0.0], color=:red, markersize=20, marker=:star5, label="Pad")

# Trail
trail = Observable(Point2f[])
lines!(ax_traj, trail, color=(:dodgerblue, 0.5), linewidth=2, label="Trajectory")

# Rocket dot
rocket_pos = Observable([Point2f(x_data[1], y_data[1])])
scatter!(ax_traj, rocket_pos, color=:white, markersize=18, strokecolor=:black, strokewidth=2)

# Rocket orientation arrow (shows tilt)
arrow_base = Observable([Point2f(x_data[1], y_data[1])])
arrow_dir  = Observable([Vec2f(0, 30)])
arrows!(ax_traj, arrow_base, arrow_dir, color=:orange, linewidth=5, arrowsize=16, label="Rocket")

# Engine flame (points out the nozzle, length ∝ thrust)
flame_max_len = 35.0  # visual length at T_max
flame_base = Observable([Point2f(x_data[1], y_data[1])])
flame_dir  = Observable([Vec2f(0, -1)])
flame_color = Observable((:red, 0.0))  # starts invisible
arrows!(ax_traj, flame_base, flame_dir, color=flame_color, linewidth=5, arrowsize=15)
# Static legend entry for thrust flame (the dynamic arrow has no label since color changes)
lines!(ax_traj, [Point2f(NaN, NaN)], color=:orangered, linewidth=5, label="Thrust")

axislegend(ax_traj, position=:rt)

# ===== RIGHT: Telemetry panels =====
# Altitude vs time
ax_alt = Axis(fig[1, 2], ylabel="Altitude (m)", title="Telemetry")
alt_line = Observable(Point2f[])
lines!(ax_alt, alt_line, color=:dodgerblue, linewidth=2)
hlines!(ax_alt, [0.0], color=:red, linestyle=:dash, linewidth=1)

# Velocity vs time
ax_vel = Axis(fig[2, 2], ylabel="Vert. Velocity (m/s)")
vel_line = Observable(Point2f[])
lines!(ax_vel, vel_line, color=:red, linewidth=2)
hlines!(ax_vel, [0.0], color=:black, linestyle=:dash, linewidth=1)

# Thrust vs time
ax_thr = Axis(fig[3, 2], xlabel="Time (s)", ylabel="Thrust (kN)")
thr_line = Observable(Point2f[])
lines!(ax_thr, thr_line, color=:orange, linewidth=2)
hlines!(ax_thr, [T_MAX/1000], color=:grey50, linestyle=:dash, linewidth=1)

# Fix telemetry x-axis limits
t_end = t_sim[end]
limits!(ax_alt, 0, t_end*1.05, -20, maximum(y_data)*1.05)
limits!(ax_vel, 0, t_end*1.05, minimum(vy_data)*1.1, 5)
limits!(ax_thr, 0, t_end*1.05, -10, T_MAX/1000*1.1)

# Status text overlay on trajectory
status_text = Observable("t = 0.0 s\ny = 500 m\nvy = -50.0 m/s")
text!(ax_traj, 0.02, 0.98, text=status_text, space=:relative, align=(:left, :top),
      fontsize=16, color=:grey30)

# --- Record ---
record(fig, joinpath(@__DIR__, "falcon9_landing.mp4"), 1:N_total; framerate=30) do i
    xi, yi = x_all[i], y_all[i]
    θi = θ_all[i]
    vyi = vy_all[i]
    Ti = T_all[i]
    ti = t_all[i]

    # Update rocket position
    rocket_pos[] = [Point2f(xi, yi)]

    # Update orientation arrow (30m long in body direction)
    arrow_len = 30.0
    arrow_base[] = [Point2f(xi, yi)]
    arrow_dir[]  = [Vec2f(-arrow_len*sin(θi), arrow_len*cos(θi))]

    # Update engine flame (opposite to body direction, length ∝ thrust)
    T_frac = Ti / T_MAX
    f_len = flame_max_len * T_frac
    flame_base[] = [Point2f(xi, yi)]
    flame_dir[]  = [Vec2f(f_len*sin(θi), -f_len*cos(θi))]
    flame_color[] = T_frac > 0.01 ? (:orangered, 0.8) : (:red, 0.0)

    # Update trail
    push!(trail[], Point2f(xi, yi))
    notify(trail)

    # Dynamic camera: track the rocket, zoom in as it descends
    cam_margin_x = max(40.0, yi * 0.15)  # tighter as rocket descends
    cam_margin_y = max(50.0, yi * 0.20)
    x_lo = min(xi, 0.0) - cam_margin_x
    x_hi = max(xi, 0.0) + cam_margin_x
    y_lo = min(yi, 0.0) - 20.0
    y_hi = yi + cam_margin_y
    limits!(ax_traj, x_lo, x_hi, y_lo, y_hi)

    # Update telemetry lines (only during sim, not hold)
    if i <= N_sim
        push!(alt_line[], Point2f(ti, yi))
        notify(alt_line)
        push!(vel_line[], Point2f(ti, vyi))
        notify(vel_line)
        push!(thr_line[], Point2f(ti, Ti/1000))
        notify(thr_line)
    end

    # Status text
    if i <= N_sim
        status_text[] = "t = $(round(ti, digits=1)) s\ny = $(round(yi, digits=1)) m\nvy = $(round(vyi, digits=1)) m/s"
    else
        fuel_used = round(M0 - sol_s3[m_s3][end], digits=0)
        status_text[] = "✓ LANDED\nt = $(round(ti, digits=1)) s\nvy = $(round(vyi, digits=2)) m/s\nFuel: $(Int(fuel_used))/$(Int(M_FUEL)) kg"
    end
end

println("Animation saved to falcon9_landing.mp4")
