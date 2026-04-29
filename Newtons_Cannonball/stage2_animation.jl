# ============================================================
# Newton's Cannonball — Stage 2: Animation
# ============================================================
# Run stage1_simulation.jl FIRST to generate solutions.
# This script produces the animation video.
# ============================================================


using GLMakie

# === Extract trajectory data (x, y in Mm) ===
trajs = [(sol[x] ./ 1e6, sol[y] ./ 1e6) for sol in solutions]

all_labels = [
    "3.0 km/s", "5.0 km/s", "6.5 km/s",
    "$(round(v_orbital/1000, digits=1)) km/s (orbit)",
    "9.0 km/s (ellipse)",
    "$(round(v_escape/1000, digits=1)) km/s (escape)"
]

# === Animation timing ===
fps = 30
frames_per_traj = [80, 80, 100, 200, 200, 150]
pause_frames = 30    # pause between trajectories
end_hold = 120       # 4 second hold at end

# Build frame schedule: (trajectory_index, fraction_to_show)
schedule = Tuple{Int,Float64}[]
for (i, nf) in enumerate(frames_per_traj)
    for f in 1:nf
        push!(schedule, (i, f / nf))
    end
    for _ in 1:pause_frames
        push!(schedule, (i, 1.0))
    end
end
for _ in 1:end_hold
    push!(schedule, (length(trajs), 1.0))
end
N_total = length(schedule)
println("Total frames: $N_total ($(round(N_total/fps, digits=1))s at $(fps)fps)")

# === Earth geometry (for plotting) ===
θ_e = range(0, 2π, length=300)
earth_x = R_earth / 1e6 .* cos.(θ_e)
earth_y = R_earth / 1e6 .* sin.(θ_e)

# === Trajectory colors ===
traj_colors = [
    Makie.RGBf(1.0, 0.3, 0.3),   # red — 3 km/s
    Makie.RGBf(1.0, 0.6, 0.2),   # orange — 5 km/s
    Makie.RGBf(1.0, 0.9, 0.2),   # gold — 6.5 km/s
    Makie.RGBf(0.2, 1.0, 0.4),   # green — orbital
    Makie.RGBf(0.6, 0.3, 1.0),   # purple — elliptical
    Makie.RGBf(1.0, 0.4, 0.8),   # pink — escape
]

# === Dynamic zoom function ===
function get_zoom(traj_idx, frac)
    tx, ty = trajs[traj_idx]
    n_show = max(1, round(Int, frac * length(tx)))

    if traj_idx <= 3
        # Sub-orbital: tight zoom centered on launch area
        max_x = maximum(abs.(tx[1:n_show]))
        max_y_above = maximum(ty[1:n_show]) - R_earth / 1e6
        max_y_below = R_earth / 1e6 - minimum(ty[1:n_show])
        extent = max(max_x, max_y_above, max_y_below, 0.5)
        lim = R_earth / 1e6 + extent * 2.5
        return (-lim * 0.6, lim * 0.6, R_earth / 1e6 - extent * 3, lim)
    elseif traj_idx == 4
        # Orbital: show full Earth + orbit
        return (-9.0, 9.0, -9.0, 9.0)
    elseif traj_idx == 5
        # Elliptical: expand as orbit grows
        max_r = maximum(sqrt.(tx[1:n_show] .^ 2 .+ ty[1:n_show] .^ 2))
        lim = max(9.0, max_r * 1.4)
        return (-lim, lim, -lim, lim)
    else
        # Escape: expand progressively
        max_r = maximum(sqrt.(tx[1:n_show] .^ 2 .+ ty[1:n_show] .^ 2))
        lim = max(9.0, max_r * 1.3)
        return (-lim, lim, -lim, lim)
    end
end

# Smooth zoom interpolation
function smooth_zoom!(prev, target, factor=0.06)
    for i in 1:4
        prev[i] += (target[i] - prev[i]) * factor
    end
    return prev
end

# === Build Figure ===
bg = Makie.RGBf(0.02, 0.02, 0.08)
fig = Makie.Figure(size=(1000, 1000), backgroundcolor=bg, figure_padding=0)

ax = Makie.Axis(fig[1, 1],
    aspect=Makie.DataAspect(),
    backgroundcolor=bg)
Makie.hidedecorations!(ax)
Makie.hidespines!(ax)

# Earth
Makie.poly!(ax, Makie.Point2f.(zip(earth_x, earth_y)),
    color=Makie.RGBf(0.15, 0.4, 0.8),
    strokecolor=Makie.RGBf(0.3, 0.6, 1.0), strokewidth=2)

# Atmosphere glow
for (r_mult, alpha) in [(1.02, 0.15), (1.04, 0.08), (1.06, 0.04)]
    gx = R_earth / 1e6 * r_mult .* cos.(θ_e)
    gy = R_earth / 1e6 * r_mult .* sin.(θ_e)
    Makie.lines!(ax, gx, gy,
        color=Makie.RGBAf(0.4, 0.7, 1.0, alpha), linewidth=3)
end

# Launch point
Makie.scatter!(ax, [0], [R_launch / 1e6], color=:white, markersize=8)

# Trajectory trail lines (one Observable per trajectory)
trail_obs = [Observable(Makie.Point2f[]) for _ in 1:6]
for i in 1:6
    Makie.lines!(ax, trail_obs[i], color=traj_colors[i], linewidth=2.5)
end

# Moving dot on current trajectory
dot_pos = Observable([Makie.Point2f(0, R_launch / 1e6)])
dot_color = Observable(traj_colors[1])
Makie.scatter!(ax, dot_pos, color=dot_color, markersize=12)

# --- Text overlays ---
# Top-left: title
Makie.text!(ax, 0.02, 0.97, text="Newton's Cannonball",
    space=:relative, align=(:left, :top),
    fontsize=30, color=:white, font=:bold)

# Top-left: velocity label (dynamic)
label_obs = Observable("")
Makie.text!(ax, 0.02, 0.90, text=label_obs,
    space=:relative, align=(:left, :top),
    fontsize=26, color=:yellow, font=:bold)

# Bottom-center: description (dynamic, large)
info_obs = Observable("")
Makie.text!(ax, 0.50, 0.08, text=info_obs,
    space=:relative, align=(:center, :bottom),
    fontsize=24, color=:white)

# Bottom-right: equation (static, subtle)
Makie.text!(ax, 0.98, 0.02, text="F = −GMm/r²",
    space=:relative, align=(:right, :bottom),
    fontsize=14, color=Makie.RGBf(0.4, 0.4, 0.4))

Makie.limits!(ax, -10, 10, -10, 10)

# === Descriptions for each trajectory ===
descriptions = [
    "Sub-orbital -> hits the ground",
    "Sub-orbital -> longer arc, still hits",
    "Sub-orbital -> almost makes it around",
    "Circular orbit -> falling = curving away",
    "Elliptical orbit -> bound but stretched",
    "Escape -> leaves Earth forever"
]

# === Record Animation ===
output_path = joinpath(@__DIR__, "newtons_cannonball.mp4")
prev_lims = [-10.0, 10.0, -10.0, 10.0]

Makie.record(fig, output_path, 1:N_total; framerate=fps) do frame
    traj_idx, frac = schedule[frame]

    # Draw current trajectory progressively
    tx, ty = trajs[traj_idx]
    n_pts = length(tx)
    show_pts = max(1, round(Int, frac * n_pts))
    trail_obs[traj_idx][] = [Makie.Point2f(tx[i], ty[i]) for i in 1:show_pts]

    # Update moving dot
    dot_pos[] = [Makie.Point2f(tx[show_pts], ty[show_pts])]
    dot_color[] = traj_colors[traj_idx]

    # Smooth dynamic zoom
    target = collect(get_zoom(traj_idx, frac))
    smooth_zoom!(prev_lims, target, 0.06)
    Makie.limits!(ax, prev_lims[1], prev_lims[2], prev_lims[3], prev_lims[4])

    # Velocity label
    v_val = [3.0, 5.0, 6.5,
        round(v_orbital / 1000, digits=1),
        9.0,
        round(v_escape / 1000, digits=1)][traj_idx]
    label_obs[] = "v₀ = $v_val km/s"

    # Description
    info_obs[] = descriptions[traj_idx]

    # End hold — summary text
    if frame > N_total - end_hold
        label_obs[] = "Same equation. Different speed."
        info_obs[] = "v_orbital = $(round(v_orbital/1000, digits=1)) km/s  •  v_escape = $(round(v_escape/1000, digits=1)) km/s"
    end
end

println("✓ Animation saved to: $output_path")
println("  Size: $(round(filesize(output_path)/1024/1024, digits=2)) MB")
println("  Duration: $(round(N_total/fps, digits=1))s at $(fps)fps")
