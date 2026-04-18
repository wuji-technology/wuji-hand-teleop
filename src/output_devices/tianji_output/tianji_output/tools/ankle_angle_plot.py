import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, CheckButtons, TextBox

# === 1. Static Geometry Parameters ===
shoulder = np.array([0.0, 0.0, 0.4])
wrist = np.array([0.4, 0.0, 0.2])
L_upper = 0.3
L_fore = 0.25

# Compute axis
axis_vec = wrist - shoulder
axis_len = np.linalg.norm(axis_vec)
axis_norm = axis_vec / axis_len

# Compute center and radius
cos_alpha = (L_upper**2 + axis_len**2 - L_fore**2) / (2 * L_upper * axis_len)
cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
center_dist = L_upper * cos_alpha
center = shoulder + axis_norm * center_dist
radius = np.sqrt(max(0, L_upper**2 - center_dist**2))

# === 2. Initialize plot ===
fig = plt.figure(figsize=(13, 9)) # Slightly wider
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.05, bottom=0.35, right=0.95) # Adjust layout

# --- Draw static elements ---
ax.quiver(0,0,0, 0.1,0,0, color='r', lw=1.5)
ax.quiver(0,0,0, 0,0.1,0, color='g', lw=1.5)
ax.quiver(0,0,0, 0,0,0.1, color='b', lw=1.5)

ax.scatter(*shoulder, color='k', s=100, label='Shoulder', zorder=10)
ax.scatter(*wrist, color='b', s=100, label='Wrist', zorder=10)
ax.text(shoulder[0], shoulder[1], shoulder[2]+0.05, "Shoulder", color='k', fontsize=10)
ax.text(wrist[0], wrist[1], wrist[2]+0.05, "Wrist", color='b', fontsize=10)
ax.plot([shoulder[0], wrist[0]], [shoulder[1], wrist[1]], [shoulder[2], wrist[2]], 'k--', alpha=0.3)

# --- Dynamic elements ---
arm_plot, = ax.plot([], [], [], 'o-', color='red', lw=3, zorder=5)
circle_plot, = ax.plot([], [], [], color='gray', alpha=0.4, linestyle=':')
elbow_scatter = ax.scatter([], [], [], color='red', s=120, zorder=10)

quiver_handles = {'ref': None, 'raw': None}
line_handles = {'proj': None} 

# === 3. Control area (sliders + input boxes) ===
ax_check = plt.axes([0.05, 0.8, 0.15, 0.1])
check = CheckButtons(ax_check, ['Show Raw Vec'], [True])

# Define control position function
def make_control(label, y_pos, val_min, val_max, val_init):
    # Slider position: [left, bottom, width, height]
    ax_s = plt.axes([0.20, y_pos, 0.50, 0.03])
    # Input box position: right next to slider
    ax_t = plt.axes([0.75, y_pos, 0.10, 0.03])
    
    slider = Slider(ax_s, label, val_min, val_max, valinit=val_init)
    text_box = TextBox(ax_t, '', initial=str(val_init), label_pad=0.05)
    
    return slider, text_box

# Create controls
s_angle, t_angle = make_control('Arm Angle', 0.20, -180, 180, 30)
s_vx, t_vx = make_control('Vec X', 0.15, -1.0, 1.0, 0.0)
s_vy, t_vy = make_control('Vec Y', 0.10, -1.0, 1.0, 0.0)
s_vz, t_vz = make_control('Vec Z', 0.05, -1.0, 1.0, -1.0)

# === 4. Synchronization logic ===

# Flag to prevent infinite loop (update -> set text -> submit -> update ...)
is_updating_from_slider = False

def update_graph(val=None):
    """Main plot update function, usually triggered by slider"""
    global is_updating_from_slider
    is_updating_from_slider = True # Mark start
    
    # 1. Get slider values
    angle = s_angle.val
    raw_vec = np.array([s_vx.val, s_vy.val, s_vz.val])
    show_raw = check.get_status()[0]

    # 2. Synchronize text box display update (keep 2 decimal places)
    t_angle.set_val(f"{angle:.1f}")
    t_vx.set_val(f"{raw_vec[0]:.2f}")
    t_vy.set_val(f"{raw_vec[1]:.2f}")
    t_vz.set_val(f"{raw_vec[2]:.2f}")

    # 3. Computation logic
    if np.linalg.norm(raw_vec) < 1e-6: raw_vec = np.array([0, 0, -1])
    raw_vec_norm = raw_vec / np.linalg.norm(raw_vec)

    temp_perp = np.cross(axis_norm, raw_vec_norm)
    if np.linalg.norm(temp_perp) < 1e-6: temp_perp = np.cross(axis_norm, np.array([1,0,0]))
    temp_perp = temp_perp / np.linalg.norm(temp_perp)
    ref_vec = np.cross(temp_perp, axis_norm)
    ref_vec = ref_vec / np.linalg.norm(ref_vec)

    theta = np.radians(angle)
    elbow_vec = radius * (np.cos(theta) * ref_vec + np.sin(theta) * temp_perp)
    elbow = center + elbow_vec

    # 4. Plot update
    arm_plot.set_data([shoulder[0], elbow[0], wrist[0]], [shoulder[1], elbow[1], wrist[1]])
    arm_plot.set_3d_properties([shoulder[2], elbow[2], wrist[2]])
    elbow_scatter._offsets3d = ([elbow[0]], [elbow[1]], [elbow[2]])

    t = np.linspace(0, 2*np.pi, 60)
    c_pts = center[:,None] + radius * (np.cos(t)*ref_vec[:,None] + np.sin(t)*temp_perp[:,None])
    circle_plot.set_data(c_pts[0], c_pts[1])
    circle_plot.set_3d_properties(c_pts[2])

    # Safely remove old arrows
    for key in ['ref', 'raw']:
        if quiver_handles[key] is not None:
            try: quiver_handles[key].remove()
            except ValueError: pass
    if line_handles['proj'] is not None:
        try: 
            if isinstance(line_handles['proj'], list) and len(line_handles['proj']) > 0:
                line_handles['proj'][0].remove()
        except ValueError: pass

    # Draw new arrows
    quiver_handles['ref'] = ax.quiver(
        center[0], center[1], center[2],
        radius*ref_vec[0], radius*ref_vec[1], radius*ref_vec[2],
        color='green', lw=2, arrow_length_ratio=0.15
    )

    if show_raw:
        quiver_handles['raw'] = ax.quiver(
            center[0], center[1], center[2],
            radius*raw_vec_norm[0], radius*raw_vec_norm[1], radius*raw_vec_norm[2],
            color='purple', lw=1.5, linestyle='--', arrow_length_ratio=0.1
        )
        tip_raw = center + radius * raw_vec_norm
        tip_ref = center + radius * ref_vec
        line_handles['proj'] = ax.plot(
            [tip_raw[0], tip_ref[0]], [tip_raw[1], tip_ref[1]], [tip_raw[2], tip_ref[2]], 
            color='orange', linestyle=':', lw=1
        )

    is_updating_from_slider = False # Mark end
    fig.canvas.draw_idle()

def submit_text(text, slider_obj):
    """Triggered when text box receives Enter, updates slider"""
    # If text update was triggered by slider, do not set slider again, otherwise infinite recursion
    if is_updating_from_slider:
        return
        
    try:
        val = float(text)
        # Limit range to prevent errors
        if val < slider_obj.valmin: val = slider_obj.valmin
        if val > slider_obj.valmax: val = slider_obj.valmax
        
        # Set slider value (this automatically triggers update_graph)
        slider_obj.set_val(val)
    except ValueError:
        pass

# === 5. Bind events ===
# Slider change -> update graph
s_angle.on_changed(update_graph)
s_vx.on_changed(update_graph)
s_vy.on_changed(update_graph)
s_vz.on_changed(update_graph)

# Text box Enter -> update slider (using lambda to pass corresponding slider object)
t_angle.on_submit(lambda text: submit_text(text, s_angle))
t_vx.on_submit(lambda text: submit_text(text, s_vx))
t_vy.on_submit(lambda text: submit_text(text, s_vy))
t_vz.on_submit(lambda text: submit_text(text, s_vz))

check.on_clicked(update_graph)

# View angle
ax.set_xlim([-0.4, 0.6])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([0, 0.7])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Start
update_graph(None)
plt.show()