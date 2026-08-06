from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

HIP_OFFSET = 31.5
KNEE_LINK  = 61.5
FOOT_LINK  = 75.5

REACH = HIP_OFFSET + KNEE_LINK + FOOT_LINK

FL_chain = Chain(name="FL", links=[
    OriginLink(),
    URDFLink(
        name="Hip",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="Knee",
        origin_translation=[0, HIP_OFFSET, 0],
        origin_orientation=[0, 0, 0],
        rotation=[-1, 0, 0],
    ),
    URDFLink(
        name="Foot",
        origin_translation=[0, 0, KNEE_LINK],
        origin_orientation=[0, 0, 0],
        rotation=[-1, 0, 0],
    ),
    URDFLink(
        name="FootTip",
        origin_translation=[0, 0, FOOT_LINK],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
    ),
], active_links_mask=[False, True, True, True, False])

fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.22)

ax_x = plt.axes([0.20, 0.13, 0.62, 0.03])
ax_y = plt.axes([0.20, 0.08, 0.62, 0.03])
ax_z = plt.axes([0.20, 0.03, 0.62, 0.03])

s_x = Slider(ax_x, "X (out)",     -150, 150, valinit=0,    valstep=1)
s_y = Slider(ax_y, "Y (forward)", -150, 150, valinit=31.5, valstep=1)
s_z = Slider(ax_z, "Z (up)",      -150, 150, valinit=100,  valstep=1)


def draw(_=None):
    elev, azim = ax.elev, ax.azim
    ax.clear()

    target = [s_x.val, s_y.val, s_z.val]
    angles = FL_chain.inverse_kinematics(target)
    tip = FL_chain.forward_kinematics(angles)[:3, 3]
    err = np.linalg.norm(tip - np.array(target))

    FL_chain.plot(angles, ax, target=target)

    ax.scatter([0], [0], [0], color='red', s=60, depthshade=False)

    ax.set_xlim(-REACH, REACH)
    ax.set_ylim(-REACH, REACH)
    ax.set_zlim(-REACH, REACH)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X (outward)")
    ax.set_ylabel("Y (forward)")
    ax.set_zlabel("Z (up)")

    deg = np.degrees(angles)
    ax.set_title(
        f"hip {deg[1]:7.2f}   knee {deg[2]:7.2f}   foot {deg[3]:7.2f}\n"
        f"err {err:.2f} mm",
        fontsize=10, family="monospace"
    )

    ax.view_init(elev=elev, azim=azim)
    fig.canvas.draw_idle()


s_x.on_changed(draw)
s_y.on_changed(draw)
s_z.on_changed(draw)

draw()
plt.show()