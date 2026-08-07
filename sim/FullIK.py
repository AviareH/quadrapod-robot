from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

HIP_OFFSET = 31.5
KNEE_LINK  = 61.5
FOOT_LINK  = 75.5

REACH_MAX = KNEE_LINK + FOOT_LINK
REACH_MIN = abs(KNEE_LINK - FOOT_LINK)

BODY_X = 40.0
BODY_Y = 60.0
LIMIT  = 240


def make_leg(mirrored, rear):
    return Chain(name="leg", links=[
        OriginLink(),
        URDFLink(name="Hip", origin_translation=[0, 0, 0],
                 origin_orientation=[0, 0, np.pi] if rear else [0, 0, 0],
                 rotation=[0, 0, -1] if mirrored else [0, 0, 1]),
        URDFLink(name="Knee", origin_translation=[0, HIP_OFFSET, 0],
                 origin_orientation=[0, 0, 0], rotation=[-1, 0, 0]),
        URDFLink(name="Foot", origin_translation=[0, 0, KNEE_LINK],
                 origin_orientation=[0, 0, 0], rotation=[-1, 0, 0]),
        URDFLink(name="FootTip", origin_translation=[0, 0, FOOT_LINK],
                 origin_orientation=[0, 0, 0], rotation=[0, 0, 0]),
    ], active_links_mask=[False, True, True, True, False])


def solve_ik(x, y, z, mirrored):
    horizontal_dist = np.hypot(x, y)
    planar_reach = horizontal_dist - HIP_OFFSET
    if planar_reach < 0.0:
        return None, None, None, "INSIDE HIP OFFSET"

    leg_span = np.hypot(planar_reach, z)
    if leg_span > REACH_MAX or leg_span < REACH_MIN:
        return None, None, None, "SPAN OUT OF RANGE"

    raw1 = np.degrees(np.arctan2(x, y))
    raw2 = np.degrees(
        np.arctan2(planar_reach, -z)
        + np.arccos(np.clip((KNEE_LINK**2 + leg_span**2 - FOOT_LINK**2)
                            / (2.0 * KNEE_LINK * leg_span), -1.0, 1.0)))
    raw3 = np.degrees(np.arccos(np.clip(
        (KNEE_LINK**2 + FOOT_LINK**2 - leg_span**2)
        / (2.0 * KNEE_LINK * FOOT_LINK), -1.0, 1.0)))

    if mirrored:
        t1, t2, t3 = 180.0 - raw1, 180.0 - raw2, raw3
    else:
        t1, t2, t3 = raw1, raw2, 180.0 - raw3

    status = "ok"
    if not all(0.0 <= a <= 180.0 for a in (t1, t2, t3)):
        status = "SERVO RANGE"
    return t1, t2, t3, status


def to_chain(t1, t2, t3, mirrored):
    if mirrored:
        return [0, np.radians(180 - t1), np.radians(t2), np.radians(180 - t3), 0]
    return [0, np.radians(t1), np.radians(180 - t2), np.radians(t3), 0]


LEGS = [
    ("FL", make_leg(False, False), np.array([-BODY_X,  BODY_Y, 0.0]), -1, False, False, '#1f77b4'),
    ("FR", make_leg(True,  False), np.array([ BODY_X,  BODY_Y, 0.0]), +1, True,  False, '#d62728'),
    ("BL", make_leg(True,  True),  np.array([-BODY_X, -BODY_Y, 0.0]), -1, True,  True,  '#2ca02c'),
    ("BR", make_leg(False, True),  np.array([ BODY_X, -BODY_Y, 0.0]), +1, False, True,  '#ff7f0e'),
]

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.26, top=0.90)

sliders = {}
col_x = [0.07, 0.31, 0.55, 0.79]
for (name, *_), x0 in zip(LEGS, col_x):
    fig.text(x0 + 0.075, 0.205, name, ha="center", fontsize=11, weight="bold")
    sliders[name] = (
        Slider(plt.axes([x0, 0.155, 0.15, 0.025]), "Out", -150, 150, valinit=50,   valstep=1),
        Slider(plt.axes([x0, 0.105, 0.15, 0.025]), "Fwd", -150, 150, valinit=0,    valstep=1),
        Slider(plt.axes([x0, 0.055, 0.15, 0.025]), "Up",  -150, 150, valinit=-100, valstep=1),
    )


def draw_body():
    c = np.array([[-BODY_X, -BODY_Y, 0], [BODY_X, -BODY_Y, 0],
                  [BODY_X, BODY_Y, 0], [-BODY_X, BODY_Y, 0],
                  [-BODY_X, -BODY_Y, 0]])
    ax.plot(c[:, 0], c[:, 1], c[:, 2], '-', color='#888', linewidth=1.5)


def draw(_=None):
    elev, azim = ax.elev, ax.azim
    ax.clear()
    draw_body()

    lines = []
    for name, chain, base, out_sign, mirrored, rear, colour in LEGS:
        out, fwd, up = (s.val for s in sliders[name])
        local_y = -fwd if rear else fwd
        t1, t2, t3, status = solve_ik(out, local_y, up, mirrored)

        target = base + np.array([out_sign * out, fwd, up])
        ax.scatter(*target, color='magenta', marker='x', s=70,
                   depthshade=False, linewidths=2)

        if t1 is None:
            lines.append(f"{name}   --       --       --       [{status}]")
            continue

        fk = chain.forward_kinematics(to_chain(t1, t2, t3, mirrored),
                                      full_kinematics=True)
        pts = np.array([m[:3, 3] for m in fk[1:]]) + base

        style = '--o' if status == "SERVO RANGE" else '-o'
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], style,
                color=colour, linewidth=3, markersize=5)

        tag = "" if status == "ok" else f"   [{status}]"
        lines.append(f"{name}   hip {t1:7.2f}   knee {t2:7.2f}   foot {t3:7.2f}{tag}")

    ax.scatter([0], [0], [0], color='red', s=50, depthshade=False)

    ax.set_xlim(-LIMIT, LIMIT)
    ax.set_ylim(-LIMIT, LIMIT)
    ax.set_zlim(-LIMIT, LIMIT)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X (right)")
    ax.set_ylabel("Y (forward)")
    ax.set_zlabel("Z (up)")
    ax.set_title("\n".join(lines), fontsize=9, family="monospace", loc="left")

    ax.view_init(elev=elev, azim=azim)
    fig.canvas.draw_idle()


for trio in sliders.values():
    for s in trio:
        s.on_changed(draw)

draw()
plt.show()