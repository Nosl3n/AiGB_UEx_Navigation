"""Husky A300 self-test (F1): drive the skid-steer base open-loop and compare against ground truth.

Run by setting the robot's controller to "husky_selftest" (instead of <extern>) in husky_test.wbt,
or with webots --batch --mode=fast. Uses the SAME skid-steer IK/FK the bridge uses
(Base.Kinematics = "skid"), so a mismatch here is a mismatch there:

    left  = (v - c*w) / R        right = (v + c*w) / R          c = track / 2
    v = R/2 (wr + wl)            w = R/(2c) (wr - wl)

Robot frame: +Y forward, +X right, +Z up. Positive w = counter-clockwise seen from above.
"""
import math
import sys

from controller import Supervisor

R = 0.1651
C = 0.5708 / 2
MOTORS = ["front_left", "rear_left", "front_right", "rear_right"]   # left pair, right pair

PHASES = [  # (label, v m/s, w rad/s, seconds)
    ("settle", 0.0, 0.0, 1.0),
    ("forward 1.0 m/s", 1.0, 0.0, 4.0),
    ("stop", 0.0, 0.0, 1.0),
    ("pivot +0.5 rad/s", 0.0, 0.5, 4.0),
    ("stop", 0.0, 0.0, 1.0),
    ("arc 0.6 m/s, -0.3 rad/s", 0.6, -0.3, 4.0),
    ("stop", 0.0, 0.0, 1.0),
]


def yaw_of(node):
    """Heading of robot +Y in the world XY plane (0 = world +Y / north), CCW positive."""
    m = node.getOrientation()          # row-major 3x3, columns = robot axes in world
    fy = (m[1], m[4])                  # robot +Y expressed in world (x, y)
    return math.atan2(-fy[0], fy[1])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    sup = Supervisor()
    dt_ms = int(sup.getBasicTimeStep())
    dt = dt_ms / 1000.0
    me = sup.getSelf()

    motors = [sup.getDevice(n) for n in MOTORS]
    enc = []
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        s = m.getPositionSensor()
        s.enable(dt_ms)
        enc.append(s)

    devices = {}
    for name in ("gps", "inertial unit", "compass", "gyro", "accelerometer", "helios", "zed", "zed-ranger"):
        d = sup.getDevice(name)
        devices[name] = d
        if d is None:
            print(f"[selftest] FAIL: device '{name}' missing", flush=True)
        else:
            d.enable(dt_ms if name not in ("helios", "zed", "zed-ranger") else 100)
    if any(d is None for d in devices.values()):
        sup.simulationQuit(1)
        return

    sup.step(dt_ms)
    prev = [s.getValue() for s in enc]
    ox, oy, oth = 0.0, 0.0, yaw_of(me)
    p0 = me.getPosition()
    ox, oy = p0[0], p0[1]
    ok = True

    for label, v, w, secs in PHASES:
        wl = (v - C * w) / R
        wr = (v + C * w) / R
        for i, m in enumerate(motors):
            m.setVelocity(wl if i < 2 else wr)
        gt_start = me.getPosition()[:2]
        th_start = yaw_of(me)
        odo_start = (ox, oy, oth)
        for _ in range(int(secs / dt)):
            if sup.step(dt_ms) == -1:
                print(f"[selftest] simulator ended during '{label}' at t={sup.getTime():.2f}s", flush=True)
                return
            cur = [s.getValue() for s in enc]
            d = [(c - p) for c, p in zip(cur, prev)]
            prev = cur
            dl = R * (d[0] + d[1]) / 2
            dr = R * (d[2] + d[3]) / 2
            ds = (dr + dl) / 2
            dth = (dr - dl) / (2 * C)
            # forward = robot +Y; heading th measured from world +Y, CCW
            ox += -ds * math.sin(oth + dth / 2)
            oy += ds * math.cos(oth + dth / 2)
            oth = wrap(oth + dth)
        gt = me.getPosition()
        gt_d = math.hypot(gt[0] - gt_start[0], gt[1] - gt_start[1])
        gt_dth = wrap(yaw_of(me) - th_start)
        odo_d = math.hypot(ox - odo_start[0], oy - odo_start[1])
        odo_dth = wrap(oth - odo_start[2])
        print(f"[selftest] {label:26s} GT: dist={gt_d:6.3f} m dyaw={math.degrees(gt_dth):7.2f}°"
              f" | wheels: dist={odo_d:6.3f} m dyaw={math.degrees(odo_dth):7.2f}°"
              f" | expected dist={abs(v) * secs:5.2f} dyaw={math.degrees(w * secs):7.2f}°", flush=True)

    gps = devices["gps"].getValues()
    rpy = devices["inertial unit"].getRollPitchYaw()
    gt = me.getPosition()
    # Antenna GT = robot pose applied to the proto's gpsTranslation (the Python getFromDevice is broken).
    m = me.getOrientation()
    lever = (0.0, -0.25, 1.02)
    ant = [gt[i] + sum(m[3 * i + j] * lever[j] for j in range(3)) for i in range(3)]
    print(f"[selftest] GPS lat/lon/alt = {gps[0]:.9f} {gps[1]:.9f} {gps[2]:.3f} | antenna GT xyz = "
          f"{ant[0]:.4f} {ant[1]:.4f} {ant[2]:.4f}", flush=True)
    print(f"[selftest] IMU rpy = {rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f} | GT heading = {yaw_of(me):.3f}", flush=True)
    rng = devices["helios"].getRangeImage()
    finite = [r for r in rng if math.isfinite(r)]
    print(f"[selftest] helios: {devices['helios'].getNumberOfLayers()} layers x "
          f"{devices['helios'].getHorizontalResolution()} = {len(rng)} rays, {len(finite)} hits, "
          f"min={min(finite) if finite else float('nan'):.2f} m", flush=True)
    z = devices["zed"]
    print(f"[selftest] zed: {z.getWidth()}x{z.getHeight()} | chassis z = {gt[2]:.4f} m "
          f"(expect ~0 with origin on the floor)", flush=True)
    if abs(gt[2]) > 0.02:
        ok = False
        print("[selftest] FAIL: robot origin is not resting on the floor", flush=True)
    print("[selftest] DONE " + ("OK" if ok else "WITH FAILURES"), flush=True)
    # Webots relays controller stdout asynchronously: step a little so the last lines reach the
    # console before the simulator exits.
    for _ in range(50):
        sup.step(dt_ms)
    sup.simulationQuit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        # Never leave a --batch Webots hanging on a dead controller.
        Supervisor().simulationQuit(1)
