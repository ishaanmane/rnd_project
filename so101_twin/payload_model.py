"""Predicted motor load for the payload experiment: static gravity torque from the URDF.

For every arm joint it answers: "to hold the arm still in this pose, with this payload in the
gripper, how much torque must this motor give?" and compares that with the motor's stall torque.

How:
- Forward kinematics on urdf/so101.urdf puts every link's centre of mass (from <inertial>) and
  the gripper tool point (gripper_frame_link) in the base frame. The URDF is Z-up, gravity -Z.
- Torque on joint j = axis_j . sum over everything the joint carries of (r - p_j) x (m g),
  where p_j is a point on the joint axis and r the centre of mass. The payload is a point mass
  at the tool point (between the jaws).
- LeRobot degrees become URDF angles the same way the twin draws the arm (twin_calib.json:
  angle = sign * degrees + offset), so the model sees exactly the pose the twin shows.

Limits of the model (say these in the report):
- Static only: no acceleration. At 10-30 deg/s the inertia terms are small next to gravity.
- It trusts the URDF masses. Real printed parts, screws and cables differ a little.
- Friction in the gears is not modelled.
- Stall torque is a peak the servo cannot hold for long; it gets hot well before that.

Only the standard library is used, so twin_server.py stays dependency free.
"""
import math
import xml.etree.ElementTree as ET

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
G = 9.81
STALL_KGCM = 30.0          # Feetech STS3215 12 V version, stall torque (datasheet)
TOOL_LINK = "gripper_frame_link"


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _tf(R, t):
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _axis_angle(a, t):
    """Rotation about unit axis a by t radians (Rodrigues), as a 4x4."""
    x, y, z = a
    c, s, v = math.cos(t), math.sin(t), 1 - math.cos(t)
    return _tf([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                [z * x * v - y * s, z * y * v + x * s, c + z * z * v]], [0.0, 0.0, 0.0])


def _apply(M, p):
    return [M[i][0] * p[0] + M[i][1] * p[1] + M[i][2] * p[2] + M[i][3] for i in range(3)]


def _origin(el):
    o = el.find("origin")
    if o is None:
        return [0.0] * 3, [0.0] * 3
    return ([float(v) for v in (o.get("xyz") or "0 0 0").split()],
            [float(v) for v in (o.get("rpy") or "0 0 0").split()])


class GravityModel:
    def __init__(self, urdf_path, calib, stall_kgcm=STALL_KGCM):
        root = ET.parse(urdf_path).getroot()
        self.calib = calib or {}
        self.stall = float(stall_kgcm)
        self.masses = {}                         # link -> (kg, centre of mass in the link frame)
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                xyz, _ = _origin(inertial)
                self.masses[link.get("name")] = (float(inertial.find("mass").get("value")), xyz)
        self.joints = []                         # in the URDF's order; parents are resolved below
        for j in root.findall("joint"):
            xyz, rpy = _origin(j)
            ax = j.find("axis")
            a = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0.0, 0.0, 0.0]
            n = math.sqrt(sum(v * v for v in a)) or 1.0
            self.joints.append({"name": j.get("name"), "type": j.get("type"),
                                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                                "T": _tf(_rpy(*rpy), xyz), "axis": [v / n for v in a]})
        children = {}
        for j in self.joints:
            children.setdefault(j["parent"], []).append(j["child"])

        def subtree(link):
            out = [link]
            for c in children.get(link, []):
                out += subtree(c)
            return out
        self.carried = {j["name"]: subtree(j["child"]) for j in self.joints if j["name"] in ARM}
        self.total_kg = sum(m for m, _ in self.masses.values())

    def _angles(self, deg):
        q = {}
        for j in ARM:
            c = self.calib.get(j) or {}
            q[j] = math.radians(float(c.get("sign", 1)) * float(deg[j]) + float(c.get("offset", 0.0)))
        return q

    def _frames(self, q):
        W = {"base_link": [[1.0 if i == k else 0.0 for k in range(4)] for i in range(4)]}
        pending = list(self.joints)
        while pending:
            rest = []
            for j in pending:
                if j["parent"] not in W:
                    rest.append(j)
                    continue
                M = _mul(W[j["parent"]], j["T"])
                if j["type"] in ("revolute", "continuous") and j["name"] in q:
                    M = _mul(M, _axis_angle(j["axis"], q[j["name"]]))
                W[j["child"]] = M
            if len(rest) == len(pending):        # broken tree: stop rather than loop forever
                break
            pending = rest
        return W

    def torques(self, deg, payload_kg=0.0):
        """deg: {joint: LeRobot degrees}. Returns {joint: holding torque in kg*cm (absolute)}."""
        q = self._angles(deg)
        W = self._frames(q)
        tool = _apply(W[TOOL_LINK], [0.0, 0.0, 0.0]) if TOOL_LINK in W else None
        joint_by_name = {j["name"]: j for j in self.joints}
        out = {}
        for name in ARM:
            j = joint_by_name[name]
            M = W[j["child"]]
            p = [M[0][3], M[1][3], M[2][3]]
            a = [sum(M[i][k] * j["axis"][k] for k in range(3)) for i in range(3)]
            tau = 0.0
            items = [(self.masses[l][0], _apply(W[l], self.masses[l][1])) for l in self.carried[name]
                     if l in self.masses and l in W]
            if tool is not None and payload_kg > 0:
                items.append((payload_kg, tool))
            for m, r in items:                     # a . ((r - p) x (0, 0, -m g))
                d = [r[0] - p[0], r[1] - p[1], r[2] - p[2]]
                f = -m * G
                cross = [d[1] * f, -d[0] * f, 0.0]
                tau += a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]
            out[name] = abs(tau) / G * 100.0       # N*m -> kg*cm
        return out

    def percent(self, deg, payload_kg=0.0):
        """{joint: % of stall torque}."""
        return {j: v / self.stall * 100.0 for j, v in self.torques(deg, payload_kg).items()}

    def path_peak(self, poses, payload_kg, samples=8):
        """Highest % per joint along straight joint-space lines through `poses` (what the
        sequencer drives), sampled `samples` times per segment. Returns ({joint: %}, {joint: index})."""
        peak = {j: 0.0 for j in ARM}
        where = {j: 0 for j in ARM}
        for i in range(len(poses)):
            a = poses[i]
            b = poses[i + 1] if i + 1 < len(poses) else a
            for s in range(samples if i + 1 < len(poses) else 1):
                f = s / samples
                pose = {j: a[j] + f * (b[j] - a[j]) for j in ARM}
                for j, v in self.percent(pose, payload_kg).items():
                    if v > peak[j]:
                        peak[j], where[j] = v, i if f < 0.5 else min(i + 1, len(poses) - 1)
        return peak, where


if __name__ == "__main__":
    import json
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    calib = json.load(open(os.path.join(here, "twin_calib.json")))
    m = GravityModel(os.path.join(here, "urdf", "so101.urdf"), calib)
    print(f"URDF mass total {m.total_kg * 1000:.0f} g, stall {m.stall:g} kg*cm")
    poses = json.load(open(os.path.join(here, "taught_poses.json")))
    for kg in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
        t = m.torques(poses["cells"]["3"]["grab"], kg)
        print(f"{kg * 1000:4.0f} g at cell 3 grab: " + "  ".join(f"{j} {v:5.1f}" for j, v in t.items()))
    sys.exit(0)
