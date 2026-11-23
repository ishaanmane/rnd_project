"""Planner for the 3x3 sorting demo.

The board has 9 cells, 6 blocks (3 of colour A, 2 of B, 1 of C) and so always
3 empty cells. One move = pick a block from a cell and put it in an empty
cell. Blocks of one colour are interchangeable.

Goal (default): A in cells 1, 4, 7   B in cells 2, 8   C in cell 6.
Cells 3, 5 and 9 end empty, so they double as parking cells.

Key fact (checked by computer on all 5040 start layouts, see selftest()):

    fewest moves = misplaced blocks + number of "loops"

A loop is a group of colours whose blocks sit in each other's home cells
while none of their home cells is empty. Then no block of the group can go
straight home, so one block must be parked first: one extra move.

Algorithms the user can choose:
    greedy_simple  move any block straight home, park one when stuck
    greedy_smart   the same, but prefer a block that sits in another colour's
                   home (moving it also frees a home cell)
    bfs            breadth-first search: always the fewest moves
    astar          A* search guided by "misplaced + loops": fewest moves,
                   usually explores far fewer layouts than BFS
    dijkstra       Dijkstra on robot TIME, not on move count: always the
                   fastest plan for the real arm (needs a TimeModel)

Why Dijkstra counts seconds: with every move costing 1, Dijkstra gives exactly
the same answer as BFS. But moves do not cost the same on the robot: cell 1 to
cell 9 takes longer than cell 4 to cell 7, and where the arm starts each move
matters too. So the Dijkstra state is (board, where the arm is) and the cost of
a move is the time the sequencer in twin_server.py needs for it, worked out from
the taught poses and the speed cap (see TimeModel). Sometimes the fastest plan
even uses one move more than the minimum.

The demo menu offers greedy_simple, bfs and dijkstra. The others stay for the
command line and the self test.

Only the standard library is used.
"""
from collections import Counter, deque
import heapq
import itertools

CELLS = tuple(range(1, 10))
DEFAULT_TARGETS = {"A": (1, 4, 7), "B": (2, 8), "C": (6,)}
# Demo block set (Nov 2026): 3 green, 2 yellow, 2 blue, 2 red. Green is always the colour with
# 3 blocks; B (2 blocks) and C (1 block) are any two of yellow, blue, red. Checking this catches
# a detector mistake (for example 3 "blue") before the robot moves. None = no check.
MAIN_COLOR = "green"
ROLE_COUNTS = {"A": 3, "B": 2, "C": 1}

ALGORITHMS = {
    "greedy_simple": "Greedy (simple)",
    "greedy_smart": "Greedy (smart)",
    "bfs": "BFS (fewest moves)",
    "astar": "A* (fewest moves)",
    "dijkstra": "Dijkstra (fastest)",
}
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


class PlanError(ValueError):
    pass


# ---- board helpers ---------------------------------------------------------------
def normalise_board(cells):
    """{"1": "blue", 2: None, ...} -> tuple of 9 entries (colour or None)."""
    out = [None] * 9
    for k, v in (cells or {}).items():
        n = int(k)
        if n not in CELLS:
            raise PlanError(f"cell {k} does not exist (use 1 to 9)")
        if v:
            out[n - 1] = str(v).strip().lower()
    return tuple(out)


def board_dict(state):
    return {str(i + 1): c for i, c in enumerate(state)}


def infer_roles(state, main=MAIN_COLOR):
    """The colour with 3 blocks is A, with 2 is B, with 1 is C. A must be `main` if given."""
    counts = Counter(c for c in state if c)
    by_count = {}
    for colour, n in counts.items():
        by_count.setdefault(n, []).append(colour)
    if sorted(counts.values()) != [1, 2, 3] or len(counts) != 3:
        found = ", ".join(f"{n} {c}" for c, n in sorted(counts.items())) or "no blocks"
        raise PlanError(f"Need 3 + 2 + 1 blocks of three different colours. Found: {found}.")
    if main and by_count[3][0] != main:
        found = ", ".join(f"{n} {c}" for c, n in sorted(counts.items()))
        raise PlanError(f"{main.capitalize()} must be the colour with 3 blocks. Found: {found}. "
                        f"Check the board (or the scan).")
    return {"A": by_count[3][0], "B": by_count[2][0], "C": by_count[1][0]}


def goal_map(roles, targets=None):
    """{cell: colour} for the finished board."""
    targets = targets or DEFAULT_TARGETS
    g = {}
    for role, cells in targets.items():
        for c in cells:
            g[int(c)] = roles[role]
    return g


def goal_state(goal):
    return tuple(goal.get(c) for c in CELLS)


def misplaced(state, goal):
    return sum(1 for i, c in enumerate(state, 1) if c and goal.get(i) != c)


def loops(state, goal):
    """Groups of colours that block each other with no free home cell.

    Returns a list of colour sets, one per loop. len() is the number of extra
    (parking) moves the best plan needs.
    """
    edges = [(goal[i], c) for i, c in enumerate(state, 1) if c and i in goal and goal[i] != c]
    holes = Counter(goal[i] for i, c in enumerate(state, 1) if not c and i in goal)
    parked = {c for i, c in enumerate(state, 1) if c and i not in goal}
    adj = {}
    for a, b in edges:
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    seen, out = set(), []
    for n in adj:
        if n in seen:
            continue
        comp, stack = set(), [n]
        while stack:
            x = stack.pop()
            if x not in comp:
                comp.add(x)
                stack.extend(adj[x])
        seen |= comp
        if not any(holes[x] for x in comp) and not (comp & parked):
            out.append(comp)
    return out


def lower_bound(state, goal):
    return misplaced(state, goal) + len(loops(state, goal))


def neighbours(state):
    for i in range(9):
        if not state[i]:
            continue
        for j in range(9):
            if state[j]:
                continue
            t = list(state)
            t[j], t[i] = t[i], None
            yield (i + 1, j + 1), tuple(t)


def apply(state, src, dst):
    if not state[src - 1]:
        raise PlanError(f"cell {src} is empty")
    if state[dst - 1]:
        raise PlanError(f"cell {dst} is not empty")
    t = list(state)
    t[dst - 1], t[src - 1] = t[src - 1], None
    return tuple(t)


# ---- algorithms ------------------------------------------------------------------
def _greedy(start, goal, smart):
    s, moves, explored = start, [], 0
    target = goal_state(goal)
    while s != target:
        explored += 1
        direct = []
        for i, c in enumerate(s, 1):
            if not c or goal.get(i) == c:
                continue
            for j in CELLS:
                if not s[j - 1] and goal.get(j) == c:
                    direct.append((i, j))
        if direct:
            if smart:      # a block sitting in another colour's home first: it frees that home
                direct.sort(key=lambda m: (m[0] not in goal, m[0], m[1]))
            src, dst = direct[0]
        else:              # stuck: park a block that is blocking a home cell
            src = next(i for i, c in enumerate(s, 1) if c and i in goal and goal[i] != c)
            dst = next(j for j in CELLS if not s[j - 1] and j not in goal)
        moves.append((src, dst))
        s = apply(s, src, dst)
        if len(moves) > 20:
            raise PlanError("greedy did not finish (bug)")
    return moves, explored


def _path(parent, s):
    moves = []
    while parent[s] is not None:
        prev, m = parent[s]
        moves.append(m)
        s = prev
    return moves[::-1]


def _bfs(start, goal):
    target = goal_state(goal)
    parent, q, explored = {start: None}, deque([start]), 0
    while q:
        s = q.popleft()
        explored += 1
        if s == target:
            return _path(parent, s), explored
        for m, t in neighbours(s):
            if t not in parent:
                parent[t] = (s, m)
                q.append(t)
    raise PlanError("no plan found")


def _astar(start, goal):
    target = goal_state(goal)
    tie = itertools.count()
    g = {start: 0}
    parent = {start: None}
    heap = [(lower_bound(start, goal), next(tie), start)]
    closed, explored = set(), 0
    while heap:
        _, _, s = heapq.heappop(heap)
        if s in closed:
            continue
        closed.add(s)
        explored += 1
        if s == target:
            return _path(parent, s), explored
        for m, t in neighbours(s):
            ng = g[s] + 1
            if ng < g.get(t, 1 << 30):
                g[t] = ng
                parent[t] = (s, m)
                heapq.heappush(heap, (ng + lower_bound(t, goal), next(tie), t))
    raise PlanError("no plan found")


def _optimal_moves(start, goal):
    return lower_bound(start, goal)      # exact, see selftest()


# ---- robot time ------------------------------------------------------------------
class TimeModel:
    """Seconds the robot needs for a plan, worked out the way twin_server.py runs it.

    poses: the taught poses (same layout as taught_poses.json: "home", "cells", "gripper").
    speed: the speed cap in deg/s (the gripper moves in %/s at the same number).

    - Every arm motion is one straight line in joint space at the speed cap, so it takes
      (largest joint change) / speed.
    - A gripper step takes (change) / speed, then the dwell (grip_dwell seconds).
    - One block move: hover over src, lower, close, lift, hover over dst, lower, open, lift.
    - A plan starts at Home (first it opens the gripper) and ends back at Home.

    Not counted: the short wait after a motion while the servos catch up (up to 1 s, see
    ARRIVE_WAIT in twin_server.py). So the real run takes a little longer than this.
    Arm positions are numbers: 0 = at Home, n = hovering over cell n.
    """

    def __init__(self, poses, speed, grip_dwell=0.6):
        self.speed = max(float(speed), 1e-3)
        self.dwell = float(grip_dwell)
        g = poses["gripper"]
        self.g_open, self.g_grip, self.g_home = float(g["open"]), float(g["grip"]), float(g["home"])
        self.home = {j: float(poses["home"][j]) for j in ARM_JOINTS}
        self.cells = {}
        for n in CELLS:
            c = (poses.get("cells") or {}).get(str(n)) or {}
            if c.get("hover") and c.get("grab"):
                self.cells[n] = tuple({j: float(c[k][j]) for j in ARM_JOINTS} for k in ("hover", "grab"))
        self._cache = {}

    def missing(self, cells=CELLS):
        return [n for n in sorted(set(cells)) if n not in self.cells]

    def require(self, cells=CELLS):
        m = self.missing(cells)
        if m:
            raise PlanError("Dijkstra needs the hover and grab poses of every cell. "
                            f"Teach cell{'s' if len(m) > 1 else ''} {', '.join(map(str, m))} first.")

    def _arm(self, a, b):
        return max(abs(a[j] - b[j]) for j in ARM_JOINTS) / self.speed

    def _grip(self, a, b):
        return abs(a - b) / self.speed + self.dwell

    def _at(self, arm):
        return self.home if arm == 0 else self.cells[arm][0]

    def move(self, arm, src, dst):
        """Seconds for one block move from src to dst, with the arm starting at `arm`."""
        key = (arm, src, dst)
        t = self._cache.get(key)
        if t is None:
            (hs, gs), (hd, gd) = self.cells[src], self.cells[dst]
            t = (self._arm(self._at(arm), hs) + 2 * self._arm(hs, gs) + self._grip(self.g_open, self.g_grip)
                 + self._arm(hs, hd) + 2 * self._arm(hd, gd) + self._grip(self.g_grip, self.g_open))
            self._cache[key] = t
        return t

    def start(self):
        """Opening the gripper at Home, before the first move."""
        return self._grip(self.g_home, self.g_open)

    def finish(self, arm):
        """Back to Home from `arm`, then the gripper to its Home value."""
        return self._arm(self._at(arm), self.home) + self._grip(self.g_open, self.g_home)

    def plan_seconds(self, moves):
        if not moves:
            return 0.0
        t, arm = self.start(), 0
        for src, dst in moves:
            t += self.move(arm, src, dst)
            arm = dst
        return t + self.finish(arm)


def _dijkstra(start, goal, tm):
    """Fewest robot seconds. A node is (board, arm position). Returns (moves, nodes explored)."""
    target = goal_state(goal)
    if start == target:
        return [], 1
    tm.require()
    end = ("end",)                       # one extra node: "sorted and back at Home"
    tie = itertools.count()
    s0 = (start, 0)
    best, parent = {s0: tm.start()}, {s0: None}
    heap = [(best[s0], next(tie), s0)]
    closed, explored = set(), 0
    while heap:
        d, _, node = heapq.heappop(heap)
        if node in closed:
            continue
        closed.add(node)
        explored += 1
        if node == end:
            break
        board, arm = node
        if board == target:              # sorted: only the trip back Home is left
            nd = d + tm.finish(arm)
            if nd < best.get(end, float("inf")):
                best[end], parent[end] = nd, (node, None)
                heapq.heappush(heap, (nd, next(tie), end))
            continue
        for (src, dst), nb in neighbours(board):
            nxt, nd = (nb, dst), d + tm.move(arm, src, dst)
            if nd < best.get(nxt, float("inf")):
                best[nxt], parent[nxt] = nd, (node, (src, dst))
                heapq.heappush(heap, (nd, next(tie), nxt))
    moves, node = [], end
    while parent[node] is not None:
        node, m = parent[node]
        if m:
            moves.append(m)
    return moves[::-1], explored


def _time_table(tm, goal):
    """For the self test: the fewest seconds from EVERY (board, arm) node, by one Dijkstra run
    backwards from the goal. Independent of _dijkstra(), so it can check it."""
    target = goal_state(goal)
    tie = itertools.count()
    heap = [(tm.finish(a), next(tie), (target, a)) for a in [0] + [n for n in CELLS if target[n - 1]]]
    table = {}
    while heap:
        w, _, node = heapq.heappop(heap)
        if node in table:
            continue
        table[node] = w
        after, dst = node
        if dst == 0:
            continue
        for src in CELLS:                # the move that led here was src -> dst
            if after[src - 1]:
                continue
            b = list(after)
            b[src - 1], b[dst - 1] = after[dst - 1], None
            b = tuple(b)
            for arm in [0] + list(CELLS):
                if arm and b[arm - 1] is None:   # the arm hovers where it last put a block
                    continue
                if (b, arm) not in table:
                    heapq.heappush(heap, (w + tm.move(arm, src, dst), next(tie), (b, arm)))
    return table


# ---- explanations ----------------------------------------------------------------
def _explain(state, src, dst, goal):
    colour = state[src - 1]
    src_home = goal.get(src)
    if goal.get(dst) == colour:
        if src_home and src_home != colour:
            return "home_frees", (f"{colour} goes home to cell {dst}. "
                                  f"This also frees cell {src} for {src_home}.")
        if src not in goal:
            return "home", f"{colour} leaves parking cell {src} and goes home to cell {dst}."
        return "home", f"{colour} goes home to cell {dst}."
    for comp in loops(state, goal):
        if colour in comp:
            names = " and ".join(sorted(comp))
            return "park", (f"Loop: {names} sit in each other's home cells and none of their "
                            f"home cells is free. Park {colour} in cell {dst} to break the loop.")
    return "park", f"Park {colour} in cell {dst} for now."


def plan(cells, algorithm="astar", targets=None, main=MAIN_COLOR, time_model=None):
    """Main entry. cells: {"1": "blue", ...}. Returns a JSON-ready dict.

    time_model: a TimeModel. Needed for "dijkstra". With it, every plan also gets
    summary["seconds"], the predicted robot time (when its cells are taught).
    """
    if algorithm not in ALGORITHMS:
        raise PlanError(f"unknown algorithm {algorithm}, use one of {', '.join(ALGORITHMS)}")
    start = normalise_board(cells)
    roles = infer_roles(start, main)
    goal = goal_map(roles, targets)
    if algorithm == "greedy_simple":
        moves, explored = _greedy(start, goal, smart=False)
    elif algorithm == "greedy_smart":
        moves, explored = _greedy(start, goal, smart=True)
    elif algorithm == "bfs":
        moves, explored = _bfs(start, goal)
    elif algorithm == "dijkstra":
        if time_model is None:
            raise PlanError("Dijkstra needs the taught poses to work out robot time.")
        moves, explored = _dijkstra(start, goal, time_model)
    else:
        moves, explored = _astar(start, goal)
    seconds = None
    if time_model is not None and not time_model.missing(c for m in moves for c in m):
        seconds = round(time_model.plan_seconds(moves), 1)

    steps, s = [], start
    for k, (src, dst) in enumerate(moves, 1):
        kind, why = _explain(s, src, dst, goal)
        steps.append({"n": k, "src": src, "dst": dst, "color": s[src - 1], "kind": kind, "why": why})
        s = apply(s, src, dst)
    assert s == goal_state(goal)

    m, lp = misplaced(start, goal), loops(start, goal)
    best, n = m + len(lp), len(steps)
    if algorithm == "dijkstra":
        quality = " (also the fewest moves)" if n == best else f" ({n - best} more than the minimum, but faster)"
    else:
        quality = " (optimal)" if n == best else f" ({n - best} more than needed)"
    timing = f", {seconds:.1f} s robot time" if seconds else ""
    explored_what = "States (board + arm position)" if algorithm == "dijkstra" else "Layouts"
    return {
        "algorithm": algorithm,
        "algorithm_name": ALGORITHMS[algorithm],
        "roles": roles,
        "goal": board_dict(goal_state(goal)),
        "start": board_dict(start),
        "moves": steps,
        "summary": {
            "misplaced": m,
            "loops": [sorted(c) for c in lp],
            "minimum": best,
            "length": n,
            "optimal": n == best,             # fewest moves
            "fastest": algorithm == "dijkstra",
            "seconds": seconds,               # predicted robot time, None if unknown
            "explored": explored,
            "short": (f"Plan: {n} move{'s' if n != 1 else ''}"
                      + (f", about {seconds:.0f} s." if seconds else ".")),
            "text": (f"{m} misplaced + {len(lp)} loop{'s' if len(lp) != 1 else ''} = {best} moves minimum. "
                     f"{ALGORITHMS[algorithm]} plan: {n} moves{quality}{timing}. "
                     f"{explored_what} explored: {explored}."),
        },
    }


# ---- self test -------------------------------------------------------------------
def selftest(tm=None, sample=300):
    """Check every claim in the docstring on all 5040 start layouts.

    With a TimeModel (taught poses + speed) it also checks Dijkstra: no other planner is ever
    faster, and Dijkstra matches the exact fastest time (a separate backward search) on
    `sample` random layouts (all 5040 take about 15 minutes).
    """
    roles = {"A": "green", "B": "blue", "C": "red"}
    goal = goal_map(roles)
    target = goal_state(goal)
    items = ["green"] * 3 + ["blue"] * 2 + ["red"] + [None] * 3
    starts = sorted(set(itertools.permutations(items)), key=lambda s: [c or "" for c in s])
    dist = {target: 0}
    q = deque([target])
    while q:                                   # exact distances: BFS backwards from the goal
        s = q.popleft()
        for _, t in neighbours(s):
            if t not in dist:
                dist[t] = dist[s] + 1
                q.append(t)
    assert len(starts) == 5040
    extra = {"greedy_simple": Counter(), "greedy_smart": Counter()}
    timed = ("greedy_simple", "bfs", "dijkstra")
    secs = {a: [] for a in timed}
    moves_n = {a: [] for a in timed}
    table = _time_table(tm, goal) if tm else None
    for s in starts:
        assert dist[s] == lower_bound(s, goal), s
        cells = board_dict(s)
        for alg in ALGORITHMS:
            if alg == "dijkstra":
                continue
            p = plan(cells, alg, time_model=tm)
            n = p["summary"]["length"]
            if alg in ("bfs", "astar"):
                assert n == dist[s], (alg, s)
            else:
                extra[alg][n - dist[s]] += 1
            if tm and alg in secs:
                t = tm.plan_seconds([(m["src"], m["dst"]) for m in p["moves"]])
                assert t >= table[(s, 0)] + tm.start() - 1e-6 or not p["moves"], (alg, s)
                secs[alg].append(t)
                moves_n[alg].append(n)
        if tm:                           # the exact fastest time, from the backward table
            fastest = 0.0 if s == target else table[(s, 0)] + tm.start()
            secs["dijkstra"].append(fastest)
    loops_n = sum(1 for s in starts if loops(s, goal))
    print("5040 layouts: fewest moves = misplaced + loops on every one")
    print("BFS and A* are optimal on every one")
    print("layouts with a loop:", loops_n)
    for alg, c in extra.items():
        print(f"{alg}: extra moves -> layouts {dict(sorted(c.items()))}")
    if not tm:
        print("(no taught poses given: Dijkstra not checked)")
        return True

    import random
    rng = random.Random(0)
    picks = starts if sample >= len(starts) else rng.sample(starts, sample)
    index = {s: i for i, s in enumerate(starts)}
    longest = 0
    for s in picks:
        p = plan(board_dict(s), "dijkstra", time_model=tm)
        mv = [(m["src"], m["dst"]) for m in p["moves"]]
        assert abs(tm.plan_seconds(mv) - secs["dijkstra"][index[s]]) < 1e-6, s
        longest = max(longest, len(mv))
        moves_n["dijkstra"].append(len(mv))
    N = len(starts)
    print(f"\nRobot time at {tm.speed:g} deg/s, gripper dwell {tm.dwell:g} s (Home to Home, no settle waits):")
    print(f"Dijkstra matches the exact fastest time on {len(picks)} checked layouts "
          f"(longest Dijkstra plan there: {longest} moves)")
    print("no other planner is faster on any of the 5040 layouts")
    for alg in timed:
        ts = secs[alg]
        mv = moves_n[alg]
        print(f"  {ALGORITHMS[alg]:22s} mean {sum(ts) / N:5.1f} s   worst {max(ts):5.1f} s   "
              f"mean moves {sum(mv) / len(mv):.2f}" + ("  (sampled)" if alg == "dijkstra" else ""))
    for ref in ("greedy_simple", "bfs"):
        gain = [a - b for a, b in zip(secs[ref], secs["dijkstra"])]
        pct = [g / a * 100 for g, a in zip(gain, secs[ref]) if a > 0]
        print(f"  Dijkstra vs {ALGORITHMS[ref]}: faster on {sum(1 for g in gain if g > 0.05)} layouts, "
              f"mean {sum(gain) / N:.1f} s ({sum(pct) / len(pct):.1f} %), best {max(gain):.1f} s")
    return True


def load_time_model(path=None, speed=30.0, grip_dwell=0.6):
    """TimeModel from a taught_poses.json file (default: the one next to this file), or None."""
    import json
    import os
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "taught_poses.json")
    try:
        with open(path) as f:
            data = json.load(f)
        return TimeModel(data, speed, grip_dwell)
    except (OSError, ValueError, KeyError, TypeError):
        return None


if __name__ == "__main__":
    # python sorting.py                       example plans (the layout in the phone photo)
    # python sorting.py test [poses.json]     self test; Dijkstra too if the poses file loads
    # add --speed N to change the speed cap used for robot time (default 30 deg/s)
    import json
    import sys
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    speed = 30.0
    if "--speed" in sys.argv:
        speed = float(sys.argv[sys.argv.index("--speed") + 1])
        args = [a for a in args if a != sys.argv[sys.argv.index("--speed") + 1]]
    if args and args[0] == "test":
        tm = load_time_model(args[1] if len(args) > 1 else None, speed)
        selftest(tm)
    else:
        tm = load_time_model(None, speed)
        demo = {"1": "green", "3": "yellow", "4": "yellow", "5": "red", "7": "green", "9": "green"}
        for a in ALGORITHMS:
            if a == "dijkstra" and not tm:
                print("Dijkstra skipped: no taught_poses.json next to sorting.py")
                continue
            p = plan(demo, a, time_model=tm)
            print(p["summary"]["text"])
            for m in p["moves"]:
                print(f"  {m['n']}. {m['src']} -> {m['dst']}  {m['why']}")
        json.dump(plan(demo, "astar"), sys.stdout, indent=1) if "--json" in sys.argv else None
