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

Only the standard library is used.
"""
from collections import Counter, deque
import heapq
import itertools

CELLS = tuple(range(1, 10))
DEFAULT_TARGETS = {"A": (1, 4, 7), "B": (2, 8), "C": (6,)}
ROLE_COUNTS = {"A": 3, "B": 2, "C": 1}

ALGORITHMS = {
    "greedy_simple": "Greedy (simple)",
    "greedy_smart": "Greedy (smart)",
    "bfs": "BFS (fewest moves)",
    "astar": "A* (fewest moves)",
}


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


def infer_roles(state):
    """The colour with 3 blocks is A, with 2 is B, with 1 is C."""
    counts = Counter(c for c in state if c)
    by_count = {}
    for colour, n in counts.items():
        by_count.setdefault(n, []).append(colour)
    if sorted(counts.values()) != [1, 2, 3] or len(counts) != 3:
        found = ", ".join(f"{n} {c}" for c, n in sorted(counts.items())) or "no blocks"
        raise PlanError(f"Need 3 + 2 + 1 blocks of three different colours. Found: {found}.")
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


def plan(cells, algorithm="astar", targets=None):
    """Main entry. cells: {"1": "blue", ...}. Returns a JSON-ready dict."""
    if algorithm not in ALGORITHMS:
        raise PlanError(f"unknown algorithm {algorithm}, use one of {', '.join(ALGORITHMS)}")
    start = normalise_board(cells)
    roles = infer_roles(start)
    goal = goal_map(roles, targets)
    if algorithm == "greedy_simple":
        moves, explored = _greedy(start, goal, smart=False)
    elif algorithm == "greedy_smart":
        moves, explored = _greedy(start, goal, smart=True)
    elif algorithm == "bfs":
        moves, explored = _bfs(start, goal)
    else:
        moves, explored = _astar(start, goal)

    steps, s = [], start
    for k, (src, dst) in enumerate(moves, 1):
        kind, why = _explain(s, src, dst, goal)
        steps.append({"n": k, "src": src, "dst": dst, "color": s[src - 1], "kind": kind, "why": why})
        s = apply(s, src, dst)
    assert s == goal_state(goal)

    m, lp = misplaced(start, goal), loops(start, goal)
    best = m + len(lp)
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
            "length": len(steps),
            "optimal": len(steps) == best,
            "explored": explored,
            "text": (f"{m} misplaced + {len(lp)} loop{'s' if len(lp) != 1 else ''} = {best} moves minimum. "
                     f"{ALGORITHMS[algorithm]} plan: {len(steps)} moves"
                     f"{' (optimal)' if len(steps) == best else f' ({len(steps) - best} more than needed)'}. "
                     f"Layouts explored: {explored}."),
        },
    }


# ---- self test -------------------------------------------------------------------
def selftest():
    """Check every claim in the docstring on all 5040 start layouts."""
    roles = {"A": "blue", "B": "black", "C": "yellow"}
    goal = goal_map(roles)
    target = goal_state(goal)
    items = ["blue"] * 3 + ["black"] * 2 + ["yellow"] + [None] * 3
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
    for s in starts:
        assert dist[s] == lower_bound(s, goal), s
        cells = board_dict(s)
        for alg in ALGORITHMS:
            p = plan(cells, alg)
            n = p["summary"]["length"]
            if alg in ("bfs", "astar"):
                assert n == dist[s], (alg, s)
            else:
                extra[alg][n - dist[s]] += 1
    loops_n = sum(1 for s in starts if loops(s, goal))
    print("5040 layouts: fewest moves = misplaced + loops on every one")
    print("BFS and A* are optimal on every one")
    print("layouts with a loop:", loops_n)
    for alg, c in extra.items():
        print(f"{alg}: extra moves -> layouts {dict(sorted(c.items()))}")
    return True


if __name__ == "__main__":
    import json
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        selftest()
    else:      # example: the layout in the phone photo
        demo = {"1": "blue", "3": "black", "4": "black", "5": "yellow", "7": "blue", "9": "blue"}
        for a in ALGORITHMS:
            p = plan(demo, a)
            print(p["summary"]["text"])
            for m in p["moves"]:
                print(f"  {m['n']}. {m['src']} -> {m['dst']}  {m['why']}")
        json.dump(plan(demo, "astar"), sys.stdout, indent=1) if "--json" in sys.argv else None
