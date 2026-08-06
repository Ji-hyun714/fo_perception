#!/usr/bin/env python3

import argparse
import math
import sys
import yaml


# --------------------------------------------------
# Geometry
# --------------------------------------------------

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def solve_circle_intersection(A, B, AP, BP, tol):
    """
    두 원 교점 계산
    A 중심 반지름 AP
    B 중심 반지름 BP
    """

    ax, ay = A
    bx, by = B

    dx = bx - ax
    dy = by - ay

    d = math.hypot(dx, dy)

    if d < tol:
        raise ValueError("A and B are identical points")

    # unit vector AB
    ux = dx / d
    uy = dy / d

    # perpendicular vector
    vx = -uy
    vy = ux

    # circle intersection
    a = (AP*AP - BP*BP + d*d) / (2*d)
    h2 = AP*AP - a*a

    if h2 < -tol:
        raise ValueError("triangle_inequality_failed")

    if h2 < 0:
        h2 = 0

    h = math.sqrt(h2)

    mx = ax + a * ux
    my = ay + a * uy

    p1 = (mx + h * vx, my + h * vy)
    p2 = (mx - h * vx, my - h * vy)

    return p1, p2


# --------------------------------------------------
# Solution selection
# --------------------------------------------------

def select_solution(A, B, P1, P2, mode):
    if mode == "both":
        return [P1, P2]

    ax, ay = A
    bx, by = B

    def cross(P):
        px, py = P
        return (bx-ax)*(py-ay) - (by-ay)*(px-ax)

    c1 = cross(P1)
    c2 = cross(P2)

    if mode == "positive_cross":
        return [P1 if c1 >= 0 else P2]

    if mode == "negative_cross":
        return [P1 if c1 <= 0 else P2]

    raise ValueError(f"Unknown solution selection: {mode}")


# --------------------------------------------------
# Target solver
# --------------------------------------------------

def solve_target(A, B, target, default_mode, tol):

    if "id" not in target:
        raise ValueError("target missing id")

    tid = target["id"]

    if "AP" not in target or "BP" not in target:
        raise ValueError("target missing AP/BP")

    AP = float(target["AP"])
    BP = float(target["BP"])

    if AP <= 0 or BP <= 0:
        raise ValueError("AP/BP must be positive")

    mode = target.get("solution_selection", default_mode)

    p1, p2 = solve_circle_intersection(A, B, AP, BP, tol)

    result = select_solution(A, B, p1, p2, mode)

    return tid, result


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    if "fixed_points" not in cfg:
        raise ValueError("missing fixed_points")

    A = tuple(cfg["fixed_points"]["A"])
    B = tuple(cfg["fixed_points"]["B"])

    if A == B:
        raise ValueError("A and B cannot be identical")

    options = cfg.get("options", {})

    tol = float(options.get("tolerance", 1e-6))
    default_mode = options.get("default_solution_selection", "positive_cross")
    continue_on_error = options.get("continue_on_error", True)

    targets = cfg.get("targets", [])

    seen_ids = set()

    for t in targets:

        try:

            tid, result = solve_target(A, B, t, default_mode, tol)

            if tid in seen_ids:
                raise ValueError("duplicate id")

            seen_ids.add(tid)

            if len(result) == 1:

                x, y = result[0]
                print(f"[{tid}] status=ok, P=({x:.6f}, {y:.6f})")

            else:

                (x1, y1), (x2, y2) = result
                print(f"[{tid}] status=ok, P1=({x1:.6f},{y1:.6f}), P2=({x2:.6f},{y2:.6f})")

        except Exception as e:

            print(f"[{t.get('id','UNKNOWN')}] status=error, reason={str(e)}")

            if not continue_on_error:
                sys.exit(1)


if __name__ == "__main__":
    main()