# NHT Planner — Refactor & Cleanup Plan

## Overview

This document tracks the ongoing effort to clean up, fix, and refactor the NHT_Planner codebase.
The planner implements a **Kinodynamic RRT (KRRT)** for a tether-constrained wheeled robot (Husky UGV)
in a 2D occupancy grid environment.

---

## Phase 1 — Critical Bug Fixes ✅

These were correctness-breaking bugs affecting the algorithm's output.

| # | File | Issue | Fix | Status |
|---|------|-------|-----|--------|
| 1 | `Xstate/Xstate.cpp:31-37` | Default constructor assigned all elements to `state_elem[0]` — elements [1,2,3] were uninitialized | Fixed indices to `[1]`, `[2]`, `[3]` | ✅ Done |
| 2 | `Xstate/state_template.h:24,32` | Bounds check used `>` instead of `>=` — allowed out-of-bounds access at `index == arr_size` | Changed to `>=` | ✅ Done |
| 3 | `KRRT/KRRT.cpp:351-361` | `ObstacleFree()` returned `true` in both branches — collisions never blocked node insertion | Fixed `else` branch to `return false` | ✅ Done |
| 4 | `KRRT/KRRT.cpp:198-225` | `nearest_n_idx()` returned the first neighbor within radius, not the nearest | Replaced early `return i` with proper min-tracking loop | ✅ Done |
| 5 | `object/object.cpp:242` | Floating-point exact equality `AB == AP+PB` — almost never triggered | Replaced with `std::abs(AB-(AP+PB)) < 1e-9` | ✅ Done |

---

## Phase 2 — Remove Dead Code ✅

Remove all commented-out legacy code, unused files, inactive `#if` branches, and unused class members.

| # | Location | What to Remove | Status |
|---|----------|----------------|--------|
| 1 | `Backup.cpp` | Entire file (~722 lines of commented-out legacy code) | ✅ Done |
| 2 | `KDtree/KDtreePoint.h` | Entire file is commented out — serves no purpose | ✅ Done |
| 3 | `Xstate/Xstate.cpp:71-242` | Large block of commented-out dead code (clothoid, collision variants) | ✅ Done |
| 4 | `KRRT/KRRT.cpp` | `#if LIN_TREE` blocks — linear tree path removed; KDTree path unwrapped | ✅ Done |
| 5 | `KRRT/KRRT.cpp` | `#if CONST` / `#if !CONST` blocks — active path unwrapped, dead path removed | ✅ Done |
| 6 | `planner.cpp` | `#if TRIALS` block removed; `plan_trials()` method kept for future use | ✅ Done |
| 7 | `node/node.h` | `childs` vector — removed | ✅ Done |
| 8 | `planner.cpp` | `ObstacleFinder` class — removed | ✅ Done |
| 9 | `object/object.cpp` | `Draw_in_center()`, `Draw_object_coords()` — removed; `Draw_Path()` inlined | ✅ Done |
| 10 | `map/map.cpp` | `setWhite()`, `setBlack()`, `printMap()` — removed | ✅ Done |
| 11 | `KRRT/KRRT.cpp` | `Render()` method — removed (duplicate of render loop in `planner.cpp`) | ✅ Done |
| 12 | `KRRT/KRRT.h` | `Add_Edge`, `UpdateControl`, `nearest_n_idx`, `getPlan_vector`, `Render`, duplicate `public:` — removed | ✅ Done |

---

## Phase 3 — Architecture Refactor

Split the KRRT God Object into focused, single-responsibility classes.

### Current Problem
`KRRT` owns: planning algorithm + physics simulation + rendering + object management +
file I/O + random state + statistics. All members are public.

### Target Architecture

```
KRRT/
├── Dynamics        — rk4step(), propagate_one_step(), dynamics()
│                     Pure physics, no external dependencies
├── Sampler         — random control/state sampling
│                     Owns the random engine and distributions (currently file-scope globals)
├── KRRT            — planning algorithm only
│                     Depends on: Dynamics, Sampler, KDTree, map
└── Renderer        — all OpenGL calls, object drawing, file output
                      Separated from planning logic entirely
```

| # | Task | Status |
|---|------|--------|
| 1 | Extract `Dynamics` class from KRRT (`dynamics()`, `rk4step()`, `propagate_one_step()`) | ⬜ Pending |
| 2 | Extract `Sampler` class — move file-scope random distributions into a class | ⬜ Pending |
| 3 | Extract `Renderer` class — move all OpenGL/object calls out of KRRT | ⬜ Pending |
| 4 | Make KRRT members private, expose only a clean planning interface | ⬜ Pending |
| 5 | Remove rendering state from KRRT (`husky_robot`, `path`, `t`, `start_pos`, `goal_pos`) | ⬜ Pending |

---

## Phase 4 — Code Quality

| # | Issue | Location | Status |
|---|-------|----------|--------|
| 1 | Magic numbers scattered everywhere (`0.085`, `70.0`, `5.0`, `0.01`, `200000`, `20.0`, etc.) | `KRRT.h`, `KRRT.cpp` | ⬜ Pending |
| 2 | Hardcoded file path `C:/Users/arisa/Desktop/...` | `planner.cpp:162`, `KRRT.cpp:513` | ⬜ Pending |
| 3 | Debug `printf("%d", axis)` left in production code | `KDtree/KDtree.cpp:167` | ⬜ Pending |
| 4 | `squared_distance()` in KDTree actually returns sqrt (misnamed) | `KDtree/KDtree.cpp` | ⬜ Pending |
| 5 | `pos_idx` in `object` grows every frame and is never cleared (memory leak) | `object/object.cpp:20-30` | ⬜ Pending |
| 6 | Raw `new`/`delete` throughout — replace with `std::unique_ptr` where ownership is clear | `KRRT.cpp`, `KDtree.cpp` | ⬜ Pending |
| 7 | Inconsistent naming: `q_near` vs `x_near` vs `Knode` for the same concept | `KRRT.cpp` | ⬜ Pending |
| 8 | Missing virtual destructor on `object` base class (`tether` inherits from it) | `object/object.h` | ⬜ Pending |
| 9 | `KDTree` has no destructor — potential leak if `cleanUp()` not called | `KDtree/KDtree.h` | ⬜ Pending |

---

## Phase 5 — CMake Cleanup

| # | Task | Status |
|---|------|--------|
| 1 | Add compiler warning flags (`-Wall -Wextra`) to catch future issues | ⬜ Pending |
| 2 | Restrict `fssimplewindow` linkage to only the libraries that render (currently leaks into `map` and `KRRT` unnecessarily) | ⬜ Pending |

---

## Notes

- Third-party libraries (`public`, `MMLPlayer`) are managed as git submodules under `third_party/`.
- The active planner path is `#if !LIN_TREE` (KDTree-based). The `LIN_TREE` path has not been tested and should be removed in Phase 2.
- The active control sampling path is `#if !CONST`. The `CONST` path should be removed in Phase 2.
- Results CSV output path must be made portable before sharing or running on another machine.
