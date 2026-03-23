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

## Phase 3 — Architecture Refactor ✅

Split the KRRT God Object into focused, single-responsibility classes.

### Architecture

```
KRRT/
├── Dynamics.h   — pure physics: f(), rk4step(), euler_step(), propagate(), check_collision()
├── Sampler.h    — random sampling: owns engine + all distributions (was file-scope globals)
├── Renderer.h   — visual objects + OpenGL calls: setup(), update(), draw(), reset_trail()
├── KRRT.h       — planner: composes above, all internals private
└── KRRT.cpp     — implementation; delegates physics/sampling/rendering to sub-systems
```

| # | Task | Status |
|---|------|--------|
| 1 | Extract `Dynamics` class — `f()`, `rk4step()`, `euler_step()`, `propagate()` | ✅ Done |
| 2 | Extract `Sampler` class — file-scope globals moved into class with clean API | ✅ Done |
| 3 | Extract `Renderer` class — all visual objects and OpenGL calls isolated | ✅ Done |
| 4 | Privatize KRRT internals — only public: sub-systems, plan, render-loop state | ✅ Done |
| 5 | Remove visual objects from KRRT (`husky_robot`, `path`, `t`, `start_pos`, `goal_pos`) | ✅ Done |
| 6 | Remove dead code from KRRT.cpp (`nearest_nn_idx`, `LQR_Cost`, `map2block`, commented blocks) | ✅ Done |
| 7 | Fix hardcoded CSV path in `planner.cpp` — now writes to `results.csv` relative path | ✅ Done |

---

## Phase 4 — Code Quality

| # | Issue | Location | Status |
|---|-------|----------|--------|
| 1 | Magic numbers scattered everywhere | `KRRT.h`, `KRRT.cpp` | ✅ Already named constants (`time2exit`, `tether_length`, `tolerance`, `K`, `h`, `eps`) |
| 2 | Hardcoded file path `C:/Users/arisa/Desktop/...` | `KRRT.cpp:plan_trials()` | ✅ Done — changed to `results.csv` |
| 3 | Debug `printf("%d", axis)` left in production code | `KDtree/KDtree.cpp` | ✅ Done — removed |
| 4 | `squared_distance()` actually called `sqrt()` (misnamed, wrong pruning) | `KDtree/KDtree.cpp` | ✅ Done — removed sqrt; updated caller to compare `distance <= r*r` |
| 5 | `pos_idx` grows every frame | `object/object.cpp` | N/A — intentional trail buffer; cleared by `reset_trail()` |
| 6 | Raw `new`/`delete` throughout | `KRRT.cpp`, `KDtree.cpp`, `map.cpp` | Partial — see [Memory Management](#memory-management-phase-4-item-6) below |
| 7 | Dead commented-out `Draw_object_Angle` in object.cpp | `object/object.cpp` | ✅ Done — removed |
| 8 | Missing virtual destructor on `object` base class | `object/object.h` | ✅ Done — added `virtual ~object() = default;` |
| 9 | `KDTree` destructor was empty — leak if `cleanUp()` not called | `KDtree/KDtree.h` | ✅ Done — destructor now calls `cleanup(root)` |

---

## Phase 5 — CMake Cleanup

| # | Task | Status |
|---|------|--------|
| 1 | Add compiler warning flags (`/W4` on MSVC, `-Wall -Wextra` on GCC/Clang) | ✅ Done — applied to all project targets via `foreach` in root CMakeLists.txt; third-party libs unaffected |
| 2 | Restrict `fssimplewindow` linkage to only libraries that call OpenGL | ✅ Done — removed from `KRRT` (gets it transitively via `object`); `map` and `object` keep it (both call OpenGL directly) |

---

## Memory Management (Phase 4, Item 6)

### The Problem

In C++, when you allocate memory with `new`, you are responsible for freeing it later with `delete`.
If you forget — or if an error/exception happens before you reach the `delete` — that memory is
**leaked**: it stays reserved until the program exits, but nothing can use it.

This codebase had two places using raw `new`:

1. **The occupancy grid (`map_ptr`)** — a flat array holding the map data (0 = free, 1 = obstacle).
2. **Tree nodes (`node*`)** — every time the planner explores a new state, it creates a `node` and
   inserts it into a KD-tree (a spatial search structure).

### What is `std::unique_ptr`?

`std::unique_ptr` is a C++ standard-library wrapper around a raw pointer. It works like a normal
pointer (you can dereference it, index into it), but it **automatically frees the memory when it
goes out of scope**. You never write `delete` yourself — the language handles it for you.

```cpp
// Old — you must remember to call delete[], or it leaks:
double* data = new double[100];
// ... use data ...
delete[] data;  // easy to forget

// New — freed automatically when 'data' is destroyed:
std::unique_ptr<double[]> data = std::make_unique<double[]>(100);
// ... use data[i] exactly the same way ...
// no delete needed — cleanup is automatic
```

The word "unique" means **exactly one owner**. You cannot copy a `unique_ptr` — you can only
*move* it. This makes ownership explicit: whoever holds the `unique_ptr` is responsible for the
memory, and when that holder is destroyed, the memory is freed.

### What We Fixed — `map_ptr` (✅ Done)

The `map` class allocated its grid with `new double[height * width]` but had **no destructor** to
free it. Every time a `map` object was destroyed, that grid memory leaked.

**Fix:** Changed `double*` to `std::unique_ptr<double[]>`. Now the grid is automatically freed when
the `map` object is destroyed. All existing code that reads the grid (`map_ptr[i]`) works without
changes because `unique_ptr<double[]>` supports `[]` indexing. The one place that passes the raw
pointer to the physics engine (`Dynamics::propagate`) now calls `.get()` to obtain the underlying
`double*`.

| File | Change |
|------|--------|
| `map/map.h` | `double *map_ptr` → `std::unique_ptr<double[]> map_ptr` |
| `map/map.cpp` | `new double[...]` → `std::make_unique<double[]>(...)` ; removed unused `map(double*)` constructor |
| `KRRT/KRRT.cpp` | `map_1.map_ptr` → `map_1.map_ptr.get()` (passes raw pointer to physics code) |

### What Remains — Tree Nodes (⬜ Deferred)

Each `node` in the KD-tree has two child pointers (`left`, `right`) and a back-pointer to its
`parent`. The KD-tree *owns* these nodes (it allocates and deletes them). The planner's `plan`
vector holds pointers to some of these same nodes, but it does not own them — it just reads them.

```
KDTree (owner)
  └── root
       ├── left  ──▶ node ──▶ ...
       └── right ──▶ node ──▶ ...

plan[] (observer — points into the same nodes, does not own them)
  [0] ──▶ node (in tree)
  [1] ──▶ node (in tree)
  ...
```

Converting this to `unique_ptr` would mean:
- `node::left` and `node::right` become `std::unique_ptr<node>` (owning children).
- `node::parent` stays as a raw `node*` (non-owning back-pointer — the parent does not belong to
  the child).
- `plan` stays as `std::vector<node*>` (non-owning observers).
- The `KDTree::Insert()` method would need to accept `std::unique_ptr<node>` and *move* it into
  the tree, rather than taking a raw pointer.
- The recursive `cleanup()` function and destructor become trivial — destroying the root
  automatically cascades to all children.

This is deferred because it touches **4–5 files** and changes the KDTree's public API. The current
code is safe because the KDTree destructor (fixed in Phase 4 item 9) now properly frees all nodes
on destruction.

---

## Notes

- Third-party libraries (`public`, `MMLPlayer`) are managed as git submodules under `third_party/`.
- Results CSV now writes to `results.csv` in the working directory (was hardcoded to a user-specific desktop path).
