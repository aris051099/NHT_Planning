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
| 10 | KDTree `axis = depth` instead of `depth % 4` — pruning broken past depth 4 | `KDtree/KDtree.cpp` | ✅ Done — see [KDTree Overhaul](#kdtree-overhaul-phase-4-item-10) below |

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

#### Background: How the KD-tree works in this planner

A **KD-tree** is a data structure for organizing points in multi-dimensional space so you can
quickly answer "which existing point is closest to this new point?" — a question the planner asks
thousands of times per run.

Every time the KRRT planner explores a new robot state, it:

1. Creates a new `node` on the heap with `new node(...)`.
2. Inserts that node into the KD-tree with `Ktree.Insert(q_new)`.
3. The KD-tree places the node in the right position by comparing coordinates along alternating
   axes (x, y, theta, beta), giving it O(log n) lookups instead of checking every node.

The tree grows to **tens of thousands of nodes** during a single planning run. When planning is
done, all those nodes must be freed.

#### Current ownership model

There are **two data structures** pointing at the same `node` objects, with different roles:

```
KDTree (the OWNER — responsible for creating and destroying nodes)
  └── root
       ├── left  ──▶ node
       │               ├── left  ──▶ node ──▶ ...
       │               └── right ──▶ node ──▶ ...
       └── right ──▶ node
                       ├── left  ──▶ ...
                       └── right ──▶ ...

plan[] (an OBSERVER — borrows pointers to some of those same nodes)
  [0] ──▶ node (lives inside the tree above)
  [1] ──▶ node (lives inside the tree above)
  [2] ──▶ node (lives inside the tree above)
  ...
```

- The **KD-tree** is the owner. It holds every node through `left`/`right` child pointers.
  When the tree is destroyed, it recursively walks the tree and calls `delete` on each node.
- The **plan** is an observer. After the planner finds a path, it walks from the goal node
  back to the start (following `parent` pointers) and collects those nodes into a vector.
  The plan never allocates or frees nodes — it just reads them.

This "one owner, many observers" pattern is common. The danger is that if the owner frees the
nodes while the observer still holds pointers to them, those pointers become **dangling** — they
point to memory that no longer belongs to us. Accessing a dangling pointer is undefined behavior
(crashes, corrupted data, or worse — it silently works until it doesn't).

In our code this is safe today because `plan` is always used *before* the KDTree is destroyed, and
both live inside the same `KRRT` object. But nothing in the code *enforces* that — a future change
could accidentally break this ordering.

#### What `unique_ptr` would give us

With `unique_ptr`, the ownership is encoded in the type system — the compiler itself prevents you
from accidentally creating two owners or forgetting to free memory:

```cpp
// Current (raw pointers — ownership is implicit, enforced by convention):
struct node {
    node* left   = nullptr;   // who frees this? you have to read the code to know
    node* right  = nullptr;
    node* parent = nullptr;   // is this an owner too? no, but nothing says so
};

// With unique_ptr (ownership is explicit in the types):
struct node {
    std::unique_ptr<node> left;    // "I own my left child"
    std::unique_ptr<node> right;   // "I own my right child"
    node* parent = nullptr;        // raw pointer = "I'm just borrowing this"
};
```

With this change:
- **Automatic cleanup**: Deleting the root cascades through `left` and `right` automatically.
  No manual `cleanup()` function needed. No chance of forgetting to call it.
- **No double-free**: `unique_ptr` cannot be copied. If you try to have two owners, the compiler
  gives you an error *before the program runs*. With raw pointers, double-free is a silent bug
  that may crash unpredictably.
- **Self-documenting**: Anyone reading `node* parent` immediately knows "this is borrowed, not
  owned", because owned pointers use `unique_ptr`.

#### Why it is deferred

The change is conceptually clean but mechanically invasive:

| What changes | Why |
|-------------|-----|
| `node.h` | `left`/`right` become `unique_ptr<node>` |
| `KDtree.h` | `root` becomes `unique_ptr<node>` |
| `KDtree.cpp` | `Insert()` must accept `unique_ptr<node>` and `std::move` it into position; `cleanup()` can be removed entirely |
| `KRRT.cpp` | `new node(...)` becomes `std::make_unique<node>(...)`, then moved into the tree |
| `KRRT.h` | `plan` stays as `vector<node*>` (observer), but `getPlan()` must be careful not to take ownership |

The KDTree's recursive `insert()` function passes ownership down the tree as it descends. With
raw pointers this is just `Knode->left = q_new`. With `unique_ptr` it becomes
`Knode->left = std::move(q_new)` — the same idea, but every intermediate step must explicitly
*move* rather than copy. Missing a single `std::move` is a compile error (safe, but tedious to
get right across all code paths).

The current code is **safe without this change** because:
- The KDTree destructor (fixed in Phase 4 item 9) properly frees all nodes.
- `plan` is always consumed before the tree is destroyed.
- The `KRRT` destructor calls `CleanUp()` then clears `plan`, in that order.

---

## KDTree Overhaul (Phase 4, Item 10)

### The Bug — Pruning Stopped Working After Depth 4

A KD-tree speeds up "find the nearest point" queries by **pruning** — skipping entire branches of
the tree that provably cannot contain a closer point. In a 4D state space (x, y, theta, beta),
the tree cycles through the 4 axes at each level: level 0 splits on x, level 1 on y, level 2 on
theta, level 3 on beta, level 4 on x again, and so on.

Each node must remember which axis it splits on so the search knows how to prune. The bug was in
how the axis was stored:

```cpp
// In insert():
int axis = depth % 4;   // ✓ CORRECT — used to decide left/right placement
// ...
Knode->axis = depth;     // ✗ BUG — stored raw depth (0, 1, 2, ... 17+)
```

When the nearest-neighbor search later read `Knode->axis`, it expected a value 0–3:

```cpp
switch (Knode->axis) {
    case 0: axis_diff = target[0] - node[0]; break;  // x
    case 1: axis_diff = target[1] - node[1]; break;  // y
    case 2: axis_diff = target[2] - node[2]; break;  // theta
    case 3: axis_diff = target[3] - node[3]; break;  // beta
    // axis >= 4 ? nothing matches → axis_diff stays 0
}
```

For any node deeper than 4 levels, `axis_diff` was always 0, which meant:
- The pruning check `axis_diff² < min_distance` became `0 < min_distance` → **always true**
- Both subtrees were always searched — the tree lost its O(log n) advantage

With 200,000 nodes and a tree depth of ~17, only the top 4 levels pruned correctly. Everything
below was a brute-force scan. The tree was doing the work of a linked list.

**Fix:** `Knode->axis = depth % N_DIM` — now every node stores 0, 1, 2, or 3. The search also
uses `Knode->axis % N_DIM` as a direct array index instead of a switch statement.

### Dead Code Removed

Only 3 methods were ever called by the planner: `Insert()`, `nearest_neighbor()`, and `cleanUp()`.
Everything else was removed:

| Removed | Why |
|---------|-----|
| `AxisComparator` struct | Never used |
| `XstateIter` typedef | Never used |
| `find()` / `find_recursive()` | Never called |
| `getRoot()` | Never called |
| `setNull()` | Never called — also dangerous: leaks entire tree without freeing |
| `removeRoot()` | Never called — also dangerous: deletes root but orphans all children |
| `is_empty()` | Never called |
| `parent` param in `insert()` | Passed recursively but never used |

### Other Improvements

- **`N_DIM` constant** replaces hardcoded `4` throughout
- **`r²` squaring** moved inside the public `nearest_neighbor()` — callers pass a plain Euclidean
  radius, the method squares it once before the recursive search
- **`insert()` simplified** — the 4-way `if/else` chain replaced with direct array indexing:
  `q_new->getXstate()[axis] < Knode->reached_state[axis]`
- **File reduced** from ~170 lines to ~90 lines

---

## Notes

- Third-party libraries (`public`, `MMLPlayer`) are managed as git submodules under `third_party/`.
- Results CSV now writes to `results.csv` in the working directory (was hardcoded to a user-specific desktop path).
