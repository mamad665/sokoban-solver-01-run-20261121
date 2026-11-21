# Sokoban Solver — mamad665

Entry for make.fun `sokoban-solver-01-run-20261121`: build the best Sokoban
solver. One program (`main.ts`, Node 22, no dependencies) answers a set of 5
levels, writing the player's moves to `/output/<id>/solution.txt`.

## How it works

The generator's levels are a 1-wide hallway plus 1–4 rooms, each holding 3–4
boxes and as many goals — and a box pushed out of a room can never come back,
so every box stays in its room and a level's push-optimal count is the sum of
its rooms' optima. The solver therefore:

1. splits a level into rooms (connected floor components off the hallway row),
2. solves each room push-optimally with A* over pushes (state = player square
   + sorted box squares; heuristic = min-cost box→goal assignment on BFS
   distances; pruned by dead squares, corner / wall-run / 2×2 deadlocks),
3. stitches the room solutions together with plain walks through the hallway.

Score depends only on pushes, so walks are free. Each room is solved twice —
with and without an exit requirement — and the room whose exit costs the most
pushes is solved last, so the total stays push-optimal.

The search budget is counted in expansions, never in wall-clock time, so two
runs write identical bytes.

## Results

Practice set (in the kit): **1,000,000** — every level solved in exactly its
push-optimal count (13, 16, 29, 47, 60 pushes), in ~1.3 s total. Three
freshly drawn sets from unseen seeds also score 1,000,000 each (~2 s each).

## Local run

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see LICENSE.
