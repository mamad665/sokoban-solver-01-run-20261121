// Sokoban solver — mamad665 entry for make.fun sokoban-solver-01-run-20261121 (MIT).
//
// Strategy: the generator's levels are a 1-wide hallway plus 1-4 rooms, each room
// holding 3-4 boxes and as many goals, and a box pushed out of a room can never
// come back — so every box stays in its room and a level's push-optimal count is
// the sum of its rooms' optima. We therefore:
//   1. split the level into rooms (connected floor components off the hallway row),
//   2. solve each room push-optimally with A* over pushes (state = player square +
//      sorted box squares; heuristic = min-cost box->goal assignment on BFS
//      distances; pruned by dead squares, corner / wall-run / 2x2 deadlocks),
//   3. stitch the room solutions together with plain walks through the hallway.
// Score depends only on pushes, so walks are free. The search budget is counted
// in expansions, never in wall-clock time, so two runs write identical bytes.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Deterministic work budget for the whole set (expansions, not seconds).
let EXPANSIONS_LEFT = 1200000;

const DIRS: { dx: number; dy: number; ch: string }[] = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (includes out-of-bounds padding via checks)
  goals: number[];
  goalSet: Uint8Array;
  boxes: number[];
  player: number;
}

function parse(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goalSet = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') { goalSet[i] = 1; goals.push(i); }
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, goalSet, boxes, player };
}

// BFS over floor cells (wall[] == 0) restricted to `mask` (1 = allowed), from a
// set of start cells. `blocked` (Uint8Array, 1 = obstacle) is honoured too.
function bfs(
  w: number, mask: Uint8Array, blocked: Uint8Array | null,
  starts: number[],
): Int32Array {
  const dist = new Int32Array(mask.length).fill(-1);
  const q: number[] = [];
  for (const s of starts) {
    if (mask[s] && dist[s] < 0 && !(blocked && blocked[s])) { dist[s] = 0; q.push(s); }
  }
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const x = c % w;
    const y = (c / w) | 0;
    const d = dist[c]! + 1;
    if (x > 0 && mask[c - 1] && dist[c - 1] < 0 && !(blocked && blocked[c - 1])) { dist[c - 1] = d; q.push(c - 1); }
    if (x + 1 < w && mask[c + 1] && dist[c + 1] < 0 && !(blocked && blocked[c + 1])) { dist[c + 1] = d; q.push(c + 1); }
    if (y > 0 && mask[c - w] && dist[c - w] < 0 && !(blocked && blocked[c - w])) { dist[c - w] = d; q.push(c - w); }
    if (y + 1 < (mask.length / w | 0) && mask[c + w] && dist[c + w] < 0 && !(blocked && blocked[c + w])) { dist[c + w] = d; q.push(c + w); }
  }
  return dist;
}

// Shortest walk (letters u/d/l/r) from `from` to `to` over floor cells in `mask`,
// avoiding cells marked in `blocked`. Empty string when unreachable / equal.
function walkPath(
  w: number, h: number, mask: Uint8Array, blocked: Uint8Array | null,
  from: number, to: number,
): string {
  if (from === to) return '';
  const prev = new Int32Array(mask.length).fill(-2);
  const prevCh = new Array<string>(mask.length).fill('');
  const q: number[] = [from];
  prev[from] = -1;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    for (const dir of DIRS) {
      const x = c % w;
      const y = (c / w) | 0;
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!mask[n] || prev[n] !== -2 || (blocked && blocked[n])) continue;
      prev[n] = c;
      prevCh[n] = dir.ch;
      if (n === to) {
        let s = '';
        let cur = n;
        while (cur !== from) { s = prevCh[cur] + s; cur = prev[cur]!; }
        return s;
      }
      q.push(n);
    }
  }
  return '';
}

// Dead squares for a region: a box on one can never reach a goal.
//  - no BFS path from the cell to any goal (walls only), or
//  - corner (two perpendicular wall neighbours), or
//  - in a wall run bounded by walls on both ends with no goal in the run.
function deadTable(
  w: number, h: number, wall: Uint8Array, mask: Uint8Array,
  goals: number[], goalSet: Uint8Array,
): Uint8Array {
  const dead = new Uint8Array(mask.length);
  const toGoal = bfs(w, mask, null, goals);
  // TRUE walls only: a neighbouring hallway floor cell is not a wall, even
  // though it lies outside the box region.
  const isWall = (c: number): boolean => c < 0 || c >= mask.length || wall[c] === 1;
  for (let c = 0; c < mask.length; c++) {
    if (!mask[c] || goalSet[c]) continue;
    if (toGoal[c] < 0) { dead[c] = 1; continue; }
    const x = c % w;
    const y = (c / w) | 0;
    const up = y === 0 || isWall(c - w);
    const down = y === h - 1 || isWall(c + w);
    const left = x === 0 || isWall(c - 1);
    const right = x === w - 1 || isWall(c + 1);
    if ((up || down) && (left || right)) { dead[c] = 1; continue; }
    // Wall runs: cell against a wall on one side, run along the wall.
    // Dead only when the run is bounded by walls on both ends and holds no goal.
    const sides: { perp: number; along: number }[] = [];
    if (up) sides.push({ perp: -w, along: 1 });
    if (down) sides.push({ perp: w, along: 1 });
    if (left) sides.push({ perp: -1, along: w });
    if (right) sides.push({ perp: 1, along: w });
    for (const side of sides) {
      let bounded = true;
      let hasGoal = false;
      for (const sgn of [1, -1]) {
        let cur = c;
        for (;;) {
          if (goalSet[cur]) hasGoal = true;
          const nxt = cur + sgn * side.along;
          const cx = cur % w;
          const nx = nxt % w;
          if (nxt < 0 || nxt >= mask.length || (side.along === 1 && Math.abs(nx - cx) !== 1)) break; // grid edge: bounded
          if (!mask[nxt] || wall[nxt] === 1) break; // run hits a wall: bounded
          const beside = nxt + side.perp;
          if (!isWall(beside)) { bounded = false; break; } // run opens: not bounded
          cur = nxt;
        }
      }
      if (bounded && !hasGoal) { dead[c] = 1; break; }
    }
  }
  return dead;
}

interface PushSolution {
  moves: string;
  pushes: number;
  endPlayer: number;
}

// Push-optimal A* for one region (a room, or the whole level as fallback).
// `walkCells` = cells the player may occupy (room + the hallway cell in front
// of its door, from which the player pushes the door box inward);
// `boxCells` = cells boxes may occupy (the room only — a box pushed into the
// hallway can never come back). `startPlayer` is inside walkCells, `exitCell` =
// cell the player must be able to reach at the end (-1 = no exit requirement).
// Returns null when unsolved / budget out.
function solveRegion(
  w: number, h: number, wall: Uint8Array, walkCells: Uint8Array, boxCells: Uint8Array,
  boxes0: number[], goals: number[], goalSet: Uint8Array,
  startPlayer: number, exitCell: number,
): PushSolution | null {
  const n = goals.length;
  if (n === 0) return { moves: '', pushes: 0, endPlayer: startPlayer };
  if (boxes0.length !== n) return null;
  const dead = deadTable(w, h, wall, boxCells, goals, goalSet);
  for (const b of boxes0) if (dead[b]) return null;

  // Per-goal BFS distances over the box cells (ignoring boxes).
  const distGoal: Int32Array[] = goals.map((g) => bfs(w, boxCells, null, [g]));
  const goalOf = new Map<number, number>();
  goals.forEach((g, i) => goalOf.set(g, i));

  // Admissible heuristic: min-cost assignment boxes -> goals (brute force, n <= 6;
  // sum-of-minima fallback for larger n).
  const heuristic = (bs: number[]): number => {
    if (n <= 6) {
      let best = Infinity;
      const used = new Array<boolean>(n).fill(false);
      const rec = (i: number, acc: number): void => {
        if (acc >= best) return;
        if (i === n) { best = acc; return; }
        for (let j = 0; j < n; j++) {
          if (used[j]) continue;
          const d = distGoal[j]![bs[i]!]!;
          if (d < 0) continue; // this box cannot reach this goal: try another
          used[j] = true;
          rec(i + 1, acc + d);
          used[j] = false;
        }
      };
      rec(0, 0);
      return best === Infinity ? 1e9 : best;
    }
    let s = 0;
    for (const b of bs) {
      let m = Infinity;
      for (let j = 0; j < n; j++) { const d = distGoal[j]![b]!; if (d >= 0 && d < m) m = d; }
      if (m === Infinity) return 1e9;
      s += m;
    }
    return s;
  };

  // State storage.
  const ps: number[] = [startPlayer];
  const bs: number[][] = [boxes0.slice()];
  const parent: number[] = [-1];
  const pushFrom: number[] = [-1]; // box cell before the push that made this state
  const pushTo: number[] = [-1]; // box cell after that push
  const gs: number[] = [0];
  const best = new Map<string, number>();
  const keyOf = (p: number, b: number[]): string => p + '|' + b.join(',');
  best.set(keyOf(startPlayer, boxes0), 0);

  // Binary heap on [f, -g, idx] (prefer deeper on ties).
  const heap: [number, number, number][] = [];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ;) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  hpush([heuristic(boxes0), 0, 0]);

  const boxMask = new Uint8Array(walkCells.length);
  const solved = (b: number[]): boolean => b.every((x) => goalSet[x] === 1);

  while (heap.length > 0) {
    if (EXPANSIONS_LEFT <= 0) return null;
    const [, ng, s] = hpop();
    const g = -ng; // heap stores -g in slot 1 (deeper-first tie-break)
    if (best.get(keyOf(ps[s]!, bs[s]!)) !== s) continue;
    const curBoxes = bs[s]!;
    const curPlayer = ps[s]!;
    if (solved(curBoxes)) {
      if (exitCell < 0) {
        // fall through to reconstruction
      } else {
        boxMask.fill(0);
        for (const b of curBoxes) boxMask[b] = 1;
        const reach = bfs(w, walkCells, boxMask, [curPlayer]);
        if (reach[exitCell] < 0) continue; // cannot walk out: keep searching
      }
      // Reconstruct moves.
      const chain: { from: number; to: number }[] = [];
      for (let t = s; t > 0; t = parent[t]!) chain.push({ from: pushFrom[t]!, to: pushTo[t]! });
      chain.reverse();
      let moves = '';
      let ppos = startPlayer;
      let cur = boxes0.slice();
      for (const step of chain) {
        const d = step.to - step.from;
        const dir = DIRS.find((dd) => dd.dy * w + dd.dx === d)!;
        const stand = step.from - d; // where the player must stand to push
        const bm = new Uint8Array(walkCells.length);
        for (const b of cur) bm[b] = 1;
        const walk = walkPath(w, h, walkCells, bm, ppos, stand);
        if (walk === '' && ppos !== stand) return null; // should not happen
        moves += walk + dir.ch.toUpperCase();
        const at = cur.indexOf(step.from);
        cur[at] = step.to;
        cur.sort((a, b) => a - b);
        ppos = step.from;
      }
      return { moves, pushes: g, endPlayer: ppos };
    }
    EXPANSIONS_LEFT--;
    // Player-reachable cells with current boxes as obstacles.
    boxMask.fill(0);
    for (const b of curBoxes) boxMask[b] = 1;
    const reach = bfs(w, walkCells, boxMask, [curPlayer]);
    for (let bi = 0; bi < curBoxes.length; bi++) {
      const b = curBoxes[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (const dir of DIRS) {
        const nx = bx + dir.dx;
        const ny = by + dir.dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const to = ny * w + nx;
        if (!boxCells[to] || wall[to] === 1 || boxMask[to]) continue;
        const stand = b - (dir.dy * w + dir.dx);
        if (stand < 0 || stand >= walkCells.length || !walkCells[stand] || reach[stand] < 0) continue;
        // Deadlock checks on the moved box.
        if (dead[to]) continue;
        // 2x2 block of boxes/walls with no goal.
        let frozen = false;
        const nb = curBoxes.slice();
        nb[bi] = to;
        nb.sort((a, b2) => a - b2);
        const boxSet = new Set(nb);
        const isSolid = (c: number): boolean =>
          c < 0 || c >= walkCells.length || !walkCells[c] || wall[c] === 1 || boxSet.has(c);
        for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]] as const) {
          const x0 = (to % w) + ox;
          const y0 = ((to / w) | 0) + oy;
          if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) continue;
          const cs = [y0 * w + x0, y0 * w + x0 + 1, (y0 + 1) * w + x0, (y0 + 1) * w + x0 + 1];
          if (cs.every((c) => isSolid(c)) && cs.every((c) => goalSet[c] === 0)) { frozen = true; break; }
        }
        if (frozen) continue;
        const k = keyOf(b, nb); // player ends on the box's old square
        const seen = best.get(k);
        if (seen !== undefined && gs[seen]! <= g + 1) continue;
        const t = ps.length;
        ps.push(b);
        bs.push(nb);
        gs.push(g + 1);
        parent.push(s);
        pushFrom.push(b);
        pushTo.push(to);
        best.set(k, t);
        hpush([g + 1 + heuristic(nb), -(g + 1), t]);
      }
    }
  }
  return null;
}

function solveLevel(text: string): string {
  const lv = parse(text);
  const { w, h, wall, goals, goalSet, boxes, player } = lv;
  if (boxes.length === 0) return '';
  const floor = new Uint8Array(w * h);
  for (let i = 0; i < floor.length; i++) floor[i] = wall[i] === 0 ? 1 : 0;

  const hallwayRow = (player / w) | 0;

  // Rooms: connected floor components excluding the hallway row.
  const noHall = new Uint8Array(w * h);
  for (let i = 0; i < noHall.length; i++) noHall[i] = floor[i] && ((i / w) | 0) !== hallwayRow ? 1 : 0;
  const seen = new Uint8Array(w * h);
  interface Room { cells: Uint8Array; boxes: number[]; goals: number[]; door: number; approach: number; }
  const rooms: Room[] = [];
  for (let i = 0; i < noHall.length; i++) {
    if (!noHall[i] || seen[i]) continue;
    const comp = new Uint8Array(w * h);
    const q: number[] = [i];
    seen[i] = 1;
    comp[i] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      const x = c % w;
      const y = (c / w) | 0;
      const ns = [
        x > 0 ? c - 1 : -1, x + 1 < w ? c + 1 : -1,
        y > 0 ? c - w : -1, y + 1 < h ? c + w : -1,
      ];
      for (const n of ns) {
        if (n >= 0 && noHall[n] && !seen[n]) { seen[n] = 1; comp[n] = 1; q.push(n); }
      }
    }
    const rBoxes = boxes.filter((b) => comp[b] === 1);
    const rGoals = goals.filter((g) => comp[g] === 1);
    // Door: a room cell adjacent to a hallway-row floor cell; approach = that
    // hallway cell (the player pushes the door box inward from there — the door
    // cell itself may start out occupied by a box).
    let door = -1;
    let approach = -1;
    for (let c = 0; c < comp.length; c++) {
      if (!comp[c]) continue;
      const y = (c / w) | 0;
      for (const n of [y > 0 ? c - w : -1, y + 1 < h ? c + w : -1]) {
        if (n >= 0 && ((n / w) | 0) === hallwayRow && floor[n]) { door = c; approach = n; break; }
      }
      if (door >= 0) break;
    }
    rooms.push({ cells: comp, boxes: rBoxes, goals: rGoals, door, approach });
  }

  const allBoxesCovered = rooms.reduce((a, r) => a + r.boxes.length, 0) === boxes.length;
  const allGoalsCovered = rooms.reduce((a, r) => a + r.goals.length, 0) === goals.length;
  const roomsOk = allBoxesCovered && allGoalsCovered &&
    rooms.every((r) => r.boxes.length === r.goals.length && r.boxes.length > 0 && r.door >= 0 && r.approach >= 0);

  if (!roomsOk) {
    // Fallback: solve the whole level as one region (no exit requirement).
    const sol = solveRegion(w, h, wall, floor, floor, boxes, goals, goalSet, player, -1);
    return sol ? sol.moves : '';
  }

  // Each room is solved twice: with an exit requirement (the player must be
  // able to walk back out to the hallway) and without one. The exit costs
  // extra pushes only sometimes, and only the room solved LAST may skip the
  // exit — so the room whose exit is most expensive goes last.
  interface RoomPlan { room: Room; exit: PushSolution | null; noexit: PushSolution | null; }
  const plans: RoomPlan[] = [];
  for (const room of rooms) {
    const walkCells = room.cells.slice();
    walkCells[room.approach] = 1;
    const exit = solveRegion(
      w, h, wall, walkCells, room.cells, room.boxes, room.goals, goalSet,
      room.approach, room.approach,
    );
    const noexit = exit && exit.pushes === 0 ? exit : solveRegion(
      w, h, wall, walkCells, room.cells, room.boxes, room.goals, goalSet,
      room.approach, -1,
    );
    if (!exit && !noexit) return ''; // unsolvable room: empty answer is valid, 0 pts
    plans.push({ room, exit, noexit });
  }
  // Cost of solving a room mid-sequence is its exit solution; last is no-exit.
  const exitCost = (p: RoomPlan): number => (p.exit ? p.exit.pushes : Infinity);
  const noexitCost = (p: RoomPlan): number => (p.noexit ? p.noexit.pushes : Infinity);
  let lastIdx = 0;
  let lastGain = -Infinity;
  plans.forEach((p, i) => {
    const gain = exitCost(p) - noexitCost(p);
    if (gain > lastGain) { lastGain = gain; lastIdx = i; }
  });
  const order = plans.filter((_, i) => i !== lastIdx);
  order.push(plans[lastIdx]!);

  let curBoxes = boxes.slice();
  let curPlayer = player;
  let moves = '';
  for (let oi = 0; oi < order.length; oi++) {
    const plan = order[oi]!;
    const isLast = oi === order.length - 1;
    if (!isLast && !plan.exit) return ''; // two rooms that cannot be exited: unsolvable in sequence
    const sol = isLast ? plan.noexit! : plan.exit!;
    const room = plan.room;
    const bm = new Uint8Array(w * h);
    for (const b of curBoxes) bm[b] = 1;
    const walk = walkPath(w, h, floor, bm, curPlayer, room.approach);
    if (walk === '' && curPlayer !== room.approach) return '';
    moves += walk + sol.moves;
    const after = room.goals.slice();
    const rest = curBoxes.filter((b) => room.cells[b] !== 1);
    curBoxes = [...after, ...rest].sort((a, b) => a - b);
    if (!isLast) {
      const bm2 = new Uint8Array(w * h);
      for (const b of curBoxes) bm2[b] = 1;
      const out = walkPath(w, h, floor, bm2, sol.endPlayer, room.approach);
      if (out === '' && sol.endPlayer !== room.approach) return '';
      moves += out;
      curPlayer = room.approach;
    }
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
