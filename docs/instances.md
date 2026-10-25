# Instances and datasets

An **instance** is one benchmark problem. It is always stored in the same canonical form, independently of which
planner will solve it:

* **environment:** the workspace boundary and the obstacles (holes) as polygons in metres (WKT);
* **team:** robots with start poses, tool and body footprints, speed and turning model;
* **task:** task class, completeness tolerance τ, return-to-depot, time and memory limits, profiles;
* **evaluation settings:** evaluation resolution, sensitivity check, collision and conflict semantics.

Instances are grouped in **instance sets**, directories under `data/`. Each set has an `index.json` that assigns
every instance to the **tuning** or the **test** split.

## Instance families

| Family | What | How to get it |
|--------|------|---------------|
| IF1 | polygonal environments: AreaCoverage VM25 (25 indoor), AC300 (300 outdoor, 1–15 holes), H2 | `uninbench import areacoverage` |
| IF2 | agricultural fields with real, non-convex boundaries: the 350 fields of Fields2Benchmark | `uninbench import fields` |
| IF3 | indoor rooms, each without and with furniture (paired) | `uninbench import rooms` |
| IF4 | connectivity sweep: the 21 published SafeLog maps, subgraphs of a 20 × 20 grid from a spanning tree to an almost complete grid | `uninbench import safelog` (or a SafeLog-like `uninbench generate safelog`) |
| IF5 | synthetic maps with controlled size, obstacle density and shape, and non-convexity | `uninbench generate synthetic` |

All generation and import commands are also in the TUI and web UI (*Instances → Generate… / Import…*). Each command
writes the instances plus `index.json` and assigns splits (see [Splits](#tuning-and-test-splits)).

## Synthetic instances (IF5)

```bash
uninbench generate synthetic --out data/IF5 --count 10 \
    --sizes 20 40 80 --densities 0.05 0.1 0.2 --shapes mixed --nonconvexity 0 2 \
    --stratify obstacle_density
```

This is a full-factorial sweep: `--count` instances for every combination of sizes × densities × shapes ×
notches. The example above produces 10 × 3 × 3 × 1 × 2 = 180 instances.

| Option / parameter | Default | Meaning |
|--------------------|---------|---------|
| `--sizes` | 40 | width of the rectangular workspace in metres; height = width × `--aspect` (0.75) |
| `--densities` | 0.1 | target fraction of the workspace covered by obstacles (the realized density is stored in the metadata) |
| `--shapes` | mixed | `rect`, `polygon` (convex or L-shaped), `circle`, or `mixed` |
| `--nonconvexity` | 0 | number of rectangular notches cut into the boundary |
| `--seed` | 0 | base seed; instance *k* of combination *c* uses seed `seed + 1000·c + k` |
| `--param robot_width=1.0` | 1.0 | working width of the robot (disc) |
| `--param min_gap=3.0` | 3.0 | minimum gap between obstacles and to the boundary, in working widths |
| `--param obstacle_size=[0.06,0.16]` | | obstacle size range as a fraction of the shorter side |
| `--param notch_depth=0.25` | | notch depth as a fraction of the side |
| `--param speed=1.0 turn_cost=0.25` | | robot speed (m/s) and turning cost (s per radian) |
| `--param robot_kind=rect` | disc | tool footprint shape |

Every generated instance stores the generator parameters and its seed, so it can be regenerated exactly:

```python
from uninbench.datasets import regenerate
same = regenerate(instance)      # same.hash == instance.hash
```

The file names encode the factors, e.g. `syn_40x30_d0.10_mixed_n2_s1003.json`.

## SafeLog graphs (IF4)

SafeLog maps are subgraphs of a 4-connected grid. uninbench embeds every vertex as a square cell of
`--cell-size` metres. Every *missing* edge between neighbouring vertices becomes a thin wall. The robot's tool is
half a cell wide, so every cell is exactly one STC coarse cell. The original graph is kept in the metadata for
graph-native planners (the `graph` adapter).

```bash
# the published maps (Rybecky & Kulich, 2021; doi:10.5281/zenodo.4439404), downloaded by scripts/fetch_datasets.sh
uninbench import safelog datasets/safelog --out data/IF4
# a SafeLog-like sweep of random connected subgraphs, 21 densities (no download needed)
uninbench generate safelog --out data/IF4 --rows 20 --cols 20 --count 1
```

The published dataset is a directory with `test_nodes.txt` (`id x y`) and `test_edges0.txt` … `test_edges20.txt`
(one directed pair of node ids per line); the extra complete grid `test_edges.txt` is imported only with
`--param pattern='test_edges*.txt'`. Plain edge lists (one edge per line, `u v` with row-major vertex ids or
`r1 c1 r2 c2`, `#` comments) are read too. Importer parameters: `rows`, `cols` (20; taken from the node file when
present), `cell_size` (2.0), `robot_width` (cell/2), `pattern`.

Grid planners receive the original graph through the identity `graph` adapter (`native_graph: true`), so their
input is exactly the published map.

## Public datasets

Download them first:

```bash
scripts/fetch_datasets.sh        # → datasets/AreaCoverage-dataset, datasets/test_maps, datasets/fields
```

### AreaCoverage (IF1)

```bash
uninbench import areacoverage datasets/AreaCoverage-dataset --out data/IF1 --stratify holes
uninbench import areacoverage datasets/AreaCoverage-dataset --out data/IF1-VM25 --param subsets=[VM25]
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `subsets` | all present | any of `VM25`, `AC300`, `H2`, `RAL_main` |
| `width` | per subset | override the working width in metres |
| `limit` | – | import at most this many instances per subset |
| `flag_convention` | `from` | meaning of the route flag column (see below) |

The subsets use their own units and robot models:

* **VM25:** 1 unit = 0.1 m. The robot is the given `robot_polygon`: a square tool, with its inscribed disc as the
  body.
* **AC300:** 1 unit = 1 m. It models aerial robots: a 3 m square sensor footprint carried by a point-like
  vehicle.
* **H2** and **RAL_main:** 1 m units and a point-like body.

The dataset also contains reference solutions (`mem_*` directories). They make a planner-independent sanity check
of the evaluator:

```bash
uninbench check-evaluator datasets/AreaCoverage-dataset --out validation/evaluator_reference_routes.json
```

Every reference solution should come out *valid* with coverage ≈ 1.0. The route files mark each leg as servicing
(1) or deadheading (0). `flag_convention=from` reads the flag of a point as describing the travel *from* it to the
next point; this is the convention that reproduces complete coverage.

### Agricultural fields (IF2)

```bash
# Fields2Benchmark, 350 fields as WKT in longitude/latitude (doi:10.5281/zenodo.14524734; scripts/fetch_datasets.sh)
uninbench import fields datasets/fields2benchmark/wkt --out data/IF2 --param width=3.0
```

Geographic coordinates are projected to a local metric frame (the projection is stored in the metadata).

Accepted files, one field per file:

* **Fields2Cover GML/XML** (`<Field>` with outer and inner boundaries; a `swath_width` attribute sets the working
  width);
* **GeoJSON** (Polygon or MultiPolygon features);
* **WKT**.

Longitude/latitude coordinates are detected and projected to a local metric frame (an equirectangular projection
about the field centroid; the error is well below a centimetre at field scale). The projection is stored in the
metadata.

The field robot has a disc body of the working width and a rectangular implement (width × 0.2·width). Its speed
is 2 m/s and its turning cost 0.5 s/rad.

Parameters: `width` (3.0 m), `pattern` (`*`), `limit`.

### Indoor rooms (IF3)

```bash
uninbench import rooms datasets/test_maps --out data/IF3 --param resolution=0.05 --param width=0.5
```

For every map `<name>.png` that has a furnished variant `<name>_furnitures.png`, the importer does the following:

1. It uses the ground-truth segmentation `<name>_gt_segmentation.png`, if present, to split the map into rooms.
   Rooms are white regions separated by black lines.
2. It writes two instances per room: `<name>_roomNN_empty` and `<name>_roomNN_furnished`. They share
   `metadata.pair`, so they always land in the same split and can be compared pair by pair.
3. It vectorizes pixels exactly: free space is the union of pixel squares, simplified by half a pixel. The source
   resolution is stored with the environment, because it bounds the fidelity any adapter can reach.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `resolution` | 0.05 | metres per pixel |
| `width` | 0.5 | robot diameter (tool = body) in metres |
| `min_room_area` | 4.0 | ignore regions smaller than this (m²) |
| `limit` | – | at most this many maps |

## Published MCPP instances (validation)

The ten instances of Tang & Ma (2023) -- floor, maze and terrain grids with 4 to 12 robots -- are used to validate
MFC, MSTC\*, MIP and LS-MCPP against the coverage times published for them (see
[Tuning and validation](tuning-and-validation.md#validation-against-published-results)). `scripts/fetch_datasets.sh`
clones the authors' repository to `datasets/MIP-MCPP`; `uninbench.datasets.mip_mcpp.import_mip_mcpp` converts the
pickled graphs (read with a restricted unpickler) into canonical instances of family `published`.

## Tuning and test splits

Tuning must never see the instances results are reported on. Every generation and import therefore assigns each
instance to `tuning` or `test`, deterministically from its content hash:

* `--tuning-fraction` (default 0.2) is the share of each stratum that goes to tuning.
* `--stratify` (default `auto`) stratifies by the factor the family varies: `holes` (IF1, IF2), `furniture` (IF3),
  `graph_density` (IF4), and size, obstacle density, shape and non-convexity together (IF5). Another factor name,
  several comma-separated names, or `none` can be given. A single numeric factor is split by systematic sampling
  along its sorted values, so its whole range is represented in both splits even when every value occurs once
  (the 21 SafeLog densities).
* Paired instances (the same `metadata.pair`) always share a split.

To reassign splits, for example after adding instances:

```bash
uninbench split data/IF5 --tuning-fraction 0.2            # stratified by the family's factors
```

Experiments select a split with `split: test` (the default), `tuning` or `all`. See
[Experiments](experiments.md#instance-sources).

## The instance file

```json
{
  "schema_version": 1,
  "id": "hall",
  "family": "custom",
  "environment": {
    "boundary": "POLYGON ((0 0, 0 8, 12 8, 12 0, 0 0))",
    "holes": ["POLYGON ((6 5, 4 5, 4 3, 6 3, 6 5))"]
  },
  "team": [
    {"id": "r0", "start": [0.5, 0.5, 0.0], "tool": {"kind": "disc", "width": 1.0},
     "speed": 1.0, "turn_cost": 0.25}
  ],
  "task": {"task_class": "one_shot_direct_coverage", "tolerance": 0.02, "return_to_depot": false,
           "time_limit": 60.0, "memory_limit_mb": 4096, "profiles": ["core"]},
  "evaluation": {"resolution": null, "resolution_factor": 0.1, "sensitivity": true,
                 "sensitivity_tolerance": 0.01, "max_refinements": 1, "collision": "invalidate",
                 "conflicts": "report"},
  "metadata": {"source": "hand-made", "split": "test", "factors": {}},
  "hash": "ac03bbc3d8f5c693"
}
```

| Field | Meaning |
|-------|---------|
| `environment.boundary` | outer workspace polygon (or multipolygon), WKT, metres |
| `environment.holes` | obstacle polygons; the free space is boundary minus holes. Obstacles are never inflated here: inflation is a planner or adapter decision. |
| `environment.terrain` | optional terrain-cost raster for weighted planners |
| `environment.source_resolution` | resolution of the raster the instance came from (room maps) |
| `team[].start` | `[x, y, heading]` in metres and radians |
| `team[].tool` / `body` | footprints: `{"kind": "disc", "width": w}` or `{"kind": "rect", "width": lateral, "length": longitudinal}`. The body defaults to the tool. |
| `team[].speed`, `turn_cost` | m/s; seconds per radian of heading change |
| `team[].min_turn_radius`, `capacity`, `depot` | set by the agricultural profile |
| `task.tolerance` | τ: a run is complete if the coverage ratio is at least 1 − τ |
| `metadata.split`, `factors`, `pair`, `generator` | split, experimental factors, pairing, generator parameters |
| `hash` | content hash, excluding the split and notes; results reference it |

## Creating instances in Python

```python
from shapely.geometry import box, Point
from uninbench.instance import instance_from_polygon, default_robot

free = box(0, 0, 30, 20).difference(Point(15, 10).buffer(3))
inst = instance_from_polygon("pillar_hall", free,
                             default_robot(width=0.8, speed=0.5, start=(1, 1, 0)),
                             family="custom", factors={"pillars": 1})
inst.save("data/custom")          # → data/custom/pillar_hall.json
```

Then run `uninbench split data/custom` to create the index and splits.

## Team sizes and start configurations

Instances usually contain a single *template* robot. Experiments derive teams from it: `team_sizes: [1, 2, 4, 8]`
× `start_configs: [clustered, dispersed]`.

* **clustered**: robots on a tight lattice around the template's start, never overlapping.
* **dispersed**: robots spread over the reachable free space by farthest-point sampling.
* **given**: use the instance's own team unchanged; its size must match the team size.

Derived instances are deterministic and named `<instance>__m<m>_<config>`. They are stored with the results in
`results/<experiment>/instances/`.

## Inspecting instances

```bash
uninbench info data/IF5/syn_40x30_d0.10_mixed_n0_s0.json
uninbench adapt data/IF5/syn_40x30_d0.10_mixed_n0_s0.json --adapter grid --param coarse=true
```

`adapt` prints an adapter's transformation record: its parameters, runtime and the fidelity of the representation
(see [Planners and adapters](planners.md#adapters)).
