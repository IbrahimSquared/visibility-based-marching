# Visibility cache implementation and tests

[`VisibilityCache`](../include/solver/visibility_cache.hpp) stores visibility
values for pairs of grid cells and wave sources, including pivots. The solver
passes the cell index and source ID separately when reading or writing a value.

## Storage

The cache contains two flat vectors: one with an entry for every grid cell and
one shared by all cells for additional source entries. Each entry occupies
16 bytes:

| Field | Type | Purpose |
|---|---|---|
| `visibility` | `double` | Computed visibility value |
| `source` | `uint32_t` | Source associated with the value |
| `next` | `uint32_t` | Index of the next additional entry, or `UINT32_MAX` |

The first source evaluated at a cell uses that cell's own entry. Further sources
use entries in the shared vector, linked by their indices. Lookup compares source
IDs along this chain. Its cost depends on how many sources have been evaluated
at the cell.

A source ID of `UINT32_MAX` marks an empty entry. A cached visibility value of
zero is therefore distinct from a pair that has not been evaluated. All queried
sources are retained, and visibility values keep their original `double`
precision.

## Cache operations

| Operation | Behavior |
|---|---|
| `reset(cellCount)` | Initializes empty entries for the grid and clears additional entries, retaining vector capacity for reuse. |
| `tryGet(cell, source, value)` | Returns `true` and copies the cached value into `value`, or returns `false` and leaves `value` unchanged. |
| `set(cell, source, value)` | Updates an existing value or creates an entry for the source. |
| `size()` | Returns the number of cached cell/source pairs. |
| `cellCount()` | Returns the number of grid cells. |
| `storageBytes()` | Returns the space reserved for entries in both vectors, in bytes. |

Call `reset()` before reading or writing values. Cell indices must be below
`cellCount()`. Source IDs and indices into the shared vector must be below
`UINT32_MAX`; exceeding either limit raises `std::length_error`.

The shared vector grows as entries are added. Its links use indices, so they
remain valid when the vector reallocates. Lookups copy values out of the cache,
allowing recursive visibility evaluations to add entries safely.

## Use in the solver

The cache replaces the hash table used by the solver in commit `7aa9581`.
Visibility calculations, marching order, pivot creation and saved result-file
formats are unchanged. Console output reports **Cached visibility entries** in
place of the hash table's **Load factor**.

The included [`flat_hash_map`](../include/flat_hash_map/flat_hash_map.hpp) and
public `Solver::Map` alias remain available for source compatibility. The solver
calls its existing `hashFunction(x, y, 0)` helper to obtain the cell index, then
passes the actual source ID to the cache separately.

## Running the tests

From the repository root, build and run the cache tests with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

These tests compare stored values with a reference map and check updates,
growth, resets and source-ID limits. They also check that zero values are
cached and that stored values retain their exact bits.

To compare the solver with the previous hash-table implementation:

```sh
python3 tests/compare_visibility_cache.py --baseline 7aa9581
```

The comparison requires Linux, Python 3, Git, GCC and SFML. It builds separate
copies of both versions and compares their outputs byte for byte, including
distances, parent assignments, pivots, iteration counts and solver status. It
also checks cached-entry counts, repeated resets, and the VStar, AStar and
distance-function methods.

For a smaller set of timing tests, use:

```sh
python3 tests/compare_visibility_cache.py --baseline 7aa9581 --quick --rounds 1
```

## Benchmark setup and results

The full comparison measures computation time on 36 maps: empty maps, maps with
rectangular obstacles, mazes, noise maps and maps with multiple starting
positions. Dimensions range from 64×64 to 1024×1024 and include rectangular
grids. Four image maps from the repository are included; the huge maze is
resampled to 800×800. Another 37 cases check solver behavior, giving 73 cases
in total.

For each of the 36 timing cases, both versions run in three fresh processes.
Each process performs one warm-up solve followed by seven timed solves. Runs
execute one at a time on the same CPU core. Timing includes solver reset and
excludes output comparison and saving. The first solve is also timed separately.

Tests on an Intel Core i9-13980HX with GCC 13.3.0, compilation flags
`-O3 -DNDEBUG -flto=auto`, and SFML 2.6.1 produced identical results and
cached-entry counts in all 73 cases. For each timing case, the comparison takes
the median of the compact-cache/hash-table runtime ratios from the three
process pairs, using each process's median solve time. The median of these
36 case ratios was 0.580, corresponding to approximately 42% less computation
time. The largest case ratio was 0.892. For first solves, the median case ratio
was 0.585.

The focused cache tests also passed with AddressSanitizer and
UndefinedBehaviorSanitizer.

The comparison saves raw timings, equality checks, compiler information, source
hashes and result summaries in `build/cache-comparison/`. Use
`--output DIRECTORY` to choose another location.
