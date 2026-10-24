# Grid Search — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestGridSearch : public ::testing::Test:
    # 8 × 6 map: wall at x = 4 for y = 0..4 (gap at y = 5); cost 1 elsewhere
    GridSearch<float, 8, 6> search{ { full, aStar } }
    Cost(c) ⇒ wall ? nullopt : 1
    # ValidPath(path): starts at start, ends at goal, consecutive cells are neighbours, all cells free,
    #                  no corner cutting; its summed step cost equals the returned cost
# each case below is a TEST_F(TestGridSearch, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
eight_connected_path_goes_through_the_gap:
    Act:    Search((1,1), (6,1), Cost)
    Assert: cost ≈ 11.242641 (= 7 + 3√2; Dijkstra agrees);  ExtractPath gives 11 cells;  ValidPath
four_connected_path_is_longer:
    Arrange: connectivity faces
    Assert:  cost == 13;  14 cells;  ValidPath
astar_and_dijkstra_agree_and_astar_expands_no_more:
    Assert: for both connectivities: aStar cost == dijkstra cost;  aStar Expanded() ≤ dijkstra Expanded()
weighted_cells_are_avoided_when_cheaper:
    Arrange: free 8 × 6 map with cost 5 on x ∈ {3, 4}, y ∈ {0, 1, 2}
    Assert:  cost ≈ 6.656854 (both modes);  the path climbs to y = 3 to cross x = 3..4;  ValidPath
blocked_goal_or_closed_gap_has_no_path:
    Assert: gap closed (cell (4,5) blocked) ⇒ nullopt;  goal on the wall (4,2) ⇒ nullopt;  ExtractPath ⇒ 0
start_equal_to_goal_costs_nothing:
    Assert: Search((2,2), (2,2), Cost) == 0;  ExtractPath gives the single cell (2,2)
three_dimensional_grid_uses_diagonal_steps:
    Arrange: GridSearch<float, 3, 3, 3>, all free
    Assert:  full: Search((0,0,0), (2,2,2)) ≈ 2√3 = 3.464102;  faces: 6
extract_path_rejects_a_short_buffer:
    Assert: after the first case, ExtractPath into a 5-cell span ⇒ 0
```

## Reference vectors

- Wall map, 8-connected: `(1,1)` to `(6,1)` costs `11.242641` through the gap at `y = 5`; 4-connected `13`.
- Weighted map: `6.656854` (`4√2 + 1` around the cost-5 block).

## Edge cases

- Corner cutting enabled ⇒ diagonal moves past a blocked side cell are allowed (never shorter than
  through the gap here, which the test documents).
- A grid larger than 65 535 cells switches `Index` to 32 bits (compile-time check).
