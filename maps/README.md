# Navigation maps

`room_20261012.pgm` / `.yaml` is the failed first mapping attempt, preserved as
a diagnostic. It incorrectly cleared obstacle cells from inactive physics rays.
**Do not use it for navigation.** Save corrected maps to separate filenames and
check their alignment/coverage before using them with Nav2.

`room_20261012_corrected.pgm` / `.yaml` was collected through the corrected
navigation scan adapter. Full room walls and furniture were visually inspected;
wall cells align with the audited live wall faces within the 5cm grid tolerance.
Live footprint verification and ordinary navigation-goal tests remain pending.

`room_redo_20261012T141102.pgm` / `.yaml` is the latest changed-room map.
YAML metadata: 0.05m resolution, origin [-20,-20,0]. User confirmed saved-map
and live laser visualization working in RViz. Alignment while moving, live
footprint and navigation-goal tests remain pending. An earlier unsuccessful
save was reported; do not assume other new files are usable without inspection.
See WORKFLOW.md section 8 for loading and visualization commands.
