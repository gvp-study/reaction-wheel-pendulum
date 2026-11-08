// pivot_shim.scad — fills the gap between the pivot block's flange and the upright, around the 34 mm boss,
// so the flange (and the rib stops on it) is backed over its full face instead of only on the boss.
// For a block already printed with a flat back (v1-v3). Slip it over the boss, then bolt the block on as usual.
// Print flat, 100 % infill. If it rocks, sand it a little thinner; if there's a gap, print it at 3.2.
$fn = 128;
T  = 3.0;         // the boss height (measure the gap: flange rim to upright)
OD = 58;          // flange diameter
ID = 34.6;        // over the 34 mm boss
difference() { cylinder(h = T, d = OD); translate([0, 0, -1]) cylinder(h = T + 2, d = ID); }
