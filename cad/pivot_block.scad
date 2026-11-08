// pivot_block.scad — the pendulum's pivot: two 608 bearings and an 8 mm shaft, bolted to the green
// stand's upright where the 4015 motor is now (same 4 holes, 18 mm square).
//
//   back  : v4: a 34 mm recess in the back fits over the upright's 3 mm boss (the boss was there to keep the
//           4015's spinning can off the upright), so the whole flange sits flat on the upright and the
//           boss centres the block. Everything in front is where it was before.
//           flange sits on the upright; 4 x M3 screws come from behind the upright (through its
//           existing counterbored holes) and thread into pilot holes in the flange (self-tapping into
//           the plastic, or melt in M3 heat-set inserts if you have them)
//   front : bearing housing; both 608s go in from the front with the printed spacer between them.
//           v3: 8 crush ribs in the pocket give a real press fit (~0.25 mm interference) whatever the
//           printer's hole size does - the ribs shave/flatten as the bearing goes in. Press with a vise or a
//           socket on the OUTER race. For good: a drop of thin CA on each outer race before pressing.
//   stops : v2 - two solid ribs (not thin posts: v1's 6 mm post snapped on the first fall). Each rib's face
//           is the line the bar's edge lies on at STOP_TILT, so the bar lands flat along 16 mm of rib, and
//           the rib is fused to both the housing and the flange. Stick 2-3 mm foam tape on the faces.
//   shaft : 8 mm, about 52 mm long - from 1 mm behind the upright (magnet glued on its end, AS5048A
//           behind that) to the pendulum bar's clamp in front
//
// PART = "block" | "spacer" | "both".  Print the block flange-down, the spacer flat; no supports.

PART = "both";

$fn = 96;

// stand (must match motor_mount.scad)
HOLE_PCD  = 18 * sqrt(2);     // the 4 screw holes, 18 mm square
BOSS_D    = 34;
// bearings
BRG_D     = 22.4;             // 608 pocket: clearance; the crush ribs below do the gripping
RIB_TIP_D = 21.75;            // crush-rib tips: the 22.0 bearing squeezes them 0.12 mm each side
RIB_N     = 8;
RIB_W     = 1.2;
BRG_W     = 7;
SPACER_L  = 6;                // between the two bearings
LIP_D     = 15;               // rear lip: supports the bearing's outer race only
// block
FLANGE_D  = 58;
FLANGE_T  = 10;              // 3.2 of it is the boss recess -> 6.8 mm in front of the boss, as before
RECESS_D  = 34.6;             // prints ~34.2: slips over the boss
RECESS_H  = 3.2;              // boss is 3 high: a hair deeper so the flange rim lands on the upright
HOUSING_D = 30;
PILOT_D   = 2.9;              // prints ~2.5: an M3 screw cuts its own thread (or a heat-set insert)
PILOT_DEPTH = 5.5;
// end stops (ribs) - OFF from v5: catching the bar 14-29 mm from the pivot levers the impact into the
// bearings (6.5x the bar's momentum); the stops move out to ~150 mm (the centre of percussion) on an arch
RIBS      = false;
STOP_TILT = 60;               // bar tilt (deg from upright) where it lands on a rib
BAR_HALF  = 8;                // half the bar's width
FOAM      = 2.5;              // foam tape thickness on the rib face
RIB_T     = 7;                // rib thickness behind its face
RIB_R     = [14, 29];         // rib runs from the housing out to the flange edge
STOP_LEN  = 30;               // ribs reach 40 mm from the upright, as before

HOUSING_L = BRG_W + SPACER_L + BRG_W;
T_ALL     = FLANGE_T + HOUSING_L;

module block() difference() {
    union() {
        cylinder(h = FLANGE_T, d = FLANGE_D);
        cylinder(h = T_ALL, d = HOUSING_D);
        if (RIBS) for (s = [-1, 1]) mirror([s < 0 ? 1 : 0, 0, 0])      // right rib, and its mirror on the left
            intersection() {
                cylinder(h = FLANGE_T + STOP_LEN, r = RIB_R[1]);
                // bar at STOP_TILT to the right: its edge line is offset BAR_HALF from the axis; the rib
                // starts FOAM beyond that line and is RIB_T thick
                rotate(-STOP_TILT) translate([BAR_HALF + FOAM, RIB_R[0] - 4, 0]) cube([RIB_T, RIB_R[1] - RIB_R[0] + 8, FLANGE_T + STOP_LEN]);
            }
    }
    translate([0, 0, FLANGE_T]) difference() {                                          // both bearings + spacer
        cylinder(h = HOUSING_L + 1, d = BRG_D);
        for (i = [0 : RIB_N - 1]) rotate(i * 360 / RIB_N + 22.5) translate([RIB_TIP_D / 2 + RIB_W / 2, 0, 0]) {
            cylinder(h = HOUSING_L - 2, d = RIB_W, $fn = 16);
            translate([0, 0, HOUSING_L - 2]) cylinder(h = 2.01, d1 = RIB_W, d2 = 0.2, $fn = 16);  // lead-in
        }
    }
    translate([0, 0, -1]) cylinder(h = FLANGE_T + 2, d = LIP_D);                        // shaft passes back
    translate([0, 0, -1]) cylinder(h = RECESS_H + 1, d = RECESS_D);                     // fits over the boss
    for (a = [45 : 90 : 315]) rotate(a) translate([HOLE_PCD / 2, 0, -1])                // M3 into the flange
        cylinder(h = RECESS_H + PILOT_DEPTH + 1, d = PILOT_D);
    translate([0, 0, T_ALL - 0.6]) cylinder(h = 1, d1 = BRG_D, d2 = BRG_D + 1.2);       // lead-in for pressing
}

module spacer() difference() {               // sits between the bearings' outer races
    cylinder(h = SPACER_L, d = RIB_TIP_D - 0.4);             // slides past the ribs
    translate([0, 0, -1]) cylinder(h = SPACER_L + 2, d = LIP_D);
}

if (PART == "block") block();
else if (PART == "spacer") spacer();
else { block(); translate([FLANGE_D / 2 + 20, 0, 0]) spacer(); }
