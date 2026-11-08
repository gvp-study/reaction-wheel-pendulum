// pendulum_bar.scad — the reaction-wheel pendulum's bar.
//   bottom : split clamp on the 8 mm pivot shaft (one M3 x 12 + nut squeezes it)
//   top    : motor plate for the 4015, same as the green stand: 4 x M2.5 on the 18 mm square into the motor's
//            brass standoffs, a 34 mm boss and a 16 mm bore that clear the MT6701 board and its plugs
//   length : L_PIVOT_TO_MOTOR from the shaft axis to the motor axis
//   sides  : 16 mm wide where it passes the pivot block's stop posts (they stop it at about 60 deg)
//   slots  : pairs of cable-tie slots along the bar for the 8-wire bundle
//
// Orientation in use: the flat face without the boss faces the pivot block; the motor bolts on the boss
// side, flywheel facing away from the stand.
// Print that flat face down (as modelled), no supports. 4 walls, 30-40 % infill.

$fn = 64;

L_PIVOT_TO_MOTOR = 150;
BAR_W   = 16;
BAR_T   = 8;            // thickness along the shaft axis

// shaft clamp
SHAFT_D = 8.3;          // prints ~8.0: snug on the rod, the screw does the clamping
HUB_D   = 24;
HUB_L   = 12;           // along the shaft (sticks out 4 mm past the bar's front face)
SLIT_W  = 1.2;
M3_D    = 3.4; M3_HEAD = 6.0; M3_NUT_AF = 5.7;

// motor plate (matches motor_mount.scad)
PLATE_D  = 46;
BOSS_D   = 34; BOSS_H = 3;
HOLE_PCD = 18 * sqrt(2);
M25_D    = 2.9; M25_HEAD = 5.0;
M25_SINK = 4;           // counterbore on the flat (stand-side) face: M2.5 x 10 leaves ~3 mm in the standoffs
CENTER_D = 16;

// cable-tie slots
TIE_Y   = [40, 80, 120];
TIE_SLOT = [3, 1.8];    // width along the bar, gap across

module bar2d() hull() { circle(d = HUB_D); translate([0, L_PIVOT_TO_MOTOR]) circle(d = PLATE_D); }

difference() {
    union() {
        // the bar: a strip from hub to plate
        linear_extrude(BAR_T) hull() { circle(d = BAR_W); translate([0, L_PIVOT_TO_MOTOR]) circle(d = BAR_W); }
        cylinder(h = HUB_L, d = HUB_D);                                           // shaft hub
        translate([0, L_PIVOT_TO_MOTOR, 0]) {
            cylinder(h = BAR_T, d = PLATE_D);                                     // motor plate
            translate([0, 0, BAR_T - 0.01]) cylinder(h = BOSS_H + 0.01, d = BOSS_D);
        }
    }
    // shaft bore + clamp slit (downward, away from the bar)
    translate([0, 0, -1]) cylinder(h = HUB_L + 2, d = SHAFT_D);
    translate([-SLIT_W / 2, -HUB_D, -1]) cube([SLIT_W, HUB_D, HUB_L + 2]);
    // clamp screw across the slit, head on one side, nut pocket on the other
    translate([0, -(SHAFT_D / 2 + 3.6), HUB_L / 2]) rotate([0, 90, 0]) {
        translate([0, 0, -HUB_D]) cylinder(h = 2 * HUB_D, d = M3_D);
        translate([0, 0, 4.5]) cylinder(h = HUB_D, d = M3_HEAD);
        translate([0, 0, -HUB_D - 4.5]) cylinder(h = HUB_D, d = M3_NUT_AF / cos(30), $fn = 6);
    }
    // motor plate holes
    translate([0, L_PIVOT_TO_MOTOR, 0]) {
        translate([0, 0, -1]) cylinder(h = BAR_T + BOSS_H + 2, d = CENTER_D);
        for (a = [45 : 90 : 315]) rotate(a) translate([HOLE_PCD / 2, 0, 0]) {
            translate([0, 0, -1]) cylinder(h = BAR_T + BOSS_H + 2, d = M25_D);
            translate([0, 0, -1]) cylinder(h = M25_SINK + 1, d = M25_HEAD);
        }
    }
    // cable-tie slots through the bar, one each side
    for (y = TIE_Y, s = [-1, 1]) translate([s * (BAR_W / 2 - 2.5) - TIE_SLOT[1] / 2, y - TIE_SLOT[0] / 2, -1])
        cube([TIE_SLOT[1], TIE_SLOT[0], BAR_T + 2]);
}
