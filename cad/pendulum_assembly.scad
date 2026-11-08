// pendulum_assembly.scad — the whole reaction-wheel pendulum, as it goes together on the green stand.
// Preview only (F5). Put this file in the same folder as the other .scad/.stl files.
//
//   TILT     pendulum angle from upright (deg); the stop posts catch it at about +-60
//   EXPLODE  pull the parts apart along the shaft (mm) to see the order
//   SHOW_*   hide parts to see inside

TILT = 0;
EXPLODE = 0;
SHOW_STAND = true; SHOW_BLOCK = true; SHOW_SENSOR = true; SHOW_BAR = true; SHOW_MOTOR = true;

use <motor_mount.scad>            // the green stand (bracket module)
$fn = 64;

AXIS_Z = 75; BOSS_H = 3; UP_T = 7;

// frame on the pivot axis: origin on the boss face, +z along the shaft out over the desk edge, +y up
module axis_frame() translate([0, -BOSS_H, AXIS_Z]) rotate([90, 0, 0]) children();
module ex(n) translate([0, 0, n * EXPLODE]) children();

module bearing() color("#9aa") difference() { cylinder(h = 7, d = 22); translate([0, 0, -1]) cylinder(h = 9, d = 8); }
module flywheel() color("silver") difference() {
    union() {
        difference() { cylinder(h = 4, d = 80, $fn = 120); translate([0, 0, -1]) cylinder(h = 6, d = 68, $fn = 120); }
        cylinder(h = 4, d = 26);
        for (a = [90, 210, 330]) rotate(a) translate([0, -3, 0]) cube([36, 6, 4]);
    }
    translate([0, 0, -1]) cylinder(h = 6, d = 10);
}

// ---- stand ----
if (SHOW_STAND) color("#3ec43e") bracket();

// ---- pivot block, bearings, spacer, shaft ----
axis_frame() {
    if (SHOW_BLOCK) {
        ex(0) color("#2fa82f") translate([0, 0, -BOSS_H]) import("pivot_block.stl");   // v4 back sits on the upright
        ex(1) { translate([0, 0, 7]) bearing(); color("#e0e0e0") translate([0, 0, 14]) import("pivot_spacer.stl"); translate([0, 0, 20]) bearing(); }
    }
    ex(1.5) color("#ccc") translate([0, 0, -11]) cylinder(h = 52, d = 8);          // 8 mm shaft, 1 mm behind the upright
    if (SHOW_SENSOR) ex(-1) color("#7788ff") translate([0, 0, -13]) cylinder(h = 2, d = 5);   // magnet on the shaft end

    // ---- pendulum bar on the shaft, tilting about it ----
    rotate([0, 0, -TILT]) {
        if (SHOW_BAR) ex(2.5) color("orange") translate([0, 0, 28]) import("pendulum_bar.stl");
        if (SHOW_MOTOR) ex(3.5) translate([0, 150, 28 + 8 + 3]) {
            color("#222") cylinder(h = 20, d = 46);                                    // 4015 (stator + rotor can)
            translate([0, 0, 20]) { color("gold") import("wheel_adapter.stl");        // adapter v2
                                    translate([0, 0, 6.5]) flywheel(); }
        }
    }
}

// ---- AS5048A holder on the back of the upright, wire slot up ----
if (SHOW_SENSOR) translate([0, UP_T + 10, AXIS_Z]) rotate([90, 0, 0]) translate([0, 0, 0]) {
    translate([0, 0, -EXPLODE]) color("#ff8c00") import("as5048_holder.stl");
    translate([0, 0, 3.2 - EXPLODE / 2]) color("#1a1a1a") translate([-8.5, -7.5, 0]) cube([17, 15, 1.6]);   // board
}
