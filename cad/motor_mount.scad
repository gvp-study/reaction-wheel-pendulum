// motor_mount.scad — bench mount for the 4015 gimbal BLDC (MT6701 encoder, hollow shaft)
//
// An L bracket: the base sits on the table (C-clamp it, or screw it down through the
// four M5 holes); the upright stands at the table edge and holds the motor by the
// threaded brass inserts in the back of its stator, shaft horizontal and pointing
// out past the edge. Whatever you bolt to the rotor can swing a full circle in front
// of the table. A raised boss keeps the spinning rotor can clear of the plate, and a
// centre bore leaves room for the encoder board and lets its cable out the back.
//
// MEASURE the motor and set the [Motor] values before printing.
// PART = "test": a 4 mm plate with just the boss holes (~10 min) to check the hole pattern.
// Print the bracket as modelled (base on the bed), no supports.

PART = "mount";          // "mount" | "test"

/* [Motor — measure these] */
N_HOLES   = 4;           // threaded inserts in the back of the motor
HOLE_PCD  = 18 * sqrt(2);  // inserts on an 18 mm square (centre to centre) -> 25.46 mm circle
SCREW_D   = 3.4;         // clearance for the screws (M3 = 3.4, M2.5 = 2.9)
HEAD_D    = 6.5;         // screw-head counterbore (M3 socket head = 6.5)
CENTER_D  = 16;          // clear bore for the encoder / cable at the back (must stay inside HOLE_PCD - SCREW_D - 3)
MOTOR_OD  = 46;          // rotor can diameter (clearance only)
BOSS_D    = 34;          // the mount face the stator sits on (smaller than the rotor can)
BOSS_H    = 3;           // keeps the rotor can this far off the upright

/* [Bracket] */
AXIS_Z    = 75;          // shaft height above the table
WIDTH     = 110;          // base width
UP_W      = 64;           // upright width
BASE_D    = 90;          // base depth onto the table
BASE_T    = 6;
UP_T      = 7;           // upright thickness (the screws pass through this + the boss)
GUSSET_T  = 6;
M5_D      = 5.5;         // base holes for screwing down

$fn = 96;
UP_H = AXIS_Z + MOTOR_OD / 2 + 10;

// motor frame: origin on the boss face, z along the shaft (pointing -y, out over the edge)
module motor_frame() translate([0, -BOSS_H, AXIS_Z]) rotate([90, 0, 0]) children();

module teardrop_bore(d, h)      // horizontal bore that prints without support
    linear_extrude(h) hull() { circle(d = d); translate([0, d / 2 * 1.2]) rotate(45) square(d / 2 * 0.85, center = true); }

module bracket() difference() {
    union() {
        translate([-WIDTH / 2, 0, 0]) cube([WIDTH, BASE_D, BASE_T]);                         // base
        translate([-UP_W / 2, 0, 0]) cube([UP_W, UP_T, UP_H]);                                // upright
        translate([0, 0, AXIS_Z]) rotate([90, 0, 0]) cylinder(h = BOSS_H, d = BOSS_D);        // boss
        for (x = [-UP_W / 2, UP_W / 2 - GUSSET_T]) translate([x, 0, 0])                       // side gussets
            rotate([90, 0, 90]) linear_extrude(GUSSET_T)
                polygon([[UP_T - 0.01, BASE_T - 0.01], [BASE_D - 10, BASE_T - 0.01], [UP_T - 0.01, AXIS_Z - 10]]);
    }
    // centre bore (teardrop top, hidden behind the motor)
    translate([0, UP_T + 1, AXIS_Z]) rotate([90, 0, 0]) rotate([0, 0, 0]) teardrop_bore(CENTER_D, UP_T + BOSS_H + 2);
    // motor screws + counterbores on the back of the upright
    for (i = [0 : N_HOLES - 1]) let(a = 360 / N_HOLES * i + 45)
        translate([HOLE_PCD / 2 * cos(a), 0, AXIS_Z + HOLE_PCD / 2 * sin(a)]) rotate([90, 0, 0]) {
            translate([0, 0, -UP_T - 1]) cylinder(h = UP_T + BOSS_H + 2, d = SCREW_D);
            translate([0, 0, -UP_T - 1]) cylinder(h = UP_T - 4 + 1, d = HEAD_D);   // leaves 4 mm + boss under the head
        }
    // M5 hold-down holes in the base (or use a C-clamp across the middle)
    for (x = [-WIDTH / 2 + 14, WIDTH / 2 - 14], y = [UP_T + 14, BASE_D - 12])
        translate([x, y, -1]) cylinder(h = BASE_T + 2, d = M5_D);
}

module test_plate() difference() {
    cylinder(h = 4, d = BOSS_D + 8);
    translate([0, 0, -1]) cylinder(h = 6, d = CENTER_D);
    for (i = [0 : N_HOLES - 1]) let(a = 360 / N_HOLES * i + 45)
        translate([HOLE_PCD / 2 * cos(a), HOLE_PCD / 2 * sin(a), -1]) cylinder(h = 6, d = SCREW_D);
}

assert(CENTER_D < HOLE_PCD - HEAD_D - 2, "centre bore would cut into the screw holes: measure and adjust CENTER_D / HOLE_PCD");
if (PART == "test") test_plate();
else {
    bracket();
    %motor_frame() cylinder(h = 20, d = MOTOR_OD);                                  // ghost: motor
    %motor_frame() translate([-6, -125, 21]) cube([12, 140, 6]);                    // ghost: a 125 mm test arm
}
