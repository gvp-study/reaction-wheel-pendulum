// stop_arch.scad — hard stops for the reaction-wheel pendulum, catching the BAR far from the pivot.
//
// Why out here: stopping a swinging bar with an impulse P at distance r from the pivot leaves an impulse
//   R = m*l*w * (1 - l_eq / r)   on the bearings, with l_eq = J/(m*l) = g/lambda^2 = 150 mm (measured).
// The old ribs caught it at ~20 mm: R ~ 6.5x the bar's momentum, which knocked the bearings out every fall.
// These pads catch it 88-114 mm out (R ~ 0.5x) and take ~5x less force themselves.
//
// Two halves that bolt together over the top of the green upright and to its front face:
//   - each half: a 12 mm plate against the upright's front face, reaching out to a pad on the line the bar's
//     edge lies on at STOP_TILT; the pad reaches forward into the bar's plane (31-39 mm in front of the upright)
//   - centre: the halves lap over the top of the upright, 2 x M3 x 12 vertical (heads on the left half, nuts
//     in the right half)
//   - upright: 1 x M3 x 20 per half through the upright (drill 3.4 mm using the half as a guide), nut behind
// Everything sits in front of the upright and behind the bar, so the bar, motor and wheel never touch it
// except where the bar's edge lands on a pad. Stick 2-3 mm foam tape on each pad face.
// Take the arch off for hanging tests: the pads stop the bar from swinging down.
//
// Print each half with the face that goes against the upright DOWN (as exported), no supports.
// PART = "right" | "left" | "print" (both, flat) | "both" (on a ghost of the stand, bar at +STOP_TILT)

PART = "both";
$fn = 64;

// stand (motor_mount.scad) and pendulum
AXIS_Z  = 75;       // pivot height above the stand's base bottom
UP_W    = 64;       // upright width
UP_T    = 7;
UP_TOP  = AXIS_Z + 46 / 2 + 10;     // 108
FLANGE_CLR = 30.5;  // pivot block flange is 58 mm: keep this far from the axis near the upright
BAR_PLANE = [31, 39];               // bar's back and front faces, mm in front of the upright
BAR_HALF  = 8;

// stops
STOP_TILT = 60;     // deg from upright where the bar lands
FOAM      = 3;
PAD_R     = [88, 114];   // along the bar (clear of the cable ties at 80 and 120)
PAD_ROOT  = 24;     // pad thickness behind the face at the plate
PAD_TIP   = 10;     // ... and at its front end
PAD_FRONT = BAR_PLANE[1] + 2;
// plate
PLATE_T = 12;
CTR = [106, 113, 120];   // centre lap: right half z 106-113, left half z 113-120
M3 = 3.4; M3_HEAD = 6.5; M3_NUT = 5.7;
UP_BOLT = [27, 100];     // through the upright (x, z)

function p(r, o) = [r * sin(STOP_TILT) + o * cos(STOP_TILT), AXIS_Z + r * cos(STOP_TILT) - o * sin(STOP_TILT)];
function pad(o1, o2) = [p(PAD_R[0], o1), p(PAD_R[1], o1), p(PAD_R[1], o2), p(PAD_R[0], o2)];
O_FACE = BAR_HALF + FOAM;

// local frame: X = x, Y = height, Z = distance in front of the upright. Print frame = this frame.
module half_local(lower) difference() {
    union() {
        linear_extrude(PLATE_T) difference() {
            union() {
                hull() {
                    translate([12, CTR[0]]) square([24, CTR[2] - CTR[0]]);
                    translate([21, 86]) square([15, CTR[2] - 86]);
                    polygon(pad(O_FACE, O_FACE + PAD_ROOT));
                }
                translate([-12, lower ? CTR[0] : CTR[1]]) square([24.01, CTR[1] - CTR[0]]);   // centre lap
            }
            translate([0, AXIS_Z]) circle(r = FLANGE_CLR, $fn = 128);
            if (!lower) translate([-13, CTR[0] - 1]) square([25, CTR[1] - CTR[0] + 1]);       // room for the other half
        }
        hull() {                                                                                // pad
            translate([0, 0, PLATE_T - 0.01]) linear_extrude(0.01) polygon(pad(O_FACE, O_FACE + PAD_ROOT));
            translate([0, 0, PAD_FRONT - 0.01]) linear_extrude(0.01) polygon(pad(O_FACE, O_FACE + PAD_TIP));
        }
    }
    // upright bolt, head counterbored in the front face
    translate([UP_BOLT[0], UP_BOLT[1], -1]) { cylinder(h = PLATE_T + 2, d = M3); translate([0, 0, PLATE_T - 3 + 1]) cylinder(h = 5, d = M3_HEAD); }
    // centre lap bolts (vertical in use)
    for (x = [-6, 6]) translate([x, CTR[0] - 1, PLATE_T / 2]) rotate([-90, 0, 0]) {
        cylinder(h = CTR[2] - CTR[0] + 2, d = M3);
        if (lower) cylinder(h = 2.6 + 1, d = M3_NUT / cos(30), $fn = 6);                      // nut from below
        else translate([0, 0, CTR[2] - CTR[0] + 1 - 3]) cylinder(h = 5, d = M3_HEAD);          // head from above
    }
}

module right_local() half_local(true);
module left_local()  mirror([1, 0, 0]) half_local(false);

// in the stand's frame: x across, y = 0 at the upright's front face (bar side is -y), z up
module place() rotate([90, 0, 0]) children();

if (PART == "right") right_local();
else if (PART == "left") left_local();
else if (PART == "print") { right_local(); translate([-8, 40, 0]) left_local(); }
else if (PART == "both") {
    place() { color("orange") right_local(); color("gold") left_local(); }
    // ghosts: upright, pivot flange, bar at +STOP_TILT and upright
    %translate([-UP_W / 2, 0, 0]) cube([UP_W, UP_T, UP_TOP]);
    %translate([0, 0, AXIS_Z]) rotate([90, 0, 0]) cylinder(h = 10, d = 58);
    for (a = [0, STOP_TILT]) %translate([0, -BAR_PLANE[0], AXIS_Z]) rotate([90, 0, 0]) rotate([0, 0, -a]) {
        linear_extrude(8) hull() { circle(d = 16); translate([0, 150]) circle(d = 16); }
        translate([0, 150, 0]) cylinder(h = 8, d = 46);
    }
}
