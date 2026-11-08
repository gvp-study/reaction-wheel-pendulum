// esc_box.scad (v3) — enclosure for the B-G431B-ESC1 (ST-LINK part still attached), 31 x 42 x 13 mm.
// Every cable leaves through a connector held IN THE WALL, so a tug on a cable pulls on the box, never on
// the solder pads:
//
//   signal wall (bottom) : MOTOR white 3-pin phase connector (8 x 4.75)
//                          ENC   MX1.25 5-pin female housing
//                          TILT  MX1.25 3-pin female housing
//   power wall (top)     : 12 V barrel jack (hex, 10 AF, 38 long, in a channel along the left side, mouth flush)
//                          micro-USB opening
//   lid                  : window over the whole board (LEDs, pot, button reachable; MOSFETs breathe)
//
// A 4.5 mm wire gutter runs the full length between the jack channel and the board, joining the top and
// bottom chambers: the jack's wires fold back up it to the power pads, and wires from the top end of the
// board (J3 tilt, phases) run down it to the signal wall.
//
// The box splits at the board's top face. Every outlet sits on that plane, so the base holds the lower half of
// each connector and the lid clamps the upper half. The board rests on two side ledges; four fingers on the
// lid press its long edges down. 4 x M2.5 x 16 screws at the corners (self-tap into the base).
// Looking down, component side up: ESC strip LEFT, ST-LINK strip RIGHT, USB at the TOP end.
// PART = "base" | "lid" | "both" (assembled) | "print" (both, laid out for printing). No supports.

PART = "both";
$fn = 48;

// ---- board ----
BX = 31.4;  BY = 42.4;       // 31 x 42 + clearance
BOT_CLR = 5;                 // room under the board for its bottom-side parts
PCB = 1.6;
// ---- box ----
W = 2.4;  F = 2;             // wall, floor
CH_BOT = 18;                 // chamber at the signal wall
CH_TOP = 14;                 // chamber at the power wall (USB plug reaches across it)
GUT = 4.5;                   // wire gutter beside the jack channel
JACK_AF = 10.35; JACK_L = 38; JACK_X = 6.1;
RAIL = 3; LEDGE = 1.5;       // side rails; the board overlaps them by LEDGE
// ---- MX1.25 female housings: 5P = 8.0 x 3.2, 3P = 5.5 x 3.2 (measure yours and adjust) ----
MX_W = [8.3, 5.8];           // ENC (5P), TILT (3P) pocket widths
MX_H = 3.6;                  // pocket height: body + clearance + latch ramp
MX_L = 6.5;                  // housing length from mating face to the back (wires exit there)
MX_GAP = 6;
MX_IN = 10;                  // block depth inside the wall
// ---- motor phase connector ----
CONN_W = 8.3; CONN_H = 5.05; CONN_L = 8;     // housing, wall to its back
// ---- micro-USB plug opening ----
USB_W = 14; USB_H = 9; USB_X_FROM_RIGHT = 7; // centre measured from the board's right edge
// ---- lid ----
LID_TOP = 2;  TOP_ROOM = 6;

// derived
JACK_COL = JACK_X * 2 + 1.5;                 // jack channel block width
BX0 = JACK_COL + GUT + (RAIL - LEDGE);       // board's left edge
BY0 = CH_BOT;
IW  = BX0 + BX + (RAIL - LEDGE) + 0.5;
IL  = CH_BOT + BY + CH_TOP;
ZL  = F + BOT_CLR;           // ledge
ZS  = ZL + PCB;              // split plane = board top
ZT  = ZS + TOP_ROOM + LID_TOP;
JACK_BACK = IL + W - JACK_L; // jack's back end, inside
CONN_X = JACK_X + 0.8;       // motor connector: bottom-left, under the jack column
MX_X = [BX0 + 10, BX0 + 10 + MX_W[0] / 2 + MX_GAP + MX_W[1] / 2];   // ENC, TILT
USB_X  = BX0 + BX - USB_X_FROM_RIGHT;
EARS = [[-W - 1.5, -W - 1.5], [IW + W + 1.5, -W - 1.5], [-W - 1.5, IL + W + 1.5], [IW + W + 1.5, IL + W + 1.5]];

module hex_jack(l) rotate([-90, 0, 0]) rotate([0, 0, 30]) cylinder(h = l, d = JACK_AF / cos(30), $fn = 6);

// solid blocks that hold the outlets (both halves are cut from these)
module blocks(z0, z1) {
    translate([0, 0, z0]) {
        translate([0, JACK_BACK - 2, 0]) cube([JACK_COL, IL - JACK_BACK + 2, z1 - z0]);              // jack channel
        translate([MX_X[0] - MX_W[0] / 2 - 2.4, 0, 0])
            cube([MX_X[1] + MX_W[1] / 2 - MX_X[0] + MX_W[0] / 2 + 4.8, MX_IN, z1 - z0]);             // ENC + TILT
        translate([CONN_X - CONN_W / 2 - 2, 0, 0]) cube([CONN_W + 4, CONN_L + 2 - W, z1 - z0]);      // MOTOR
    }
}

// everything cut through both halves
module outlets() {
    translate([JACK_X, JACK_BACK, ZS]) hex_jack(JACK_L + 0.1);                              // jack, mouth in the top wall
    translate([JACK_X, JACK_BACK + 1, ZS]) rotate([90, 0, 0]) cylinder(h = 5, d = 5);       // its cable, out the back
    for (i = [0, 1]) {
        translate([MX_X[i] - MX_W[i] / 2, -W - 0.1, ZS - MX_H / 2]) cube([MX_W[i], MX_L + 0.1, MX_H]);   // housing
        translate([MX_X[i] - (MX_W[i] - 1.8) / 2, -W + MX_L - 0.01, ZS - 1.1])
            cube([MX_W[i] - 1.8, MX_IN + W - MX_L + 1, 2.2]);                                // wires out the back
    }
    translate([CONN_X - CONN_W / 2, -W - 0.1, ZS - CONN_H / 2]) cube([CONN_W, CONN_L + 0.1, CONN_H]);  // motor
    translate([CONN_X - 3, -W + CONN_L - 0.01, ZS - 1.5]) cube([6, 4, 3]);                  // phase wires out the back
    translate([USB_X - USB_W / 2, IL - 1, ZS + 1.5 - USB_H / 2]) cube([USB_W, W + 2, USB_H]);
}

module ear(h, z0) for (e = EARS) translate([e[0], e[1], z0]) cylinder(h = h, d = 8);

module base() difference() {
    union() {
        difference() {
            translate([-W, -W, 0]) cube([IW + 2 * W, IL + 2 * W, ZS]);
            translate([0, 0, F]) cube([IW, IL, ZS]);
        }
        blocks(F - 0.01, ZS);
        for (x = [BX0 - (RAIL - LEDGE), BX0 + BX - LEDGE]) translate([x, BY0 - 2, 0]) cube([RAIL, BY + 4, ZS]);  // rails
        ear(ZS, 0);
    }
    translate([BX0, BY0, ZL]) cube([BX, BY, 10]);                                            // board pocket on the ledges
    outlets();
    for (i = [0 : 5]) translate([BX0 + 4 + i * 4.6, BY0 + 6, -1]) cube([1.8, BY - 12, F + 2]);  // vents under the board
    for (e = EARS) translate([e[0], e[1], -1]) cylinder(h = ZS + 2, d = 2.2);                // M2.5 pilot
}

module lid() difference() {
    union() {
        difference() {
            translate([-W, -W, ZS]) cube([IW + 2 * W, IL + 2 * W, ZT - ZS]);
            translate([0, 0, ZS - 1]) cube([IW, IL, ZT - ZS - LID_TOP + 1]);
        }
        blocks(ZS, ZT - LID_TOP + 0.01);
        for (x = [BX0 - (RAIL - LEDGE), BX0 + BX - LEDGE], y = [BY0 + 3, BY0 + BY - 9])     // clamp fingers
            translate([x, y, ZS]) cube([RAIL, 6, ZT - ZS]);
        ear(ZT - ZS, ZS);
    }
    translate([BX0 + LEDGE, BY0 + LEDGE, ZS - 1]) cube([BX - 2 * LEDGE, BY - 2 * LEDGE, 20]);  // window
    outlets();
    for (e = EARS) translate([e[0], e[1], ZS - 1]) { cylinder(h = 20, d = 2.9); translate([0, 0, ZT - ZS - 1.5]) cylinder(h = 5, d = 5.6); }
    // labels, engraved in the top
    translate([0, 0, ZT - 0.6]) linear_extrude(1) {
        translate([CONN_X, 5, 0]) text("MOTOR", size = 2.2, halign = "center", font = "Liberation Sans:style=Bold");
        translate([MX_X[0], 5, 0]) text("ENC", size = 2.6, halign = "center", font = "Liberation Sans:style=Bold");
        translate([MX_X[1], 5, 0]) text("TILT", size = 2.6, halign = "center", font = "Liberation Sans:style=Bold");
        translate([JACK_X, IL - 6, 0]) text("12V", size = 2.6, halign = "center", font = "Liberation Sans:style=Bold");
        translate([USB_X, IL - 6, 0]) text("USB", size = 2.6, halign = "center", font = "Liberation Sans:style=Bold");
    }
}

if (PART == "base") base();
else if (PART == "lid") translate([0, 0, ZT]) rotate([180, 0, 0]) lid();       // top face down on the bed
else if (PART == "print") { base(); translate([IW + 2 * W + 14, IL, ZT]) rotate([180, 0, 0]) lid(); }
else if (PART == "both") { color("#3ec43e") base(); color("#2f8f2f", 0.85) lid(); }
