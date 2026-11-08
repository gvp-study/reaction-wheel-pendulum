// wheel_adapter.scad (v3) — mounts the 80 mm aluminium flywheel on the 4015 rotor, CENTRED and FLAT.
//
// v3: the motor face has to sit dead flat on the rotor or the wheel wobbles. So that face now has
//   - a 10 mm x 0.8 mm relief in the middle (clears the rotor's centre boss / shaft end / any first-layer blob)
//   - 0.5 mm chamfers on its outer edge and on every hole and nut pocket (squashed first layer can't bulge
//     into them and lift the part)
//   - nut pockets 2.8 deep, so the M2.5 nuts sit 0.8 below the face and can never stand proud; this also
//     brings the nuts up to meet the M2.5 x 10 wheel screws sooner
//   Before fitting: hold it flat-face down on glass or a tile; it must not rock. If it does, rub it on fine
//   sandpaper laid on the glass until it doesn't.
//
//   4015 rotor face ── 4 x M2.5 x 5 (14 mm square) ── adapter ── 4 x M2.5 x 10 + plain M2.5 nuts ── flywheel
//
// v2: a 10 mm spigot fits the wheel's centre hole, so the wheel can only sit concentric (v1's slots let it
// sit off-centre and wobble). The 4 lock nuts sit in fixed hex pockets on the motor side at the wheel's
// 16 mm hole circle, can't turn or slide, and are trapped once the adapter is on the rotor.
//
// Assembly:
//   1. Press the 4 plain M2.5 nuts into the hex pockets on the flat (motor) side (snug; they sit flush).
//   2. Screw the adapter to the rotor with 4 x M2.5 x 5 (same screws as the test disc; heads sink in).
//   3. Drop the wheel over the spigot, put 4 x M2.5 x 10 (no longer: the tips must not reach the rotor)
//      through the wheel into the nuts below, tighten
//      in a cross pattern.
// Print flat (motor) side down, no supports, 100 % infill. If the spigot is tight in the wheel, sand it lightly.

$fn = 96;

OD        = 32;      // adapter diameter (rotor can is 46)
T         = 6.5;     // plate thickness
ROTOR_SQ  = 14;      // rotor screw square (centre to centre)
M25_D     = 2.9;     // M2.5 clearance
M25_HEAD  = 5.5;     // M2.5 head counterbore (4.8 printed too small for a 4.5 mm socket head)
M25_UNDER = 2.0;     // plastic left under the M2.5 heads (as on the test disc: M2.5 x 5 -> 3 mm into the rotor)
WHEEL_PCD = 16;      // wheel's M3 holes: opposite holes 16 mm apart (measured)
M3_D      = 2.9;     // wheel screws are M2.5 (name kept from v1)
NUT_AF    = 5.3;     // plain M2.5 nut (5.0 across flats): snug, prints ~5.0-5.1 so the nut can't turn
NUT_DEPTH = 2.8;     // nut sits 0.8 below the motor face (and can't tumble: 2.0 nut in a 2.8 pocket)
RELIEF_D  = 10;      // centre relief on the motor face (stays inside the nut pockets)
RELIEF_H  = 0.8;
CH        = 0.5;     // first-layer chamfers
SPIGOT_D  = 9.85;    // fits the wheel's 10 mm centre hole
SPIGOT_H  = 3.0;     // less than the wheel's 4 mm hub, so the screws clamp the wheel flat on the plate
CENTER_D  = 6;       // through hole (hollow shaft clearance)

difference() {
    union() {
        hull() { translate([0, 0, CH]) cylinder(h = T - CH, d = OD); cylinder(h = CH, d = OD - 2 * CH); }
        translate([0, 0, T - 0.01]) cylinder(h = SPIGOT_H + 0.01, d = SPIGOT_D);
    }
    translate([0, 0, -1]) cylinder(h = T + SPIGOT_H + 2, d = CENTER_D);
    translate([0, 0, -1]) cylinder(h = RELIEF_H + 1, d = RELIEF_D);                         // centre relief
    // rotor screws on the 14 mm square (45 deg), heads recessed from the wheel side
    for (a = [45 : 90 : 315]) rotate(a) translate([ROTOR_SQ / sqrt(2), 0, 0]) {
        translate([0, 0, -1]) cylinder(h = T + 2, d = M25_D);
        translate([0, 0, -0.01]) cylinder(h = CH, d1 = M25_D + 2 * CH, d2 = M25_D);           // chamfer
        translate([0, 0, M25_UNDER]) cylinder(h = T, d = M25_HEAD);
    }
    // wheel screws at 0/90/180/270 on the 16 mm circle, lock-nut hex pockets on the motor side
    for (a = [0 : 90 : 270]) rotate(a) translate([WHEEL_PCD / 2, 0, 0]) {
        translate([0, 0, -1]) cylinder(h = T + 2, d = M3_D);
        translate([0, 0, -0.01]) rotate(30) cylinder(h = NUT_DEPTH, d = NUT_AF / cos(30), $fn = 6);
        translate([0, 0, -0.01]) rotate(30) cylinder(h = CH, d1 = (NUT_AF + 2 * CH) / cos(30), d2 = NUT_AF / cos(30), $fn = 6);
    }
}
