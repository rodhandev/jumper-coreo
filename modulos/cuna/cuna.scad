// Jumper phone cradle ("cuna"): the phone in landscape, leaning back on the shell behind the visor,
// held by a MagSafe ring and a lip, released by pressing a printed spring tongue from behind.
//
// Coordinates are Jumper's base_link frame in millimetres (x forward, y left, z up), so the STL drops
// straight into the MJCF. caparazon.stl is the shell + display module in that frame, written by
// construir.py from the jumper checkout; the cradle's underside is cut from it, so it sits flush and
// sticks on with VHB tape: no holes in the robot.
//
// Why each choice (measured 15-oct, see README.md):
// - landscape, 30 deg from vertical: the phone rises ~65 mm, not ~140 (portrait), so the centre of mass
//   stays low; and it sits behind the visor, so the eyes stay in view.
// - MagSafe ring: the dance accelerates the phone at most 1.4 m/s^2 (~0.3 N for 200 g); the ring holds
//   10-15 N against the plate and the lip takes the weight.
// - spring tongue: the "press and it lets go" of the Under Desk Phone Holder's printed trigger, redrawn:
//   a cantilever inside the ring's hole, flush at rest; pressing its tab pushes the phone 3 mm off the
//   magnet, and the phone lifts out diagonally.

include <BOSL2/std.scad>

/* [Phone] */
phone_long = 160;     // along y
phone_short = 75;     // up the plate
slot = 12;            // phone + case thickness the lip takes

/* [Cradle] */
tilt = 30;            // degrees back from vertical
x0 = 0;               // where the plate's front face meets the shelf (base_link, mm)
z0 = 78;
plate_w = 130;        // shorter than the phone: its USB-C ends stay free for a cable
plate_h = 60;
plate_t = 4;
shelf_t = 3;
lip_h = 7;            // front stop above the shelf
rib_y = [-60, -35, 35, 60];
rib_t = 3;
skin_t = 3;           // saddle on the shell
foot_x = [-34, 16];   // saddle footprint
foot_y = 68;

/* [Magnet and tongue] */
ring_od = 56.5;       // N52 MagSafe ring 56 x 46 x 1.5, with clearance
ring_id = 45.5;
ring_d = 1.6;
ring_z = 30;          // ring centre, up the plate
tongue_w = 16;
tongue_t = 2.4;       // strain at 3 mm of travel over 34 mm: 3*t*d/(2L^2) = 0.9 %, under PLA's ~2.5 %
tongue_z = [12, 46];  // root, free end (up the plate)
cut = 1.5;            // the slot around the tongue
tab = 10;             // how far the finger tab stands out behind

$fn = 64;

// The plate's frame: X = out of the plate's front face, Y = along the phone, Z = up the plate.
module plate_frame() { translate([x0, 0, z0]) rotate([0, -tilt, 0]) children(); }

module local() {
    // plate, with the ring's recess and the tongue's slot
    difference() {
        translate([-plate_t, -plate_w/2, -shelf_t]) cube([plate_t, plate_w, plate_h + shelf_t]);
        translate([-ring_d, 0, ring_z]) rotate([0, 90, 0]) tube(h = ring_d + 0.01, or = ring_od/2, ir = ring_id/2, anchor = BOTTOM);
        // U-shaped cut, the tongue's root left whole
        translate([-plate_t - 0.1, -tongue_w/2 - cut, tongue_z[0]])
            difference() {
                cube([plate_t + 0.2, tongue_w + 2*cut, tongue_z[1] - tongue_z[0] + cut]);
                translate([0, cut, -0.1]) cube([plate_t + 0.2, tongue_w, tongue_z[1] - tongue_z[0] + 0.1]);
            }
        // the tongue is thinner than the plate, flush at the front
        translate([-plate_t - 0.1, -tongue_w/2, tongue_z[0] + 3]) cube([plate_t - tongue_t + 0.1, tongue_w, tongue_z[1] - tongue_z[0]]);
    }
    // finger tab at the tongue's free end, behind
    translate([-tongue_t - tab, -tongue_w/2, tongue_z[1] - 8]) cuboid([tab + 0.5, tongue_w, 8], rounding = 1.5, edges = "X", anchor = LEFT + FRONT + BOTTOM);
    // shelf and front lip
    translate([-plate_t, -plate_w/2, -shelf_t]) cube([plate_t + slot + 3, plate_w, shelf_t]);
    translate([slot, -plate_w/2, 0]) cuboid([3, plate_w, lip_h], rounding = 1.4, edges = [TOP + RIGHT], anchor = LEFT + FRONT + BOTTOM);
}

module ribs() {
    for (y = rib_y)
        hull() {
            plate_frame() translate([-plate_t, y - rib_t/2, -shelf_t]) cube([plate_t + slot + 3, rib_t, 0.01]);
            plate_frame() translate([-plate_t, y - rib_t/2, -shelf_t]) cube([0.01, rib_t, plate_h * 0.8]);
            translate([foot_x[0], y - rib_t/2, 30]) cube([foot_x[1] - foot_x[0], rib_t, 0.01]);
        }
}

module shell() { import("caparazon.stl", convexity = 6); }

module saddle() {
    intersection() {
        translate([foot_x[0], -foot_y/2, 0]) cube([foot_x[1] - foot_x[0], foot_y, 120]);
        translate([0, 0, skin_t]) shell();
    }
}

difference() {
    union() {
        plate_frame() local();
        ribs();
        saddle();
    }
    shell();
}
