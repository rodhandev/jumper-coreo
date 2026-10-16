// Jumper phone holder, upright ("vertical"): the phone stands in portrait right behind the visor, leaning back
// 6 deg, its screen to the person, in a sleeve whose wings flow down onto the shell.
//
// Rodhan (15-oct): no print and no mechanism, it is a showpiece; a structure that holds the phone, with curves
// so it blends with the robot, the phone close to the body and in portrait. So: one piece, no magnet or latch.
// Coordinates are Jumper's base_link frame in mm; caparazon.stl (caparazon.py) is the shell top it sits on.
//   openscad -D 'parte="cuna"' -o cuna-vertical.stl cuna-vertical.scad     (parte: cuna, cuerpo, pantalla, isla, camara)

include <BOSL2/std.scad>
include <telefono.scad>

parte = "cuna";
x_back = 0;        // the phone's back face, where it meets the sleeve floor
z_bottom = 74;     // the phone's bottom edge (the shell top is ~70 mm here)
lean = 6;          // degrees back from vertical
wall = 3.2;
sleeve_h = 30;     // how high the sleeve holds the phone
rest_h = 78;       // the back rest, up the phone's back
foot = [50, 58];   // the footprint's half-lengths on the shell (x, y): the wings' reach

$fn = 48;

// the phone's frame placed on the robot: bottom edge at z_bottom, leaning back
module en_robot() { translate([x_back, 0, z_bottom]) rotate([0, -lean, 0]) translate([0, 0, phone[2] / 2]) children(); }
module shell() { import("caparazon.stl", convexity = 6); }

module sleeve() {
    en_robot() difference() {
        // a rounded cup around the phone's lower end
        translate([phone[0] / 2, 0, -phone[2] / 2 + sleeve_h / 2 - wall])
            cuboid([phone[0] + 2 * wall, phone[1] + 2 * wall, sleeve_h], rounding = 6.5);
        telefono_cuerpo(grow = 0.4);
        // the front opened in one smooth curve (half an ellipse), so the screen shows down to its last 7 mm and
        // the sides rise like the wings of the shell
        translate([phone[0] + wall, 0, -phone[2] / 2 + 7 + 60]) scale([1, (phone[1] - 8) / 120, 1])
            rotate([0, 90, 0]) cylinder(h = 4 * wall, r = 60, center = true, $fn = 96);
    }
}

module rest() {
    // a tongue up the back, as wide as the camera block leaves free, rounded at the top
    en_robot() difference() {
        hull() {
            translate([-wall / 2, 0, -phone[2] / 2 + 6]) cuboid([wall, 44, 2], rounding = 0.9, edges = "X");
            translate([-wall / 2, 0, -phone[2] / 2 + rest_h - 14]) rotate([0, 90, 0]) cylinder(h = wall, r = 14, center = true);
        }
        telefono_cuerpo(grow = 0.4);
    }
}

module wings() {
    // the sleeve's floor blended into an oval patch of the shell's own surface: the hull of the two is the flare
    hull() {
        en_robot() translate([phone[0] / 2, 0, -phone[2] / 2 - wall / 2])
            cuboid([phone[0] + 2 * wall, phone[1] + 2 * wall, wall], rounding = wall / 2);
        intersection() {
            translate([x_back - 6, 0, 0]) scale([foot[0], foot[1], 1]) cylinder(h = 120, r = 1);
            difference() { translate([0, 0, 2.5]) shell(); translate([0, 0, 1.2]) shell(); }
        }
    }
}

if (parte == "cuna") {
    difference() {
        union() { sleeve(); rest(); wings(); }
        shell();
        en_robot() telefono_cuerpo(grow = 0.4);
    }
} else {
    en_robot() telefono(parte);
}
