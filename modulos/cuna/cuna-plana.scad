// Jumper phone holder, flat ("plana"): the phone lies in portrait along the shell, screen up, following the
// shell's slope, in a bumper frame that wraps all four sides and comes over the screen's border, on a bed whose
// skirt flows down into the shell.
//
// Rodhan (15-oct): no print and no mechanism, a showpiece that holds the phone, curved to blend with the robot,
// the phone close to the body and in portrait. The flat one rides lowest: nothing for a claw to reach.
// Rodhan (16-oct, on the first version with four corner hooks): «the phone is not held by the structure» -- the
// hooks were too small to read. So a continuous frame, scooped low in the middle of the long sides (where a
// finger takes the phone out) and open at the bottom end (USB-C), and a skirt instead of a slab.
// Coordinates are Jumper's base_link frame in mm; caparazon.stl (caparazon.py) is the shell top it sits on.
//   openscad -D 'parte="cuna"' -o cuna-plana.stl cuna-plana.scad     (parte: cuna, cuerpo, pantalla, isla, camara)

include <BOSL2/std.scad>
include <telefono.scad>

parte = "cuna";
x_c = -15;         // the phone's centre along the shell
z_c = 69.5;        // its back face there: 2 mm over the highest point of the shell under it
slope = 8;         // degrees, the shell's fall towards the back (72 mm at x 50, 53 mm at x -80)
wall = 3.0;
lip = 3.5;         // how far the frame comes over the screen's border
skirt = 9;         // how far beyond the frame the skirt reaches out on the shell

$fn = 48;

// screen up (phone X -> +z), top of the phone towards the back (phone Z -> -x), tilted with the shell
module en_robot() { translate([x_c, 0, z_c]) rotate([0, -90 - slope, 0]) children(); }
module shell() { import("caparazon.stl", convexity = 6); }

// the phone's outline grown by `g`, as a slab from `x0` to `x1` along the phone's X (thickness)
module contorno(g, x0, x1) {
    translate([(x0 + x1) / 2, 0, 0])
        cuboid([x1 - x0, phone[1] + 2 * g, phone[2] + 2 * g], rounding = corner + g, edges = "X");
}

module frame() {
    // the bumper: around the phone, up past its glass, the screen open but for `lip`
    en_robot() difference() {
        intersection() {
            telefono_cuerpo(grow = wall);
            contorno(wall, -wall, phone[0] + 1.2);
        }
        translate([phone[0] + 5, 0, 0])
            cuboid([10, phone[1] - 2 * lip, phone[2] - 2 * lip], rounding = corner - lip, edges = "X");
        // a finger scoop in the middle of each long side, one smooth curve
        for (s = [-1, 1]) translate([phone[0] + 18, s * (phone[1] / 2 + wall), 0])
            scale([1, 1, 2.4]) rotate([90, 0, 0]) cylinder(h = 4 * wall, r = 16, center = true, $fn = 96);
        // the bottom end (towards the robot's front) open for a USB-C cable
        translate([phone[0] / 2, 0, -phone[2] / 2]) cuboid([phone[0] + 2, 18, 4 * wall], rounding = 3, edges = "X");
    }
}

module bed() {
    // under the phone's back, down to the shell: the outline grown by the wall, cut by the shell later
    en_robot() contorno(wall, -26, 0);
}

module skirt_() {
    // the bed's underside blended into a band of the shell's own surface a little wider than it: the flare
    hull() {
        en_robot() contorno(wall, -1, 0);
        intersection() {
            en_robot() contorno(wall + skirt, -40, 0);
            difference() { translate([0, 0, 1.8]) shell(); translate([0, 0, 0.8]) shell(); }
        }
    }
}

if (parte == "cuna") {
    difference() {
        union() { frame(); bed(); skirt_(); }
        shell();
        en_robot() telefono_cuerpo(grow = 0.4);
    }
} else {
    en_robot() telefono(parte);
}
