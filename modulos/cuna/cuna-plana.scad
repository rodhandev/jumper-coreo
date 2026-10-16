// Jumper phone holder, flat ("plana"): the phone lies in portrait along the shell, screen up, following the
// shell's slope, held at its four corners by curved hooks on a tray shaped to the shell.
//
// Rodhan (15-oct): no print and no mechanism, a showpiece that holds the phone, curved to blend with the robot,
// the phone close to the body and in portrait. The flat one rides lowest: nothing for a claw to reach.
// Coordinates are Jumper's base_link frame in mm; caparazon.stl (caparazon.py) is the shell top it sits on.
//   openscad -D 'parte="cuna"' -o cuna-plana.stl cuna-plana.scad     (parte: cuna, cuerpo, pantalla, isla, camara)

include <BOSL2/std.scad>
include <telefono.scad>

parte = "cuna";
x_c = -15;         // the phone's centre along the shell
z_c = 70;          // its back face there: 2.5 mm over the highest point of the shell under it
slope = 8;         // degrees, the shell's fall towards the back (72 mm at x 50, 53 mm at x -80)
wall = 2.6;
hook = 17;         // how far each corner hook reaches along the edges
lip = 2.2;         // how far it comes over the screen's border

$fn = 48;

// screen up (phone X -> +z), top of the phone towards the back (phone Z -> -x), tilted with the shell
module en_robot() { translate([x_c, 0, z_c]) rotate([0, -90 - slope, 0]) children(); }
module shell() { import("caparazon.stl", convexity = 6); }

module tray() {
    // the phone's outline grown by the wall, 30 mm deep under its back; what is inside the shell is cut away,
    // so what stays is a bed shaped to the shell (30 mm stays above the height field's floor at z = 20 even at the low back end)
    en_robot() translate([-15, 0, 0])
        cuboid([30, phone[1] + 2 * wall, phone[2] + 2 * wall], rounding = corner + wall, edges = "X");
}

module hooks() {
    // a grown phone, kept only near its corners: four rounded caps over the corners
    en_robot() difference() {
        telefono_cuerpo(grow = wall);
        // keep the corners: cut a cross out of the middle of each side
        cube([40, phone[1] - 2 * hook, 400], center = true);
        cube([40, 400, phone[2] - 2 * hook], center = true);
        // open the screen, all but `lip` over its border
        translate([phone[0] + 5, 0, 0])
            cuboid([10, phone[1] - 2 * lip, phone[2] - 2 * lip], rounding = corner - lip, edges = "X");
    }
}

if (parte == "cuna") {
    difference() {
        union() { tray(); hooks(); }
        shell();
        en_robot() telefono_cuerpo(grow = 0.4);
    }
} else {
    en_robot() telefono(parte);
}
