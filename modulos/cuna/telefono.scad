// A generic 6.1" phone, so the video shows a phone and not a box (Rodhan, 15-oct: «use a real phone model»).
// The size and mass of today's common 6.1" phones (147.6 x 71.6 x 7.8 mm, ~172 g); no brand, no logo.
// Its frame: origin at the centre of the back face, X out of the screen, Y across, Z along (top = +Z).
// Each part is a colour of its own in the MJCF: telefono(parte) draws one.

include <BOSL2/std.scad>

phone = [7.8, 71.6, 147.6];   // thickness, width, length
corner = 9;                   // the outline's corner radius
edge = 2.4;                   // the rounding of the frame's edges
bezel = 1.8;                  // black border around the screen

module telefono_cuerpo(grow = 0) {
    // the outline rounded at `corner`, every edge softened at `edge`
    s = phone + [2 * grow, 2 * grow, 2 * grow];
    translate([phone[0] / 2, 0, 0])
        intersection() {
            cuboid(s, rounding = corner + grow, edges = "X");
            cuboid(s, rounding = edge + grow, except = "X");
        }
}

module telefono(parte = "cuerpo") {
    if (parte == "cuerpo") {
        telefono_cuerpo();
    } else if (parte == "pantalla") {
        // the glass, 0.2 mm proud of the frame so it is seen
        translate([phone[0] - 0.1, 0, 0])
            cuboid([0.3, phone[1] - 2 * bezel, phone[2] - 2 * bezel], rounding = corner - bezel, edges = "X");
    } else if (parte == "isla") {
        // the camera pill at the top of the screen
        translate([phone[0] + 0.1, 0, phone[2] / 2 - 11])
            cuboid([0.3, 20, 6.2], rounding = 3.0, edges = "X");
    } else if (parte == "camara") {
        // the camera block on the back, top corner, with three lenses
        translate([0, phone[1] / 2 - 21, phone[2] / 2 - 21]) {
            translate([-0.75, 0, 0]) cuboid([1.6, 32, 32], rounding = 7, edges = "X");
            for (p = [[-7, 7], [7, 0], [-7, -7]])
                translate([-1.9, p[0], p[1]]) rotate([0, 90, 0]) cylinder(h = 1.6, r = 5.6, center = true, $fn = 40);
        }
    }
}
