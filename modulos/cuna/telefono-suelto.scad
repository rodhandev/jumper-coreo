// The phone on its own, lying screen up on a table, its centre at the origin, top towards +y: for a set where it
// sits beside the robot instead of in a holder (Rodhan, 17-oct). Same parts as telefono.scad.
//   openscad -D 'parte="cuerpo"' -o stl/suelto-cuerpo.stl telefono-suelto.scad
include <BOSL2/std.scad>
include <telefono.scad>

parte = "cuerpo";
// phone X (out of the screen) -> +z, Y (across) -> +x, Z (along, top) -> +y; the back on z = 0
multmatrix([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]) telefono(parte);
