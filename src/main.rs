//! coreo — de un guion de comportamiento a un clip de movimiento para Jumper.
//!
//! El guion dice QUÉ hace el robot y CUÁNDO (en pulsos de la música o en segundos);
//! las posturas dicen CÓMO es cada pose; coreo interpola, espeja la tenaza derecha,
//! comprueba topes y velocidades y escribe una tabla de 22 articulaciones a 50 Hz.
//! El paso a `.npz` entrenable (pose del cuerpo, entrada y salida desde HOME) lo hace
//! `puente/clip_a_npz.py` con las herramientas del repo de Jumper.
//!
//!   coreo compilar guiones/saludo.guion -o salidas/saludo.json
//!   coreo posturas
//!
//! Principio rector: el guion propone, el software decide. Una postura o un grupo que
//! no existen son un error con la línea; un tope superado también, nunca un recorte
//! silencioso.

mod guion;
mod robot;

use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn opt(args: &[String], name: &str, default: &str) -> PathBuf {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(default))
}

fn run(args: &[String]) -> Result<(), String> {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let robot_path = opt(args, "--robot", &base.join("robot.json").to_string_lossy());
    let posturas_path = opt(args, "--posturas", &base.join("posturas/base.toml").to_string_lossy());
    let robot = robot::Robot::cargar(&robot_path, &posturas_path)?;

    match args.first().map(String::as_str) {
        Some("compilar") => {
            let src = args.get(1).ok_or("falta el guion: coreo compilar <guion> -o <clip.json>")?;
            let dest = opt(args, "-o", "clip.json");
            let texto = std::fs::read_to_string(src).map_err(|e| format!("{src}: {e}"))?;
            let g = guion::leer(&texto).map_err(|e| format!("{src}:{e}"))?;
            let clip = guion::compilar(&g, &robot).map_err(|e| format!("{src}: {e}"))?;
            std::fs::write(&dest, serde_json::to_string(&clip).unwrap())
                .map_err(|e| format!("{}: {e}", dest.display()))?;
            println!(
                "{} -> {}: {:.2} s, {} filas a {} Hz, {} caras, {} sonidos",
                src,
                dest.display(),
                clip.duracion,
                clip.q.len(),
                clip.hz,
                clip.cara.len(),
                clip.sonidos.len()
            );
            println!(
                "  velocidad máxima {:.2} rad/s en {} (límite {:.1})",
                clip.informe.vel_max, clip.informe.vel_max_en, robot::VEL_MAX
            );
            Ok(())
        }
        Some("posturas") => {
            for (grupo, nombres) in robot.catalogo() {
                println!("{grupo}: {}", nombres.join(", "));
            }
            Ok(())
        }
        _ => Err("uso: coreo compilar <guion> -o <clip.json> | coreo posturas  \
                  [--robot robot.json] [--posturas posturas/base.toml]"
            .into()),
    }
}
