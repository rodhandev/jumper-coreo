//! El bailarín automático: de los pulsos de una pista a un guion completo.
//!
//!   coreo bailar <pulsos.json> --estilo romantico|rapido|clasico [--compases N] [--semilla S]
//!                [--instrumentos <tramos.json>] -o baile.guion
//!
//! Empieza en el primer compás que detectó `zym-compas pulsos`, llena cada hueco con un paso del estilo
//! (`pasos/estilos.toml`) sin repetir el mismo dos veces seguidas, y cierra con una celebración. El guion sale
//! como texto para poder leerlo y afinarlo a mano: la máquina propone, el guion se revisa.
//!
//! Con `--instrumentos` (lo que escribe `puente/instrumento.py --json`: qué instrumento suena en cada tramo), el
//! robot toca el que suena (Rodhan, 16-oct): en cada hueco, si el tramo tiene un paso de instrumento, va ése, y
//! puede repetirse mientras siga sonando; si no («mecerse»), sigue el estilo.

use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

#[derive(Deserialize)]
struct Pulsos {
    pulsos: Vec<f64>,
    #[serde(default)]
    compases: Vec<f64>,
}

#[derive(Deserialize)]
struct Tramos {
    /// [inicio s, fin s, paso, puntuación]
    tramos: Vec<(f64, f64, String, f64)>,
}

#[derive(Deserialize)]
struct Estilo {
    cara: String,
    pasos: Vec<String>,
}

fn dura(pasos: &Path, nombre: &str) -> Result<f64, String> {
    let f = pasos.join(format!("{nombre}.paso"));
    let t = std::fs::read_to_string(&f).map_err(|e| format!("{}: {e}", f.display()))?;
    t.lines()
        .find_map(|l| {
            l.trim()
                .strip_prefix("dura ")
                .and_then(|x| x.trim().parse().ok())
        })
        .ok_or(format!("{}: falta `dura`", f.display()))
}

pub fn bailar(
    pulsos_json: &Path,
    estilo: &str,
    compases: Option<usize>,
    semilla: u64,
    pasos: &Path,
    instrumentos: Option<&Path>,
) -> Result<String, String> {
    let tramos = match instrumentos {
        Some(f) => {
            let t: Tramos = serde_json::from_str(
                &std::fs::read_to_string(f).map_err(|e| format!("{}: {e}", f.display()))?,
            )
            .map_err(|e| format!("{}: {e}", f.display()))?;
            t.tramos
        }
        None => Vec::new(),
    };
    let p: Pulsos = serde_json::from_str(
        &std::fs::read_to_string(pulsos_json)
            .map_err(|e| format!("{}: {e}", pulsos_json.display()))?,
    )
    .map_err(|e| format!("{}: {e}", pulsos_json.display()))?;
    let estilos: BTreeMap<String, Estilo> = toml::from_str(
        &std::fs::read_to_string(pasos.join("estilos.toml"))
            .map_err(|e| format!("estilos.toml: {e}"))?,
    )
    .map_err(|e| format!("estilos.toml: {e}"))?;
    let e = estilos.get(estilo).ok_or_else(|| {
        format!(
            "estilo «{estilo}» no existe; hay: {}",
            estilos.keys().cloned().collect::<Vec<_>>().join(", ")
        )
    })?;

    // El primer compás, como índice de pulso: el baile arranca en un «uno».
    let b0 = match p.compases.first() {
        Some(c) => {
            p.pulsos
                .iter()
                .enumerate()
                .min_by(|a, b| (a.1 - c).abs().partial_cmp(&(b.1 - c).abs()).unwrap())
                .unwrap()
                .0
        }
        None => 0,
    };
    let ultimo = p.pulsos.len().saturating_sub(4); // 4 pulsos de cierre dentro de la pista
    let mut g = String::new();
    let ruta = pulsos_json.canonicalize().map_err(|e| e.to_string())?;
    g += &format!("# Generado por `coreo bailar` (estilo {estilo}, semilla {semilla}). Se puede editar a mano.\n");
    g += &format!("musica {}\n", ruta.display());
    g += &format!("@{b0} cara {}\n@{b0} sonido encendido_de_robot\n", e.cara);

    let mut b = b0 as f64 + 1.0; // un pulso de respiro antes del primer paso
    let mut k = (semilla as usize) % e.pasos.len();
    let mut anterior = String::new();
    let mut hechos = 0;
    while compases.map_or(true, |n| hechos < n) {
        // el instrumento que suena en este pulso, si hay uno reconocido
        let t = p.pulsos.get(b as usize).copied().unwrap_or(f64::INFINITY);
        let toca = tramos
            .iter()
            .find(|(a, z, paso, _)| *a <= t && t < *z && paso != "mecerse")
            .map(|x| x.2.clone());
        let mut nombre = e.pasos[k % e.pasos.len()].clone();
        if let Some(i) = toca {
            nombre = i;
        } else if nombre == anterior {
            k += 1;
            nombre = e.pasos[k % e.pasos.len()].clone();
        }
        let d = dura(pasos, &nombre)?;
        if b + d > ultimo as f64 {
            break;
        }
        g += &format!("@{b} paso {nombre}\n");
        b += d;
        k += 1;
        hechos += 1;
        anterior = nombre;
    }
    if hechos == 0 {
        return Err("la pista es demasiado corta para un solo paso".into());
    }
    g += &format!(
        "@{b} cara sonrisa\n@{b} sonido celebracion\n@{} fin\n",
        b + 2.0
    );
    Ok(g)
}

#[cfg(test)]
mod pruebas {
    use super::*;

    #[test]
    fn toca_el_instrumento_que_suena_y_si_no_sigue_el_estilo() {
        let dir = std::env::temp_dir().join(format!("coreo-bailar-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("estilos.toml"), "[e]\ncara = \"sonrisa\"\npasos = [\"mecerse\"]\n").unwrap();
        for p in ["mecerse", "violin"] {
            std::fs::write(dir.join(format!("{p}.paso")), "dura 4\n").unwrap();
        }
        let pulsos: Vec<f64> = (0..40).map(|i| i as f64 * 0.5).collect();
        std::fs::write(dir.join("p.json"), serde_json::json!({ "pulsos": pulsos }).to_string()).unwrap();
        // violín de 0 a 5 s (pulsos 0-9, a medio segundo), nada reconocido después
        std::fs::write(dir.join("t.json"), r#"{"tramos": [[0, 5, "violin", 0.3], [5, 20, "mecerse", 0]]}"#).unwrap();
        let g = bailar(&dir.join("p.json"), "e", None, 0, &dir, Some(&dir.join("t.json"))).unwrap();
        assert!(g.contains("@1 paso violin\n@5 paso violin\n@9 paso violin\n@13 paso mecerse\n"), "{g}");
        let sin = bailar(&dir.join("p.json"), "e", None, 0, &dir, None).unwrap();
        assert!(!sin.contains("violin"));
        std::fs::remove_dir_all(&dir).ok();
    }
}
