//! El robot (robot.json, exportado del MJCF por puente/exportar_robot.py) y las posturas.

use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

/// Velocidad articular máxima que se acepta, rad/s. Medido el 14-oct sobre los clips
/// del repo a 50 Hz: el baile `dance` (crab, ya entrenado y desplegado) llega a 12,86
/// con p99 6,0; los gestos se quedan en 3,3–7,8; `dance_brazilian` llega a 22. Se toma
/// el nivel del crab: rápido, y probado que una política lo sigue.
pub const VEL_MAX: f64 = 12.0;

#[derive(Deserialize)]
struct RobotJson {
    joints: Vec<String>,
    home: BTreeMap<String, f64>,
    range: BTreeMap<String, [f64; 2]>,
    mirror_keep_sign: Vec<String>,
}

pub struct Robot {
    pub joints: Vec<String>,
    pub home: Vec<f64>,
    pub range: Vec<[f64; 2]>,
    keep_sign: Vec<String>,
    /// "tenaza.arriba" -> {"J0": …}; "patas.de_pie" -> {"LM_J0_joint": …}
    posturas: BTreeMap<String, BTreeMap<String, f64>>,
}

/// Los grupos que un guion puede mover.
pub const GRUPOS: &[&str] = &["tenaza_izq", "tenaza_der", "tenazas", "patas"];

impl Robot {
    pub fn cargar(robot: &Path, posturas: &Path) -> Result<Self, String> {
        let r: RobotJson = serde_json::from_str(
            &std::fs::read_to_string(robot).map_err(|e| format!("{}: {e}", robot.display()))?,
        )
        .map_err(|e| format!("{}: {e}", robot.display()))?;
        let tabla: toml::Table = std::fs::read_to_string(posturas)
            .map_err(|e| format!("{}: {e}", posturas.display()))?
            .parse()
            .map_err(|e| format!("{}: {e}", posturas.display()))?;
        let mut p = BTreeMap::new();
        for (grupo, v) in &tabla {
            let Some(sub) = v.as_table() else { continue };
            for (nombre, valores) in sub {
                let mut m = BTreeMap::new();
                for (k, x) in valores.as_table().ok_or(format!("[{grupo}.{nombre}] no es una tabla"))? {
                    let x = x.as_float().or(x.as_integer().map(|i| i as f64));
                    m.insert(k.clone(), x.ok_or(format!("[{grupo}.{nombre}] {k} no es un número"))?);
                }
                p.insert(format!("{grupo}.{nombre}"), m);
            }
        }
        Ok(Robot {
            home: r.joints.iter().map(|j| r.home[j]).collect(),
            range: r.joints.iter().map(|j| r.range[j]).collect(),
            joints: r.joints,
            keep_sign: r.mirror_keep_sign,
            posturas: p,
        })
    }

    pub fn indice(&self, joint: &str) -> Option<usize> {
        self.joints.iter().position(|j| j == joint)
    }

    /// Los objetivos (índice de articulación, ángulo) de `postura` en `grupo`.
    pub fn objetivos(&self, grupo: &str, postura: &str) -> Result<Vec<(usize, f64)>, String> {
        // Una postura de las dos tenazas escrita entera manda sobre el espejo de una sola.
        if grupo == "tenazas" {
            if let Some(valores) = self.posturas.get(&format!("tenazas.{postura}")) {
                return valores
                    .iter()
                    .map(|(j, &x)| self.indice(j).map(|i| (i, x)).ok_or(format!("[tenazas.{postura}]: {j} desconocida")))
                    .collect();
            }
        }
        let (familia, lados): (&str, &[&str]) = match grupo {
            "tenaza_izq" => ("tenaza", &["LF"]),
            "tenaza_der" => ("tenaza", &["RF"]),
            "tenazas" => ("tenaza", &["LF", "RF"]),
            "patas" => ("patas", &[]),
            _ => return Err(format!("grupo «{grupo}» no existe; hay: {}", GRUPOS.join(", "))),
        };
        let clave = format!("{familia}.{postura}");
        let valores = self.posturas.get(&clave).ok_or_else(|| {
            format!("postura «{postura}» no existe para {familia}; hay: {}", self.nombres(familia).join(", "))
        })?;
        let mut out = Vec::new();
        if familia == "patas" {
            for (j, &x) in valores {
                out.push((self.indice(j).ok_or(format!("[{clave}]: articulación {j} desconocida"))?, x));
            }
        } else {
            // Las tenazas se escriben para la izquierda; la derecha es su espejo. Se espeja
            // el desplazamiento desde HOME y no el ángulo: el robot no es simétrico del todo
            // (HOME: LF_J3 -1.374, RF_J3 1.4173), y espejar el ángulo dejaba `reposo` de la
            // derecha a 0.043 rad de HOME.
            for lado in lados {
                for (j, &x) in valores {
                    let nombre = format!("{lado}_{j}_joint");
                    let i = self.indice(&nombre).ok_or(format!("[{clave}]: {j} no es J0..J4"))?;
                    let izq = self.indice(&format!("LF_{j}_joint")).unwrap();
                    let signo = if *lado == "LF" || self.keep_sign.iter().any(|k| k == &nombre) { 1.0 } else { -1.0 };
                    out.push((i, self.home[i] + signo * (x - self.home[izq])));
                }
            }
        }
        Ok(out)
    }

    fn nombres(&self, familia: &str) -> Vec<String> {
        self.posturas
            .keys()
            .filter_map(|k| k.strip_prefix(&format!("{familia}.")).map(String::from))
            .collect()
    }

    pub fn catalogo(&self) -> Vec<(&'static str, Vec<String>)> {
        vec![
            ("tenaza (tenaza_izq, tenaza_der, tenazas)", self.nombres("tenaza")),
            ("tenazas, sólo las dos juntas", self.nombres("tenazas")),
            ("patas", self.nombres("patas")),
        ]
    }
}
