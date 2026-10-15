//! El guion: lectura y compilación a una tabla de articulaciones.
//!
//! ```text
//! # saludo.guion
//! pulso 100                                  # bpm: los tiempos van en pulsos; «2.5s» es en segundos
//! @0   cara sonrisa
//! @1   patas apoyo_atras en 1                 # llega en 1 pulso
//! @1   tenaza_izq arriba en 1 suave           # suave (mínimo tirón, por defecto) | lineal
//! @2   alternar tenaza_izq arriba_b arriba veces 6 cada 0.5
//! @5   sonido celebracion
//! @6   tenaza_izq reposo en 1
//! @6   patas de_pie en 1
//! @8   fin
//! ```
//!
//! Con `musica <pulsos.json>` (el que escribe `zym-compas pulsos`) en lugar de `pulso`,
//! `@8` es el octavo pulso REAL de la pista, siga o no un tempo fijo; las duraciones
//! (`en`, `cada`) van en pulsos del tempo medio. El clip empieza en el primer pulso y
//! guarda en `musica_desfase` dónde cae en la pista, para alinear el vídeo.
//!
//! Todo empieza en HOME. Un movimiento parte de donde esté la articulación cuando
//! empieza, así que dos órdenes que se solapan se encadenan en vez de saltar.

use crate::robot::{Robot, VEL_MAX};
use serde::Serialize;

pub const HZ: u32 = 50;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Curva {
    Suave,
    Lineal,
}

impl Curva {
    fn f(self, u: f64) -> f64 {
        let u = u.clamp(0.0, 1.0);
        match self {
            // Mínimo tirón (Flash & Hogan): velocidad y aceleración nulas en los extremos.
            Curva::Suave => u * u * u * (10.0 - 15.0 * u + 6.0 * u * u),
            Curva::Lineal => u,
        }
    }
}

#[derive(Debug)]
pub enum Orden {
    Postura {
        grupo: String,
        postura: String,
        en: f64,
        curva: Curva,
    },
    Alternar {
        grupo: String,
        a: String,
        b: String,
        veces: u32,
        cada: f64,
        curva: Curva,
    },
    Cara(String),
    Sonido(String),
    Fin,
}

#[derive(Debug)]
pub struct Linea {
    pub n: usize,
    pub t: f64,
    pub orden: Orden,
}

#[derive(Debug, Default)]
pub struct Guion {
    pub pulso: Option<f64>,
    /// Los pulsos de la pista (segundos en la pista), si el guion va sobre una música.
    pub pulsos_musica: Option<Vec<f64>>,
    pub lineas: Vec<Linea>,
}

impl Guion {
    /// Un tiempo del guion a segundos: «2.5s» siempre en segundos; un número a secas,
    /// en pulsos si hay `pulso` y en segundos si no.
    fn segundos(&self, s: &str) -> Result<f64, String> {
        let (num, en_seg) = match s.strip_suffix('s') {
            Some(n) => (n, true),
            None => (s, false),
        };
        let x: f64 = num.parse().map_err(|_| format!("«{s}» no es un tiempo"))?;
        if x < 0.0 {
            return Err(format!("tiempo negativo «{s}»"));
        }
        Ok(match (en_seg, self.pulso) {
            (false, Some(bpm)) => x * 60.0 / bpm,
            _ => x,
        })
    }

    /// Un instante del guion a segundos: como `segundos`, pero con música un número a
    /// secas es un pulso de la pista (interpolado entre dos, extrapolado al tempo medio
    /// después del último), contado desde el primero.
    fn instante(&self, s: &str) -> Result<f64, String> {
        let Some(p) = &self.pulsos_musica else {
            return self.segundos(s);
        };
        if s.ends_with('s') {
            return self.segundos(s);
        }
        let b: f64 = s.parse().map_err(|_| format!("«{s}» no es un tiempo"))?;
        if b < 0.0 {
            return Err(format!("tiempo negativo «{s}»"));
        }
        let i = b.floor() as usize;
        let t = if i + 1 < p.len() {
            p[i] + (b - i as f64) * (p[i + 1] - p[i])
        } else {
            p[p.len() - 1] + (b - (p.len() - 1) as f64) * 60.0 / self.pulso.unwrap()
        };
        Ok(t - p[0])
    }
}

#[derive(serde::Deserialize)]
struct Pulsos {
    bpm: f64,
    pulsos: Vec<f64>,
}

/// Lee el texto de un guion; `dir` es su carpeta, desde la que se buscan los ficheros que
/// nombra. Los errores llevan el número de línea delante.
pub fn leer(texto: &str, dir: &std::path::Path) -> Result<Guion, String> {
    let mut g = Guion::default();
    for (i, crudo) in texto.lines().enumerate() {
        let n = i + 1;
        let linea = crudo.split('#').next().unwrap().trim();
        if linea.is_empty() {
            continue;
        }
        let err = |e: String| format!("{n}: {e}\n    {}", crudo.trim());
        let p: Vec<&str> = linea.split_whitespace().collect();
        if p[0] == "pulso" {
            let bpm: f64 = p
                .get(1)
                .and_then(|x| x.parse().ok())
                .ok_or_else(|| err("pulso <bpm>".into()))?;
            if !g.lineas.is_empty() {
                return Err(err("`pulso` va antes de la primera orden".into()));
            }
            g.pulso = Some(bpm);
            continue;
        }
        if p[0] == "musica" {
            let f = dir.join(p.get(1).ok_or_else(|| err("musica <pulsos.json>".into()))?);
            let texto =
                std::fs::read_to_string(&f).map_err(|e| err(format!("{}: {e}", f.display())))?;
            let ps: Pulsos =
                serde_json::from_str(&texto).map_err(|e| err(format!("{}: {e}", f.display())))?;
            if !g.lineas.is_empty() || g.pulso.is_some() {
                return Err(err(
                    "`musica` va antes de la primera orden y en lugar de `pulso`".into(),
                ));
            }
            if ps.pulsos.len() < 2 {
                return Err(err(format!("{}: menos de dos pulsos", f.display())));
            }
            g.pulso = Some(ps.bpm);
            g.pulsos_musica = Some(ps.pulsos);
            continue;
        }
        let t = p[0]
            .strip_prefix('@')
            .ok_or_else(|| err("una orden empieza con @<tiempo>".into()))?;
        let t = g.instante(t).map_err(err)?;
        let orden = leer_orden(&g, &p[1..]).map_err(err)?;
        g.lineas.push(Linea { n, t, orden });
    }
    if !g.lineas.iter().any(|l| matches!(l.orden, Orden::Fin)) {
        return Err("0: falta `@<tiempo> fin`: el clip necesita un final explícito".into());
    }
    Ok(g)
}

fn leer_orden(g: &Guion, p: &[&str]) -> Result<Orden, String> {
    let defecto = if g.pulso.is_some() { "1" } else { "0.5s" };
    // Las opciones «en X», «cada X», «veces N» y la curva, en cualquier orden.
    let valor = |clave: &str| {
        p.iter()
            .position(|w| *w == clave)
            .and_then(|i| p.get(i + 1))
            .copied()
    };
    let curva = if p.contains(&"lineal") {
        Curva::Lineal
    } else {
        Curva::Suave
    };
    match p.first().copied() {
        Some("cara") => Ok(Orden::Cara(p.get(1).ok_or("cara <expresión>")?.to_string())),
        Some("sonido") => Ok(Orden::Sonido(
            p.get(1).ok_or("sonido <nombre>")?.to_string(),
        )),
        Some("fin") => Ok(Orden::Fin),
        Some("alternar") => {
            if p.len() < 4 {
                return Err(
                    "alternar <grupo> <postura_a> <postura_b> veces <n> cada <tiempo>".into(),
                );
            }
            Ok(Orden::Alternar {
                grupo: p[1].into(),
                a: p[2].into(),
                b: p[3].into(),
                veces: valor("veces")
                    .unwrap_or("2")
                    .parse()
                    .map_err(|_| "veces <n>: n entero")?,
                cada: g.segundos(valor("cada").unwrap_or(defecto))?,
                curva,
            })
        }
        Some(grupo) => {
            let postura = p.get(1).ok_or(format!("{grupo} <postura> [en <tiempo>]"))?;
            Ok(Orden::Postura {
                grupo: grupo.into(),
                postura: postura.to_string(),
                en: g.segundos(valor("en").unwrap_or(defecto))?,
                curva,
            })
        }
        None => Err("orden vacía".into()),
    }
}

#[derive(Serialize)]
pub struct Informe {
    pub vel_max: f64,
    pub vel_max_en: String,
}

#[derive(Serialize)]
pub struct Clip {
    pub joints: Vec<String>,
    pub hz: u32,
    pub duracion: f64,
    pub pulso: Option<f64>,
    /// Instantes de cada pulso, en segundos, hasta el final.
    pub pulsos: Vec<f64>,
    /// Con música: segundo de la pista en el que empieza el clip (su primer pulso).
    pub musica_desfase: Option<f64>,
    pub q: Vec<Vec<f64>>,
    pub cara: Vec<(f64, String)>,
    pub sonidos: Vec<(f64, String)>,
    pub informe: Informe,
}

struct Mov {
    n: usize,
    t0: f64,
    t1: f64,
    joint: usize,
    objetivo: f64,
    curva: Curva,
}

pub fn compilar(g: &Guion, robot: &Robot) -> Result<Clip, String> {
    let fin = g
        .lineas
        .iter()
        .filter(|l| matches!(l.orden, Orden::Fin))
        .map(|l| l.t)
        .fold(f64::INFINITY, f64::min);
    let mut movs: Vec<Mov> = Vec::new();
    let mut cara = Vec::new();
    let mut sonidos = Vec::new();
    let mut empujar = |n: usize,
                       t0: f64,
                       en: f64,
                       grupo: &str,
                       postura: &str,
                       curva: Curva|
     -> Result<(), String> {
        for (joint, objetivo) in robot
            .objetivos(grupo, postura)
            .map_err(|e| format!("línea {n}: {e}"))?
        {
            let [lo, hi] = robot.range[joint];
            if objetivo < lo || objetivo > hi {
                return Err(format!(
                    "línea {n}: {postura} lleva {} a {objetivo:.3}, fuera de su tope [{lo}, {hi}]",
                    robot.joints[joint]
                ));
            }
            movs.push(Mov {
                n,
                t0,
                t1: t0 + en.max(1e-6),
                joint,
                objetivo,
                curva,
            });
        }
        Ok(())
    };
    for l in &g.lineas {
        if l.t > fin {
            return Err(format!(
                "línea {}: la orden está después de `fin` ({:.2} s > {fin:.2} s)",
                l.n, l.t
            ));
        }
        match &l.orden {
            Orden::Postura {
                grupo,
                postura,
                en,
                curva,
            } => empujar(l.n, l.t, *en, grupo, postura, *curva)?,
            Orden::Alternar {
                grupo,
                a,
                b,
                veces,
                cada,
                curva,
            } => {
                for k in 0..*veces {
                    let p = if k % 2 == 0 { a } else { b };
                    empujar(l.n, l.t + k as f64 * cada, *cada, grupo, p, *curva)?;
                }
            }
            Orden::Cara(e) => cara.push((l.t, e.clone())),
            Orden::Sonido(s) => sonidos.push((l.t, s.clone())),
            Orden::Fin => {}
        }
    }
    // Orden estable: a igual tiempo de inicio manda la línea posterior del guion.
    movs.sort_by(|a, b| a.t0.partial_cmp(&b.t0).unwrap().then(a.n.cmp(&b.n)));

    let dt = 1.0 / HZ as f64;
    let filas = (fin / dt).round() as usize + 1;
    let nj = robot.joints.len();
    let mut actual = robot.home.clone();
    // Por articulación: (valor de partida, índice del movimiento activo).
    let mut activo: Vec<Option<(f64, usize)>> = vec![None; nj];
    let mut siguiente = 0;
    let mut q = Vec::with_capacity(filas);
    for k in 0..filas {
        let t = k as f64 * dt;
        while siguiente < movs.len() && movs[siguiente].t0 <= t + 1e-9 {
            let m = &movs[siguiente];
            activo[m.joint] = Some((actual[m.joint], siguiente));
            siguiente += 1;
        }
        for j in 0..nj {
            if let Some((desde, i)) = activo[j] {
                let m = &movs[i];
                let u = (t - m.t0) / (m.t1 - m.t0);
                actual[j] = desde + (m.objetivo - desde) * m.curva.f(u);
            }
        }
        q.push(actual.clone());
    }

    let mut vel_max = 0.0;
    let mut vel_max_en = String::from("-");
    for k in 1..q.len() {
        for j in 0..nj {
            let v = (q[k][j] - q[k - 1][j]).abs() / dt;
            if v > vel_max {
                vel_max = v;
                vel_max_en = format!("{} a {:.2} s", robot.joints[j], k as f64 * dt);
            }
        }
    }
    if vel_max > VEL_MAX {
        return Err(format!(
            "demasiado rápido: {vel_max:.1} rad/s en {vel_max_en} (límite {VEL_MAX}); alarga el `en` o el `cada`"
        ));
    }
    let pulsos = match (&g.pulsos_musica, g.pulso) {
        (Some(p), _) => p
            .iter()
            .map(|t| t - p[0])
            .take_while(|t| *t <= fin + 1e-9)
            .collect(),
        (None, Some(bpm)) => (0..)
            .map(|i| i as f64 * 60.0 / bpm)
            .take_while(|t| *t <= fin + 1e-9)
            .collect(),
        (None, None) => Vec::new(),
    };
    Ok(Clip {
        joints: robot.joints.clone(),
        hz: HZ,
        duracion: fin,
        pulso: g.pulso,
        pulsos,
        musica_desfase: g.pulsos_musica.as_ref().map(|p| p[0]),
        q,
        cara,
        sonidos,
        informe: Informe {
            vel_max,
            vel_max_en,
        },
    })
}

#[cfg(test)]
mod pruebas {
    use super::*;

    #[test]
    fn los_tiempos_van_en_pulsos_salvo_con_s() {
        let g = leer(
            "pulso 120\n@2 cara a\n@1.5s cara b\n@3 fin\n",
            std::path::Path::new("."),
        )
        .unwrap();
        assert_eq!(g.lineas[0].t, 1.0); // 2 pulsos a 120 bpm
        assert_eq!(g.lineas[1].t, 1.5);
    }

    #[test]
    fn sin_fin_es_un_error() {
        assert!(leer("@0 cara a\n", std::path::Path::new(".")).is_err());
    }

    #[test]
    fn con_musica_un_numero_es_un_pulso_real_de_la_pista() {
        let dir = std::env::temp_dir().join(format!("coreo-prueba-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // Un tempo que se acelera: el pulso 2 cae a 1.0 s de la pista y no a 2 x 60/100.
        std::fs::write(
            dir.join("p.json"),
            r#"{"bpm": 100, "pulsos": [0.5, 1.0, 1.5], "compases": [0.5]}"#,
        )
        .unwrap();
        let g = leer("musica p.json\n@1.5 cara a\n@3 fin\n", &dir).unwrap();
        assert!((g.lineas[0].t - 0.75).abs() < 1e-9); // entre el pulso 1 (0.5) y el 2 (1.0), desde 0.5
        assert!((g.lineas[1].t - (1.0 + 0.6)).abs() < 1e-9); // pasado el último, al tempo medio
    }

    #[test]
    fn la_curva_suave_no_salta_en_los_extremos() {
        let c = Curva::Suave;
        assert_eq!(c.f(0.0), 0.0);
        assert_eq!(c.f(1.0), 1.0);
        // Pendiente casi nula junto a los extremos: lo que distingue a la suave de la lineal.
        assert!(c.f(0.01) < 1e-4 && Curva::Lineal.f(0.01) > 1e-3);
    }
}
