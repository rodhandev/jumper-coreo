//! Los sonidos de Jumper: sintetizados aquí, sin muestras de nadie.
//!
//! Por qué no los de Zymbot: los de Zymbot son clips de Mixkit (su licencia prohíbe darlos sueltos, y una variación
//! sigue siendo obra derivada) o la voz «luma» de Zym Compas, que es la identidad de Zymbot. Publicar en el concurso
//! concede a KingKong un derecho perpetuo y comercial sobre la obra (bases, 15-oct), así que lo que suene en sus
//! vídeos tiene que ser nuestro y poder cederse: este módulo, Apache-2.0 como el resto del repo.
//!
//! Timbre de CANGREJO, distinto del de Zymbot: chasquidos de tenaza (ruido filtrado con un «ping» resonante) y
//! gorjeos cortos y brillantes (seno + 3.er armónico). Las categorías son las del diccionario de Zymbot, sin las
//! excluidas («repetitivos», «ruido molesto»). Determinista: la misma tabla da los mismos ficheros.

use std::path::Path;

pub const SR: u32 = 48_000;

/// Un evento de la frase: gorjeo (f0 -> f1 Hz), chasquido de tenaza, o trino (f Hz con vibrato).
#[derive(Clone, Copy)]
enum Ev {
    Gorjeo(f32, f32, f32),  // f0, f1, duración s
    Tenaza(f32),            // brillo 0–1
    Trino(f32, f32),        // f, duración s
    Campana(f32, f32, f32), // trino golpeado que se apaga: f, duración s, caída s
    Silencio(f32),          // s
}
use Ev::*;

/// La tabla: categoría -> frase. Contornos: subir = positivo/pregunta, bajar = negativo/cierre.
const TABLA: &[(&str, &[Ev])] = &[
    (
        "hablar",
        &[
            Gorjeo(900., 1100., 0.07),
            Silencio(0.03),
            Gorjeo(1000., 850., 0.06),
            Silencio(0.03),
            Gorjeo(950., 1200., 0.08),
        ],
    ),
    (
        "afirmacion corta",
        &[Tenaza(0.6), Silencio(0.04), Gorjeo(800., 1300., 0.09)],
    ),
    (
        "afirmacion larga",
        &[
            Tenaza(0.6),
            Silencio(0.05),
            Gorjeo(700., 1000., 0.1),
            Silencio(0.03),
            Gorjeo(1000., 1500., 0.14),
        ],
    ),
    (
        "negacion",
        &[
            Gorjeo(1100., 700., 0.1),
            Silencio(0.05),
            Gorjeo(1100., 600., 0.12),
        ],
    ),
    (
        "duda corta",
        &[
            Gorjeo(800., 950., 0.08),
            Silencio(0.06),
            Gorjeo(850., 1250., 0.12),
        ],
    ),
    (
        "desconcierto corto",
        &[Gorjeo(1200., 700., 0.07), Gorjeo(700., 1150., 0.09)],
    ),
    (
        "matatiempo corto",
        &[Tenaza(0.3), Silencio(0.12), Tenaza(0.3)],
    ),
    (
        "excusa",
        &[
            Gorjeo(900., 750., 0.09),
            Silencio(0.04),
            Gorjeo(800., 900., 0.08),
            Gorjeo(900., 700., 0.12),
        ],
    ),
    ("censura", &[Trino(1600., 0.35)]),
    (
        "atencion",
        &[
            Tenaza(0.9),
            Silencio(0.06),
            Tenaza(0.9),
            Silencio(0.05),
            Gorjeo(1300., 1700., 0.08),
        ],
    ),
    (
        "alerta",
        &[
            Gorjeo(1500., 1900., 0.1),
            Silencio(0.05),
            Gorjeo(1500., 1900., 0.1),
            Silencio(0.05),
            Gorjeo(1500., 1900., 0.1),
        ],
    ),
    (
        "encendido de robot",
        &[
            Gorjeo(250., 900., 0.35),
            Tenaza(0.7),
            Silencio(0.05),
            Tenaza(0.7),
            Gorjeo(900., 1500., 0.15),
        ],
    ),
    (
        "apagado",
        &[Gorjeo(1200., 250., 0.45), Silencio(0.05), Tenaza(0.3)],
    ),
    (
        "acomodarse",
        &[
            Tenaza(0.4),
            Silencio(0.07),
            Tenaza(0.5),
            Silencio(0.05),
            Gorjeo(600., 750., 0.1),
        ],
    ),
    ("movimiento", &[Tenaza(0.5), Silencio(0.06), Tenaza(0.45)]),
    (
        "sorpresa",
        &[
            Gorjeo(700., 1800., 0.12),
            Silencio(0.03),
            Trino(1800., 0.15),
        ],
    ),
    (
        // Rodhan, 16-oct: de la familia A (jumper_timbre.py), la subida con cola de campana «le da un toque
        // robótico»; es la principal y las demás salen a veces (VARIANTES).
        "celebracion",
        &[
            Gorjeo(900., 1350., 0.07),
            Gorjeo(1350., 1800., 0.07),
            Campana(1800., 0.6, 0.35),
        ],
    ),
    (
        "exito",
        &[
            Gorjeo(800., 1000., 0.07),
            Gorjeo(1000., 1350., 0.07),
            Gorjeo(1350., 1800., 0.14),
        ],
    ),
    (
        "error",
        &[
            Gorjeo(500., 420., 0.12),
            Silencio(0.05),
            Gorjeo(420., 300., 0.2),
        ],
    ),
    ("descontento", &[Gorjeo(700., 520., 0.18), Tenaza(0.3)]),
    (
        "enojo",
        &[
            Tenaza(1.0),
            Tenaza(1.0),
            Tenaza(1.0),
            Gorjeo(600., 450., 0.15),
        ],
    ),
    (
        "tristeza",
        &[
            Gorjeo(700., 420., 0.3),
            Silencio(0.08),
            Gorjeo(520., 350., 0.35),
        ],
    ),
    (
        "alegria",
        &[
            Gorjeo(1100., 1500., 0.08),
            Silencio(0.03),
            Gorjeo(1200., 1700., 0.1),
            Tenaza(0.6),
        ],
    ),
    (
        "risa",
        &[
            Gorjeo(1300., 1100., 0.05),
            Silencio(0.03),
            Gorjeo(1350., 1100., 0.05),
            Silencio(0.03),
            Gorjeo(1400., 1100., 0.05),
            Silencio(0.03),
            Gorjeo(1450., 1000., 0.07),
        ],
    ),
    (
        "miedo",
        &[Trino(1500., 0.2), Silencio(0.04), Trino(1650., 0.15)],
    ),
    (
        "pensando",
        &[
            Tenaza(0.25),
            Silencio(0.18),
            Tenaza(0.25),
            Silencio(0.18),
            Gorjeo(700., 820., 0.12),
        ],
    ),
    (
        "saludo",
        &[
            Tenaza(0.6),
            Gorjeo(800., 1400., 0.1),
            Silencio(0.04),
            Gorjeo(1200., 1000., 0.12),
        ],
    ),
    (
        "despedida",
        &[
            Gorjeo(1200., 1000., 0.1),
            Silencio(0.04),
            Gorjeo(1000., 700., 0.16),
            Tenaza(0.4),
        ],
    ),
    (
        "curiosidad",
        &[
            Gorjeo(700., 900., 0.07),
            Silencio(0.05),
            Gorjeo(800., 1450., 0.14),
        ],
    ),
    (
        "sueno",
        &[
            Gorjeo(500., 380., 0.4),
            Silencio(0.2),
            Gorjeo(450., 360., 0.45),
        ],
    ),
    (
        "carino",
        &[
            Gorjeo(900., 1150., 0.12),
            Silencio(0.04),
            Gorjeo(1150., 950., 0.16),
        ],
    ),
];

/// Otras versiones de una categoría, con su peso: el montaje elige una al azar (con semilla) cada vez que suena,
/// para que la misma emoción no suene idéntica diez veces. La principal (TABLA) se lleva lo que sobra hasta 1.
/// Rodhan, 16-oct: «a4 le da un toque robótico, aunque también podrían usarse las otras en menor medida».
const VARIANTES: &[(&str, f32, &[Ev])] = &[
    // la celebración original: dos chasquidos, subida y trino
    (
        "celebracion",
        0.14,
        &[
            Tenaza(0.8),
            Tenaza(0.8),
            Gorjeo(900., 1350., 0.07),
            Gorjeo(1350., 1800., 0.07),
            Trino(1800., 0.25),
        ],
    ),
    // sin chasquidos
    (
        "celebracion",
        0.13,
        &[
            Gorjeo(900., 1350., 0.07),
            Gorjeo(1350., 1800., 0.07),
            Trino(1800., 0.25),
        ],
    ),
    // cinco semitonos más grave
    (
        "celebracion",
        0.13,
        &[
            Gorjeo(674.2, 1011.4, 0.08),
            Gorjeo(1011.4, 1348.5, 0.08),
            Trino(1348.5, 0.32),
        ],
    ),
];

struct Azar(u64);
impl Azar {
    fn ruido(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 % 20_001) as f32 / 10_000.0 - 1.0
    }
}

/// Envolvente sin clics: ataque y caída de `a` segundos.
fn env(i: usize, n: usize, a: f32) -> f32 {
    let k = (a * SR as f32) as usize;
    let k = k.max(1).min(n / 2).max(1);
    if i < k {
        i as f32 / k as f32
    } else if i + k > n {
        (n - i) as f32 / k as f32
    } else {
        1.0
    }
}

/// Un tono de f0 a f1 Hz (contorno exponencial, suena natural) con vibrato `vib` a 28 Hz:
/// seno + 3.er armónico, el brillo de «bicho» que separa a Jumper de la voz de Zymbot.
fn tono(out: &mut Vec<f32>, f0: f32, f1: f32, d: f32, vib: f32) {
    let tau = std::f32::consts::TAU;
    let n = (d * SR as f32) as usize;
    let mut fase = 0.0f32;
    for i in 0..n {
        let u = i as f32 / n as f32;
        let f = f0 * (f1 / f0).powf(u) * (1.0 + vib * (tau * 28.0 * i as f32 / SR as f32).sin());
        fase += tau * f / SR as f32;
        out.push(0.42 * (fase.sin() + 0.28 * (3.0 * fase).sin()) * env(i, n, 0.008));
    }
}

/// El trino de `tono`, pero golpeado y apagándose como una campanilla: seno con 2.º y 3.er armónicos suaves,
/// ataque de 4 ms y caída exponencial de `caida` s.
fn campana(out: &mut Vec<f32>, f: f32, d: f32, caida: f32) {
    let tau = std::f32::consts::TAU;
    let n = (d * SR as f32) as usize;
    let mut fase = 0.0f32;
    for i in 0..n {
        let t = i as f32 / SR as f32;
        fase += tau * f * (1.0 + 0.08 * (tau * 28.0 * t).sin()) / SR as f32;
        let a = (-t / caida).exp() * (t / 0.004).min(1.0);
        out.push(
            0.5 * (fase.sin() + 0.18 * (2.0 * fase).sin() + 0.06 * (3.0 * fase).sin())
                * a
                * env(i, n, 0.01),
        );
    }
}

fn render(ev: &[Ev], semilla: u64) -> Vec<f32> {
    let mut az = Azar(semilla | 1);
    let mut out = Vec::new();
    let tau = std::f32::consts::TAU;
    for e in ev {
        match *e {
            Gorjeo(f0, f1, d) => tono(&mut out, f0, f1, d, 0.0),
            Trino(f, d) => tono(&mut out, f, f, d, 0.08),
            Campana(f, d, caida) => campana(&mut out, f, d, caida),
            Tenaza(brillo) => {
                // chasquido: ruido con caída de 18 ms + ping resonante a 2,2–3,4 kHz
                let n = (0.045 * SR as f32) as usize;
                let fp = 2200.0 + 1200.0 * brillo;
                let mut prev = 0.0f32;
                for i in 0..n {
                    let t = i as f32 / SR as f32;
                    let r = az.ruido();
                    let hp = r - prev; // paso alto de primer orden: quita el «soplido» grave
                    prev = r;
                    let s = 0.55 * hp * (-t / 0.006).exp()
                        + 0.35 * (tau * fp * t).sin() * (-t / 0.018).exp();
                    out.push((0.5 + 0.5 * brillo) * s);
                }
            }
            Silencio(d) => out.extend(std::iter::repeat(0.0).take((d * SR as f32) as usize)),
        }
    }
    // pico a -3 dBFS: todas las categorías al mismo nivel, como los de Zymbot
    let pico = out.iter().fold(0.0f32, |m, x| m.max(x.abs())).max(1e-6);
    let mut out: Vec<f32> = out.iter().map(|x| x * 0.708 / pico).collect();
    // 150 ms de silencio al final: algunos reproductores (VLC, 15-oct) cortaban el último gorjeo de los ficheros cortos.
    out.extend(std::iter::repeat(0.0).take((0.15 * SR as f32) as usize));
    out
}

fn wav(path: &Path, x: &[f32]) -> std::io::Result<()> {
    let datos: Vec<u8> = x
        .iter()
        .flat_map(|v| ((v.clamp(-1.0, 1.0) * 32767.0) as i16).to_le_bytes())
        .collect();
    let mut h = Vec::with_capacity(44 + datos.len());
    h.extend(b"RIFF");
    h.extend((36 + datos.len() as u32).to_le_bytes());
    h.extend(b"WAVEfmt ");
    h.extend(16u32.to_le_bytes());
    h.extend(1u16.to_le_bytes()); // PCM
    h.extend(1u16.to_le_bytes()); // mono
    h.extend(SR.to_le_bytes());
    h.extend((SR * 2).to_le_bytes());
    h.extend(2u16.to_le_bytes());
    h.extend(16u16.to_le_bytes());
    h.extend(b"data");
    h.extend((datos.len() as u32).to_le_bytes());
    h.extend(datos);
    std::fs::write(path, h)
}

fn archivo(cat: &str) -> String {
    format!("jumper_{}.wav", cat.replace(' ', "_"))
}

/// Escribe un .wav por categoría y `diccionario.json` en `dir`.
pub fn generar(dir: &Path) -> Result<Vec<(String, f64)>, String> {
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    let mut dic = serde_json::Map::new();
    let mut hechos = Vec::new();
    for (i, (cat, ev)) in TABLA.iter().enumerate() {
        let x = render(ev, 0x9E37_79B9_7F4A_7C15 ^ (i as u64 + 1));
        let f = archivo(cat);
        wav(&dir.join(&f), &x).map_err(|e| format!("{f}: {e}"))?;
        let dur = x.len() as f64 / SR as f64;
        let otras: Vec<_> = VARIANTES.iter().filter(|(c, _, _)| c == cat).collect();
        let resto = 1.0 - otras.iter().map(|(_, p, _)| p).sum::<f32>();
        let mut lista = vec![
            serde_json::json!({ "clip": f, "dur": (dur * 100.0).round() / 100.0, "generado": true,
                                                 "preferido": true, "peso": (resto as f64 * 100.0).round() / 100.0 }),
        ];
        for (k, (_, peso, ev)) in otras.iter().enumerate() {
            let x = render(
                ev,
                0x9E37_79B9_7F4A_7C15 ^ (i as u64 + 1) ^ ((k as u64 + 2) << 32),
            );
            let f = archivo(&format!("{cat}_{}", k + 2));
            wav(&dir.join(&f), &x).map_err(|e| format!("{f}: {e}"))?;
            let dur = x.len() as f64 / SR as f64;
            lista.push(serde_json::json!({ "clip": f, "dur": (dur * 100.0).round() / 100.0, "generado": true,
                                           "preferido": false, "peso": (*peso as f64 * 100.0).round() / 100.0 }));
        }
        dic.insert(cat.to_string(), serde_json::Value::Array(lista));
        hechos.push((cat.to_string(), dur));
    }
    let doc = serde_json::json!({
        "estilo": "cangrejo: chasquidos de tenaza y gorjeos (sintetizado por coreo, src/sonidos.rs)",
        "licencia": "Apache-2.0 (como el repo); sin muestras de terceros",
        "volumen_sugerido": 0.51,
        "emociones": dic,
    });
    std::fs::write(
        dir.join("diccionario.json"),
        serde_json::to_string_pretty(&doc).unwrap(),
    )
    .map_err(|e| e.to_string())?;
    Ok(hechos)
}

#[cfg(test)]
mod pruebas {
    use super::*;

    #[test]
    fn ninguna_categoria_satura_ni_queda_muda() {
        for (i, (cat, ev)) in TABLA.iter().enumerate() {
            let x = render(ev, i as u64 + 1);
            let pico = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert!((0.70..=0.71).contains(&pico), "{cat}: pico {pico}");
            assert!(x.len() > SR as usize / 20, "{cat}: dura menos de 50 ms");
        }
    }

    #[test]
    fn las_variantes_suenan_y_sus_pesos_dejan_sitio_a_la_principal() {
        for (cat, peso, ev) in VARIANTES {
            assert!(
                TABLA.iter().any(|(c, _)| c == cat),
                "{cat}: variante sin categoría principal"
            );
            let x = render(ev, 1);
            let pico = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert!(
                (0.70..=0.71).contains(&pico) && *peso > 0.0,
                "{cat}: pico {pico}"
            );
        }
        let celebra: f32 = VARIANTES
            .iter()
            .filter(|(c, _, _)| *c == "celebracion")
            .map(|(_, p, _)| p)
            .sum();
        assert!(celebra < 0.5, "la principal debe ser la más frecuente");
    }

    #[test]
    fn determinista() {
        let (_, ev) = TABLA[0];
        assert_eq!(render(ev, 7), render(ev, 7));
    }
}
