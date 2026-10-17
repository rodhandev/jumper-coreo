"""What the phone's screen shows, drawn per frame: the guion's `telefono <estado>` orders become moving screens.

Rodhan (16-oct): «on the screen there should be a music player when music plays, a call screen when a call comes
in…; find the most common uses of the phone». The common moments a phone on a desk companion lives through,
each drawn here with no third-party logo or brand (a generic chat bubble, not any messenger's):

    reposo        lock screen: time and date
    musica        music player: cover, title, progress and a moving equaliser (the music comes from the phone)
    musica_fiesta the same player on a party track (other title and colours)
    llamada       incoming call: caller, pulsing rings, answer / decline
    mensaje       a chat notification sliding down over the lock screen
    videollamada  a video call in progress
    alarma        alarm ringing: big time, snooze / stop
    notificacion  a calendar reminder card
    cargando      charging: a ring filling up
    termostato    smart-home heating dial going down («Jumper, turn down the heating»)
    arena V1 V2 [hit1|hit2|draw]   «Jumper Arena», the battle from the app (17-oct): both health bars (0-100), a
                  fire button, and a flash on a hit or the draw. Its arguments go in the state: `telefono arena 100 70 hit2`

    imagen(estado, t, w=390, h=844) -> PIL.Image     t: seconds since the state began
    python pantallas.py <dir>                         writes one still of each, for a look

The canvas is the screen in portrait, top up. English text: the video is for an international audience.
"""

import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

FONT = "/usr/share/fonts/opentype/inter/Inter-{}.otf"
CIAN, BLANCO, GRIS = (0, 195, 227), (245, 247, 250), (150, 158, 170)
VERDE, ROJO = (52, 199, 89), (255, 69, 58)
ESTADOS = ("reposo", "musica", "musica_fiesta", "llamada", "mensaje", "videollamada", "alarma", "notificacion", "cargando",
           "termostato", "arena")


def fuente(peso, px):
    try:
        return ImageFont.truetype(FONT.format(peso), px)
    except OSError:
        return ImageFont.load_default()


def fondo(w, h, a=(18, 32, 68), b=(70, 30, 90)):
    """A vertical gradient wallpaper with a soft glow, so the screen never reads as a flat box."""
    im = Image.new("RGB", (w, h))
    d = ImageDraw.Draw(im)
    for y in range(h):
        u = y / h
        d.line([(0, y), (w, y)], fill=tuple(int(a[i] * (1 - u) + b[i] * u) for i in range(3)))
    glow = Image.new("RGB", (w, h), (0, 0, 0))
    ImageDraw.Draw(glow).ellipse([w * 0.1, h * 0.15, w * 0.9, h * 0.55], fill=(40, 90, 140))
    return Image.blend(im, Image.composite(glow, im, glow.convert("L").filter(ImageFilter.GaussianBlur(60))), 0.5)


def barra_estado(d, w, s):
    d.text((24 * s, 14 * s), "9:41", font=fuente("SemiBold", int(15 * s)), fill=BLANCO)
    x = w - 30 * s
    d.rounded_rectangle([x - 22 * s, 17 * s, x, 28 * s], radius=3 * s, outline=BLANCO, width=max(1, int(1.5 * s)))
    d.rectangle([x - 20 * s, 19 * s, x - 6 * s, 26 * s], fill=BLANCO)


def centrado(d, w, y, texto, f, color):
    d.text((w / 2, y), texto, font=f, fill=color, anchor="mt")


def boton(d, cx, cy, r, color, glifo, s):
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color)
    if glifo == "tel":   # a handset: two rounded blobs joined by an arc
        d.arc([cx - r * 0.5, cy - r * 0.5, cx + r * 0.5, cy + r * 0.5], 200, 340, fill=BLANCO, width=int(7 * s))
    elif glifo == "x":
        k = r * 0.35
        d.line([cx - k, cy - k, cx + k, cy + k], fill=BLANCO, width=int(5 * s))
        d.line([cx - k, cy + k, cx + k, cy - k], fill=BLANCO, width=int(5 * s))


def reposo(w, h, t, s):
    im = fondo(w, h)
    d = ImageDraw.Draw(im)
    barra_estado(d, w, s)
    centrado(d, w, 120 * s, "Friday, October 17", fuente("Medium", int(19 * s)), BLANCO)
    centrado(d, w, 150 * s, "9:41", fuente("Bold", int(92 * s)), BLANCO)
    return im, d


def musica(w, h, t, s, titulo="Moonlight Waltz", autor="Jumper's romantic mix", tinta=((60, 20, 50), (20, 10, 30)),
           portada=(200, 70, 110), barras=(255, 140, 170)):
    im = fondo(w, h, *tinta)
    d = ImageDraw.Draw(im)
    barra_estado(d, w, s)
    m = 34 * s
    cov = [m, 90 * s, w - m, 90 * s + (w - 2 * m)]
    d.rounded_rectangle(cov, radius=18 * s, fill=portada)
    cx, cy = (cov[0] + cov[2]) / 2, (cov[1] + cov[3]) / 2
    for k in range(6, 0, -1):   # cover art: concentric warm rings and a heart
        r = k * 24 * s
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(255, 160 - 12 * k, 150), width=int(3 * s))
    hr = 30 * s
    d.polygon([(cx, cy + hr), (cx - hr * 1.1, cy - hr * 0.1), (cx, cy - hr * 0.3), (cx + hr * 1.1, cy - hr * 0.1)],
              fill=BLANCO)
    d.ellipse([cx - hr * 1.15, cy - hr * 0.75, cx - hr * 0.0, cy + hr * 0.25], fill=BLANCO)
    d.ellipse([cx, cy - hr * 0.75, cx + hr * 1.15, cy + hr * 0.25], fill=BLANCO)
    y = cov[3] + 26 * s
    d.text((m, y), titulo, font=fuente("Bold", int(26 * s)), fill=BLANCO)
    d.text((m, y + 36 * s), autor, font=fuente("Medium", int(18 * s)), fill=GRIS)
    y += 86 * s
    total, pos = 154.0, 31.0 + t
    d.rounded_rectangle([m, y, w - m, y + 6 * s], radius=3 * s, fill=(90, 80, 100))
    xe = m + (w - 2 * m) * min(pos / total, 1)
    d.rounded_rectangle([m, y, xe, y + 6 * s], radius=3 * s, fill=BLANCO)
    d.ellipse([xe - 8 * s, y - 5 * s, xe + 8 * s, y + 11 * s], fill=BLANCO)
    f = fuente("Medium", int(14 * s))
    d.text((m, y + 16 * s), f"{int(pos) // 60}:{int(pos) % 60:02d}", font=f, fill=GRIS)
    d.text((w - m, y + 16 * s), f"-{int(total - pos) // 60}:{int(total - pos) % 60:02d}", font=f, fill=GRIS, anchor="ra")
    y += 70 * s   # transport: previous, pause, next
    d.rectangle([w / 2 - 14 * s, y, w / 2 - 4 * s, y + 36 * s], fill=BLANCO)
    d.rectangle([w / 2 + 4 * s, y, w / 2 + 14 * s, y + 36 * s], fill=BLANCO)
    for sg in (-1, 1):
        x0 = w / 2 + sg * 100 * s
        for k in (0, 1):
            xa = x0 + sg * (k * 16 - 8) * s
            d.polygon([(xa, y + 6 * s), (xa, y + 30 * s), (xa + sg * 18 * s, y + 18 * s)], fill=BLANCO)
    y += 70 * s   # equaliser, moving with the music
    n = 18
    bw = (w - 2 * m) / n
    for i in range(n):
        a = 0.5 + 0.5 * math.sin(t * (5 + i % 5) + i * 1.7) * math.sin(t * 2.3 + i)
        bh = (10 + 60 * abs(a)) * s
        d.rounded_rectangle([m + i * bw + 2 * s, y + 70 * s - bh, m + (i + 1) * bw - 2 * s, y + 70 * s], radius=3 * s,
                            fill=barras)
    return im, d


def musica_fiesta(w, h, t, s):
    return musica(w, h, t, s, "Turn Up!", "Jumper's party mix", ((20, 30, 80), (10, 60, 70)), (40, 170, 210),
                  (90, 230, 200))


def llamada(w, h, t, s):
    im = fondo(w, h, (20, 40, 40), (10, 20, 30))
    d = ImageDraw.Draw(im)
    barra_estado(d, w, s)
    centrado(d, w, 90 * s, "Incoming call", fuente("Medium", int(18 * s)), GRIS)
    centrado(d, w, 120 * s, "Mom", fuente("Bold", int(44 * s)), BLANCO)
    cx, cy, r = w / 2, 330 * s, 70 * s
    for k in range(3):   # rings pulsing outwards, once a second
        u = (t + k / 3) % 1.0
        rr = r + u * 90 * s
        c = tuple(int(v * (1 - u) + 20 * u) for v in (80, 200, 170))
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=c, width=int(3 * s))
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(90, 160, 150))
    centrado(d, w, cy - 34 * s, "M", fuente("Bold", int(64 * s)), BLANCO)
    wob = math.sin(t * 18) * 4 * s if (t % 1.0) < 0.5 else 0   # the answer button shakes while it rings
    boton(d, w * 0.27, h - 150 * s, 38 * s, ROJO, "x", s)
    boton(d, w * 0.73 + wob, h - 150 * s, 38 * s, VERDE, "tel", s)
    f = fuente("Medium", int(15 * s))
    d.text((w * 0.27, h - 100 * s), "Decline", font=f, fill=BLANCO, anchor="mt")
    d.text((w * 0.73, h - 100 * s), "Accept", font=f, fill=BLANCO, anchor="mt")
    return im, d


def mensaje(w, h, t, s):
    im, d = reposo(w, h, t, s)
    u = min(t / 0.35, 1.0)   # slides down in 0.35 s, eased
    u = 1 - (1 - u) ** 3
    y0 = (-130 + u * 400) * s
    m = 14 * s
    card = Image.new("RGBA", im.size, (0, 0, 0, 0))
    cd = ImageDraw.Draw(card)
    cd.rounded_rectangle([m, y0, w - m, y0 + 110 * s], radius=22 * s, fill=(245, 245, 248, 235))
    cd.rounded_rectangle([m + 14 * s, y0 + 16 * s, m + 50 * s, y0 + 52 * s], radius=9 * s, fill=VERDE)
    bx, by = m + 32 * s, y0 + 33 * s    # a plain speech bubble: no messenger's logo
    cd.ellipse([bx - 11 * s, by - 9 * s, bx + 11 * s, by + 8 * s], fill=BLANCO)
    cd.polygon([(bx - 8 * s, by + 4 * s), (bx - 12 * s, by + 12 * s), (bx - 2 * s, by + 7 * s)], fill=BLANCO)
    cd.text((m + 62 * s, y0 + 16 * s), "Messages", font=fuente("SemiBold", int(15 * s)), fill=(60, 60, 70))
    cd.text((w - m - 14 * s, y0 + 16 * s), "now", font=fuente("Medium", int(14 * s)), fill=(120, 120, 130), anchor="ra")
    cd.text((m + 62 * s, y0 + 42 * s), "Ana", font=fuente("Bold", int(17 * s)), fill=(20, 20, 25))
    cd.text((m + 62 * s, y0 + 66 * s), "Dinner at 8? I'll bring dessert", font=fuente("Medium", int(16 * s)),
            fill=(40, 40, 50))
    im = Image.alpha_composite(im.convert("RGBA"), card).convert("RGB")
    return im, ImageDraw.Draw(im)


def videollamada(w, h, t, s):
    im = fondo(w, h, (90, 110, 130), (40, 50, 60))
    d = ImageDraw.Draw(im)
    cx, cy = w / 2, h * 0.42 + math.sin(t * 1.3) * 6 * s   # the other person, a soft silhouette that moves a little
    d.ellipse([cx - 80 * s, cy - 110 * s, cx + 80 * s, cy + 70 * s], fill=(200, 170, 150))
    d.ellipse([cx - 170 * s, cy + 90 * s, cx + 170 * s, cy + 420 * s], fill=(60, 90, 140))
    d.rounded_rectangle([w - 120 * s, 60 * s, w - 20 * s, 200 * s], radius=14 * s, fill=(30, 35, 45))
    d.ellipse([w - 92 * s, 95 * s, w - 48 * s, 145 * s], fill=(180, 190, 200))
    barra_estado(d, w, s)
    centrado(d, w, 60 * s, "Mom  ·  02:1" + str(int(4 + t) % 10), fuente("SemiBold", int(16 * s)), BLANCO)
    boton(d, w / 2, h - 110 * s, 34 * s, ROJO, "x", s)
    return im, d


def alarma(w, h, t, s):
    im = fondo(w, h, (60, 40, 20), (20, 15, 10))
    d = ImageDraw.Draw(im)
    barra_estado(d, w, s)
    pulse = 0.5 + 0.5 * math.sin(t * 6)
    centrado(d, w, 150 * s, "Alarm", fuente("SemiBold", int(22 * s)), (255, 200 + int(40 * pulse), 120))
    centrado(d, w, 190 * s, "7:00", fuente("Bold", int(100 * s)), BLANCO)
    d.rounded_rectangle([40 * s, h - 260 * s, w - 40 * s, h - 200 * s], radius=30 * s, fill=(255, 160, 40))
    centrado(d, w, h - 246 * s, "Snooze", fuente("Bold", int(22 * s)), (40, 25, 5))
    centrado(d, w, h - 150 * s, "Stop", fuente("SemiBold", int(20 * s)), BLANCO)
    return im, d


def notificacion(w, h, t, s):
    im, d = reposo(w, h, t, s)
    u = 1 - (1 - min(t / 0.35, 1.0)) ** 3
    y0 = (-140 + u * 410) * s
    m = 14 * s
    d.rounded_rectangle([m, y0, w - m, y0 + 120 * s], radius=22 * s, fill=(245, 245, 248))
    d.rounded_rectangle([m + 14 * s, y0 + 16 * s, m + 50 * s, y0 + 52 * s], radius=9 * s, fill=(255, 255, 255),
                        outline=(220, 220, 225))
    d.text((m + 32 * s, y0 + 22 * s), "17", font=fuente("Bold", int(18 * s)), fill=ROJO, anchor="mt")
    d.text((m + 62 * s, y0 + 16 * s), "Calendar", font=fuente("SemiBold", int(15 * s)), fill=(60, 60, 70))
    d.text((m + 62 * s, y0 + 42 * s), "Team meeting", font=fuente("Bold", int(17 * s)), fill=(20, 20, 25))
    d.text((m + 62 * s, y0 + 68 * s), "In 10 minutes · Room 2", font=fuente("Medium", int(16 * s)), fill=(40, 40, 50))
    return im, d


def cargando(w, h, t, s):
    im = Image.new("RGB", (w, h), (5, 6, 8))
    d = ImageDraw.Draw(im)
    pct = min(62 + t * 2, 100)
    cx, cy, r = w / 2, h * 0.42, 90 * s
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(40, 45, 50), width=int(12 * s))
    d.arc([cx - r, cy - r, cx + r, cy + r], -90, -90 + 3.6 * pct, fill=VERDE, width=int(12 * s))
    centrado(d, w, cy - 30 * s, f"{int(pct)}%", fuente("Bold", int(48 * s)), BLANCO)
    centrado(d, w, cy + r + 30 * s, "Charging", fuente("Medium", int(18 * s)), GRIS)
    return im, d


def termostato(w, h, t, s):
    im = fondo(w, h, (30, 30, 40), (60, 25, 15))
    d = ImageDraw.Draw(im)
    barra_estado(d, w, s)
    centrado(d, w, 70 * s, "Living room", fuente("SemiBold", int(22 * s)), BLANCO)
    centrado(d, w, 102 * s, "Heating", fuente("Medium", int(16 * s)), (255, 150, 90))
    temp = max(23.0 - t * 1.5, 20.0)   # turned down: 23 -> 20 over 2 s
    cx, cy, r = w / 2, h * 0.42, 130 * s
    d.arc([cx - r, cy - r, cx + r, cy + r], 135, 405, fill=(60, 60, 70), width=int(16 * s))
    frac = (temp - 15) / 15
    col = (int(80 + 175 * frac), int(140 - 40 * frac), int(255 - 200 * frac))
    d.arc([cx - r, cy - r, cx + r, cy + r], 135, 135 + 270 * frac, fill=col, width=int(16 * s))
    a = math.radians(135 + 270 * frac)
    d.ellipse([cx + r * math.cos(a) - 14 * s, cy + r * math.sin(a) - 14 * s, cx + r * math.cos(a) + 14 * s,
               cy + r * math.sin(a) + 14 * s], fill=BLANCO)
    centrado(d, w, cy - 50 * s, f"{temp:.0f}°", fuente("Bold", int(80 * s)), BLANCO)
    centrado(d, w, cy + r + 40 * s, "Set by Jumper", fuente("Medium", int(16 * s)), GRIS)
    return im, d


def arena(w, h, t, s, v1="100", v2="100", evento=""):
    im = Image.new("RGB", (w, h), (12, 10, 24))
    d = ImageDraw.Draw(im)
    for y in range(0, h, int(6 * s)):   # a faint grid: a game, not a utility
        d.line([(0, y), (w, y)], fill=(20, 18, 40))
    barra_estado(d, w, s)
    centrado(d, w, 52 * s, "JUMPER ARENA", fuente("Bold", int(26 * s)), (255, 220, 90))
    for i, (nom, v, col) in enumerate((("JUMPER", int(v1), CIAN), ("JUMPY", int(v2), (255, 95, 162)))):
        y = (110 + i * 70) * s
        d.text((24 * s, y), nom, font=fuente("Bold", int(16 * s)), fill=col)
        d.rounded_rectangle([24 * s, y + 24 * s, w - 24 * s, y + 44 * s], radius=10 * s, fill=(40, 36, 60))
        if v > 0:
            d.rounded_rectangle([24 * s, y + 24 * s, 24 * s + (w - 48 * s) * v / 100, y + 44 * s], radius=10 * s, fill=col)
        d.text((w - 24 * s, y), f"{v}", font=fuente("Bold", int(16 * s)), fill=BLANCO, anchor="ra")
    cx, cy = w / 2, h * 0.62   # the fire button, pulsing
    r = (70 + 6 * math.sin(t * 8)) * s
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(230, 60, 70))
    centrado(d, w, cy - 14 * s, "FIRE", fuente("Bold", int(26 * s)), BLANCO)
    if evento in ("hit1", "hit2") and t < 0.8:
        col = CIAN if evento == "hit2" else (255, 95, 162)
        centrado(d, w, h * 0.38, "HIT!", fuente("Bold", int((64 + 30 * max(0, 0.3 - t)) * s)), col)
    if evento == "draw":
        centrado(d, w, h * 0.36, "DRAW", fuente("Bold", int(72 * s)), (255, 220, 90))
    return im, d


DIBUJO = {e: globals()[e] for e in ESTADOS}


def imagen(estado: str, t: float, w: int = 390, h: int = 844) -> Image.Image:
    nombre, *args = estado.split()
    if nombre not in DIBUJO:
        raise ValueError(f"telefono {estado}: unknown state; there are {', '.join(ESTADOS)}")
    return DIBUJO[nombre](w, h, max(t, 0.0), w / 390, *args)[0]


if __name__ == "__main__":
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for e in ESTADOS:
        imagen(e, 1.2).save(out / f"{e}.png")
    print(f"{len(ESTADOS)} screens in {out}")
