"""
Conversor de hora/fecha a ángulos solares — Toledo, España
Ecuaciones consistentes con graficar_ciclo_solar()
"""

import numpy as np
from datetime import datetime, date, timezone, timedelta


# ---------------------------------------------------------------------------
# Constantes geográficas de Toledo
# ---------------------------------------------------------------------------
LAT_DEG = 39.866       # latitud  (°N)
LON_DEG = -4.028       # longitud (°E, negativo = oeste)
LAT = np.radians(LAT_DEG)


# ---------------------------------------------------------------------------
# Horario de verano (CEST/CET) — reglas europeas
# Idéntico al widget: último domingo de marzo y de octubre
# ---------------------------------------------------------------------------

def _ultimo_domingo(anyo: int, mes: int) -> date:
    """Último domingo del mes dado (mes en 1-12)."""
    # Último día del mes
    if mes == 12:
        ultimo = date(anyo + 1, 1, 1) - timedelta(days=1)
    else:
        ultimo = date(anyo, mes + 1, 1) - timedelta(days=1)
    # Retroceder hasta el domingo (weekday 6)
    dias_atras = (ultimo.weekday() + 1) % 7   # lunes=0 … domingo=6
    return ultimo - timedelta(days=dias_atras)


def utc_offset_spain(dt_local: datetime) -> int:
    """
    Devuelve el offset UTC aplicado en España (CET=1, CEST=2)
    a partir de la fecha/hora local introducida.

    Cambio de hora:
      - Último domingo de marzo  a las 02:00 local → 03:00 (entra CEST)
      - Último domingo de octubre a las 03:00 local → 02:00 (entra CET)
    """
    anyo = dt_local.year
    inicio_verano = datetime.combine(
        _ultimo_domingo(anyo, 3), datetime.min.time()
    ).replace(hour=2)                          # 02:00 hora local
    fin_verano = datetime.combine(
        _ultimo_domingo(anyo, 10), datetime.min.time()
    ).replace(hour=3)                          # 03:00 hora local

    es_verano = inicio_verano <= dt_local < fin_verano
    return 2 if es_verano else 1


# ---------------------------------------------------------------------------
# Fórmulas astronómicas (mismas que en graficar_ciclo_solar)
# ---------------------------------------------------------------------------

def dia_del_anyo(d: date) -> int:
    return d.timetuple().tm_yday


def declinacion(doy: int) -> float:
    """Declinación solar en grados. Misma fórmula que el script original."""
    return 23.44 * np.sin(np.radians((360 / 365) * (doy - 81)))


def ecuacion_del_tiempo(doy: int) -> float:
    """
    Ecuación del tiempo en minutos.
    (No estaba en el script original, pero es necesaria para la corrección hora↔solar.)
    """
    B = 2 * np.pi / 365 * (doy - 81)
    return 9.87 * np.sin(2 * B) - 7.53 * np.cos(B) - 1.5 * np.sin(B)


def angulo_horario(hora_local: float, utc_offset: int, eqt: float) -> float:
    """
    Ángulo horario en grados.
    hora_local : horas decimales en hora oficial española
    Retorna 0 al mediodía solar, negativo por la mañana, positivo por la tarde.
    """
    # Hora UTC → hora solar verdadera
    hora_utc = hora_local - utc_offset
    hora_solar = hora_utc + LON_DEG / 15

    # Corrección por ecuación del tiempo y diferencia con meridiano de referencia
    meridiano_ref = -utc_offset * 15              # meridiano del huso (15°E → -15°*offset)
    corr_min = eqt + (LON_DEG - meridiano_ref) * 4
    hora_solar_corr = hora_solar + corr_min / 60

    return (hora_solar_corr - 12) * 15


def angulo_cenital(ha_deg: float, dec_deg: float) -> float:
    """
    Ángulo cenital en grados.
    Misma fórmula que el script original (arccos del producto escalar).
    """
    ha  = np.radians(ha_deg)
    dec = np.radians(dec_deg)
    cos_z = (np.sin(dec) * np.sin(LAT) +
             np.cos(dec) * np.cos(LAT) * np.cos(ha))
    return float(np.degrees(np.arccos(np.clip(cos_z, -1, 1))))


def angulo_acimutal(ha_deg: float, zenital_deg: float, dec_deg: float) -> float:
    """
    Ángulo acimutal en grados, con la misma convención del script original:
      0° = Sur, −90° = Este, +90° = Oeste
      (o equivalentemente: Norte=±180°, Este=−90°, Oeste=+90°, Sur=0°)
    Devuelve NaN si el sol está bajo el horizonte.
    """
    if zenital_deg >= 90:
        return float("nan")

    zen = np.radians(zenital_deg)
    dec = np.radians(dec_deg)

    cos_a = np.clip(
        (np.sin(dec) - np.cos(zen) * np.sin(LAT)) /
        (np.sin(zen) * np.cos(LAT)),
        -1, 1
    )
    acim = float(np.degrees(np.arccos(cos_a)))

    # Corrección tarde/mañana (igual que el script original)
    if ha_deg > 0:
        acim = 360 - acim
    acim -= 180
    return acim


# ---------------------------------------------------------------------------
# Amanecer / atardecer / mediodía solar
# ---------------------------------------------------------------------------

def eventos_solares(doy: int, dec_deg: float, utc_offset: int, eqt: float):
    """
    Devuelve (amanecer, mediodia_solar, atardecer) en minutos desde medianoche
    en hora oficial española. Devuelve None si el sol no sale/no se pone.
    """
    dec = np.radians(dec_deg)
    cos_ha_sr = -np.tan(LAT) * np.tan(dec)

    if cos_ha_sr > 1:
        return None, None, None   # sol permanece bajo el horizonte
    if cos_ha_sr < -1:
        return None, None, None   # sol permanece sobre el horizonte (ártico)

    ha_sr_deg = np.degrees(np.arccos(cos_ha_sr))

    meridiano_ref = -utc_offset * 15
    corr_min = eqt + (LON_DEG - meridiano_ref) * 4

    noon_min   = 12 * 60 - corr_min + utc_offset * 60
    sr_min     = noon_min - ha_sr_deg * 4
    ss_min     = noon_min + ha_sr_deg * 4

    return sr_min, noon_min, ss_min


def _fmt_min(minutos: float) -> str:
    if minutos is None or not np.isfinite(minutos):
        return "—"
    m = round(minutos)
    h = (m // 60) % 24
    mm = m % 60
    return f"{h:02d}:{mm:02d}"


# ---------------------------------------------------------------------------
# Función principal
# ---------------------------------------------------------------------------

def convertir(dt: datetime) -> dict:
    """
    Recibe un datetime en hora oficial española (sin info de zona horaria)
    y devuelve un diccionario con todos los ángulos y datos derivados.
    """
    d = dt.date()
    hora_local = dt.hour + dt.minute / 60 + dt.second / 3600

    offset  = utc_offset_spain(dt)
    doy     = dia_del_anyo(d)
    dec     = declinacion(doy)
    eqt     = ecuacion_del_tiempo(doy)
    ha      = angulo_horario(hora_local, offset, eqt)
    zen     = angulo_cenital(ha, dec)
    elev    = 90 - zen
    acim    = angulo_acimutal(ha, zen, dec)

    sr_min, noon_min, ss_min = eventos_solares(doy, dec, offset, eqt)
    daylen = (ss_min - sr_min) if (sr_min is not None) else None

    return {
        "fecha_hora"        : dt.strftime("%Y-%m-%d %H:%M"),
        "huso"              : f"UTC+{offset} ({'CEST' if offset == 2 else 'CET'})",
        "dia_del_anyo"      : doy,
        "declinacion_deg"   : round(dec, 4),
        "ecuacion_tiempo_min": round(eqt, 2),
        "angulo_horario_deg": round(ha, 4),
        "angulo_cenital_deg": round(zen, 4),
        "elevacion_deg"     : round(elev, 4),
        "angulo_acimutal_deg": round(acim, 4) if not np.isnan(acim) else None,
        "sol_visible"       : elev > 0,
        "amanecer"          : _fmt_min(sr_min),
        "mediodia_solar"    : _fmt_min(noon_min),
        "atardecer"         : _fmt_min(ss_min),
        "horas_de_luz"      : (f"{int(daylen//60)}h {round(daylen%60)}min"
                               if daylen is not None else "—"),
    }


def imprimir(resultado: dict) -> None:
    sep = "─" * 46
    print(f"\n{'ÁNGULOS SOLARES — TOLEDO, ESPAÑA':^46}")
    print(sep)
    print(f"  Fecha/hora  : {resultado['fecha_hora']}")
    print(f"  Huso horario: {resultado['huso']}")
    print(sep)
    print(f"  Día del año       : {resultado['dia_del_anyo']}")
    print(f"  Declinación solar : {resultado['declinacion_deg']:>8.2f}°")
    print(f"  Ecuación tiempo   : {resultado['ecuacion_tiempo_min']:>+8.1f} min")
    print(f"  Ángulo horario    : {resultado['angulo_horario_deg']:>8.2f}°")
    print(sep)
    if resultado["sol_visible"]:
        print(f"  ☀  Sol sobre el horizonte")
        print(f"  Ángulo cenital    : {resultado['angulo_cenital_deg']:>8.2f}°")
        print(f"  Elevación solar   : {resultado['elevacion_deg']:>8.2f}°")
        acim = resultado["angulo_acimutal_deg"]
        print(f"  Ángulo acimutal   : {acim:>8.2f}°" if acim is not None else "  Ángulo acimutal   :      —")
    else:
        print(f"  ☾  Sol bajo el horizonte")
        print(f"  Ángulo cenital    : {resultado['angulo_cenital_deg']:>8.2f}°")
        print(f"  Elevación solar   : {resultado['elevacion_deg']:>8.2f}°")
    print(sep)
    print(f"  Amanecer      : {resultado['amanecer']}")
    print(f"  Mediodía solar: {resultado['mediodia_solar']}")
    print(f"  Atardecer     : {resultado['atardecer']}")
    print(f"  Horas de luz  : {resultado['horas_de_luz']}")
    print(sep)


# ---------------------------------------------------------------------------
# Hora local del PC → ángulos solares ahora mismo
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    ahora = datetime.now()
    resultado = convertir(ahora)
    imprimir(resultado)