#!/usr/bin/env python3

from math import sqrt

# ─────────────────────────────────────────────
# PARAMETER FISIKA (KONSTANTA)
# ─────────────────────────────────────────────
G   = 9.807  # percepatan gravitasi (m/s^2)
RHO = 1.225  # kerapatan udara (kg/m^3)

# ─────────────────────────────────────────────
# PARAMETER PAYLOAD
# ─────────────────────────────────────────────
MASS        = 0.3
DIMS        = (0.08, 0.10, 0.03)  # width, height, thickness (m)
CD_FRONT    = 1.07
CD_TOP      = 1.00
FLIP_TIME   = 0.15
DT          = 0.01
ORIENTATION = 'fixed'  # 'fixed' | 'broadside' | 'flip'


# ═════════════════════════════════════════════
# FUNGSI: SIMULASI DROP
# ═════════════════════════════════════════════
def simulate_drop(
    groundspeed, height, airspeed,
    mass=MASS,
    dims=DIMS,
    Cd_front=CD_FRONT,
    Cd_top=CD_TOP,
    orientation=ORIENTATION,
    flip_time=FLIP_TIME,
    dt=DT
):
    """
    Simulasi jatuh bebas 2D dengan drag anisotropik.
    
    Parameter:
        groundspeed : kecepatan terhadap tanah (m/s)
        height      : ketinggian AGL (m)
        airspeed    : kecepatan terhadap udara (m/s)
    
    Return:
        x_hit : jarak horizontal saat mendarat (m)
        t_hit : waktu tempuh (s)
    """
    W, H, T = dims
    A_front = W * H   # luas muka depan 0.008 m^2
    A_top   = W * T   # luas muka atas  0.0024 m^2

    CdA_x = Cd_front * A_front  # drag horizontal

    # Kondisi awal
    x, y = 0.0, float(height)
    vx   = float(groundspeed)
    vy   = 0.0
    t    = 0.0

    # Kecepatan angin efektif
    wind = float(groundspeed) - float(airspeed)

    # Simpan langkah sebelumnya untuk interpolasi
    x_prev, y_prev, vx_prev, vy_prev, t_prev = x, y, vx, vy, t

    while y > 0.0:
        vrel_x = vx - wind
        vrel_y = vy

        # Pilih CdA sumbu Y sesuai orientasi
        if orientation == 'fixed':
            CdA_y = Cd_top * A_top
        elif orientation == 'broadside':
            CdA_y = Cd_front * A_front
        elif orientation == 'flip':
            s     = min(1.0, max(0.0, t / max(1e-6, flip_time)))
            CdA_y = (1.0 - s) * (Cd_top * A_top) + s * (Cd_front * A_front)
        else:
            CdA_y = Cd_top * A_top

        # Drag per sumbu
        ax_drag = -0.5 * RHO * CdA_x * vrel_x * abs(vrel_x) / mass
        ay_drag = -0.5 * RHO * CdA_y * vrel_y * abs(vrel_y) / mass

        # Total percepatan
        ax = ax_drag
        ay = G + ay_drag

        # Integrasi trapezoidal
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        x_new  = x + 0.5 * (vx + vx_new) * dt
        y_new  = y - 0.5 * (vy + vy_new) * dt
        t_new  = t + dt

        # Simpan prev dan update
        x_prev, y_prev, vx_prev, vy_prev, t_prev = x, y, vx, vy, t
        x, y, vx, vy, t = x_new, y_new, vx_new, vy_new, t_new

    # Interpolasi linear mendarat tepat di y=0
    if y_prev > 0:
        frac  = y_prev / (y_prev - y) if (y_prev - y) != 0 else 1.0
        x_hit = x_prev + frac * (x - x_prev)
        t_hit = t_prev + frac * (t - t_prev)
    else:
        x_hit, t_hit = x, t

    return x_hit, t_hit


def calc_horizontal_travel_dist(groundspeed, height, airspeed, orientation=ORIENTATION):
    """
    Wrapper sederhana untuk simulate_drop.
    Return: jarak horizontal saja (meter)
    """
    x_hit, _ = simulate_drop(groundspeed, height, airspeed, orientation=orientation)
    return x_hit