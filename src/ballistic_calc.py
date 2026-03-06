from math import sqrt

# ====== Parameter Umum ======
G   = 9.807        # percepatan gravitasi (m/s^2)
RHO = 1.225        # kerapatan udara (kg/m^3) ~ laut/udara standar

def simulate_drop(
    groundspeed, height, airspeed,
    mass=0.3,
    dims=(0.08, 0.10, 0.03),   # (width, height, thickness) meter -> 8x10x3 cm
    Cd_front=1.07,             # Cd untuk muka depan 8x10 cm (aliran arah horizontal)
    Cd_top=1.00,               # Cd untuk muka atas/bawah 8x3 cm (aliran arah vertikal "fixed")
    orientation='fixed',       # 'fixed' | 'broadside' | 'flip'
    flip_time=0.15,            # detik, hanya dipakai jika orientation='flip'
    dt=0.01                    # time step (s)
):
    """
    Simulasi jatuh bebas 2D dgn drag anisotropik (beda Cd*A utk sumbu X & Y).
    - Horizontal (sumbu X): pakai muka 8x10 cm (A_front)
    - Vertikal (sumbu Y):   pakai atas/bawah 8x3 cm (A_top) atau transisi ke A_front (flip)
    Return: (jarak_x_saat_menyentuh_tanah, waktu_jatuh)
    """
    W, H, T = dims
    A_front = W * H         # 8x10 cm  -> 0.08 * 0.10 = 0.008 m^2
    A_top   = W * T         # 8x3  cm  -> 0.08 * 0.03 = 0.0024 m^2

    # Produk Cd*A untuk sumbu X (horizontal)
    CdA_x = Cd_front * A_front

    # Kondisi awal
    x, y = 0.0, float(height)   # y: ketinggian (m) diukur dari tanah, turun ke 0
    vx   = float(groundspeed)   # m/s, searah +x
    vy   = 0.0                  # m/s, positif = ke bawah
    t    = 0.0

    # Angin (positif jika angin tailwind, negatif headwind); dari data drone:
    # groundspeed = airspeed + wind  => wind = groundspeed - airspeed
    wind = float(groundspeed) - float(airspeed)

    # Simpan langkah sebelumnya untuk interpolasi waktu-tabrak tanah
    x_prev, y_prev, vx_prev, vy_prev, t_prev = x, y, vx, vy, t

    while y > 0.0:
        # Kecepatan relatif terhadap udara
        vrel_x = vx - wind
        vrel_y = vy

        # Pilih CdA pada sumbu vertikal sesuai model orientasi
        if orientation == 'fixed':
            CdA_y = Cd_top * A_top
        elif orientation == 'broadside':
            CdA_y = Cd_front * A_front
        elif orientation == 'flip':
            s = min(1.0, max(0.0, t / max(1e-6, flip_time)))   # 0→1
            CdA_y = (1.0 - s) * (Cd_top * A_top) + s * (Cd_front * A_front)
        else:
            CdA_y = Cd_top * A_top  # default aman

        # Drag komponen (pakai bentuk v*|v| supaya arah otomatis berlawanan dgn v)
        ax_drag = -0.5 * RHO * CdA_x * vrel_x * abs(vrel_x) / mass
        ay_drag = -0.5 * RHO * CdA_y * vrel_y * abs(vrel_y) / mass

        # Total percepatan
        ax = ax_drag
        ay = G + ay_drag   # +G karena vy positif ke bawah

        # Integrasi semi-implisit (trapezoidal utk posisi)
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        x_new  = x + 0.5 * (vx + vx_new) * dt
        y_new  = y - 0.5 * (vy + vy_new) * dt  # kurangi karena vy positif ke bawah
        t_new  = t + dt

        # Simpan prev dan update
        x_prev, y_prev, vx_prev, vy_prev, t_prev = x, y, vx, vy, t
        x, y, vx, vy, t = x_new, y_new, vx_new, vy_new, t_new

    # Interpolasi linear mendarat tepat di y=0
    # fraksi dari langkah terakhir saat y melintasi 0
    if y_prev > 0:
        frac = y_prev / (y_prev - y) if (y_prev - y) != 0 else 1.0
        x_hit = x_prev + frac * (x - x_prev)
        t_hit = t_prev + frac * (t - t_prev)
    else:
        x_hit, t_hit = x, t

    return x_hit, t_hit


def terminal_velocity(mass, CdA_y):
    """Kecepatan jatuh terminal (positif = ke bawah) utk CdA_y tertentu."""
    return sqrt((2.0 * mass * G) / (RHO * CdA_y))

def calc_horizontal_travel_dist(
    groundspeed, height, airspeed, drop_index=0,
    mass=0.3,
    dims=(0.08, 0.10, 0.03),
    Cd_front=1.07,
    Cd_top=1.00,
    orientation='fixed',   # 'fixed' | 'broadside' | 'flip'
    flip_time=0.15,
    dt=0.01
):

    x_hit, _ = simulate_drop(
        groundspeed, height, airspeed,
        mass=mass, dims=dims,
        Cd_front=Cd_front, Cd_top=Cd_top,
        orientation=orientation, flip_time=flip_time, dt=dt
    )
    return x_hit
