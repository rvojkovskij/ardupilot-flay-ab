#!/usr/bin/env python3
"""
Скрипт керування дроном через RC Override у режимі STABILIZE.
Маршрут: Точка А (50.450739, 30.461242) → Точка Б (50.443326, 30.448078)
Висота польоту: 200 метрів. Yaw фіксований весь політ!
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

CONNECTION_STRING = 'tcp:127.0.0.1:5762'

POINT_A      = (50.450739, 30.461242)
POINT_B      = (50.443326, 30.448078)
TARGET_ALT   = 200.0
EARTH_RADIUS = 6371000

RC_NEUTRAL    = 1500
RC_MIN        = 1000
RC_MAX        = 2000
THROTTLE_HOLD = 1470

ARRIVAL_RADIUS = 5.5

# ═══════════════════════════════════════════════════════════════════
# ДОПОМІЖНІ ФУНКЦІЇ
# ═══════════════════════════════════════════════════════════════════

def get_distance_m(lat1, lon1, lat2, lon2):
    """Відстань між GPS точками у метрах (Haversine)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a    = (math.sin(dphi/2)**2 +
            math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return EARTH_RADIUS * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def send_rc(vehicle, ch1=RC_NEUTRAL, ch2=RC_NEUTRAL,
            ch3=RC_NEUTRAL, ch4=RC_NEUTRAL):
    """
    RC Override. CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw.
    Відправляти кожні ~50мс інакше автопілот скине override.
    """
    vehicle.channels.overrides = {
        '1': clamp(int(ch1), RC_MIN, RC_MAX),
        '2': clamp(int(ch2), RC_MIN, RC_MAX),
        '3': clamp(int(ch3), RC_MIN, RC_MAX),
        '4': clamp(int(ch4), RC_MIN, RC_MAX),
    }


def get_speed(vehicle):
    vx = vehicle.velocity[0]
    vy = vehicle.velocity[1]
    return math.sqrt(vx**2 + vy**2)


def pos_correction(vehicle, point_b, pos_kp, pos_max):
    """
    Розраховує Roll і Pitch для утримання позиції над точкою Б.
    Проектує вектор зміщення на осі дрона з урахуванням поточного Hdg.
    """
    loc      = vehicle.location.global_relative_frame
    curr_lat = loc.lat
    curr_lon = loc.lon
    curr_hdg = vehicle.heading

    dlat_m    = (point_b[0] - curr_lat) * 111320
    dlon_m    = (point_b[1] - curr_lon) * 111320 * math.cos(
        math.radians(curr_lat))
    hdg_rad   = math.radians(curr_hdg)
    fwd_err   =  dlat_m * math.cos(hdg_rad) + dlon_m * math.sin(hdg_rad)
    right_err = -dlat_m * math.sin(hdg_rad) + dlon_m * math.cos(hdg_rad)

    pitch_out = RC_NEUTRAL - clamp(fwd_err   * pos_kp, -pos_max, pos_max)
    roll_out  = RC_NEUTRAL + clamp(right_err * pos_kp, -pos_max, pos_max)
    return pitch_out, roll_out


# ═══════════════════════════════════════════════════════════════════
# ВИМІРЮВАННЯ HOVER THROTTLE
# ═══════════════════════════════════════════════════════════════════

def measure_hover_throttle(vehicle):
    """
    Читає реальний hover throttle з параметрів ArduPilot.
    Додає корекцію якщо значення занижене.
    """
    for param_name in ['MOT_THST_HOVER', 'THR_MID']:
        val = vehicle.parameters.get(param_name, None)
        if val is not None:
            hover_us = int(1000 + val * 1000) if param_name == 'MOT_THST_HOVER' \
                       else int(1000 + val)
            print(f"  {param_name}: {val:.3f} → {hover_us} мкс")
            if hover_us < 1430:
                corrected = max(hover_us + 80, 1450)
                print(f"  ⚠️  Занижене! {hover_us} → {corrected} мкс")
                return corrected
            if hover_us > 1650:
                print(f"  ⚠️  Завищене! → 1500 мкс")
                return 1500
            return hover_us
    print("  ⚠️  Не знайдено, використовуємо 1470 мкс")
    return 1470


# ═══════════════════════════════════════════════════════════════════
# ЗЛІТ
# ═══════════════════════════════════════════════════════════════════

def arm_and_takeoff(vehicle, altitude):
    """Arm і зліт до заданої висоти в GUIDED."""
    print("Очікуємо pre-arm checks...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode  = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("  Очікуємо arm...")
        time.sleep(1)

    print(f"Злітаємо до {altitude}м...")
    vehicle.simple_takeoff(altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Висота: {alt:.1f}м")
        if alt >= altitude - 1.0:
            print(f"✅ Висота {alt:.1f}м!")
            break
        time.sleep(0.3)


# ═══════════════════════════════════════════════════════════════════
# НАВІГАЦІЯ А → Б (YAW ФІКСОВАНИЙ)
# ═══════════════════════════════════════════════════════════════════

def navigate_fixed_yaw(vehicle, target_lat, target_lon,
                       target_alt, arrival_radius=5.5):
    """
    Навігація з фіксованим Yaw весь політ.
    CH4 = RC_NEUTRAL = 1500 завжди — Yaw не крутиться.
    Roll + Pitch керують рухом в глобальних координатах.
    """
    ALT_KP  = 3.5
    ALT_MAX = 150
    LOOP_DT = 0.05

    fixed_yaw_deg = vehicle.heading
    print(f"🔒 Yaw зафіксовано: {fixed_yaw_deg}° (не зміниться весь політ)")

    prev_time = time.time()
    prev_dist = get_distance_m(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        target_lat, target_lon
    )
    print(f"Початкова відстань: {prev_dist:.1f}м")

    while True:
        loc      = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt
        curr_hdg = vehicle.heading
        speed    = get_speed(vehicle)

        dist = get_distance_m(curr_lat, curr_lon, target_lat, target_lon)

        now           = time.time()
        dt            = now - prev_time
        closing_speed = (prev_dist - dist) / dt if dt > 0 else 0
        prev_time     = now
        prev_dist     = dist

        if dist < arrival_radius:
            print(f"\n✅ Прибули! Відстань: {dist:.2f}м")
            send_rc(vehicle, ch3=THROTTLE_HOLD)
            time.sleep(0.5)
            break

        north_err = (target_lat - curr_lat) * 111320
        east_err  = (target_lon - curr_lon) * 111320 * math.cos(
            math.radians(curr_lat))

        speed_factor = clamp(1.0 - (speed - 3.0) * 0.05, 0.4, 1.0)

        if dist > 100:
            pos_kp = 8.0 * speed_factor
        elif dist > 50:
            pos_kp = 12.0 * speed_factor
        elif dist > 20:
            pos_kp = 18.0 * speed_factor
        elif dist > 10:
            pos_kp = 28.0 * speed_factor
        elif dist > 5:
            pos_kp = 40.0
        else:
            pos_kp = 55.0

        POS_MAX = 300
        if speed > 6.0 and dist < 150:
            POS_MAX = int(300 * clamp(dist / 150.0, 0.3, 1.0))

        hdg_rad   = math.radians(curr_hdg)
        fwd_err   =  north_err * math.cos(hdg_rad) + east_err * math.sin(hdg_rad)
        right_err = -north_err * math.sin(hdg_rad) + east_err * math.cos(hdg_rad)

        pitch_out = RC_NEUTRAL - clamp(fwd_err   * pos_kp, -POS_MAX, POS_MAX)
        roll_out  = RC_NEUTRAL + clamp(right_err * pos_kp, -POS_MAX, POS_MAX)

        alt_err      = target_alt - curr_alt
        throttle_out = THROTTLE_HOLD + clamp(alt_err * ALT_KP, -ALT_MAX, ALT_MAX)

        print(f"Dist:{dist:6.1f}м | Alt:{curr_alt:5.1f}м | "
              f"Hdg:{curr_hdg:3.0f}° | "
              f"N:{north_err:+6.1f}м | E:{east_err:+6.1f}м | "
              f"Spd:{speed:4.1f}м/с | Cls:{closing_speed:+4.1f}")

        send_rc(vehicle,
                ch1=roll_out,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=RC_NEUTRAL)  # Yaw = нейтраль завжди

        time.sleep(LOOP_DT)


# ═══════════════════════════════════════════════════════════════════
# ПОСАДКА (YAW ФІКСОВАНИЙ, ШВИДКА)
# ═══════════════════════════════════════════════════════════════════

def safe_land(vehicle, point_b):
    """
    Посадка з фіксованим Yaw.
    Максимальна швидкість спуску 3м/с — контрольована через throttle.
    """
    print("\nПочинаємо процедуру посадки...")
    vehicle.channels.overrides = {}
    time.sleep(0.3)

    print("Вимірюємо hover throttle...")
    hover_thr = measure_hover_throttle(vehicle)
    print(f"  Hover: {hover_thr} мкс")

    print("STABILIZE: спуск до землі (Yaw фіксований)...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(0.5)

    last_vz     = []
    touch_count = 0
    start_time  = time.time()

    while True:
        loc      = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt
        curr_hdg = vehicle.heading
        vz       = vehicle.velocity[2]  # + = вниз, - = вгору

        dist = get_distance_m(curr_lat, curr_lon,
                              point_b[0], point_b[1])

        # ── Висота над землею ─────────────────────────────────────────────────
        # Після спуску з 200м баро дрейфує ~-10м
        # Додаємо 10м компенсацію для оцінки реальної висоти
        h = curr_alt + 10.0

        # ── Цільова швидкість спуску ──────────────────────────────────────────
        # Обмежена 3м/с як вимагається
        if h > 100:
            target_vz = 3.0
        elif h > 50:
            target_vz = 3.0
        elif h > 20:
            target_vz = 2.5
        elif h > 10:
            target_vz = 1.5
        elif h > 3:
            target_vz = 1.0
        else:
            target_vz = 0.5

        # ── Throttle: ЖОРСТКИЙ обмежувач швидкості ────────────────────────────
        # Головне виправлення: якщо падаємо швидше ніж треба — різко гальмуємо
        # Не використовуємо простий PID бо він не встигає при 10+м/с
        if vz > target_vz + 2.0:
            # Критично швидко — максимальне гальмування
            throttle_out = hover_thr + 200
        elif vz > target_vz + 0.5:
            # Швидше ніж треба — гальмуємо пропорційно
            over = vz - target_vz
            throttle_out = hover_thr + int(over * 60)
        elif vz < target_vz - 0.5:
            # Повільніше ніж треба — трохи зменшуємо газ
            throttle_out = hover_thr - 50
        else:
            # Нормальна швидкість
            throttle_out = hover_thr - 30

        # Обмеження: не більше hover+250 щоб не злетіти
        throttle_out = clamp(throttle_out, 1150, hover_thr + 250)

        # ── Позиційна корекція ────────────────────────────────────────────────
        if h < 20:
            pos_kp  = 60.0
            pos_max = 280
        else:
            pos_kp  = 35.0
            pos_max = 250

        pitch_out, roll_out = pos_correction(vehicle, point_b,
                                             pos_kp, pos_max)

        print(f"  Alt:{curr_alt:+.1f}м | H:{h:+.1f}м | "
              f"Dist:{dist:.2f}м | Vz:{vz:+.2f}(ціль:{target_vz}) | "
              f"THR:{int(throttle_out)} | touch:{touch_count}/6")

        send_rc(vehicle,
                ch1=roll_out,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=RC_NEUTRAL)  # Yaw фіксований

        # ── Визначення торкання ───────────────────────────────────────────────
        last_vz.append(abs(vz))
        if len(last_vz) > 10:
            last_vz.pop(0)
        avg_vz = sum(last_vz) / len(last_vz)
        
        # Торкання = швидкість впала до нуля після того як спускались
        was_descending = len(last_vz) >= 10 and max(last_vz) < 0.85
        if avg_vz < 0.15 and was_descending:
            touch_count += 1
            # Вимикаємо газ при торканні
            send_rc(vehicle, ch3=1000, ch4=RC_NEUTRAL)
        else:
            touch_count = 0

        print(f"last_vz: {last_vz}, avg_vz: {avg_vz}, was_descending: {was_descending}")

        if touch_count >= 6:
            print("✅ Торкання землі підтверджено!")
            vehicle.channels.overrides = {'3': 1000}
            time.sleep(0.5)
            vehicle.channels.overrides = {}
            break

        if not vehicle.armed:
            print("✅ Авто disarm!")
            break

        if time.time() - start_time > 120:
            print("⚠️  Таймаут 120с!")
            break

        time.sleep(0.05)

    # Фінальний звіт
    time.sleep(1.0)
    final      = vehicle.location.global_frame
    final_dist = get_distance_m(final.lat, final.lon,
                                point_b[0], point_b[1])
    final_alt  = vehicle.location.global_relative_frame.alt

    print(f"\n{'='*50}")
    print(f"🎯 Точність посадки: {final_dist:.2f}м від точки Б")
    print(f"📍 Позиція: {final.lat:.6f}, {final.lon:.6f}")
    print(f"📏 Висота баро: {final_alt:+.2f}м (дрейф норм.)")
    print(f"{'='*50}")

    return final_dist


# ═══════════════════════════════════════════════════════════════════
# ГОЛОВНА ПРОГРАМА
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"Підключаємось до {CONNECTION_STRING}...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)

    loc         = vehicle.location.global_frame
    dist_from_a = get_distance_m(loc.lat, loc.lon, POINT_A[0], POINT_A[1])
    print(f"Коптер зараз:   {loc.lat:.6f}, {loc.lon:.6f}")
    print(f"Відстань від А: {dist_from_a:.1f}м")

    if dist_from_a > 100:
        print("⚠️  Далеко від точки А! Перезапусти SITL.")
        vehicle.close()
        return

    arm_and_takeoff(vehicle, TARGET_ALT)
    time.sleep(1)

    print("Перемикаємо на STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)

    fixed_yaw = vehicle.heading
    print(f"🔒 Yaw зафіксовано: {fixed_yaw}° (не зміниться весь політ)")
    print("Починаємо навігацію до точки Б...\n")

    navigate_fixed_yaw(vehicle, POINT_B[0], POINT_B[1],
                       TARGET_ALT, ARRIVAL_RADIUS)

    safe_land(vehicle, POINT_B)
    vehicle.close()


if __name__ == '__main__':
    main()