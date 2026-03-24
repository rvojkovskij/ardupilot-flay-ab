#!/usr/bin/env python3
"""
Завдання: Зліт + навігація A→Б у режимі STABILIZE через RC Override
Точка А: 50.450739, 30.461242
Точка Б: 50.443326, 30.448078
Висота: 100 м
Для реального польоту — з надійною посадкою
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

CONNECTION_STRING = 'tcp:127.0.0.1:5762'

POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 100.0

RC_NEUTRAL    = 1500
RC_MIN        = 1000
RC_MAX        = 2000
THROTTLE_HOLD = 1470  # зменшуй якщо тримає вище 100м (спробуй 1460, 1450)

ARRIVAL_RADIUS = 2.0

# ─── Допоміжні функції ────────────────────────────────────────────────────────

def get_distance_m(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def get_bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def heading_error(current, target):
    err = target - current
    if err > 180:  err -= 360
    if err < -180: err += 360
    return err

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def send_rc(vehicle, ch1=RC_NEUTRAL, ch2=RC_NEUTRAL,
            ch3=RC_NEUTRAL, ch4=RC_NEUTRAL):
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

def arm_and_takeoff(vehicle, target_altitude):
    print("Очікуємо pre-arm checks...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Встановлюємо GUIDED, arm...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("  Очікуємо arm...")
        time.sleep(1)

    print(f"Злітаємо до {target_altitude} м...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Висота: {alt:.1f} м")
        if alt >= target_altitude - 1.0:
            print(f"✅ Досягнуто {alt:.1f} м!")
            break
        time.sleep(0.3)

def safe_land(vehicle, point_b):
    """
    Надійна посадка для реального дрона.
    Двоетапна: спочатку GUIDED до 10м над точкою Б, потім LAND.
    """
    print("\nПочинаємо процедуру посадки...")
    vehicle.channels.overrides = {}
    time.sleep(0.3)

    # ── Крок 1: знижуємось до 10м в GUIDED точно над точкою Б ───────────────
    print("Знижуємось до 10м у GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(0.5)

    hover_point = LocationGlobalRelative(point_b[0], point_b[1], 10.0)
    vehicle.simple_goto(hover_point)

    while True:
        alt  = vehicle.location.global_relative_frame.alt
        dist = get_distance_m(
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            point_b[0], point_b[1]
        )
        print(f"  До точки Б: {dist:.1f}м | Висота: {alt:.1f}м")
        if dist < 1.5 and abs(alt - 10.0) < 2.0:
            print("✅ Зависаємо над точкою Б на 10м")
            break
        time.sleep(0.5)

    # ── Крок 2: стабілізація 2 секунди ───────────────────────────────────────
    print("Стабілізація...")
    time.sleep(2.0)

    # ── Крок 3: фінальна посадка через LAND ──────────────────────────────────
    print("Фінальна посадка (LAND)...")
    vehicle.mode = VehicleMode("LAND")

    last_vz     = []   # історія вертикальної швидкості
    touch_count = 0
    start_time  = time.time()

    while True:
        alt   = vehicle.location.global_relative_frame.alt
        vz    = vehicle.velocity[2]  # м/с, + = вниз
        armed = vehicle.armed

        last_vz.append(abs(vz))
        if len(last_vz) > 8:
            last_vz.pop(0)

        avg_vz = sum(last_vz) / len(last_vz)

        print(f"  Висота: {alt:+.2f}м | Vz: {vz:+.3f}м/с | "
              f"Avg Vz: {avg_vz:.3f}м/с | Armed: {armed}")

        # ── Торкання визначаємо по швидкості, НЕ по висоті ───────────────────
        # Коли дрон торкнувся землі — вертикальна швидкість різко падає до ~0
        # Не залежить від дрейфу баровисотомира
        touching = (avg_vz < 0.08 and len(last_vz) >= 8)

        if touching:
            touch_count += 1
            print(f"  Торкання підтверджується ({touch_count}/6)...")
        else:
            touch_count = 0

        if touch_count >= 6:
            print("✅ Торкання землі підтверджено по швидкості!")
            break

        # Запасний варіант 1 — ArduPilot сам розармував
        if not armed:
            print("✅ Автоматичний disarm — посадка завершена!")
            break

        # Запасний варіант 2 — якщо щось пішло не так, чекаємо максимум 60с
        if time.time() - start_time > 60:
            print("⚠️  Таймаут посадки 60с — зупиняємо примусово!")
            break

        time.sleep(0.2)

    # ── Фінальна точність ─────────────────────────────────────────────────────
    time.sleep(1.0)
    final      = vehicle.location.global_frame
    final_dist = get_distance_m(final.lat, final.lon, point_b[0], point_b[1])
    final_alt  = vehicle.location.global_relative_frame.alt

    print(f"\n{'='*50}")
    print(f"🎯 Точність посадки: {final_dist:.2f} м від точки Б")
    print(f"📍 Фінальна позиція: {final.lat:.6f}, {final.lon:.6f}")
    print(f"📏 Висота після посадки: {final_alt:+.2f} м (дрейф баро)")
    print(f"{'='*50}")

    return final_dist

# ─── Основна програма ─────────────────────────────────────────────────────────

def main():
    print(f"Підключаємось до {CONNECTION_STRING}...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)

    # ── Перевірка стартової позиції ───────────────────────────────────────────
    loc = vehicle.location.global_frame
    dist_from_a = get_distance_m(loc.lat, loc.lon, POINT_A[0], POINT_A[1])
    print(f"Коптер зараз:   {loc.lat:.6f}, {loc.lon:.6f}")
    print(f"Відстань від А: {dist_from_a:.1f} м")

    if dist_from_a > 100:
        print("⚠️  Коптер далеко від точки А! Перезапусти SITL.")
        vehicle.close()
        return

    # ── Зліт ──────────────────────────────────────────────────────────────────
    arm_and_takeoff(vehicle, TARGET_ALT)
    time.sleep(1)

    # ── STABILIZE ─────────────────────────────────────────────────────────────
    print("Перемикаємо на STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)

    print("Починаємо навігацію до точки Б...\n")

    YAW_KP  = 5.0
    YAW_MAX = 100
    ALT_KP  = 3.0
    ALT_MAX = 150
    LOOP_DT = 0.05

    prev_time = time.time()
    prev_dist = get_distance_m(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        POINT_B[0], POINT_B[1]
    )

    while True:
        loc      = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt
        curr_hdg = vehicle.heading

        dist    = get_distance_m(curr_lat, curr_lon, POINT_B[0], POINT_B[1])
        bearing = get_bearing(curr_lat, curr_lon, POINT_B[0], POINT_B[1])
        yaw_err = heading_error(curr_hdg, bearing)
        speed   = get_speed(vehicle)

        now           = time.time()
        dt            = now - prev_time
        closing_speed = (prev_dist - dist) / dt if dt > 0 else 0
        prev_time     = now
        prev_dist     = dist

        print(f"Dist: {dist:7.1f}м | Alt: {curr_alt:6.1f}м | "
              f"Hdg: {curr_hdg:3.0f}° | Bear: {bearing:3.0f}° | "
              f"Speed: {speed:4.1f}м/с | Closing: {closing_speed:+.1f}м/с")

        # ── Прибуття ──────────────────────────────────────────────────────────
        if dist < ARRIVAL_RADIUS:
            print(f"\n✅ Прибули! Відстань до точки Б: {dist:.2f} м")
            break

        # ── Yaw ───────────────────────────────────────────────────────────────
        yaw_out = RC_NEUTRAL + clamp(yaw_err * YAW_KP, -YAW_MAX, YAW_MAX)

        # ── Pitch ─────────────────────────────────────────────────────────────
        if abs(yaw_err) < 15:
            if dist > 200:
                pitch_offset = 250
            elif dist > 50:
                pitch_offset = 180
            elif dist > 20:
                pitch_offset = 100
            elif dist > 5:
                pitch_offset = 50
            else:
                pitch_offset = 25
            pitch_out = RC_NEUTRAL - pitch_offset
        else:
            pitch_out = RC_NEUTRAL

        # ── Throttle ──────────────────────────────────────────────────────────
        alt_err      = TARGET_ALT - curr_alt
        throttle_out = THROTTLE_HOLD + clamp(alt_err * ALT_KP, -ALT_MAX, ALT_MAX)

        send_rc(vehicle,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=yaw_out)

        time.sleep(LOOP_DT)

    # ── Посадка ───────────────────────────────────────────────────────────────
    safe_land(vehicle, POINT_B)
    vehicle.close()

if __name__ == '__main__':
    main()
