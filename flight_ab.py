#!/usr/bin/env python3
"""
Скрипт керування дроном через RC Override у режимі STABILIZE.
Маршрут: Точка А (50.450739, 30.461242) → Точка Б (50.443326, 30.448078)
Висота польоту: 100 метрів
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

CONNECTION_STRING = 'tcp:127.0.0.1:5762'

POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT  = 200.0
EARTH_RADIUS = 6371000

RC_NEUTRAL    = 1500
RC_MIN        = 1000
RC_MAX        = 2000
THROTTLE_HOLD = 1470

# ── ВИПРАВЛЕННЯ 1: зменшили з 8.0 до 3.0м ───────────────────────────────────
# Менший радіус = точніше прибуття = точніша посадка
ARRIVAL_RADIUS = 5.5 # 3.0

# ═══════════════════════════════════════════════════════════════════
# ДОПОМІЖНІ ФУНКЦІЇ
# ═══════════════════════════════════════════════════════════════════

def get_distance_m(lat1, lon1, lat2, lon2):
    """Відстань між GPS точками у метрах (Haversine)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return EARTH_RADIUS * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def get_bearing(lat1, lon1, lat2, lon2):
    """Азимут від точки 1 до точки 2 (0=Північ, 90=Схід)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam)*math.cos(phi2)
    y = (math.cos(phi1)*math.sin(phi2) -
         math.sin(phi1)*math.cos(phi2)*math.cos(dlam))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def heading_error(current, target):
    """
    Різниця курсів (-180..+180).
    + = треба вправо, - = треба вліво.
    """
    err = target - current
    if err >  180: err -= 360
    if err < -180: err += 360
    return err


def clamp(val, lo, hi):
    """Обмежує значення у діапазоні [lo, hi]."""
    return max(lo, min(hi, val))


def send_rc(vehicle, ch1=RC_NEUTRAL, ch2=RC_NEUTRAL,
            ch3=RC_NEUTRAL, ch4=RC_NEUTRAL):
    """
    Відправляє RC Override.
    CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw.
    Відправляти кожні ~50мс інакше автопілот скине override.
    """
    vehicle.channels.overrides = {
        '1': clamp(int(ch1), RC_MIN, RC_MAX),
        '2': clamp(int(ch2), RC_MIN, RC_MAX),
        '3': clamp(int(ch3), RC_MIN, RC_MAX),
        '4': clamp(int(ch4), RC_MIN, RC_MAX),
    }


def get_speed(vehicle):
    """Горизонтальна швидкість у м/с."""
    vx = vehicle.velocity[0]
    vy = vehicle.velocity[1]
    return math.sqrt(vx**2 + vy**2)


# ═══════════════════════════════════════════════════════════════════
# ВИМІРЮВАННЯ HOVER THROTTLE
# ═══════════════════════════════════════════════════════════════════

def measure_hover_throttle(vehicle):
    """
    Читає hover throttle з параметрів ArduPilot.
    MOT_THST_HOVER: 0.0..1.0 → конвертуємо в мкс.
    Додано захист від занижених значень.
    """
    for param_name in ['MOT_THST_HOVER', 'THR_MID']:
        val = vehicle.parameters.get(param_name, None)
        if val is not None:
            if param_name == 'THR_MID':
                # THR_MID вже в діапазоні 0..1000
                hover_us = int(1000 + val)
            else:
                # MOT_THST_HOVER: 0.0..1.0
                hover_us = int(1000 + val * 1000)

            print(f"  {param_name}: {val:.3f} → {hover_us} мкс")

            # ── ВИПРАВЛЕННЯ 2: захист від занижених значень ───────────────────
            # MOT_THST_HOVER часто занижений бо ArduPilot вчиться під навантаженням.
            # Мінімум 1430 мкс — нижче цього дрон точно буде падати.
            if hover_us < 1430:
                corrected = max(hover_us + 80, 1450)
                print(f"  ⚠️  Значення занижене! Корегуємо: {hover_us} → {corrected} мкс")
                return corrected

            if hover_us > 1650:
                print(f"  ⚠️  Значення завищене! Використовуємо 1500 мкс")
                return 1500

            return hover_us

    print("  ⚠️  Параметр не знайдено, використовуємо 1470 мкс")
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
            print(f"✅ Висота {alt:.1f}м досягнута!")
            break
        time.sleep(0.3)


# ═══════════════════════════════════════════════════════════════════
# НАВІГАЦІЯ А → Б
# ═══════════════════════════════════════════════════════════════════

def navigate_to_target(vehicle, target_lat, target_lon,
                       target_alt, arrival_radius=3.0):
    """
    Навігація в STABILIZE через RC Override.

    Використовує позиційне керування через Roll + Pitch:
    - Розкладаємо вектор до цілі на осі дрона (вперед/вбік)
    - Pitch = рух вперед/назад
    - Roll  = рух вбік (компенсація бокового зносу від вітру)
    - Yaw   = повертаємо ніс до цілі
    - Throttle = утримання висоти 100м

    Швидкість автоматично зменшується при наближенні до цілі.
    """

    # Yaw: мкс на градус помилки курсу
    YAW_KP  = 2.5
    YAW_KI  = 0.08   # інтегральна складова для компенсації вітру
    YAW_MAX = 80

    # Throttle: корекція висоти
    ALT_KP  = 3.0
    ALT_MAX = 150

    LOOP_DT = 0.05  # 20 Гц

    yaw_integral = 0.0

    prev_time = time.time()
    prev_dist = get_distance_m(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        target_lat, target_lon
    )

    print(f"Початкова відстань: {prev_dist:.1f}м")
    print(f"Bearing А→Б: "
          f"{get_bearing(POINT_A[0], POINT_A[1], target_lat, target_lon):.1f}°")

    while True:
        loc      = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt
        curr_hdg = vehicle.heading

        dist    = get_distance_m(curr_lat, curr_lon, target_lat, target_lon)
        bearing = get_bearing(curr_lat, curr_lon, target_lat, target_lon)
        yaw_err = heading_error(curr_hdg, bearing)
        speed   = get_speed(vehicle)

        now           = time.time()
        dt            = now - prev_time
        closing_speed = (prev_dist - dist) / dt if dt > 0 else 0
        prev_time     = now
        prev_dist     = dist

        # ── Перевірка прибуття ────────────────────────────────────────────────
        if dist < arrival_radius:
            print(f"\n✅ Прибули! Відстань: {dist:.2f}м")
            # Нейтраль — гасимо швидкість
            send_rc(vehicle, ch3=THROTTLE_HOLD)
            time.sleep(0.5)
            break

        # ── Yaw PI: повертаємо ніс до цілі ───────────────────────────────────
        if abs(yaw_err) < 30:
            yaw_integral += yaw_err * LOOP_DT
            yaw_integral  = clamp(yaw_integral, -20.0, 20.0)
        else:
            yaw_integral = 0.0

        yaw_out = RC_NEUTRAL + clamp(
            yaw_err * YAW_KP + yaw_integral * YAW_KI,
            -YAW_MAX, YAW_MAX
        )

        # ── Позиційне керування Roll + Pitch ──────────────────────────────────
        # Зміщення до цілі в метрах (по осях північ/схід)
        dlat_m = (target_lat - curr_lat) * 111320
        dlon_m = (target_lon - curr_lon) * 111320 * math.cos(
            math.radians(curr_lat)
        )

        # Проекція на осі дрона з урахуванням поточного курсу
        hdg_rad   = math.radians(curr_hdg)
        fwd_err   =  dlat_m * math.cos(hdg_rad) + dlon_m * math.sin(hdg_rad)
        right_err = -dlat_m * math.sin(hdg_rad) + dlon_m * math.cos(hdg_rad)

        # ── ВИПРАВЛЕННЯ 3: динамічний pos_kp залежно від швидкості ───────────
        # Якщо летимо швидко і близько — зменшуємо pos_kp щоб не пролетіти.
        # speed_factor: при 8м/с і dist<50м зменшує kp в 2 рази.
        # Динамічний pos_kp: далеко — помірно, близько — агресивно
        # speed_factor зменшує kp при великій швидкості (захист від перельоту)
        speed_factor = clamp(1.0 - (speed - 3.0) * 0.05, 0.4, 1.0)

        if dist > 50:
            pos_kp = 10.0 * speed_factor
        elif dist > 20:
            pos_kp = 16.0 * speed_factor
        elif dist > 10:
            pos_kp = 22.0 * speed_factor
        elif dist > 5:
            pos_kp = 35.0   # збільшили — вітер не зупинить
        else:
            pos_kp = 50.0   # максимум на фіналі

        # Обмеження pitch при великій швидкості і малій відстані
        POS_MAX = 280
        if speed > 6.0 and dist < 100:
            POS_MAX = int(280 * clamp(dist / 100.0, 0.3, 1.0))

        pitch_out = RC_NEUTRAL - clamp(fwd_err   * pos_kp, -POS_MAX, POS_MAX)
        roll_out  = RC_NEUTRAL + clamp(right_err * pos_kp, -POS_MAX, POS_MAX)

        # ── Throttle: утримання висоти ────────────────────────────────────────
        alt_err      = target_alt - curr_alt
        throttle_out = THROTTLE_HOLD + clamp(alt_err * ALT_KP,
                                             -ALT_MAX, ALT_MAX)

        print(f"Dist:{dist:6.1f}м | Alt:{curr_alt:5.1f}м | "
              f"Hdg:{curr_hdg:3.0f}° | Bear:{bearing:3.0f}° | "
              f"YawErr:{yaw_err:+5.1f}° | Fwd:{fwd_err:+6.1f}м | "
              f"Right:{right_err:+6.1f}м | "
              f"Spd:{speed:4.1f}м/с | Cls:{closing_speed:+4.1f}")

        send_rc(vehicle,
                ch1=roll_out,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=yaw_out)

        time.sleep(LOOP_DT)


# ═══════════════════════════════════════════════════════════════════
# ПОСАДКА
# ═══════════════════════════════════════════════════════════════════

def safe_land(vehicle, point_b):
    """
    Триетапна точна посадка в точку Б.

    Етап 1: Вимірюємо hover throttle.
    Етап 2: GUIDED — швидко спускаємось до 5м над точкою Б.
    Етап 3: STABILIZE + RC Override — контрольований спуск
            з Vz регулятором і позиційною корекцією.
    """
    print("\nПочинаємо процедуру посадки...")
    vehicle.channels.overrides = {}
    time.sleep(0.3)

    # ── Етап 1: вимірюємо hover throttle ─────────────────────────────────────
    print("Вимірюємо hover throttle...")
    hover_thr = measure_hover_throttle(vehicle)
    print(f"  Hover: {hover_thr} мкс")

    # ── Етап 2: GUIDED до 5м над точкою Б ────────────────────────────────────
    print("GUIDED: спускаємось до 5м над точкою Б...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(0.5)

    # ── ВИПРАВЛЕННЯ 5: airspeed=5 — швидше спускаємось ───────────────────────
    vehicle.simple_goto(
        LocationGlobalRelative(point_b[0], point_b[1], 5.0),
        airspeed=5
    )

    timeout = time.time() + 120  # максимум 2 хвилини на спуск
    while True:
        alt  = vehicle.location.global_relative_frame.alt
        dist = get_distance_m(
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            point_b[0], point_b[1]
        )
        print(f"  Dist:{dist:.1f}м | Alt:{alt:.1f}м")

        #if dist < 1.0 and abs(alt - 5.0) < 1.5:
        if dist < 0.3 and abs(alt - 5.0) < 1.5:
            print("✅ Над точкою Б на 5м")
            break

        # Таймаут — якщо не може дістатись за 2 хвилини
        if time.time() > timeout:
            print("⚠️  Таймаут GUIDED спуску — продовжуємо з поточної позиції")
            break

        time.sleep(0.5)

    print("Стабілізація 2с...")
    time.sleep(2.0)

    # ── Етап 3: STABILIZE — контрольований спуск ─────────────────────────────
    print("STABILIZE: фінальна посадка з RC Override...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(0.5)

    # Цільова швидкість спуску (м/с, + = вниз)
    # 1.0 м/с = плавний спуск (знизили з 1.5 для кращого контролю)
    TARGET_VZ = 1.0

    # P-регулятор вертикальної швидкості
    # Більше = швидша реакція на зміну Vz
    VZ_KP  = 80.0   # збільшили з 60 для кращого гальмування при падінні
    VZ_MAX = 200    # збільшили максимум корекції

    # Позиційна корекція (компенсація вітру під час спуску)
    POS_KP  = 20.0
    POS_MAX = 200

    last_vz     = []
    touch_count = 0
    start_time  = time.time()

    while True:
        loc      = vehicle.location.global_relative_frame
        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt
        curr_hdg = vehicle.heading
        vz       = vehicle.velocity[2]  # + = вниз

        # ── Throttle Vz регулятор ─────────────────────────────────────────────
        # Якщо висота <= 0.2м — торкнулись землі, вимикаємо мотори.
        # Це запобігає відскоку: після торкання Vz стає від'ємним (летить вгору)
        # і регулятор без цієї перевірки дав би максимальний газ THR:1669.
        if curr_alt <= 0.2:
            throttle_out = 1000  # мотори вимкнути — залишаємось на землі
        else:
            # vz_err > 0: падаємо швидше ніж треба → додаємо газ
            # vz_err < 0: падаємо повільніше       → зменшуємо газ
            vz_err       = vz - TARGET_VZ
            throttle_out = hover_thr - clamp(vz_err * VZ_KP, -VZ_MAX, VZ_MAX)
            # Мінімум 1200 — не даємо газу впасти занадто низько
            throttle_out = clamp(throttle_out, 1200, RC_MAX)

        # ── Позиційна корекція Roll/Pitch ─────────────────────────────────────
        dlat_m = (point_b[0] - curr_lat) * 111320
        dlon_m = (point_b[1] - curr_lon) * 111320 * math.cos(
            math.radians(curr_lat)
        )

        hdg_rad   = math.radians(curr_hdg)
        fwd_err   =  dlat_m * math.cos(hdg_rad) + dlon_m * math.sin(hdg_rad)
        right_err = -dlat_m * math.sin(hdg_rad) + dlon_m * math.cos(hdg_rad)

        pitch_out = RC_NEUTRAL - clamp(fwd_err   * POS_KP, -POS_MAX, POS_MAX)
        roll_out  = RC_NEUTRAL + clamp(right_err * POS_KP, -POS_MAX, POS_MAX)

        dist_to_b = get_distance_m(curr_lat, curr_lon,
                                   point_b[0], point_b[1])

        print(f"  Alt:{curr_alt:+.2f}м | Dist:{dist_to_b:.2f}м | "
              f"Vz:{vz:+.2f}м/с (ціль:{TARGET_VZ}) | "
              f"THR:{int(throttle_out)} | "
              f"Pitch:{int(pitch_out)} | Roll:{int(roll_out)}")

        send_rc(vehicle,
                ch1=roll_out,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=RC_NEUTRAL)

        # ── Визначення торкання землі ─────────────────────────────────────────
        last_vz.append(abs(vz))
        if len(last_vz) > 8:
            last_vz.pop(0)
        avg_vz = sum(last_vz) / len(last_vz)

        # Торкання = вертикальна швидкість впала до нуля + низька висота
        touching = (avg_vz < 0.12 and len(last_vz) >= 8 and curr_alt < 1.5)

        if touching:
            touch_count += 1
            print(f"  Торкання підтверджується ({touch_count}/6)...")
        else:
            touch_count = 0

        if touch_count >= 6:
            print("✅ Торкання землі підтверджено!")
            vehicle.channels.overrides = {'3': 1000}
            time.sleep(0.5)
            vehicle.channels.overrides = {}
            break

        if not vehicle.armed:
            print("✅ Авто disarm!")
            break

        if time.time() - start_time > 90:
            print("⚠️  Таймаут 90с!")
            break

        time.sleep(0.05)  # 20 Гц

    # ── Фінальний звіт ────────────────────────────────────────────────────────
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

def navigate_to_target_no_yaw(vehicle, target_lat, target_lon,
                             target_alt, arrival_radius=3.0):
    """
    Навігація БЕЗ зміни yaw (yaw фіксований).

    Використовує глобальні координати:
    - North → Pitch
    - East  → Roll
    """

    # ALT_KP  = 3.0
    ALT_KP = 2.5 if target_alt > 150 else 3.5
    ALT_MAX = 150

    LOOP_DT = 0.05

    fixed_yaw = vehicle.heading
    print(f"🔒 Фіксуємо Yaw: {fixed_yaw}°")

    while True:
        loc = vehicle.location.global_relative_frame

        curr_lat = loc.lat
        curr_lon = loc.lon
        curr_alt = loc.alt

        # ── Відстань ─────────────────────────────────────────
        dist = get_distance_m(curr_lat, curr_lon,
                              target_lat, target_lon)

        if dist < arrival_radius:
            print(f"\n✅ Прибули: {dist:.2f}м")
            send_rc(vehicle, ch3=THROTTLE_HOLD)
            time.sleep(0.5)
            break

        # ── Глобальна помилка (метри) ───────────────────────
        north_err = (target_lat - curr_lat) * 111320
        east_err  = (target_lon - curr_lon) * 111320 * math.cos(
            math.radians(curr_lat)
        )

        # ── Динамічний коефіцієнт ───────────────────────────
        if dist > 50:
            pos_kp = 8.0
        elif dist > 20:
            pos_kp = 14.0
        elif dist > 10:
            pos_kp = 20.0
        elif dist > 5:
            pos_kp = 30.0
        else:
            pos_kp = 45.0

        POS_MAX = 300

        # ❗ ГОЛОВНА ЗМІНА
        pitch_out = RC_NEUTRAL - clamp(north_err * pos_kp,
                                       -POS_MAX, POS_MAX)

        roll_out  = RC_NEUTRAL + clamp(east_err * pos_kp,
                                       -POS_MAX, POS_MAX)

        # ── Висота ──────────────────────────────────────────
        alt_err      = target_alt - curr_alt
        throttle_out = THROTTLE_HOLD + clamp(alt_err * ALT_KP,
                                             -ALT_MAX, ALT_MAX)

        # ── YAW ФІКСОВАНИЙ ─────────────────────────────────
        yaw_out = RC_NEUTRAL

        print(f"Dist:{dist:6.1f}м | Alt:{curr_alt:5.1f}м | "
              f"N:{north_err:+6.1f} | E:{east_err:+6.1f}")

        send_rc(vehicle,
                ch1=roll_out,
                ch2=pitch_out,
                ch3=throttle_out,
                ch4=yaw_out)

        time.sleep(LOOP_DT)


# ═══════════════════════════════════════════════════════════════════
# ГОЛОВНА ПРОГРАМА
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"Підключаємось до {CONNECTION_STRING}...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)

    # Перевірка стартової позиції
    loc         = vehicle.location.global_frame
    dist_from_a = get_distance_m(loc.lat, loc.lon,
                                 POINT_A[0], POINT_A[1])
    print(f"Коптер зараз:   {loc.lat:.6f}, {loc.lon:.6f}")
    print(f"Відстань від А: {dist_from_a:.1f}м")

    if dist_from_a > 100:
        print("⚠️  Далеко від точки А! Перезапусти SITL.")
        vehicle.close()
        return

    # Зліт
    arm_and_takeoff(vehicle, TARGET_ALT)
    time.sleep(1)

    # STABILIZE для RC Override навігації
    print("Перемикаємо на STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)

    print("Починаємо навігацію до точки Б...\n")

    # Навігація
    # navigate_to_target(vehicle,
    #                    POINT_B[0], POINT_B[1],
    #                    TARGET_ALT,
    #                    ARRIVAL_RADIUS)

    navigate_to_target_no_yaw(vehicle, POINT_B[0], POINT_B[1], TARGET_ALT)
    # navigate_ultra_precise(vehicle, POINT_B[0], POINT_B[1], TARGET_ALT)

    # Посадка
    safe_land(vehicle, POINT_B)
    vehicle.close()


if __name__ == '__main__':
    main()