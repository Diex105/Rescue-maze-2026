import sensor, image, time, math, ml
from pyb import UART
from ulab import numpy as np

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=22000)
clock = time.clock()

uart = UART(1, 115200, timeout_char=1000)

senal_silence_until = 0
SENAL_SILENCE_MS = 4000

ultima_letra_enviada = None
letras_cooldown = {}
COOLDOWN_LETRA_MS = 12000

net = ml.Model("trained.tflite", load_to_fb=True)
try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception:
    labels = ["omega", "phi", "psi"]

min_confidence = 0.86
threshold_list = [(math.ceil(min_confidence * 255), 255)]
CONFIRM_FRAMES_LETRA = 1
contador_modelo = 0
letra_candidata = None

# === VARIABLES PARA TARGETS ===
CONFIRM_FRAMES_TARGET  = 6
MAX_MISS_FRAMES_TARGET = 2
contador_target        = 0
miss_frames_target     = 0
target_candidato_kits  = None

targets_cooldown   = {}
COOLDOWN_TARGET_MS = 12000
# ==============================

IPM_X_ROTATION = 0
IPM_Y_ROTATION = 0.0
IPM_Z_ROTATION = 0.0
IPM_ZOOM       = 1
IMG_W, IMG_H   = 240, 240
ROI_MARGIN     = 20
ROI            = (ROI_MARGIN, ROI_MARGIN, IMG_W - 2*ROI_MARGIN, IMG_H - 2*ROI_MARGIN)

FC_X_STRIDE      = 2
FC_Y_STRIDE      = 2
FC_THRESHOLD     = 3000
FC_MIN_MAGNITUDE = 2
FC_X_MARGIN      = 10
FC_Y_MARGIN      = 10
FC_R_MARGIN      = 10
R_MIN            = 5
R_MAX            = 140

SAMPLES_PER_RING = 12
RING_OFFSETS     = [0.08, 0.25, 0.45, 0.68, 0.88]

COLOR_THRESHOLDS = {
    "Negro"    : (15, 34, -11, 60, -128, 127),
    "Rojo"     : (0, 100, 21, 127, -128, 127),
    "Amarillo" : (0, 100,   2,  20,  28, 127),
    "Verde"    : (0, 100, -55, -30, 33, 61),
    "Azul"     : (0, 100, -28,  -5, -85, -11),
}

COLOR_VALUES = {
    "Negro": -2, "Rojo": -1, "Amarillo": 0, "Verde": 1, "Azul": 2, "?": None,
}

# ==========================================
# 2. FUNCIONES DE ESTADO / UART
# ==========================================

def senal_is_silent():
    return time.ticks_diff(senal_silence_until, time.ticks_ms()) > 0

def reset_deteccion():
    global contador_modelo, letra_candidata
    contador_modelo = 0
    letra_candidata = None

def reset_deteccion_target():
    global contador_target, target_candidato_kits, miss_frames_target
    contador_target       = 0
    target_candidato_kits = None
    miss_frames_target    = 0

def letra_a_kits(letra):
    letra = letra.lower().strip()
    if letra == "omega": return 0
    if letra == "psi":   return 1
    if letra == "phi":   return 2
    return None

def enviar_kits_uart(cantidad):
    try:
        cantidad = int(cantidad)
        if cantidad not in [0, 1, 2]:
            print("UART NO ENVIADO -> cantidad invalida:", cantidad)
            return None
        uart.write("{}\n".format(cantidad).encode('utf-8'))
        print("KITS:", cantidad)
        return cantidad
    except ValueError:
        print("UART NO ENVIADO -> Valor no numerico")
        return None

def letra_en_cooldown(letra):
    letra = letra.lower().strip()
    now = time.ticks_ms()
    if letra in letras_cooldown:
        return time.ticks_diff(letras_cooldown[letra], now) > 0
    return False

def target_en_cooldown(kits):
    now = time.ticks_ms()
    if kits in targets_cooldown:
        return time.ticks_diff(targets_cooldown[kits], now) > 0
    return False

def enviar_senal_letra(dato):
    global senal_silence_until, ultima_letra_enviada
    letra = dato.replace("L:", "").lower().strip()

    if letra_en_cooldown(letra):
        print("LETRA EN COOLDOWN, IGNORADA:", letra)
        return

    kits = letra_a_kits(letra)
    if kits is None:
        print("LETRA DESCONOCIDA, IGNORADA:", letra)
        return

    enviar_kits_uart(kits)
    ultima_letra_enviada = letra
    letras_cooldown[letra] = time.ticks_add(time.ticks_ms(), COOLDOWN_LETRA_MS)
    senal_silence_until    = time.ticks_add(time.ticks_ms(), SENAL_SILENCE_MS)
    reset_deteccion()

# ==========================================
# 3. FUNCIONES DE PROCESAMIENTO VISUAL
# ==========================================

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]
    x_scale  = inputs[0].roi[2] / ow
    y_scale  = inputs[0].roi[3] / oh
    scale    = min(x_scale, y_scale)
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]
    out = [[] for _ in range(oc)]
    for i in range(oc):
        channel_data = np.array(outputs[0][0, :, :, i], dtype=np.float)
        img_channel  = image.Image(channel_data * 255.0)
        blobs = img_channel.find_blobs(
            threshold_list, x_stride=1, y_stride=1,
            area_threshold=1, pixels_threshold=1
        )
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = img_channel.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            out[i].append((x, y, w, h, score))
    return out

def classify_color(lab):
    if lab is None: return "?"
    L, A, B    = lab
    best_color = "?"
    best_dist  = float('inf')
    for color_name, (L_lo, L_hi, A_lo, A_hi, B_lo, B_hi) in COLOR_THRESHOLDS.items():
        L_mid = (L_lo + L_hi) / 2.0
        A_mid = (A_lo + A_hi) / 2.0
        B_mid = (B_lo + B_hi) / 2.0
        dist  = math.sqrt(((L - L_mid)*0.5)**2 + (A - A_mid)**2 + (B - B_mid)**2)
        if dist < best_dist:
            best_dist  = dist
            best_color = color_name
    if best_dist > 60: return "?"
    return best_color

def analyze_target(img, cx, cy, r):
    colors = []
    values = []
    for multiplier in RING_OFFSETS:
        r_mid  = r * multiplier
        angles = [2 * math.pi * j / SAMPLES_PER_RING for j in range(SAMPLES_PER_RING)]
        labs   = []
        for angle in angles:
            px = int(cx + r_mid * math.cos(angle))
            py = int(cy + r_mid * math.sin(angle))
            if 0 <= px < img.width() and 0 <= py < img.height():
                pixel = img.get_pixel(px, py)
                if pixel is not None:
                    if not (pixel[0] == 0 and pixel[1] == 0 and pixel[2] == 0 and IPM_ZOOM <= 1.0):
                        labs.append(image.rgb_to_lab(pixel))
        if not labs:
            color = "?"
        else:
            labs.sort(key=lambda p: p[0])
            color = classify_color(labs[len(labs) // 2])
        colors.append(color)
        values.append(COLOR_VALUES.get(color, None))

    valid         = all(v is not None for v in values)
    colores_vivos = ["Rojo", "Amarillo", "Verde", "Azul"]
    conteo_vivos  = sum(1 for c in colors if c in colores_vivos)
    conteo_blanco = colors.count("?")
    conteo_negro  = colors.count("Negro")

    if conteo_vivos == 0:  valid = False
    if conteo_blanco >= 3: valid = False
    if conteo_negro  >= 4: valid = False

    total = sum(v for v in values if v is not None) if valid else None
    return colors, values, total, valid

def draw_overlay(img, cx, cy, r, colors, total):
    DRAW_COLORS = {
        "Negro"    : (  0,   0,   0),
        "Rojo"     : (255,   0,   0),
        "Amarillo" : (255, 255,   0),
        "Verde"    : (  0, 200,   0),
        "Azul"     : (  0,   0, 255),
        "?"        : (128, 128, 128),
    }
    img.draw_circle(cx, cy, r + 3, color=(255, 255, 255), thickness=2)
    for i, multiplier in enumerate(RING_OFFSETS):
        r_mid           = int(r * multiplier)
        color_detectado = colors[i]
        rgb = DRAW_COLORS.get(color_detectado, (128, 128, 128))
        r5  = (rgb[0] >> 3) & 0x1F
        g6  = (rgb[1] >> 2) & 0x3F
        b5  = (rgb[2] >> 3) & 0x1F
        col565 = (r5 << 11) | (g6 << 5) | b5
        img.draw_circle(cx, cy, r_mid, color=col565, thickness=2)
        for j in range(SAMPLES_PER_RING):
            angle = 2 * math.pi * j / SAMPLES_PER_RING
            px = int(cx + r_mid * math.cos(angle))
            py = int(cy + r_mid * math.sin(angle))
            if 0 <= px < img.width() and 0 <= py < img.height():
                img.draw_circle(px, py, 2, color=(255, 255, 255), thickness=1)
    label = "S=" + str(total) if total is not None else "S=?"
    img.draw_string(cx - 12, cy - 6, label, color=(255, 255, 255), scale=1)

# ==========================================
# 4. LOOP PRINCIPAL
# ==========================================

while True:
    clock.tick()
    img = sensor.snapshot()
    img.rotation_corr(
        x_rotation = IPM_X_ROTATION,
        y_rotation = IPM_Y_ROTATION,
        z_rotation = IPM_Z_ROTATION,
        zoom       = IPM_ZOOM
    )

    target_detectado = False
    kits_calculados  = 0

    # --- 4.1 Busqueda de Targets (Circulos) ---

    # Copia preprocesada solo para Hough (no afecta colores)
    img_det = img.copy()
    img_det.histeq()    # normaliza contraste entre targets con distinta iluminacion
    img_det.gaussian(1) # suaviza ruido antes de Hough

    circles = img_det.find_circles(
        roi       = ROI,
        x_stride  = FC_X_STRIDE,
        y_stride  = FC_Y_STRIDE,
        threshold = FC_THRESHOLD,
        x_margin  = FC_X_MARGIN,
        y_margin  = FC_Y_MARGIN,
        r_margin  = FC_R_MARGIN,
        r_min     = R_MIN,
        r_max     = R_MAX
    )
    img.draw_rectangle(ROI, color=(0, 60, 0), thickness=1)

    # Filtra por magnitude minima — descarta circulos debiles/ruidosos
    if circles:
        circles = [c for c in circles if c.magnitude() >= FC_MIN_MAGNITUDE]

    if circles:
        best = max(circles, key=lambda c: c.magnitude())
        cx = best.x()
        cy = best.y()
        r  = best.r()

        # Muestra magnitude para calibrar FC_MIN_MAGNITUDE
        img.draw_string(4, 210, "MAG: %.0f" % best.magnitude(), color=(0, 200, 255), scale=1)

        if (R_MIN <= r <= R_MAX) and not (cx - r < 0 or cx + r >= IMG_W or cy - r < 0 or cy + r >= IMG_H):
            # Analiza colores sobre imagen ORIGINAL sin preprocesar
            colors, values, total, valid = analyze_target(img, cx, cy, r)
            if valid and total is not None:
                if target_en_cooldown(total):
                    img.draw_string(cx - 20, cy, "COOLDOWN S=" + str(total), color=(255, 0, 0), scale=1)
                else:
                    draw_overlay(img, cx, cy, r, colors, total)
                    target_detectado = True
                    kits_calculados  = total
        else:
            img.draw_string(4, 4, "Target fuera limites", color=(200, 100, 0), scale=1)

    # --- 4.2 Confirmacion de frames para TARGETS ---
    if target_detectado:
        miss_frames_target = 0

        img.draw_string(5, 5, "TARGET ACTIVO: S=" + str(kits_calculados), color=(0, 255, 0), scale=2)

        if kits_calculados == target_candidato_kits:
            contador_target += 1
        else:
            target_candidato_kits = kits_calculados
            contador_target       = 1

        img.draw_string(5, 25, "CNT TARGET: %d/%d" % (contador_target, CONFIRM_FRAMES_TARGET), color=(255, 255, 0), scale=1)

        if contador_target >= CONFIRM_FRAMES_TARGET:
            if not senal_is_silent():
                enviar_kits_uart(kits_calculados)
                targets_cooldown[kits_calculados] = time.ticks_add(time.ticks_ms(), COOLDOWN_TARGET_MS)
                senal_silence_until = time.ticks_add(time.ticks_ms(), SENAL_SILENCE_MS)
                reset_deteccion_target()

        continue

    else:
        miss_frames_target += 1
        img.draw_string(5, 5, "MISS: %d/%d" % (miss_frames_target, MAX_MISS_FRAMES_TARGET), color=(200, 100, 0), scale=1)

        if miss_frames_target >= MAX_MISS_FRAMES_TARGET:
            reset_deteccion_target()

    # --- 4.3 Control de Silencio Global ---
    if senal_is_silent():
        img.draw_string(5, 5, "SENAL SILENT")
        img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
        continue

    # --- 4.4 Busqueda de Letras (Modelo TFLite) ---
    detectado   = False
    mejor_score = 0
    mejor_letra = None

    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0:
            continue
        if len(detection_list) == 0:
            continue
        for x, y, w, h, score in detection_list:
            if score >= min_confidence:
                detectado = True
                if score > mejor_score:
                    mejor_score = score
                    mejor_letra = labels[i]

    if not detectado:
        reset_deteccion()
        img.draw_string(5, 5, "NO LETTER")
        img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
        continue

    if mejor_letra == letra_candidata:
        contador_modelo += 1
    else:
        letra_candidata = mejor_letra
        contador_modelo = 1

    img.draw_string(5, 5,  "CAND: %s" % str(letra_candidata))
    img.draw_string(5, 20, "CNT: %d"  % contador_modelo)

    if contador_modelo >= CONFIRM_FRAMES_LETRA:
        if senal_is_silent():
            img.draw_string(5, 40, "SILENCIO SENAL", color=(0, 255, 0))
        else:
            enviar_senal_letra("L:%s" % letra_candidata)

    img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
