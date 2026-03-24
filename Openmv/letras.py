# ============================================================
# OPENMV - UART RAPIDO Y DETECCION SELECTIVA
# Envia rapido, sin cooldowns por tipo de dato.
# Despues de enviar, entra en silencio temporal para no
# redetectar durante la accion del robot.
# ============================================================

import sensor, image, time, math, ml
from pyb import UART
from ulab import numpy as np

# =============================
# CONFIG CAMARA
# =============================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time= 4000)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

clock = time.clock()

# Sube baudrate si ambos lados lo soportan bien
uart = UART(3, 115200, timeout_char=2)

# =============================
# SILENCIO DESPUES DE ENVIAR
# =============================

uart_silence_until = 0
UART_SILENCE_MS = 4500    # ajusta a lo que tarda tu teensy en ejecutar la accion

# =============================
# ESTADOS
# =============================

DECISION = 0
DIANA = 1
MODELO = 2

estado = DECISION

# =============================
# CONFIRMACIONES MINIMAS
# =============================

CONFIRM_FRAMES_LETRA = 1
CONFIRM_FRAMES_DIANA = 0

contador_modelo = 0
contador_diana = 0

letra_candidata = None
historial_diana = []

# =============================
# THRESHOLDS COLORES DIANA
# =============================

threshold_rojo = (20, 80, 30, 80, 10, 60)
threshold_amarillo = (30, 90, -10, 10, 40, 90)
threshold_verde = (20, 80, -80, -20, -10, 40)
threshold_azul = (20, 70, 0, 40, -80, -20)

thresholds_diana = [
    threshold_rojo,
    threshold_amarillo,
    threshold_verde,
    threshold_azul
]

# =============================
# PUNTOS DIANA
# =============================

puntos_color = {
    "negro": -2,
    "rojo": -1,
    "amarillo": 0,
    "verde": 1,
    "azul": 2,
    "desconocido": 0
}

# =============================
# MODELO FOMO
# =============================

net = ml.Model("trained.tflite", load_to_fb=True)
labels = [line.rstrip('\n') for line in open("labels.txt")]

min_confidence = 0.90
threshold_list = [(math.ceil(min_confidence * 255), 255)]

# =============================
# FUNCIONES AUXILIARES
# =============================

def uart_is_silent():
    return time.ticks_diff(uart_silence_until, time.ticks_ms()) > 0

def enviar_uart(dato):
    global uart_silence_until, estado
    if dato == "L:f":
        return

    uart.write(dato + "\n")
    print("UART ->", dato)

    # Pausa la deteccion completamente
    uart_silence_until = time.ticks_add(time.ticks_ms(), UART_SILENCE_MS)

    # limpiar estado interno para no arrastrar basura
    reset_deteccion()
    estado = DECISION

def reset_deteccion():
    global contador_modelo, contador_diana, letra_candidata, historial_diana
    contador_modelo = 0
    contador_diana = 0
    letra_candidata = None
    historial_diana = []

def hay_color_diana(img):
    blobs = img.find_blobs(
        thresholds_diana,
        pixels_threshold=180,
        area_threshold=180,
        merge=True
    )

    for b in blobs:
        if b.w() >= 14 and b.h() >= 14:
            return True

    return False

def detectar_color_lab(stats):
    l = stats.l_mean()
    a = stats.a_mean()
    b = stats.b_mean()

    if l < 25:
        return "negro"
    if a > 5 and b > 10:
        return "rojo"
    if b > 35 and a < 15:
        return "amarillo"
    if a < -20:
        return "verde"
    if b < -10:
        return "azul"

    return "desconocido"

def moda(lista):
    if not lista:
        return None
    return max(set(lista), key=lista.count)

def puntuar_diana(img, c):
    cx = c.x()
    cy = c.y()
    r_total = c.r()

    radios = [
        int(r_total * 0.2),
        int(r_total * 0.4),
        int(r_total * 0.6),
        int(r_total * 0.8),
        int(r_total)
    ]

    puntuacion_total = 0
    anillos_validos = 0

    for i in range(5):
        r_interno = 0 if i == 0 else radios[i - 1]
        r_externo = radios[i]
        r_medio = int((r_interno + r_externo) / 2)

        colores = []

        for ang in range(0, 360, 15):
            rad = math.radians(ang)
            px = int(cx + math.cos(rad) * r_medio)
            py = int(cy + math.sin(rad) * r_medio)

            if 5 < px < img.width() - 5 and 5 < py < img.height() - 5:
                roi = (px - 3, py - 3, 6, 6)
                stats = img.get_statistics(roi=roi)
                color = detectar_color_lab(stats)
                colores.append(color)

        if colores:
            color_final = moda(colores)
            puntuacion_total += puntos_color[color_final]
            anillos_validos += 1

    if anillos_validos < 4:
        return None

    return puntuacion_total

# =============================
# FOMO POST PROCESS
# =============================

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]

    out = [[] for _ in range(oc)]

    for i in range(oc):
        channel_data = np.array(outputs[0][0, :, :, i], dtype=np.float)
        img_channel = image.Image(channel_data * 255.0)

        blobs = img_channel.find_blobs(
            threshold_list,
            x_stride=1,
            y_stride=1,
            area_threshold=1,
            pixels_threshold=1
        )

        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect

            score = img_channel.get_statistics(
                thresholds=threshold_list,
                roi=rect
            ).l_mean() / 255.0

            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)

            out[i].append((x, y, w, h, score))

    return out

# =============================
# LOOP PRINCIPAL
# =============================

print("Sistema Vision UART rapido")

while True:
    clock.tick()
    img = sensor.snapshot()

    # Durante silencio: no detectar nada, solo dejar pasar tiempo
    if uart_is_silent():
        img.draw_string(5, 5, "UART SILENT")
        img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
        continue

    # --------------------------------------------------------
    # DECISION
    # --------------------------------------------------------
    if estado == DECISION:
        if hay_color_diana(img):
            estado = DIANA
            contador_diana = 0
            historial_diana = []
        else:
            estado = MODELO
            contador_modelo = 0
            letra_candidata = None

    # --------------------------------------------------------
    # DIANA
    # --------------------------------------------------------
    elif estado == DIANA:
        circles = img.find_circles(
            threshold=2200,
            r_min=30,
            r_max=130,
            r_step=2
        )

        if not circles:
            reset_deteccion()
            estado = DECISION
            continue

        c = max(circles, key=lambda x: x.r())
        puntuacion = puntuar_diana(img, c)

        if puntuacion is None:
            reset_deteccion()
            estado = DECISION
            continue

        historial_diana.append(puntuacion)
        contador_diana += 1

        if contador_diana >= CONFIRM_FRAMES_DIANA:
            puntuacion_final = moda(historial_diana)
            if puntuacion_final is not None:
                enviar_uart("%d" % puntuacion_final)
            else:
                reset_deteccion()
                estado = DECISION

    # --------------------------------------------------------
    # MODELO
    # --------------------------------------------------------
    elif estado == MODELO:
        detectado = False
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
            estado = DECISION
            continue

        if mejor_letra == letra_candidata:
            contador_modelo += 1
        else:
            letra_candidata = mejor_letra
            contador_modelo = 1

        if contador_modelo >= CONFIRM_FRAMES_LETRA:
            enviar_uart("L:%s" % letra_candidata)

    img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
