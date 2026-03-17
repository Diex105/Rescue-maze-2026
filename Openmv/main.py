# ============================================================
# SISTEMA INTELIGENTE: DECISION + DIANA + MODELO FOMO
# ============================================================

import sensor, image, time, math, ml, uos, gc
from pyb import UART
from ulab import numpy as np

# =============================
# CONFIG CAMARA
# =============================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240,240))
sensor.skip_frames(time=8000)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

clock = time.clock()
uart = UART(3,230400)

# =============================
# ESTADOSa
# =============================

DECISION = 0
DIANA = 1
MODELO = 2

estado = DECISION

contador_diana = 0
contador_modelo = 0

CONFIRM_FRAMES = 1

# =============================
# VENTANA DE DECISION (3 SEG)
# =============================

VENTANA_MS = 0
inicio_ventana = time.ticks_ms()

buffer_datos = []

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
# DETECTOR RAPIDO COLOR
# =============================

def hay_color_diana(img):

    blobs = img.find_blobs(
        thresholds_diana,
        pixels_threshold=200,
        area_threshold=200,
        merge=True
    )

    if blobs:
        return True
    return False


# =============================
# PUNTOS DIANA
# =============================

puntos_color = {
"negro":-2,
"rojo":-1,
"amarillo":0,
"verde":1,
"azul":2,
"desconocido":0
}

def detectar_color_lab(stats):

    l = stats.l_mean()
    a = stats.a_mean()
    b = stats.b_mean()

    if l < 25: return "negro"
    if a > 5 and b > 10: return "rojo"
    if b > 35 and a < 15: return "amarillo"
    if a < -20: return "verde"
    if b < -10: return "azul"

    return "desconocido"


# =============================
# MODELO FOMO
# =============================

net = ml.Model("trained.tflite", load_to_fb=True)
labels = [line.rstrip('\n') for line in open("labels.txt")]

colors = [
(255,0,0),
(0,255,0),
(255,255,0),
(0,0,255),
(255,0,255),
(0,255,255),
(255,255,255)
]

min_confidence = 0.75
threshold_list = [(math.ceil(min_confidence * 255), 255)]

# ============================================================
# FOMO POST PROCESS
# ============================================================

def fomo_post_process(model, inputs, outputs):

    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh

    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]

    l = [[] for i in range(oc)]

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

            l[i].append((x, y, w, h, score))

    return l


# ============================================================
# LOOP PRINCIPAL
# ============================================================

print("Sistema Inteligente Vision")

while True:

    clock.tick()
    img = sensor.snapshot()

# ============================================================
# MODO DECISION
# ============================================================

    if estado == DECISION:

        if hay_color_diana(img):

            estado = DIANA
            contador_diana = 0

        else:

            estado = MODELO
            contador_modelo = 0


# ============================================================
# MODO DIANA
# ============================================================

    elif estado == DIANA:

        circles = img.find_circles(
            threshold=2500,
            r_min=40,
            r_max=120,
            r_step=3
        )

        if not circles:
            estado = DECISION
            continue

        c = max(circles, key=lambda x: x.r())

        cx = c.x()
        cy = c.y()
        r_total = c.r()

        img.draw_circle(cx,cy,r_total,color=(255,0,0))

        radios = [
        int(r_total*0.2),
        int(r_total*0.4),
        int(r_total*0.6),
        int(r_total*0.8),
        int(r_total)
        ]

        puntuacion_total = 0

        for i in range(5):

            if i == 0:
                r_interno = 0
            else:
                r_interno = radios[i-1]

            r_externo = radios[i]
            r_medio = int((r_interno+r_externo)/2)

            colores = []

            for ang in range(0,360,22):

                rad = math.radians(ang)

                px = int(cx + math.cos(rad)*r_medio)
                py = int(cy + math.sin(rad)*r_medio)

                if 5 < px < img.width()-5 and 5 < py < img.height()-5:

                    roi = (px-3,py-3,6,6)
                    stats = img.get_statistics(roi=roi)

                    color = detectar_color_lab(stats)
                    colores.append(color)

            if colores:

                color_final = max(set(colores),key=colores.count)
                puntos = puntos_color[color_final]
                puntuacion_total += puntos

        contador_diana += 1

        if contador_diana >= CONFIRM_FRAMES:

            buffer_datos.append("%d" % puntuacion_total)

            estado = DECISION


# ============================================================
# MODO MODELO
# ============================================================

    elif estado == MODELO:

        detectado = False
        mejor_score = 0
        letra_detectada = None

        for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):

            if i == 0:
                continue

            if len(detection_list) == 0:
                continue

            for x, y, w, h, score in detection_list:

                center_x = math.floor(x + (w / 2))
                center_y = math.floor(y + (h / 2))

                img.draw_circle((center_x, center_y, 12), color=colors[i])

                if score >= min_confidence:

                    detectado = True

                    if score > mejor_score:

                        mejor_score = score
                        letra_detectada = labels[i]

        if detectado:

            contador_modelo += 1

        else:

            estado = DECISION
            contador_modelo = 0

        if contador_modelo >= CONFIRM_FRAMES:

            if letra_detectada:

                buffer_datos.append("L:%s" % letra_detectada)

            estado = DECISION


# ============================================================
# DECISION FINAL CADA 3 SEGUNDOS
# ============================================================

    if time.ticks_diff(time.ticks_ms(), inicio_ventana) > VENTANA_MS:

        if buffer_datos:

            conteo = {}

            for d in buffer_datos:

                if d not in conteo:
                    conteo[d] = 0

                conteo[d] += 1

            mejor = max(conteo, key=conteo.get)

            print("DECISION FINAL ->", mejor)


            if mejor.startswith("L:f"):
                print("Letra F detectada, no se envia al UART")
            else:
                uart.write(mejor + "\n")

        buffer_datos = []

        inicio_ventana = time.ticks_ms()

    img.draw_string(5,225,"FPS: %.1f"%clock.fps())
