import sensor, image, time, math, ml
from pyb import Pin, DAC
from ulab import numpy as np

# ============================
# CONFIG CAMARA
# ============================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=9000)

sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

clock = time.clock()

# ============================
# 2 PINES
# ============================
dac = DAC("P6", bits=12)
pin_aviso = Pin("P4", Pin.OUT_PP)
pin_aviso.low()

senal_silence_until = 0
SENAL_SILENCE_MS = 3500

# ============================
# NUEVO: CONTROL DE DUPLICADOS
# ============================
ultima_letra_enviada = None
letras_cooldown = {}
COOLDOWN_LETRA_MS = 8000  # 12 segundos

# ============================
# MODELO FOMO
# ============================
net = ml.Model("trained.tflite", load_to_fb=True)
labels = [line.rstrip('\n') for line in open("labels.txt")]

min_confidence = 0.85
threshold_list = [(math.ceil(min_confidence * 255), 255)]

# ============================
# CONFIRMACION
# ============================
CONFIRM_FRAMES_LETRA = 1
contador_modelo = 0
letra_candidata = None

# ============================
# FUNCIONES AUXILIARES
# ============================
def senal_is_silent():
    return time.ticks_diff(senal_silence_until, time.ticks_ms()) > 0

def reset_deteccion():
    global contador_modelo, letra_candidata
    contador_modelo = 0
    letra_candidata = None

def poner_voltaje_letra(letra):
    if letra == "omega":
        dac.write(993)
        return 0.80
    elif letra == "psi":
        dac.write(2048)
        return 1.65
    elif letra == "phi":
        dac.write(3475)
        return 2.80
    return None

def letra_en_cooldown(letra):
    now = time.ticks_ms()
    if letra in letras_cooldown:
        return time.ticks_diff(letras_cooldown[letra], now) > 0
    return False

def enviar_senal(dato):
    global senal_silence_until, ultima_letra_enviada

    if dato == "L:f":
        return

    letra = dato.replace("L:", "")

    # 🚫 BLOQUEAR POR COOLDOWN
    if letra_en_cooldown(letra):
        print("LETRA EN COOLDOWN, IGNORADA:", letra)
        return

    voltaje = poner_voltaje_letra(letra)
    if voltaje is None:
        return

    pin_aviso.high()
    time.sleep_ms(100)
    pin_aviso.low()

    print("SENAL ->", dato, " V=", voltaje)

    # guardar última enviada y establecer cooldown
    ultima_letra_enviada = letra
    letras_cooldown[letra] = time.ticks_add(time.ticks_ms(), COOLDOWN_LETRA_MS)

    senal_silence_until = time.ticks_add(time.ticks_ms(), SENAL_SILENCE_MS)
    reset_deteccion()

# ============================
# FOMO POST PROCESS
# ============================
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

# ============================
# LOOP PRINCIPAL
# ============================
print("Sistema Vision - Solo letras por 2 pines")

while True:
    clock.tick()
    img = sensor.snapshot()

    if senal_is_silent():
        img.draw_string(5, 5, "SENAL SILENT")
        img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
        continue

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
        img.draw_string(5, 5, "NO LETTER")
        img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
        continue

    if mejor_letra == letra_candidata:
        contador_modelo += 1
    else:
        letra_candidata = mejor_letra
        contador_modelo = 1

    img.draw_string(5, 5, "CAND: %s" % str(letra_candidata))
    img.draw_string(5, 20, "CNT: %d" % contador_modelo)

    if contador_modelo >= CONFIRM_FRAMES_LETRA:
        if senal_is_silent():
            img.draw_string(5, 40, "SILENCIO SENAL", color=(0, 255, 0))
        else:
            enviar_senal("L:%s" % letra_candidata)

    img.draw_string(5, 225, "FPS: %.1f" % clock.fps())
