# Waveshare I2C Protocol (ESP32-S3 -> ESP32-C3)

Este proyecto, en perfil waveshare_7, envia comandos I2C al ESP32-C3 auxiliar
para controlar RGB y sonido.

## Bus I2C
- SDA: GPIO8
- SCL: GPIO9
- Frecuencia: 400000 Hz
- Direccion C3: 0x2A (configurable en src/hw/hw_profile.h)

## Formato de comandos
- Byte 0: CMD
- Bytes siguientes: payload segun comando

### CMD 0xA1 - RGB ON/OFF
Payload:
- Byte 1: r_on (0 o 1)
- Byte 2: g_on (0 o 1)
- Byte 3: b_on (0 o 1)

Ejemplo: [0xA1, 1, 0, 1] -> rojo ON, verde OFF, azul ON

### CMD 0xB1 - Beep
Payload:
- Byte 1: freq_hz LSB
- Byte 2: freq_hz MSB
- Byte 3: dur_ms LSB
- Byte 4: dur_ms MSB
- Byte 5: duty (0..255)

Ejemplo: [0xB1, 0xDC, 0x05, 0x64, 0x00, 0x80] -> 1500 Hz, 100 ms, duty 128

### CMD 0xB2 - Stop beep
Sin payload.

## Nota de integracion
Si usas otra direccion I2C en el C3, cambiar HW_C3_I2C_ADDR en src/hw/hw_profile.h.
