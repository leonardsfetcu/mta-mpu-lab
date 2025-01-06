import time
import board
import busio
import digitalio
import adafruit_bmp280

# Creează un obiect SPI
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

# Creează un obiect pentru pinul CS (Chip Select)
cs = digitalio.DigitalInOut(board.D5)  # Specifică pinul utilizat pentru CS (în acest exemplu D5)

# Inițializează senzorul BMP280 cu SPI
bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, cs)

# Configurează senzorul (opțional)
bmp280.sea_level_pressure = 1013.25  # Presiunea la nivelul mării (hPa)

# Loop pentru citirea datelor
while True:
    print(f"Temperature: {bmp280.temperature:.2f} °C")
    print(f"Pressure: {bmp280.pressure:.2f} hPa")
    time.sleep(1)
