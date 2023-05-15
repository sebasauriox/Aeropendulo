"""
Este código es un script de Python que utiliza un microcontrolador ESP32
para controlar un aeropendulo utilizando un controlador PID y un MPU6050.

También tiene una pantalla OLED y un teclado matricial para mostrar información y ajustar los parámetros del controlador PID.
Además, tiene una conexión Wi-Fi y un servidor web integrado para interactuar con el sistema desde un navegador web.

El código se divide en varias secciones.
En la primera sección, se importan las librerías necesarias y se definen algunas variables y constantes, como el arreglo de bits para los logos del tecnm y el ite,
las constantes del controlador PID y la frecuencia del PWM.

En la segunda sección, se configura el bus I2C y se inicializa la pantalla OLED.
También se define una matriz de teclas y se crea una función para leer el estado de las teclas.

En la tercera sección, se definen funciones para crear el frame buffer y mostrar imágenes en la pantalla OLED.

En la cuarta sección, se define una función para conectar el microcontrolador a una red Wi-Fi.

En la quinta sección, se define una función para iniciar un servidor web en el microcontrolador.

En la sexta sección, se inicia la tarea secundaria en el núcleo secundario y se llama a la función de conexión Wi-Fi.

En la última sección, se define una función principal para controlar el sistema de balanceo utilizando un controlador PID y un MPU6050.
La función utiliza el teclado matricial para ajustar los parámetros del controlador PID y mostrar información en la pantalla OLED.
También utiliza la conexión Wi-Fi y el servidor web para interactuar con el sistema desde un navegador web.

"""



import machine
import ssd1306
import utime
import framebuf
import images
import math
import network
import ujson
import usocket
import _thread

logo1 = images.logo_tecnm() #Importa el arreglo de bits para el logo del tecnm
logo2 = images.logo_ite() #Importa el arreglo de bits para el logo del ite

# Definición de las constantes del controlador PID
Kp = 1.30
Ki = 0.28
Kd = 0.5
# Variables para el cálculo del error y la acción de control
error_sum = 0
last_error = 0

# Frecuencia del PWM en Hz
frecuencia = 60
# Rango del duty cycle (0-100%)
rango_duty_cycle = 11

# Configuración del bus I2C y la dirección del MPU6050
i2c0 = machine.I2C(scl=machine.Pin(19), sda=machine.Pin(5))
mpu_addr = 0x68
i2c0.writeto_mem(mpu_addr, 0x6B, b'\x00')  # Desbloqueo de la MPU6050
# Definir los pines de los renglones y las columnas
rows = [machine.Pin(12), machine.Pin(13), machine.Pin(14), machine.Pin(15)]
cols = [machine.Pin(25), machine.Pin(26), machine.Pin(27), machine.Pin(32)]

# Definir los valores de las teclas
keys = [
    ['1', '2', '3', 'A'],
    ['4', '5', '6', 'B'],
    ['7', '8', '9', 'C'],
    ['*', '0', '#', 'D']
]

# Inicializar la pantalla OLED
i2c1 = machine.I2C(-1, machine.Pin(22), machine.Pin(21))
oled = ssd1306.SSD1306_I2C(128, 64, i2c1)

# Arreglo vacío para almacenar teclas presionadas
pressed_keys = []

#CONECTAR WIFI
def connect_wifi(ssid, password):
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('Conectando a la red Wi-Fi...')
        sta_if.active(True)
        sta_if.connect(ssid, password)
        while not sta_if.isconnected():
            pass
    print('Conexión Wi-Fi establecida:', sta_if.ifconfig())

def start_server():
    addr = usocket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = usocket.socket()
    s.bind(addr)
    s.listen(1)
    print('Servidor escuchando en', addr)
    while True:
        conn, addr = s.accept()
        print('Conexión recibida desde', addr)
        request = conn.recv(1024)
        print('Solicitud recibida:', request)
        with open('index.html', 'r') as archivo:
            contenido = archivo.read()
        
        response = contenido
        conn.sendall(response)
        conn.close()

#Configurar conexión Wi-Fi
ssid = "INFINITUM2248_2.4"
password = "PH4FbmX82N"
connect_wifi(ssid, password)

# Función que se ejecutará en el núcleo secundario
def tarea_secundaria():
    start_server()

# Iniciar la tarea secundaria en el núcleo secundario
_thread.start_new_thread(tarea_secundaria, (), {})

# Función para leer el estado de las teclas
def get_key():
    # Establecer todos los pines de los renglones como salidas en alto
    for row in rows:
        row.init(mode=machine.Pin.OUT, value=1)

    # Establecer todos los pines de las columnas como entradas con pull-up
    for col in cols:
        col.init(mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)

    # Escanear el teclado
    key = None
    for i, row in enumerate(rows):
        row.value(0)
        for j, col in enumerate(cols):
            if col.value() == 0:
                key = keys[i][j]
                break
        row.value(1)
        if key is not None:
            break

    # Esperar un momento para evitar la detección múltiple de una misma tecla
    utime.sleep_ms(300)

    return key

def create_frame_buffer(logo, WIDTH, HEIGHT):
    buffer = bytearray(logo)  # crea un bytearray de imagen
    fb = framebuf.FrameBuffer(buffer, WIDTH, HEIGHT, framebuf.MONO_HLSB)  # crea un objeto de frame buffer a partir del bytearray de imagen
    return fb

def images(oled, fb):
    oled.fill(0)    # Limpia la pantalla del dispositivo
    oled.blit(fb, 0, 0)    # Muestra la imagen almacenada en el frame buffer en la posición (0,0) de la pantalla
    oled.show()    # Actualiza la pantalla con la imagen mostrada
    utime.sleep_ms(1500)    # Espera 3000 milisegundos (3 segundos)
    oled.fill(0)    # Limpia la pantalla del dispositivo nuevamente

bufer_1 = create_frame_buffer(logo1, 128, 64) #Crea frame buffer para logo1
bufer_2 = create_frame_buffer(logo2, 128, 64) #Crea frame buffer para logo2

images(oled,bufer_1) #crea objeto para logo1
images(oled,bufer_2) #crea objeto para logo2

def angulo_a_duty_cycle(angulo):
    # El ángulo debe estar en el rango de 0 a 180 grados
    angulo = max(0, min(90, angulo))
    # El duty cycle está en el rango de 0 a 100%
    duty_cycle = (angulo / 90) * rango_duty_cycle
    
    return duty_cycle


def calibrate_mpu():
    oled.fill(0)
    oled.text("Calibrando",0,20)
    oled.text("Sensores",0,30)

    oled.show()
    print("Mantenga el dispositivo en posición horizontal y sin movimiento durante la calibración...")
    utime.sleep(5)
    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    gyro_x_offset = 0
    gyro_y_offset = 0
    gyro_z_offset = 0
    n_samples = 1000
    
    for i in range(n_samples):
        accel_x = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3B, 2), 'big') / 16384.0
        accel_y = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3D, 2), 'big') / 16384.0
        accel_z = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3F, 2), 'big') / 16384.0
        gyro_x = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x43, 2), 'big') / 131.0
        gyro_y = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x45, 2), 'big') / 131.0
        gyro_z = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x47, 2), 'big') / 131.0

        accel_x_offset += accel_x
        accel_y_offset += accel_y
        accel_z_offset += accel_z
        gyro_x_offset += gyro_x
        gyro_y_offset += gyro_y
        gyro_z_offset += gyro_z

    accel_x_offset /= n_samples
    accel_y_offset /= n_samples
    accel_z_offset /= n_samples
    gyro_x_offset /= n_samples
    gyro_y_offset /= n_samples
    gyro_z_offset /= n_samples

    print("Offsets de calibración:\n")
    print("Accel X: ", accel_x_offset)
    print("Accel Y: ", accel_y_offset)
    print("Accel Z: ", accel_z_offset)
    print("Gyro X: ", gyro_x_offset)
    print("Gyro Y: ", gyro_y_offset)
    print("Gyro Z: ", gyro_z_offset)
    
    oled.fill(0)
    oled.text("Offsets:",0,0)
    oled.text("Accel X:{:.f} ".format(accel_x_offset),0,10)
    oled.text("Accel Y:{:.f} ".format(accel_y_offset),0,20)
    oled.text("Accel Z:{:.f} ".format(accel_z_offset),0,30)
    oled.text("Gyro X:{:.f} ".format(gyro_x_offset),0,40)
    oled.text("Gyro Y:{:.f} ".format(gyro_y_offset),0,50)
    oled.text("Gyro Z:{:.f} ".format(gyro_z_offset),0,55)
    oled.show()
    utime.sleep(10)

    return accel_x_offset, accel_y_offset, accel_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def compute(self, process_variable):
        error = self.setpoint - process_variable
        self.integral += error
        derivative = error - self.last_error
        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.last_error = error
        return output
    

# Read data from the MPU6050 and save it to a JSON file
data = {}
with open("data.json", "w") as f:
        f.write("")
# Bucle principal
while True:
    key = get_key()
    oled.fill(0)
    if key != "*":
        oled.text("Control",0,0)
        oled.text("AeroPendulo",0,16)
        oled.text("Presiona * para continuar",0,25)
        oled.text("para continuar",0,35)
        oled.show()
    else:
        break

while True:
    key = get_key()
    oled.fill(0)

    if key is not None and key != "#" and key !="*":
        if len(pressed_keys) < 3:
            pressed_keys.append(key)
        else:
            pressed_keys.pop(0)
            pressed_keys.append(key)
        oled.text("Ingresa el", 0, 0)
        oled.text("Angulo de ref", 0, 16)
        oled.text("#: guardar", 0, 25)
        oled.text("*: borrar",0, 35)
        oled.text("".join(pressed_keys), 0, 50)
        oled.show()

    if key == "*":
        pressed_keys.clear()
        oled.text("Ingresa el", 0, 0)
        oled.text("Angulo de ref", 0, 16)
        oled.text("#: guardar", 0, 25)
        oled.text("",0, 50)
        oled.text("*: borrar",0, 35)
        oled.show()

    elif key == "#":
        oled.text("Angulo en grad:", 0, 0)
        oled.text("".join(pressed_keys), 0, 25)
        oled.show()
        setpoint = int("".join(pressed_keys))
        print(setpoint)
        utime.sleep(5)
        duty_cycle = angulo_a_duty_cycle(angulo= setpoint)
        # Mostrar el duty cycle en pantalla
        oled.fill(0)
        oled.text("% Duty Cicle:",0,0)
        oled.text("{:.2f}".format(duty_cycle), 0, 25)
        oled.show()
        print(int(duty_cycle))
        # Configurar el PWM con el duty cycle calculado
        pwm = machine.PWM(machine.Pin(4))
        pwm.freq(frecuencia)
        pwm.duty(int(duty_cycle))
        utime.sleep(5)
        
        
        accel_x_offset, accel_y_offset, accel_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset = calibrate_mpu()
    
    
        while True:
            
            # Lectura de los valores de los sensores
            accel_x = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3B, 2), 'big')
            accel_y = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3D, 2), 'big')
            accel_z = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x3F, 2), 'big')
            temp_raw = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x41, 2), 'big')
            gyro_x = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x43, 2), 'big')
            gyro_y = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x45, 2), 'big')
            gyro_z = int.from_bytes(i2c0.readfrom_mem(mpu_addr, 0x47, 2), 'big')
            #Uso de los offsets
            accel_x = accel_x - accel_x_offset
            accel_y = accel_y - accel_y_offset
            accel_z = accel_z - accel_z_offset
            gyro_x = gyro_x - gyro_x_offset
            gyro_y = gyro_y - gyro_y_offset
            gyro_z = gyro_z - gyro_z_offset
                        
            # Conversión de los valores a unidades físicas
            accel_x = accel_x / 16384.0
            accel_y = accel_y / 16384.0
            accel_z = accel_z / 16384.0
            if temp_raw >= 0x8000:  # Si el valor es negativo, se convierte a complemento a dos
                temp_raw = -((temp_raw ^ 0xFFFF) + 1)
            if -40 <= temp_raw/340.0+36.53 <= 85:  # Verificación de rango de temperatura válida
                temp = temp_raw/340.0+36.53
            else:
                temp = None  #
            gyro_x = gyro_x / 131.0
            gyro_y = gyro_y / 131.0
            gyro_z = gyro_z / 131.0
            angle_x = (180/3.141592) * math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
            angle_y = (180/3.141592) * math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
            
            # Objeto de la clase PID 
            pid = PID(Kp, Ki, Kd, setpoint)
            # Cálculo del control PID
            control_signal = pid.compute(angle_x)
            # Se actualiza el valor de duty cicle calculado
            print(control_signal)
            print(setpoint)
            pwm.duty(int(abs(duty_cycle+control_signal)))
            test = duty_cycle+control_signal
            print(int(test))
            
            # Impresión de los valores de los sensores
            print('/////////////////////////////////////////////////////\n')
            print('Accel X: {:.2f} g'.format(accel_x))
            print('Accel Y: {:.2f} g'.format(accel_y))
            print('Accel Z: {:.2f} g'.format(accel_z))
            if temp is not None:
                print('Temperature: {:.2f} C'.format(temp))
            print('Gyro X: {:.2f} deg/s'.format(gyro_x))
            print('Gyro Y: {:.2f} deg/s'.format(gyro_y))
            print('Gyro Z: {:.2f} deg/s'.format(gyro_z))
            print(f'Angulo en x: {angle_x:.2f}°')
            print(f'Angulo en y: {angle_y:.2f}°')
            print('/////////////////////////////////////////////////////\n')
            data['accel_x'] = accel_x
            data['accel_y'] = accel_y
            data['accel_z'] = accel_z
            data['gyro_x'] = gyro_x
            data['gyro_y'] = gyro_y
            data['gyro_z'] = gyro_z
            
        
            utime.sleep(0.5)  # Espera de medio segundo antes de volver a leer los sensores
            # Mostrar los valores en la pantalla OLED
            oled.fill(0)
            oled.text("Accel///inclin", 0, 0)
            oled.text('X: {:.f} g'.format(accel_x),0,20)
            oled.text('Y: {:.f} g'.format(accel_y),0,30)
            oled.text('Deg in x: {:.f}'.format(angle_x),0,40)
            oled.text('Deg in y: {:.f}'.format(angle_y),0,50)
            oled.show()
            
            
            
            # Esperar un segundo antes de volver a leer los valores
            utime.sleep(0.001)
            # Almacenamiento de los datos en el archivo JSON
            with open('data.json', 'a') as f:
            # Agregar el caracter de salto de línea al final del objeto JSON
                f.write(f'{ujson.dumps(data)}' + ',\n')
