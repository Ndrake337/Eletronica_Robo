# Import necessary modules
from machine import Pin , PWM
import bluetooth
from ble_simple_peripheral import BLESimplePeripheral
import time

# Configura o pino do LED embutido
led = Pin("LED", Pin.OUT)

# Acende o LED
led.on()

# Configuração dos pinos de controle para os Motores
motor1_pin1 = Pin(7, Pin.OUT)  # Pino GP7 (direção 1)
motor1_pin2 = Pin(8, Pin.OUT)  # Pino GP8 (direção 2)

pwm_pin_1 = PWM(Pin(5))         # Pino GP5 para controle de velocidade com PWM para o Motor 1
pwm_pin_2 = PWM(Pin(21))        # Pino GP21 para controle de velocidade com PWM para o Motor 2

motor2_pin1 = Pin(20, Pin.OUT)  # Pino GP20 (direção 1)
motor2_pin2 = Pin(28, Pin.OUT)  # Pino GP28 (direção 2)

# Configuração da frequência do PWM
pwm_pin_1.freq(1000)  # Define a frequência do PWM para 1 kHz
pwm_pin_2.freq(1000)  # Define a frequência do PWM para 1 kHz

# Função para ajustar a velocidade do motor
def set_speed(duty_cycle, pwm_pin):
    pwm_pin.duty_u16(int(duty_cycle * 65535 / 100))  # Define o ciclo de trabalho (0 a 100%)

# Função para acionar o motor em uma direção
def motor_forward(motor_pin1, motor_pin2, pwm_pin, speed=100):
    motor_pin1.value(1)
    motor_pin2.value(0)
    set_speed(speed, pwm_pin)
    print("Motor para frente")

# Função para acionar o motor na direção oposta
def motor_reverse(motor_pin1, motor_pin2, pwm_pin, speed=100):
    motor_pin1.value(0)
    motor_pin2.value(1)
    set_speed(speed, pwm_pin)
    print("Motor para trás")

# Função para parar o motor
def motor_stop(motor_pin1, motor_pin2, pwm_pin):
    motor_pin1.value(0)
    motor_pin2.value(0)
    set_speed(0, pwm_pin)  # Desliga o PWM
    print("Motor parado")
    


# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()

# Create an instance of the BLESimplePeripheral class with the BLE object
sp = BLESimplePeripheral(ble)

# Configura o pino do LED embutido
led = Pin("LED", Pin.OUT)

# Acende o LED
led.on()

led_t = time.time()
led_a = True

# Define a callback function to handle received data
def on_rx(data):
    print("Data received: ", data)  # Print the received data
    global led_state  # Access the global variable led_state
    if data == b'forward\r\n':
        motor_forward(motor1_pin1, motor1_pin2, pwm_pin_1, 100)
    elif data == b'backward\r\n':
        motor_reverse(motor1_pin1, motor1_pin2, pwm_pin_1, 100)
    elif data == b'stop\r\n':
        motor_stop(motor2_pin1, motor2_pin2, pwm_pin_2)

conn = False

# Start an infinite loop
while True:
    if sp.is_connected():  # Check if a BLE connection is established
        sp.on_write(on_rx)  # Set the callback function for data reception
        led.on()
        led_a = True
        conn = True
    else:
        if conn:
            motor_stop(motor2_pin1, motor2_pin2, pwm_pin_2)
            conn = False
        if time.time() - led_t >= 0.5: # Pisca LED se Bluetooth não estiver conectado
            led_t = time.time()
            led_a = not led_a
            if led_a:
                led.on()
            else:
                led.off()
        
        

led.off()