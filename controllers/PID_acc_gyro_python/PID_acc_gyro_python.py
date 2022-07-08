from controller import Robot, Accelerometer, Gyro
from math import atan, pi, sqrt

TIME_STEP =       16

# Limita o acionamento do motor
# No mundo real, ele já vai ter um limite natural, então isso
# não é necessário
MAX_SPEED =       10

# Constante do filtro da leitura do ângulo
FILTER_CONSTANT = 0.999

# Constantes do controlador
GANHO_P = 0.5
GANHO_D = 0.8
GANHO_I = 0.005

robot = Robot()

# Inicializa os sensores
acel = Accelerometer('accelerometer')
acel.enable(TIME_STEP)

gyro = Gyro('gyro')
gyro.enable(TIME_STEP)

# Inicializa os motores
wheels = []
wheels_names = ['motor1', 'motor2']
for i, name in enumerate(wheels_names):
    wheels.append(robot.getDevice(name))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
   
   

def get_angle(angulo_antigo: float, first_read: bool = False) -> float:
    '''Lê o acelerômetro e o giroscópio e descobre a inclinação

        :param angulo_antigo: valor do último ângulo filtrado lido
        :param first_read: se é a primeira leitura ou não (usado para ignorar o filtro)
        :returns: angulo filtrado no formato
    '''
    x, z, y = acel.getValues()
    
    # Usar uma das duas fórmulas
    # Para o meu caso foi a segunda
    #angulo_bruto = atan(x/(sqrt(y**2 + z**2))) * 180/pi
    angulo_bruto  = atan(y/(sqrt(x**2 + z**2))) * 180.0/pi
    
    # A primeira medição apenas retorna o ângulo medido direto
    if first_read:
        return angulo_bruto
            
    gx, gy, gz = gyro.getValues()
    # Converte rad/s para graus/s
    gx *= -180.0/pi
    # Converte graus por segundo em graus deslocados desde a última medição
    gx *= TIME_STEP/1000.0
        
    # Formula usada para filrar leitura de ângulo
    angulo_filtrado = FILTER_CONSTANT*(angulo_antigo + gx) + (1-FILTER_CONSTANT)*angulo_bruto
            
    return angulo_filtrado


# Força a inicialização pro sensor retornar um numero valido
# Obs: O acelerômetro só inicializa depois de 32ms
robot.step(32)

angulo = get_angle(0, first_read = True)
print("Ângulo inicial: ", angulo)

erro = -angulo
erro_acumulado = 0


while robot.step(TIME_STEP) != -1:
    angulo = get_angle(angulo)
                
    # Controle PID
    erro_diferenca = angulo - erro
    erro = angulo
    erro_acumulado += erro
    
    controle_motor = GANHO_P * erro\
                     + GANHO_D * erro_diferenca\
                     + GANHO_I * erro_acumulado
       
    # Comentar se o robô estiver girando no sentido contrário        
    controle_motor = -controle_motor
               
    if controle_motor > MAX_SPEED:
        controle_motor = MAX_SPEED
    elif controle_motor < -MAX_SPEED:
        controle_motor = -MAX_SPEED
       
    wheels[0].setVelocity(controle_motor)
    wheels[1].setVelocity(controle_motor)
    
    print(angulo, controle_motor, erro, erro_diferenca, erro_acumulado)
