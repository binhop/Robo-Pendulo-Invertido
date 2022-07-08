/*
 * File:          PID_acc_gyro_C.c
 * Date: 07/2022
 * Description: Controle PID do robô pêndulo invertido
 * Author: Mundo Projetado
 * Modifications:
 */

// ----- Inclusões -----
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>

#include <stdio.h>
#include <math.h>


// ----- Definições -----
// Constantes da simulação
#define TIME_STEP       16

// Constante do filtro dos sensores
#define FILTER_CONSTANT 0.999

// Limite do motor
#define MAX_SPEED       10

// Constantes do controlador
#define GANHO_P   0.5
#define GANHO_D   0.8
#define GANHO_I   0.005


// ----- Protótipo das funções -----
/**
 * @brief Lê o acelerômetro e o giroscópio e retorna o valor do ângulo filtrado
 * @param angulo_antigo Valor do último ângulo filtrado lido
 * @param first_read Se é a primeira leitura ou não (usado para ignorar o filtro)
 * @return Ângulo filtrado no formato de float
 */
float get_angle(WbDeviceTag accel, WbDeviceTag gyro, float angulo_antigo, bool first);



int main(int argc, char **argv) 
{
  // Variáveis para o controle do robô
  float angulo;
  float erro;
  float erro_diferenca;
  float erro_acumulado = 0;
  float controle_motor = 0;
  
  wb_robot_init();
  
  // Inicializa sensores
  WbDeviceTag accel = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accel, TIME_STEP);
  
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, TIME_STEP);
  
  
  // Inicializa motores
  WbDeviceTag wheels[2];
  char wheels_names[2][8] = {
    "motor1", "motor2",
   };
   
  for(int i = 0; i < 2; i++)
  {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }
  
  // Força a inicialização pro sensor retornar um numero valido
  // Obs: O acelerômetro só inicializa depois de 32ms
  wb_robot_step(32);
  
  angulo = get_angle(accel, gyro, 0, true);
  erro = -angulo;
          
  printf("Ângulo inicial: %.2f\n", angulo);
  
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    angulo = get_angle(accel, gyro, angulo, false);
             
    // Controle PID
    erro_diferenca = angulo - erro;
    erro = angulo;
    erro_acumulado += erro;
    
    
        
    controle_motor = GANHO_P * erro\
                     + GANHO_D * erro_diferenca\
                     + GANHO_I * erro_acumulado;
       
    // Comentar se o robô estiver girando no sentido contrário        
    controle_motor = -controle_motor;
               
    if(controle_motor > MAX_SPEED)
    {
      controle_motor = MAX_SPEED;
    }
    else if(controle_motor < -MAX_SPEED)
    {
      controle_motor = -MAX_SPEED;
    }
       
    wb_motor_set_velocity(wheels[0], controle_motor);
    wb_motor_set_velocity(wheels[1], controle_motor);
    printf("Ang:%.2f - Mot:%.2f - Erro:%.2f - Erro_diff:%.2f - Erro_acum:%.2f\n", angulo, 
                                                                                  controle_motor,
                                                                                  erro,
                                                                                  erro_diferenca,
                                                                                  erro_acumulado);
    
  };


  wb_robot_cleanup();

  return 0;
}


float get_angle(WbDeviceTag accel, WbDeviceTag gyro, float angulo_antigo, bool first_read)
{
  float x, y, z;
  
  const double *accel_reads;
  float angulo_bruto;
  float angulo_filtrado;
  
  const double *gyro_reads;
  float gyro_read;
  
  accel_reads = wb_accelerometer_get_values(accel);
  x = (float)accel_reads[0];
  y = (float)accel_reads[2];
  z = (float)accel_reads[1]; // Considero que o eixo z é o eixo vertical
  
  // Usar uma das duas fórmulas
  // Para o meu caso foi a segunda
  //angulo_raw = atan(x/(sqrt(y*y + z*z))) * 180/M_PI;
  angulo_bruto  = atan(y/(sqrt(x*x + z*z))) * 180.0/M_PI;
  
  // A primeira medição apenas retorna o ângulo medido direto
  if(first_read)
  {
    return angulo_bruto;
  }
          
  gyro_reads = wb_gyro_get_values(gyro);
  gyro_read = (float)gyro_reads[0];
  // Converte rad/s para graus/s
  gyro_read *= -180.0/M_PI;
  // Converte graus por segundo em graus deslocados desde a última medição
  gyro_read *= TIME_STEP/1000.0;
      
  // Formula usada para filrar leitura de ângulo
  angulo_filtrado = FILTER_CONSTANT*(angulo_antigo + gyro_read) + (1-FILTER_CONSTANT)*angulo_bruto;
          
  return angulo_filtrado;
}