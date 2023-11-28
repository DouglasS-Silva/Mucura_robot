/*
   Código para testes dos motores com encoder do robô mucura
   Data: 13/0[/2023
*/

#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

DC_motor_controller motorR, motorL;
TwoMotors both(&motorR, &motorL);

// variáveis de controle interno
#define RR 10 // motor JGA25: 21 // motor maior (JGB37): 30
#define kp 0//1.5
#define ki 0//0.9
#define kd 0//0.05

#define in1
// Botão:
#define button 12
#define LED 8

//float Read = sensor.readRangeContinuousMillimeters();

extern void interrupt_motor () {   // Função interrupt_motor
  motorR.isr();              // Chama o método isr(), que realiza a contagem do pulso
  //Serial.println("pulse");

}

extern void interrupt_motor_L () {   // Função interrupt_motor
  motorL.isr();              // Chama o método isr(), que realiza a contagem do pulso
  //Serial.println("pulse");
}



void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.init();

  // Pinos usados:
  motorR.hBridge(11, 5, 6); // pinos da ponte H
  motorR.setEncoderPin(2, 4); // pinos do encoder (sempre o pino de interrupção vem primeiro)
  attachInterrupt(digitalPinToInterrupt(2), interrupt_motor, FALLING); // ativa a interrupção para contagem dos pulsos
  //motorR.debugMaxVel();
  motorR.setRR(RR); // razão da caixa de redução
  motorR.setPPR(7);
  //motorR.setPIDconstants(kp, ki, kd); // constantes do PID

  motorL.hBridge(11, 9, 10); // pinos da ponte H
  motorL.setEncoderPin(3, 7); // pinos do encoder (sempre o pino de interrupção vem primeiro)
  attachInterrupt(digitalPinToInterrupt(3), interrupt_motor_L, FALLING); // ativa a interrupção para contagem dos pulsos
  //  motorL.debugMaxVel();
  motorL.setRR(RR); // razão da caixa de redução
  motorL.setPPR(7);
  //motorL.setPIDconstants(kp, ki, kd); // constantes do PID

  DDRB |= 1;
  PORTB |= 1;

  DDRB &= ~(1 << 4); // pino do botão como input
  while ((PINB >> 4) & 1); // Aguarda o botão ser pressionado
  PORTB &= ~1;
  sensor.startContinuous();




}

void loop() {
  sensor.readRangeContinuousMillimeters();
  float Read = sensor.readRangeContinuousMillimeters();
  //Serial.println(Read);
  motorR.run(87);
  motorL.run(80);

  if (Read < 70) {
    motorR.run(-120);
    motorL.run(-40);
    delay(1500);
  }


}
