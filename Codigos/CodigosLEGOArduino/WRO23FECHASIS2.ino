
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//

#include <Pixy2.h>
#include <Servo.h>

// This is the main Pixy object
Pixy2 pixy;
Servo direccion;

const int PuntoMedio = 110;
const int PuntoMinimoDerecha = 40;
const int PuntoMaximoIzquierda = 180;


//uint16_t m_signature;

int velocidad = 30; //Esta velocidad es la que le ponemos al LEGO EV3, por lo que a más velocidad deberá necesitar menos tiempo para esquivar los obstaculos.
int tiempoDeGiro = -62.5 * velocidad + 4875;

const int TriggerFrontPin = 8;
const int EchoFrontPin = 9;
int durationFront;
int distanceFrontCm;

const int TriggerDchaPin = 3;
const int EchoDchaPin = 4;
int durationDcha;
int distanceDchaCm;

const int distDchaSetPoint = 20;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();

  direccion.attach(2);
  direccion.write(PuntoMedio);
  Serial.println("PosicionPuntoMedio");
  delay(2000);
  direccion.write(PuntoMinimoDerecha);
  Serial.println("PosicionMinimoDerecha");
  delay(2000);
  direccion.write(PuntoMaximoIzquierda);
  Serial.println("PosicionMaximoIzquierda");
  delay(2000);
  direccion.write(PuntoMedio);
  Serial.println("PosicionPuntoMedio");

  pinMode(TriggerFrontPin, OUTPUT);
  pinMode(EchoFrontPin, INPUT);

  pinMode(TriggerDchaPin, OUTPUT);
  pinMode(EchoDchaPin, INPUT);
}


void loop()
{
  direccion.write(PuntoMedio);
  Serial.println("Recto");
  delay(1500);

  digitalWrite(TriggerDchaPin,LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerDchaPin,HIGH);
  delayMicroseconds(5);//Entre 5 y 10ms dependiendo del sensor
  digitalWrite(TriggerDchaPin,LOW);
  int durationDcha=pulseIn(EchoDchaPin, HIGH);
  int distanceDchaCm=0.017*durationDcha;
 
  Serial.print(durationDcha);
  Serial.print("  microseconds,  ");
  Serial.print(distanceDchaCm);
  Serial.println(" cm por la derecha");
  delay(100);

  if (distanceDchaCm >= distDchaSetPoint) {
    direccion.write(PuntoMinimoDerecha);
    Serial.println("Derecha");
    delay(3000);//tiempoDeGiro
  }
  else  {
    direccion.write(PuntoMaximoIzquierda);
    Serial.println("Izquierda");
    delay(3000);//tiempoDeGiro
  }

  Slalom();

}

void mideDistFront()
{

  digitalWrite(TriggerFrontPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerFrontPin, HIGH);
  delayMicroseconds(5);//Entre 5 y 10ms dependiendo del sensor
  digitalWrite(TriggerFrontPin, LOW);
  int durationFront = pulseIn(EchoFrontPin, HIGH);
  int distanceFrontCm = 0.017 * durationFront;

  Serial.print(durationFront);
  Serial.print("  microseconds,  ");
  Serial.print(distanceFrontCm);
  Serial.println("cm de frente");

  delay(100);
}

void mideDistDcha()
{

  digitalWrite(TriggerDchaPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerDchaPin, HIGH);
  delayMicroseconds(5);//Entre 5 y 10ms dependiendo del sensor
  digitalWrite(TriggerDchaPin, LOW);
  int durationDcha = pulseIn(EchoDchaPin, HIGH);
  int distanceDchaCm = 0.017 * durationDcha;

  Serial.print(durationDcha);
  Serial.print("  microseconds,  ");
  Serial.print(distanceDchaCm);
  Serial.println("cm por la derecha");

  delay(100);
}

void Slalom() {
  int i;
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);

    for (i = 0; i < pixy.ccc.numBlocks; i++)
    {
      Serial.print("La Signatura es = ");
      Serial.println(pixy.ccc.blocks[i].m_signature);

      if (pixy.ccc.blocks[i].m_signature == 1) {
        direccion.write(PuntoMinimoDerecha);
        delay(tiempoDeGiro);
      } else if (pixy.ccc.blocks[i].m_signature == 3) {
        direccion.write(PuntoMaximoIzquierda);
        delay(tiempoDeGiro);
      }
      delay(250);
    }
  }
}
