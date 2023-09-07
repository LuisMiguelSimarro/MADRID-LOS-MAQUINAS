
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
int tiempoDeGiro = -62.5*velocidad+4875;


void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();

  direccion.attach(2);
  direccion.write(PuntoMedio);
  delay(2000);
  direccion.write(PuntoMinimoDerecha);
  delay(2000);
  direccion.write(PuntoMaximoIzquierda);
  delay(2000);
  direccion.write(PuntoMedio);
}

void loop()
{
  direccion.write(PuntoMedio);
        delay(15);
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
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
      //pixy.ccc.blocks[i] ;
      //Block blocks[i];
      Serial.println(  );
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
