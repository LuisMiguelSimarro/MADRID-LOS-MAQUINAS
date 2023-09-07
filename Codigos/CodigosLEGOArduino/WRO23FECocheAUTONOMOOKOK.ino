/*WRO2022-FUTURES ENGINEERS: "COCHE AUTÓNOMO": WALL FOLLOWER
   CHASIS TAMIYA TL-01, TL-02 O BYCMO 1
   1 MOTOR DE PROPULSIÓN. TRACCIÓN 4X4. DIFERENCIALES DELANTE Y DETRÁS.SUSPENSIONES Y AMORTIGUADORES
   1 SERVO DE DIRECCIÓN
   MOTOR CONTROLADO POR CHIP L293 O L298 o bts 7960 de 43 Amperios
   SENSORES UTILIZADOS:
   - ULTRASÓNICOS: SI (1)
   - GYRO-ACCEL MPU6050: NO
   - CAMARAS DE DETECCIÓN DE OBJETOS ALTA SENSIBILIDAD: NO
  SE TRATA DE UN WALL FOLLOWER: MEDIMOS DISTANCIA Y CON EL VALOR DE LA DISTANCIA REQUERIDA CALCULAMOS EL ERROR
  USA PID y FILTROS
  PARA QUE EL COCHE CIRCULE BIEN Y HAGA BIEN EL WALLFOLLOWER TENEIS QUE AJUSTAR LOS VALORES DE:

  - SPEED
  - Distancia Requerida: setPointDistanceDcha
  - Kp
  - Kd
  - Ki
  - FILTRO2MIN
  - FILTRO2MAX
  - FILTRO3
  - dualRateIzq
  - dualRateDcha

  LA DISTANCIA REQUERIDA: setPointDistanceDCHA AHORA ESTA EN 15 cm
*/


//COMIENZA EL PROGRAMA:
//LIBRERIAS USADAS:

#include <Servo.h>

//VARIABLES A CONTROLAR:
const int Speed = 100;//ENTRE 0 Y 255
const int SpeedManiobra = 45;//ENTRE 0 Y 255
const int setPointDistanceIzq = 25;//DISTANCIA REQUERIDA POR LA IZQUIERDA (TENEMOS EL SENSOR US EN LA IZQUIERDA)
const int setPointDistanceFront = 15;//DISTANCIA REQUERIDA DE FRENTE (TENEMOS EL SENSOR US2 EN EL FRONTAL DEL COCHE)
const int KpCERCA = 2;//AJUSTAR SEGÚN REACCIÓN DEL VEHÍCULO
const int KpLEJOS = 1;//AJUSTAR SEGÚN REACCIÓN DEL VEHÍCULO
const int KdCERCA = 8;
const int KdLEJOS = 4;
const int Ki = 0.001;
const int FILTRO1 = 1;//FILTRO PARA LA DISTANCIA MEDIDA
const int FILTRO2MIN = 3;//FILTRO PARA LA DISTANCIA MEDIDA, RANGO INFERIOR
const int FILTRO2MAX = 330;//FILTRO PARA LA DISTANCIA MEDIDA, RANGO SUPERIOR
const int FILTRO3 = 15;// FILTRO PARA EL ERROR
// EL DUAL RATE DEBE SER MAYOR SI LAS CURVAS A TOMAR SON MAS CERRADAS
const int DUALRATE = 60;// PONER MAS DUAL RATE SI LAS CURVAS SON MÁS CERRADAS Y TIENE QUE GIRAR MÁS

const int VALORMIN = 0; //OJO ESTE VALOR ES IMPORTANTE PARA QUE EL SERVO NO SE BLOQUEE. HACER LA PRUEBA CON EL PROGRAMA DEL RANGO SERVO
const int VALORMAX = 60; //OJO ESTE VALOR ES IMPORTANTE PARA QUE EL SERVO NO SE BLOQUEE. HACER LA PRUEBA CON EL PROGRAMA DEL RANGO SERVO

Servo steeringServo;
const int rearMotorPinEnable = 3;
const int rearMotorPin1 = 10;
const int rearMotorPin2 = 11;

int servoPosition = 90;
const int centerPosition = (VALORMIN + VALORMAX) / 2;

const int disparoPinIzq = 4;
const int ecoPinIzq = 5;
long int durationIzq;
int distanceMedidaIzq = 0;//DISTANCIA MEDIDA POR LA IZQDAconst int Speed = 200; //de 0 a 255, PWM)

const int disparoPinFront = 6;//para la Mega es el 52
const int ecoPinFront = 7;//para l Mega es el 53
long int durationFront;
int distanceMedidaFront = 0;//DISTANCIA MEDIDA POR LA IZQDAconst int Speed = 200; //de 0 a 255, PWM)


int error = 0;
int oldErr = 0;
int errSum = 0;

void setup() {
  digitalWrite(12, LOW);
  steeringServo.attach(2);
  steeringServo.write(centerPosition);
  pinMode(rearMotorPinEnable, OUTPUT);
  pinMode(rearMotorPin1, OUTPUT);
  pinMode(rearMotorPin2, OUTPUT);
  pinMode(disparoPinIzq, OUTPUT);
  pinMode(ecoPinIzq, INPUT);
  pinMode(disparoPinFront, OUTPUT);
  pinMode(ecoPinFront, INPUT);
  Serial.begin(9600);
  delay(1000);
  steeringServo.write(VALORMIN);
  delay(1000);
  steeringServo.write(VALORMAX);
  delay(1000);
  steeringServo.write(centerPosition);
  delay(2000);
}

void mideDistanciaIzq ()
{
  digitalWrite(disparoPinIzq, LOW);
  delayMicroseconds(5);
  digitalWrite(disparoPinIzq, HIGH);
  delayMicroseconds(10);
  digitalWrite(disparoPinIzq, LOW);
  durationIzq = pulseIn(ecoPinIzq, HIGH);
  distanceMedidaIzq = 0.017 * durationIzq;

}

void mideDistanciaFront ()
{
  digitalWrite(disparoPinFront, LOW);
  delayMicroseconds(5);
  digitalWrite(disparoPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(disparoPinFront, LOW);
  durationFront = pulseIn(ecoPinFront, HIGH);
  distanceMedidaFront = 0.017 * durationFront;

}

void loop() {
  mideDistanciaFront ();
  //imprime();
  while (distanceMedidaFront > setPointDistanceFront) {
    mideDistanciaFront ();
    digitalWrite(12, LOW);
    analogWrite(rearMotorPinEnable, Speed);
    digitalWrite(rearMotorPin1, LOW);
    digitalWrite(rearMotorPin2, HIGH);

    mideDistanciaIzq();//LA CLAVE ES FILTRAR LOS DATOS DE LOS VALORES DE DISTANCIA OBTENIDOS

    if (distanceMedidaIzq > FILTRO1) {//PRIMER FILTRO
      //imprime();
      if ((distanceMedidaIzq > FILTRO2MIN) and (distanceMedidaIzq < FILTRO2MAX)) { //SEGUNDO FILTRO RANGO MINIMO y MAXIMO
        error = distanceMedidaIzq - setPointDistanceIzq;

        if (error < 0) {
          if (error < -FILTRO3) {
            error = -FILTRO3;
            calculaCERCA();

          } else {
            calculaCERCA();
          }
        } else {
          if (error > FILTRO3) {
            error = FILTRO3;
            calculaLEJOS();

          } else {
            calculaLEJOS();
          }

        }
      } else {
        steeringServo.write(centerPosition);
      }
    }
  }
  digitalWrite(12, HIGH);
  steeringServo.write(centerPosition);
  analogWrite(rearMotorPinEnable, SpeedManiobra);
  digitalWrite(rearMotorPin1, HIGH);
  digitalWrite(rearMotorPin2, HIGH);
  delay(500);
  steeringServo.write(VALORMIN);
  analogWrite(rearMotorPinEnable, SpeedManiobra);
  digitalWrite(rearMotorPin1, HIGH);
  digitalWrite(rearMotorPin2, LOW);
  delay(1000);
  steeringServo.write(VALORMAX);
  analogWrite(rearMotorPinEnable, SpeedManiobra);
  digitalWrite(rearMotorPin1, LOW);
  digitalWrite(rearMotorPin2, HIGH);
  delay(1000);


  //imprime();

}

void calculaCERCA() { //ERROR NEGATIVO

  int correccion = KpCERCA * error + KdCERCA * (error - oldErr) + Ki * (error + errSum);
  //correccion = constrain(correccion, -DUALRATE,0);
  correccion = map(correccion, 0, -20, 0, -DUALRATE); //Aqui poner entre -30 y -90 (el -DUALRATE)
  servoPosition = centerPosition + correccion;
  steeringServo.write(servoPosition);
  oldErr = error;
  errSum = error + errSum;
}

void calculaLEJOS() { //ERROR POSITIVO
  int correccion = KpLEJOS * error + KdLEJOS * (error - oldErr) + Ki * (error + errSum);
  //correccion = constrain(correccion, 0,DUALRATE);
  correccion = map(correccion, 0, 2, 0, DUALRATE);//Aqui poner entre 30 y 90 (el DUALRATE)
  servoPosition = centerPosition + correccion;
  steeringServo.write(servoPosition);
  oldErr = error;
  errSum = error + errSum;
}


void imprime() {
  Serial.print("  La distancia a la izquierda es: ");
  Serial.println(distanceMedidaIzq);
  Serial.print("  La distancia de Frente es: ");
  Serial.println(distanceMedidaFront);
  Serial.print("  La Posición del Servo es: ");
  Serial.println(servoPosition);
  Serial.print("  La Velocidad del Coche es: ");
  Serial.println(Speed);
  Serial.println("  ");
  delay(500);
}
