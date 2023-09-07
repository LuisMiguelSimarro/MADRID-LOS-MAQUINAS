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
const int Speed = 35;//ENTRE 0 Y 255: REDUCIR ESTE VALOR SI QUEREMOS QUE VAYA MÁS DESPACIO
const int setPointDistance = 25;//DISTANCIA REQUERIDA PR LA DERECHA (TENEMOS EL SENSOR US EN LA DERECHA)
const int KpCERCA = 2;//AJUSTAR SEGÚN REACCIÓN DEL VEHÍCULO
const int KpLEJOS = 1;//AJUSTAR SEGÚN REACCIÓN DEL VEHÍCULO
const int KdCERCA = 8;
const int KdLEJOS = 4;
const int Ki = 0.001;
const int FILTRO1 = 1;//FILTRO PARA LA DISTANCIA MEDIDA
const int FILTRO2MIN = 3;//FILTRO PARA LA DISTANCIA MEDIDA, RANGO INFERIOR
const int FILTRO2MAX = 80;//FILTRO PARA LA DISTANCIA MEDIDA, RANGO SUPERIOR
const int FILTRO3 = 15;// FILTRO PARA EL ERROR
// EL DUAL RATE DEBE SER MAYOR SI LAS CURVAS A TOMAR SON MAS CERRADAS
const int DUALRATE = 30;// PONER MAS DUAL RATE SI LAS CURVAS SON MÁS CERRADAS Y TIENE QUE GIRAR MÁS

const int VALORMIN=0;
const int VALORMAX=80;

Servo steeringServo;
const int rearMotorPinEnable = 3;
const int rearMotorPin1 = 10;
const int rearMotorPin2 = 11;

int servoPosition = 90;
const int centerPosition = (VALORMIN+VALORMAX)/2;

const int disparoPin = 4;
const int ecoPin = 5;
long int durationRight;
int distanceMedidaIzqda = 0;//DISTANCIA MEDIDA POR LA IZQDAconst int Speed = 200; //de 0 a 255, PWM)
int error = 0;
int oldErr = 0;
int errSum = 0;

void setup() {
  steeringServo.attach(2);
  steeringServo.write(centerPosition);
  pinMode(rearMotorPinEnable, OUTPUT);
  pinMode(rearMotorPin1, OUTPUT);
  pinMode(rearMotorPin2, OUTPUT);
  pinMode(disparoPin, OUTPUT);
  pinMode(ecoPin, INPUT);
  Serial.begin(9600);
  delay(1000);
  steeringServo.write(0);
  delay(1000);
  steeringServo.write(130);
  delay(1000);
  steeringServo.write(centerPosition);
}

void mideDistanciaIzqda ()
{
  digitalWrite(disparoPin, LOW);
  delayMicroseconds(5);
  digitalWrite(disparoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(disparoPin, LOW);
  durationRight = pulseIn(ecoPin, HIGH);
  distanceMedidaIzqda = 0.017 * durationRight;

}

void loop() {

  analogWrite(rearMotorPinEnable, Speed);
  digitalWrite(rearMotorPin1, LOW);
  digitalWrite(rearMotorPin2, HIGH);
  
  mideDistanciaIzqda();//LA CLAVE ES FILTRAR LOS DATOS DE LOS VALORES DE DISTANCIA OBTENIDOS

  if (distanceMedidaIzqda > FILTRO1) {//PRIMER FILTRO
    //imprime();
    if ((distanceMedidaIzqda > FILTRO2MIN) and (distanceMedidaIzqda < FILTRO2MAX)) { //SEGUNDO FILTRO RANGO MINIMO y MAXIMO
      error = distanceMedidaIzqda - setPointDistance;
      
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
  //
  
  //imprime();
  
}

void calculaCERCA(){//ERROR NEGATIVO
   
   int correccion = KpCERCA * error + KdCERCA *(error-oldErr)+ Ki*(error + errSum);
   //correccion = constrain(correccion, -DUALRATE,0);
   correccion = map(correccion, 0, -20, 0,-DUALRATE);//Aqui poner entre -30 y -90 (el -DUALRATE)
   servoPosition = centerPosition + correccion;
   steeringServo.write(servoPosition);
   oldErr=error;
   errSum = error + errSum;
}

void calculaLEJOS(){//ERROR POSITIVO
   int correccion = KpLEJOS * error + KdLEJOS *(error-oldErr)+ Ki*(error + errSum);
   //correccion = constrain(correccion, 0,DUALRATE);
   correccion = map(correccion, 0, 5, 0, DUALRATE);//Aqui poner entre 30 y 90 (el DUALRATE)
   servoPosition = centerPosition + correccion;
   steeringServo.write(servoPosition);
   oldErr=error;
   errSum = error + errSum;
}


void imprime() {
  Serial.print("  La distancia a la izquierda es: ");
  Serial.println(distanceMedidaIzqda);
  Serial.print("  La Posición del Servo es: ");
  Serial.println(servoPosition);
  Serial.print("  La Velocidad del Coche es: ");
  Serial.println(Speed);
  Serial.println("  ");
  delay(500);
}
