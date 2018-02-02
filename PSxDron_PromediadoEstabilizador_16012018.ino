/*
	RightHatX ---------> Canal0 (Rotacion sobre mismo eje derecha/izquierda)
	RightHatY ------------> Canal1 (Cámara arriba/abajo)
	R2 -----------------> Canal2 (Potencia)
	LeftHatX --------------> Canal4 (GirIz/GirDr)
	
	****Los pulsos PWM van de 1ms a 2 ms donde 1.5 ms es el punto neutro.****
	
	Comandos PSx
	---------------
	joystick ------> PS3.getAnalogHat(LeftHatX) PS3.getAnalogHat(LeftHatY) PS3.getAnalogHat(RightHatX) PS3.getAnalogHat(RightHatY)
	Boton dig -----> PS3.getButtonClick(TRIANGLE) PS3.getButtonClick(CIRCLE) PS3.getButtonClick(CROSS) PS3.getButtonClick(SQUARE)
	Boton analog --> PS3.getAnalogButton(R2) PS3.getAnalogButton(L2)
	Mov dig -------> PS3.getButtonClick(UP) PS3.getButtonClick(RIGHT) PS3.getButtonClick(DOWN) PS3.getButtonClick(LEFT)
	---------------
*/
//////////////////////DEFINEs///////////////////////////////
//PS3
#include <PS3USB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

//Timer preciso
#include <eRCaGuy_Timer2_Counter.h>

#define chanel_number 6  //numero de canales
#define default_servo_value 1500  //valor por defecto del servo
#define PPM_FrLen 22000  //Tiempo de frame en uSec (1ms = 1000µs)
#define PPM_PulseLen 300  //longitud del pulso
#define onState 1  //Polaridad: 1 positivo, 0 negativo
#define sigPin 3  //Pîn de la señal PPM en la Arduino

#define Granularidad 60 //Numero de muestras para promediar

//////////////////////////////////////////////////////////////////

//////////////////////////////////////GLOBAL//////////////////////////////////////
USB Usb;
PS3USB PS3(&Usb);

int ppm[chanel_number]; //Trama PPM
int Refppm[chanel_number][Granularidad];//Promediado
int IndexRefppm[chanel_number];//Recorrer el vector de promediado
/////////////////////////////////

int Cnt; //Gestion ejecucion funciones loop (Mando<-->Trama)
boolean ConAvanzado; //Maniobras simples false, Maniobras complejas True

//Variables Generador PPM
float lastFrLen;
float lastServo;
float lastPulse;
boolean PPM_run;
boolean pulse;
boolean pulseStart;
byte counter;
byte part;
/////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*
	Funcion Estabilizar(): Estabiliza el valor del ppm promediando los valores de entrada. 
						   Promedio en funcion de la granularidad.
*/
void Estabilizar(int idPPM)
{

	Refppm[idPPM][IndexRefppm[idPPM]] = ppm[idPPM];
	
	for(int p=0; p<Granularidad;p++)
	{
		ppm[idPPM] += Refppm[idPPM][IndexRefppm[idPPM]];
	}
	
	ppm[idPPM] /=Granularidad;
	
	if ( (IndexRefppm[idPPM]> (Granularidad-1)))
		IndexRefppm[idPPM] = 0;
	else
		IndexRefppm[idPPM]++;
}

/*
	Funcion PSxCom(): Comprueba si se ha recibido una orden desde el mando PS3 y transforma dicha orden en un valor de PWM
	para el canal asociado.
	
	Boton R2: Da valores de 0 a 255. (no pulsado,pulsado al max)
	Joystick: Da valores de 0 a 255. Eje X: 0<-----127----->255  EjeY: Y 0<-----127-----> -Y 255
	
*/
void PSxCom() 
{
  //Fijamos los canales 3 y 5
  ppm[3]=1500;
  ppm[5]=1500;
  
  Usb.Task();
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) 
  {
	//Gestion complejidad maniobras
	if(PS3.getButtonClick(L1))
	{
		ConAvanzado =! ConAvanzado;
	}
	
	//Acelerar <---> Canal 2
    if(PS3.getAnalogButton(R2))
	{	
		ppm[2] = constrain(PS3.getAnalogButton(R2), 10, 245);
		Estabilizar(2);
		ppm[2] = map(ppm[2], 10, 245, 1100,1900);
	}
	
	//Camara arriba y Camara abajo <---> Canal1
	if(PS3.getAnalogHat(RightHatY))
	{
		ppm[1] = constrain(PS3.getAnalogHat(RightHatY), 10, 245);
		Estabilizar(1);
		ppm[1] = map(ppm[1], 10, 245, 1900,1100);
	}
	
	//Rotacion derecha y Rotacion izquierda <---> Canal0
	if(PS3.getAnalogHat(RightHatX) && !ConAvanzado)
	{
		ppm[0] = constrain(PS3.getAnalogHat(RightHatX), 10, 245);
		Estabilizar(0);
		ppm[0] = map(ppm[0], 10, 245, 1100,1900);
	}

	//Giro Derecha y Giro Izquierda <---> Canal4
	if(PS3.getAnalogHat(LeftHatX) && !ConAvanzado)
	{
		ppm[4] = constrain(PS3.getAnalogHat(LeftHatX), 10, 245);
		Estabilizar(4);
		ppm[4] = map(ppm[4], 10, 245, 1100,1900);
	}
	
	//Desplazamiento complejo
	if(PS3.getAnalogHat(RightHatX) && ConAvanzado)
	{
		ppm[0] = constrain(PS3.getAnalogHat(RightHatX), 10, 245);
		Estabilizar(0);
		ppm[0] = map(ppm[0], 10, 245, 1100,1900);
		
		ppm[4] = constrain(PS3.getAnalogHat(RightHatX), 10, 245);
		Estabilizar(4);
		ppm[4] = map(ppm[4], 10, 245, 1100,1900);
	}
  }
  
  Cnt=1;
}

/*
	Funcion ppmWrite(): Genera la trama PPM a transmitir en funcion del valor de los pwm. Trama de duracion 22ms
*/
void ppmWrite()
{  
  if(timer2.get_micros() - lastFrLen >= PPM_FrLen)
  {  
    lastFrLen = timer2.get_micros();
    PPM_run = true;
	Serial.println(" ");
  }

  if(counter >= chanel_number)
  {
    PPM_run = false;
    counter = 0;
	
    Cnt=0;
  }

  if(PPM_run)
  {
    if (part)
	{  
      pulse = true;
      part = false;
      lastServo = timer2.get_micros();
    }
    else
	{  
		if(timer2.get_micros() - lastServo >= ppm[counter])
		{
		  counter++;
		  part = true;
		}
    }
  }

  if(pulse)
  {
    if(pulseStart == true)
    {  
		digitalWrite(sigPin, !onState); 

		pulseStart = false;
		lastPulse = timer2.get_micros();
    }
    else
    {  
      if(timer2.get_micros() - lastPulse >= PPM_PulseLen)
      {
		digitalWrite(sigPin, onState);
    
        pulse = false;
        pulseStart = true;
      }
    }
  }
}
/*
	Funcion setup(): Inicializa el puerto serie para pruebas, establece el valor por defecto de los servo, 
	configura el USB para recivir ordenes del mando PS3 y configura el vector utilizado para el promediado.
*/
void setup()
{  
  timer2.setup();
  Serial.begin(115200);
 
  lastFrLen =0;
  lastServo =0;
  lastPulse=0;
  PPM_run =false;
  pulse=false;
  pulseStart = true;
  counter =0;
  part = true;
  
  Cnt = 0;
  
  ConAvanzado=false;
  
  for(int i=0; i<chanel_number; i++)
  {
    ppm[i] = default_servo_value;
	IndexRefppm[i] = 0;//determina la primera posicion del vector de promediado a cada canal 
	
	for( int m=0;m<Granularidad;m++)
	{
		Refppm[i][m]= 0;//Inicializamos el vector de promediado
	}
  }
  
  pinMode(sigPin, OUTPUT); //Asignamos pin 3 placa arduino como salida
  digitalWrite(sigPin,onState); //activamos pin 3 placa arduino como un 1
  
	#if !defined(__MIPSEL__)
	while (!Serial); 
	#endif

	if (Usb.Init() == -1) 
	{
		Serial.print(F("\r\nOSC did not start"));
		while (1);
	}
	Serial.println(F("\r\nPS3 USB Library Started"));
}

/*
	Funcion loop(): Ejecución en bucle del programa donde se comprueba si ha habido una orden de movimiento y
	se modifica los PWM para posteriormente generar la trama PPM a transmitir.
*/
void loop()
{
  if (Cnt==0) //Si Cnt off hacemos lectura orden movimiento
  {
	PSxCom();
  }
  else //Si Cnt on hacemos transmision PPM
  {
	ppmWrite();
  }
}
