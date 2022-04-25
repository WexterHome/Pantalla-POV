#include <FastLED.h>
#include "imgs.h"
#include <avr/pgmspace.h>

//Sensor de efecto HALL
const int HALL_SENSOR = 2;
unsigned long lastTime = 0;
unsigned long threshold = 80;

//Driver L298N y Motor CC
const int PIN_MOTOR_1 = 8;
const int PIN_MOTOR_2 = 9;
//Para controlar la velocidad del motor vamos a usar la función
//analogWrite. No podemos utilizar los pines 9 y 10 porque utilizan
//el timer 1 y nosotros ya lo utilizamos.
const int PIN_VEL_MOTOR = 5;
int motor_speed = 0;
float lap_time = 0;
volatile unsigned long lap_cont = 0;
volatile unsigned long aux_lap_cont = 0;

int resolution = 10; //Cambiamos color de los leds cada 10 grados
byte leds_samples = 360/resolution;
int leds_pos = 0;
volatile bool leds_pos_changed = false;

//LEDS
#define NUM_LEDS 70
#define LED_PIN 4
#define BRIGHTNESS 50
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];



//Prototipos de las funciones
void stopMotor();
void activateMotor();
void motorSpeedController(int speed_percentage);
float estimateLapTime();
void draw_leds(int quadrant, int leds_pos);


void setup() {
  Serial.begin(115200);
  delay(15000);
  //Pin del sensor de efecto Hall
  pinMode(HALL_SENSOR, INPUT);

  //Pines del driver y motor
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_VEL_MOTOR, OUTPUT);

  //Inicializamos leds
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  for(int i=0; i<NUM_LEDS; i++){
    leds[0].setRGB(0,0,0);
  }
  FastLED.show();
  delay(1000);
  leds[0].setRGB(255,0,0);
  FastLED.show();
  delay(1000);

  //Activa el motor y aumenta su velocidad poco a poco.
  //Cambiar el 8 del bucle for por la velocidad deseada (del 5 al 11),
  //siendo 5 un 40% y 11 un 100% de velocidad.
  activateMotor();
  for(int i=0; i<8; i++){
    motorSpeedController(motor_speed);
    motor_speed += 10;
    delay(200);
  }

  //Calculamos cuanto tiempo tarda la barra en dar una vuelta
  delay(2000);
  leds[0].setRGB(0,255,0);
  lap_time = estimateLapTime();
  Serial.print("Lap Time: ");
  Serial.println(lap_time);
  delay(3000);
  
  //Calculamos cuantos pulsos tiene que contar el timer
  //por vuelta de la barra. Hay que tener en cuenta que usamos
  //timer 1 de 16 bits y preescaler de 256  
  float pulses_number = lap_time/(0.016*leds_samples);
  
  //Configuramos timer
  TCCR1A = 0;             //Resetea el registro entero de TCCR1A a 0
  TCCR1B = 0;             //Resetea el registro entero de TCCR1B a 0
  TCCR1B |= B00000100;    //Establece CS12 a 1, es decir, prescalar a 256
  TCNT1 = 0;              //Resetea la cuenta del timer 1 a 0
  TIMSK1 |= B00000010;    //Establece OCIE1A a 1, es decir, habilita el modo de comparación con el regristro A
  OCR1A = pulses_number;  //La interrupción saltará cuando el timer cuente hasta pulses_number.
  aux_lap_cont = lap_cont;
}


void loop() {
  
  if(!digitalRead(HALL_SENSOR) && (millis()-lastTime)>threshold){
    lastTime = millis();
    leds_pos = 0;
    TCNT1 = 0;
    lap_cont++;
  }

  if(leds_pos_changed){
    leds_pos_changed = false; 
    draw_leds(leds_pos);
    FastLED.show();
  }  

  //Cuando la barra de 40 vueltas, el motor se detendrá.
  //Se puede cambiar el 40 por el número de vueltas que se quiera que de. 
  if(lap_cont>40){
    stopMotor();
    Serial.println("Fin");
    delay(20000);  
  }
}


//Para el motor
void stopMotor(){
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
}

//Activa el motor
void activateMotor(){
  digitalWrite(PIN_MOTOR_1, HIGH);
  digitalWrite(PIN_MOTOR_2, LOW); 
}

//Establece la velocidad del motor
void motorSpeedController(int speed_percentage){
  analogWrite(PIN_VEL_MOTOR, map(speed_percentage, 0, 100, 0, 255));
}

float estimateLapTime(){
  bool first_lap = true;
  int cont = 0;
  long lastTime = 0;
  int threshold = 50;
  long init_time = 0;

  while(cont<10){
    if(!digitalRead(HALL_SENSOR) && (millis()-lastTime)>threshold){
      if(first_lap){
        first_lap = false;
        init_time = millis();
      }
      else
        cont++;
      lastTime = millis();
    }
  }

  float time_for_lap = (millis()-init_time)/cont;
  return time_for_lap;
}

//Función que se encarga de cambiar el color de los leds
/*
 * Para cambiar de imagen hay que cambiar donde pone frame0 por
 * frame1, frame2 o frame3. 
 */
void draw_leds(int leds_pos){
  int row = leds_pos/resolution;
  int HALF_LEDS = NUM_LEDS/2;
  int i_aux = 0;
  
  if(leds_pos < 180){
    for(int i=0; i<HALF_LEDS; i++){
      leds[i] = pgm_read_dword(&(frame0[row][i]));
    }
    
    row += 18;
    i_aux = 0;
    for(int i=(NUM_LEDS-1); i>=HALF_LEDS; i--){
      leds[i] = pgm_read_dword(&(frame0[row][i_aux]));
      i_aux++;
    }
  }
  else{
    for(int i=0; i<HALF_LEDS; i++){
      leds[i] = pgm_read_dword(&(frame0[row][i]));
    }
    row -= 18;
    i_aux = 0;
    for(int i=(NUM_LEDS-1); i>=HALF_LEDS; i--){
      leds[i] = pgm_read_dword(&(frame0 [row][i_aux]));
      i_aux++;
    }
  }
}


///////////////////////////////////////////////////////////
////////////////////INTERRUPCIONES/////////////////////////
///////////////////////////////////////////////////////////
//Interrupción del timer
ISR(TIMER1_COMPA_vect){
  TCNT1 = 0;
  leds_pos_changed = true;
  if(leds_pos<360){
    leds_pos += resolution;
  }
}
