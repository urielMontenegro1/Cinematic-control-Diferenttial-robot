/////////////////////////////////////////////////////////////////Posiciones deseadas
float posDeseadaEnX=120.0; //en cm tiene un excedente de 50 cm
float posDeseadaEnY=60.0; //en cm+60
int act1=0;
int minimoVelocidad=11;//11
int maximoVelocidad=70;//50
int error=65;//20,50
/////////////////////////////////////////////BOTON DE REINICIO
int boton =22;
int entrada=18;
int entrada2=32;
////////////////////////////////////////////GANANCIAS   
float kv=5.0,kw=40.0;
//float kv=0.5,kw=10.0;
//////////////////////////////////////////////////////////////////parametros
float ev=0,ew=0;
float v=0,w=0,wr=0,wl=0;
/////////////////////////////////////////////////////////////////////parametros de carro

float radioRueda = 3.3; // en cm
float baseRuedas = 22; //en cm

////configuración ESP32 T-BEAM
//// Motor derecha
//int motor1Pin1 = 14;
//int motor1Pin2 = 13;
//int enable1Pin = 25;
//
////Motor izquierda
//int motor2Pin1 = 15;
//int motor2Pin2 = 35;
//int enable2Pin = 4;


// Motor derecha //configruarcion ESP32 ///
int motor1Pin1 = 5;
int motor1Pin2 = 26;
int enable1Pin = 14;

//Motor izquierda
int motor2Pin1 = 25;
int motor2Pin2 = 33;
int enable2Pin = 15;


// Setting PWM properties
const int freq = 30000;

const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
const int vel=10;
////////////////////////////////////////////////////////////////////////////////////// ENCODERS
//const byte Encoder_C1d = 33; //21 Cable amarillo pin 3 digital
//const byte Encoder_C2d = 2; // Cable verde al pin 4 digital
//const byte Encoder_C1i = 32; //22 Cable amarillo pin 3 digital
//const byte Encoder_C2i = 36; // Cable verde al pin 4 digital

//conmfiguración ESP32
const byte Encoder_C1d = 21; // Cable amarillo 
const byte Encoder_C2d = 4; // Cable verde 
const byte Encoder_C1i = 27; // Cable verde 
const byte Encoder_C2i = 13; // Cable amarillo 

byte Encoder_C1Lastd, Encoder_C1Lasti;
int pasoD, pasoI;
boolean direcciond, direccioni;

//para medir distancia dl robot
float dc,dr,di;

//es para activar el motor
int stby = 23;

float posx = 0, posy = 0, posw = 0;

boolean act;
//////////////////
char mjsUART,mjsUART1;

void setup() {
  
  posDeseadaEnX=posDeseadaEnX+50.0;
  posDeseadaEnY=posDeseadaEnY*2.0;
    //posDeseadaEnY=posDeseadaEnY*2.0;
  
    pinMode(boton,INPUT);
    pinMode(entrada,INPUT_PULLUP);
    pinMode(entrada2,INPUT_PULLUP);
    // put your setup code here, to run once:
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
  
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);
    
    pinMode(stby, OUTPUT);

    // configure LED PWM functionalitites
    ledcSetup(pwmChannel, freq, resolution);
    ledcSetup(pwmChannel2, freq, resolution); 
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(enable1Pin, pwmChannel);
//  
    ledcAttachPin(enable2Pin, pwmChannel2);

    digitalWrite(stby, 1);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    Serial.begin(115200);
}


void loop() {  
//
//    attachInterrupt(digitalPinToInterrupt(2), calculapulsoD, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(36), calculapulsoI, CHANGE);

    if(digitalRead(boton)==0){     
       posx=0;
       posy=0;
       posw=0;
       ev=0;
       ew=0;
       v=0;
       w=0;
       wr=0;
       wl=0;
       pasoD=0;
       pasoI=0;
     }

    attachInterrupt(digitalPinToInterrupt(21), calculapulsoD, CHANGE);
    attachInterrupt(digitalPinToInterrupt(27), calculapulsoI, CHANGE);


    //Serial.println("valor de pasoD:"+String(pasoD));
    //Serial.println("valor de pasoI:"+String(pasoI));

    dr = 2.0 * M_PI * radioRueda * (pasoD/748.0);
    di = 2.0 * M_PI * radioRueda * (pasoI/748.0);
    
    dc = (dr + di) / 2.0;

    posx = dc * cos(posw);
    posy = dc * sin(posw);
    posw = (dr - di) / baseRuedas;

//    Serial.println("la posicion actual en X es:" + String(posx));
//    Serial.println("la posicion actual en Y es:" + String(posy));
    
    
    //Serial.println("la posicion actual en W es:" + String(posw));

    ///////////////////////////////////////////////////////////////// CONTROL
    //errores

   // Serial.println("@"+String(posx)+","+String(posy)+"@");
    ev=sqrt((posDeseadaEnX-posx)*(posDeseadaEnX-posx)+(posDeseadaEnY-posy)*(posDeseadaEnY-posy));//error lineal
    ew=atan2((posDeseadaEnY-posy),(posDeseadaEnX-posx))-posw;//error angular
    ew = atan2(sin(ew), cos(ew));

    //velocidades 
    v = kv * ev;//velocidad lineal
    w = kw * ew;//velocidad angular

    wr = (2 * v + baseRuedas * w) / (2 * radioRueda);//velocidad de rueda derecha
    wl = (2 * v - baseRuedas * w) / (2 * radioRueda);//velocidad de rueda izquierda

    wr=abs(wr);
    wl=abs(wl);
    
    if(wr>=250){
      wr=250;
    }
    if(wl>=250) {
      wl=250;        
    }
    if(wr>=minimoVelocidad && wr<=maximoVelocidad){
      wr=maximoVelocidad;
    } 
    if(wr<minimoVelocidad){
      delay(40);
       //Serial.println("atras derecha:");
        ledcWrite(pwmChannel, 75);
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
      }
    if(wl>=minimoVelocidad && wl<=maximoVelocidad){
      wl=maximoVelocidad;
    }
    if(wl<minimoVelocidad){
      delay(40);
      //Serial.println("atras izquierda:");
         ledcWrite(pwmChannel2, 75);
         digitalWrite(motor2Pin1, LOW);
         digitalWrite(motor2Pin2, HIGH);
      }

        if(digitalRead(entrada)==1){
          while(digitalRead(entrada)==1){
           wr=0;
           wl=0;
           stby = 0;
              ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    if (digitalRead(entrada2)==1){
      while(digitalRead(entrada2)==1){
           Serial.println("!PERSONA ENCONTRADA(consulta posiciones aproximadas)!");
           delay(500);
           Serial.println("@"+String(posx)+","+String(posy)+"@");
           delay(500);
      }
    }
          } 
         } 

    if(ev<=error){
      wr=0;
       wl=0;
       stby = 0;
             ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
       //Serial.println("la posicion actual en X es:" + String(posx));
       //Serial.println("la posicion actual en Y es:" + String(posy));
       Serial.println("@"+String(posDeseadaEnX-50)+","+String(posDeseadaEnY/2)+"@");
       delay(500);
       Serial.println("!terminado aproximado!");
       delay(500);
 
       
      }

    if(posx>=posDeseadaEnX-20 && posy<=posDeseadaEnY-20 && posx>=posDeseadaEnX+20 && posy>=posDeseadaEnY+20){
       wr=0;
       wl=0;
       stby = 0;
              ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
       //Serial.println("la posicion actual en X es:" + String(posx));
       //Serial.println("la posicion actual en Y es:" + String(posy));
       Serial.println("@"+String(posDeseadaEnX-50)+","+String(posDeseadaEnY/2)+"@");
       delay(500);
       Serial.println("!terminado aproximado!");
       delay(500);
                   ledcWrite(pwmChannel, wr);
   
    
      }
       stby = 1;
    ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
/////////////////////////////////////////////////////////////////////////////////mensajes UART
//        if (Serial.available()>0){
//           //Serial.println("leyendo entrada de mensaje...");
//           //este serial
//          
//             mjsUART=Serial.read();
//             if(mjsUART=='a'){
//                 act1=1;
//              }
//             //Serial.println("!persona  en"+String(posx)+","+String(posy)+"!");
//             //Serial.println("persona encontrada en"+String(posx)+","+String(posy));
//             wr=0;
//             wl=0;
//             stby = 0;
//             
//          
//                //hacer posible while que   desde cpu  diga              
//    }  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//    Serial.println("pasoI: "+String(pasoI));
//    Serial.println("pasoD: "+String(pasoD));     
//    Serial.println("error lineal ev:"+String(ev));
//    Serial.println("error angular ew:"+String(ew));
//    Serial.println("velocidad lineal:"+String(v));
//    Serial.println("velocidad angular:"+String(w));
//    Serial.println("rueda derecha vel:"+String(wr));
//    Serial.println("rueda izquierda vel:"+String(wl));

///////////////////////////conf_2
      stby = 1;
    ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);



      
    delay(1);

}


void calculapulsoD()
{
  int Lstated = digitalRead(Encoder_C1d);
  if ((Encoder_C1Lastd == LOW) && Lstated == HIGH)
  {
    int vald = digitalRead(Encoder_C2d);
    if (vald == LOW && direcciond)
    {
      direcciond = false; //Atrás
    }
    else if (vald == HIGH && !direcciond)
    {
      direcciond = true;  //Adelante
    }
  }
  Encoder_C1Lastd = Lstated;

  if (!direcciond)  pasoD++;
  else  pasoD--;
}

void calculapulsoI()
{
  int Lstate_i = digitalRead(Encoder_C1i);
  if ((Encoder_C1Lasti == LOW) && Lstate_i == HIGH)
  {
    int vali = digitalRead(Encoder_C2i);
    if (vali == LOW && direccioni)
    {
      direccioni = false; //Atrás
    }
    else if (vali == HIGH && !direccioni)
    {
      direccioni = true;  //Adelante
    }
  }
  Encoder_C1Lasti = Lstate_i;

  if (!direccioni)  pasoI++;
  else  pasoI--;
}
