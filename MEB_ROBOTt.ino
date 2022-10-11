#include <QTRSensors.h>

#define Kp 0.09 //0.09 // Varsayılan 0.1 idi 
#define Kd 0.9 // varsayılan 4 idi ( Not: Kp < Kd) 
#define MaxSpeed 200// Varsayılan 255 idi Maksimum Hız
#define BaseSpeed 80 // Varsayılan 255 idi Çizgide kalınan en ideal hız değeri
#define NUM_SENSORS 8 // Kullanılan sensör sayısı
#define TIMEOUT 2500
#define speedturn 60 //Varsayılan değer 180 idi

#define rightMotor1 12
#define rightMotor2 13
#define rightMotorPWM 11
#define leftMotor1 3
#define leftMotor2 4
#define leftMotorPWM 5
#define motorPower 8 
#define MZ80on 6
#define MZ80yan 10


QTRSensorsRC qtrrc((unsigned char[]) {14,15,16,17,18,19,9,7} ,NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);//{7,9,19,18,17,16,15,14}

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  pinMode(MZ80on, INPUT);
  pinMode(MZ80yan, INPUT);

  delay(2000);


  int i;
   for (int i = 0; i < 150; i++)
  {
    if (0 <= i && i < 5) {move(1, 30, 1); move(0, 30, 0);}
    if (5 <= i && i < 15) {move(1, 30, 0); move(0, 30, 1);}
    if (15 <= i && i < 25) {move(1, 30, 1); move(0, 30, 0);}
    if (25 <= i && i < 35) {move(1, 30, 0); move(0, 30, 1);}
    if (35 <= i && i < 40) {move(1, 30, 1); move(0, 30, 0);}
    if (45 <= i && i < 50) {move(1, 30, 0); move(0, 30, 1);}
    
    if (i >= 50) {
      wait();
      delay(3);
    }
    
    qtrrc.calibrate();
    delay(20);
  }


//Serial.begin(9600);
}
int onsensor=1; 
int yansensor=1; 
bool zemin=0;
int lastError = 0;
unsigned int sensors[8];
int position = qtrrc.readLine(sensors, 1, zemin);
int sag=0;
int sol=0;
int ileri=0;

void loop()
{  
  sensor_oku();
  //sensor_yaz();

//(sensors[1]<100 && sensors[2]<100 && sensors[3]<100 && sensors[4]<100 && sensors[5]<100 && sensors[6]<100) ┼
//(sensors[0]>600 && sensors[1]>600 && sensors[3]<100 && sensors[4]<100 && sensors[5]<100 && sensors[6]<100 && sensors[7]<100) ┌
//(sensors[0]<100 && sensors[1]<100 && sensors[2]<100 && sensors[3]<100 && sensors[4]<100 && sensors[6]>600 && sensors[7]>600) ┐
//(sensors[0]<100 && sensors[1]<100 && sensors[3]>600 && sensors[4]>600 && sensors[6]<100)&& sensors[7]<100) > Y
//┼ geciş
 while (yansensor == LOW || onsensor ==LOW )  //Kapı
  {
    sensor_oku();
    
    wait();
   
  }
  
 while (sensors[1]<100 && sensors[2]<100 && sensors[3]<100 && sensors[4]<100 && sensors[5]<100 && sensors[6]<100)
    {
      move(1, 40, 1);
      move(0, 40, 1);
      delay(20);
      sensor_oku();
      if(sensors[1]<100 && sensors[2]<100 && sensors[3]<100 && sensors[4]<100 && sensors[5]<100 && sensors[6]<100)
      ileri=1;
    }
    while (ileri==1)
    {
    move(1, 48, 0);//Sol motor geri
    move(0, 42, 0);//Sağ motor ileri
    delay(3500);
    }
        sensor_oku();
         
        while((sensors[6]>500 || sensors[7]>500) && ( sensors[4]<100 || sensors[3]<100)  && (sensors[1]>500 || sensors[0]>500))
        {
               move(1, 40, 1);
               move(0, 40, 1);
        }
        }
          
        sensor_oku();
         
         
        {
          ileri=0;
        }
    }


   
  

//90 derece sağa dönüş
   while ((sensors[0]>500 && sensors[1]>500 && (sensors[3]<100 || sensors[4]<100) && sensors[5]<100 && (sensors[6]<100 || sensors[7]<100)))
    {
      move(1, BaseSpeed, 1);
      move(0, BaseSpeed, 1);
      sag=1;
      sensor_oku();
    }
    while (sag==1)
    {
    move(1, speedturn, 0);//Sağ motor geri
    move(0, speedturn, 1);//Sol motor ileri
        sensor_oku();
         if((sensors[6]>500 || sensors[7]>500) && (sensors[4]<100 || sensors[3]<100) && (sensors[1]>500 || sensors[0]>500) )
        {
          sag=0;
        }
    }
    //45 derece sol dönüş
   while ((sensors[0]>500 && (sensors[3]<100 || sensors[4]<100) && sensors[5]<100 && (sensors[6]<100 || sensors[7]<100)))
    {
      move(1, 40, 1);
      move(0, 40, 1);
      delay(200);
      sag=1;
      sensor_oku();
    }
    while (sag==1)
    {
    move(1, speedturn, 0);//Sağ motor geri
    move(0, 90, 1);//Sol motor ileri
        sensor_oku();
         if((sensors[6]>500 || sensors[7]>500) && (sensors[4]<100 || sensors[3]<100) && (sensors[1]>500 || sensors[0]>500) )
        {
          sag=0;
        }
    }


 //90 derece sola dönüş  
    while ((sensors[0]<100 || sensors[1]<100) && (sensors[2]<100 || sensors[3]<100) && sensors[4]<100 && sensors[6]>500 && sensors[7]>500) 
    {
      move(1, BaseSpeed, 1);
      move(0, BaseSpeed, 1);
      sol=1;
      sensor_oku();
     
    }
    while (sol==1)
    {
      
    move(1, speedturn, 1);//Sağ motor ileri
    move(0, speedturn, 0);//Sol motor geri
   
    sensor_oku();
    if((sensors[6]>500 || sensors[7]>500) && (sensors[4]<100 || sensors[3]<100) && (sensors[1]>500 || sensors[0]>500) )
        {
          sol=0;
        }
    }
//45 sag
//45 derece saga dönüş  
    while ((sensors[0]<100 || sensors[1]<100) && (sensors[2]<100 || sensors[3]<100) && sensors[4]<100  && sensors[7]>500) 
    {
      move(1, 40, 1);
      move(0, 40, 1);
      sol=1;
      sensor_oku();
     
    }
    while (sol==1)
    {
      
    move(1, 90, 1);//Sağ motor ileri
    move(0, speedturn, 0);//Sol motor geri
   
    sensor_oku();
    if((sensors[6]>500 || sensors[7]>500) && (sensors[4]<100 || sensors[3]<100) && (sensors[1]>500 || sensors[0]>500) )
        {
          sol=0;
        }
    }

//d

    

  int error = position-3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; 
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
  move(1, rightMotorSpeed, 1);//Sağ motor ileri
  move(0, leftMotorSpeed, 1);//Sol Motor ileri
}

void sensor_oku()
{
  onsensor=digitalRead(MZ80on);
  yansensor=digitalRead(MZ80yan);
  
  position = qtrrc.readLine(sensors, 1, zemin); // çizgi beyaz ise zemin değeri 0 olmalıdır.whiteLine = 0
  if (sensors[0] < 100 && sensors[7] < 100) 
  {
    zemin = 1; //çizgi siya zemin beyazdır.
  }
  if (sensors[0] > 700 && sensors[7] > 700) 
  {
    zemin = 0;//çizgi beyazdır zemin siyahdır.
  }
}
void sensor_yaz()
{
   for (unsigned char z=0; z<8; z++)
    {
      Serial.print(sensors[z]);
      Serial.println('\t');
    }
    Serial.println();
    Serial.print("On Sensör");Serial.println(onsensor);
    Serial.print("Yan Sensör");Serial.println(yansensor);
    delay(250);
}
  
void wait(){
  digitalWrite(motorPower, LOW);
}



void move(int motor, int speed, int direction){
  digitalWrite(motorPower, HIGH); //Motorları durdur.

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}
