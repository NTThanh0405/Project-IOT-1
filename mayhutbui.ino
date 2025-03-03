#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position


const int trig = 6;  
const int echo = 5;    
int tien1 = 10;       
int tien2 = 11;        
int lui1 = 12;        
int lui2 = 13;        
int dongcoservo = 9;   

int gioihan = 25;//khoảng cách nhận biết vật 
int i;
int x = 0;
unsigned long thoigian; 
int khoangcach;          
int khoangcachtrai, khoangcachphai;
int maxspeed=30;

void dokhoangcach();
void dithang(int duongdi);
void disangtrai();
void disangphai();
void dilui();
void resetdongco();
void quaycbsangphai();
void quaycbsangtrai();
void setup() {
  // put your setup code here, to run once:
  myservo.attach(9); 
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);   

  pinMode(tien1, OUTPUT);
  pinMode(tien2, OUTPUT);
  pinMode(lui1, OUTPUT);
  pinMode(lui2, OUTPUT);
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui1, LOW);
  myservo.write(90);
  delay(500);



}

void loop()
{
  khoangcach = 0;
  dokhoangcach();
  if (khoangcach > gioihan || khoangcach == 0)
  {
      dithang();
  }
  else
  {
    resetdongco();
    quaycbsangtrai();
    dokhoangcach();
    khoangcachtrai = khoangcach;
    quaycbsangphai();
    dokhoangcach();
    khoangcachphai = khoangcach;
    if (khoangcachphai < 30 && khoangcachtrai < 30) {
      dilui();
    }
    else
    {
      if (khoangcachphai >= khoangcachtrai)
      {        
        disangphai();
        delay(500);
      }
      if (khoangcachphai < khoangcachtrai)
      {
        disangtrai();
        delay(500);
      }
    }
  }

}
void dithang()
{

  digitalWrite(tien1, HIGH);
  digitalWrite(tien2, HIGH);


}

void disangphai()
{
  resetdongco();
  digitalWrite(lui1, HIGH);
  delay(2000);//thời gian lùi
  digitalWrite(lui1, LOW);


}
void disangtrai()
{
  resetdongco();
  digitalWrite(lui2, HIGH);
  delay(2000);//thời gian 
  digitalWrite(lui2, LOW);

}

void dilui()
{
  resetdongco();
  for (i = 0; i < 20; i++)
  digitalWrite(lui1, HIGH);
  digitalWrite(lui2, HIGH);
  delay(2000);


  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

void resetdongco()
{
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

void dokhoangcach()
{

  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);  
  delayMicroseconds(10); 
  digitalWrite(trig, LOW); 


  // Đo độ rộng xung HIGH ở chân echo.
  thoigian = pulseIn(echo, HIGH);

  khoangcach = thoigian / 2 / 29.412;

}


void quaycbsangtrai()
{
  myservo.write(180);              // tell servo to go to position in variable 'pos'
  delay(1000);
  dokhoangcach();
  myservo.write(90);              // tell servo to go to position in variable 'pos'
}
void quaycbsangphai()
{
  myservo.write(0);              // tell servo to go to position in variable 'pos'
  delay(1000);
  dokhoangcach();
  myservo.write(90);              // tell servo to go to position in variable 'pos'
}
void resetservo()
{
  myservo.write(90);
}
