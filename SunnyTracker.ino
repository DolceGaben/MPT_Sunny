
#include <Servo.h> // include Servo library 

// 170 horizontal MAX
Servo horizontal; // horizontal servo
int servoh = 80;   // 80;     // stand horizontal servo

int servohLimitHigh = 170;
int servohLimitLow = 20;

// 175 degrees MAX
Servo vertical;   // vertical servo 
int servov = 120; // stand vertical servo

int servovLimitHigh = 175;
int servovLimitLow = 65;


// LDR pin connections
//  name  = analogpin;
int ldrlt = 0; //LDR top left - BOTTOM LEFT    <--- BDG
int ldrrt = 1; //LDR top rigt - BOTTOM RIGHT 
int ldrld = 2; //LDR down left - TOP LEFT
int ldrrd = 3; //ldr down rigt - TOP RIGHT

void setup()
{
  Serial.begin(9600);
// servo connections
// name.attacht(pin);
  horizontal.attach(9); 
  vertical.attach(10);
  horizontal.write(65);//стандартне положення серво
  vertical.write(100);
  delay(3000);
}

void loop() 
{
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt
  
  // int dtime = analogRead(4)/20; // read potentiometers  
  // int tol = analogRead(5)/4;
  int dtime = 10;
  int tol = 50;
  
  int avt = (lt + rt) / 2; // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (lt + ld) / 2; // average value left
  int avr = (rt + rd) / 2; // average value right

  int dvert = avt - avd; // перевірка на різницю вгору і вниз
  int dhoriz = avl - avr;// перевірка на різницю вліво і вправо
  
  Serial.print(avt);
  Serial.print(" ");
  Serial.print(avd);
  Serial.print(" ");
  Serial.print(avl);
  Serial.print(" ");
  Serial.print(avr);
  Serial.print("   ");
  Serial.print(dtime);
  Serial.print("   ");
  Serial.print(tol);
  Serial.println(" ");
  delay(25);//для плавних рухів
    
  if (-1*tol > dvert || dvert > tol) // перевірка чи є різниця щоб змінити vertical angle
  {
  if (avt > avd)
  {
    servov = ++servov;
     if (servov > servovLimitHigh) 
     { 
      servov = servovLimitHigh;
     }
  }
  else if (avt < avd)
  {
    servov= --servov;
    if (servov < servovLimitLow)
  {
    servov = servovLimitLow;
  }
  }
  vertical.write(servov);
  }
  
  if (-1*tol > dhoriz || dhoriz > tol) // перевірка чи є різниця щоб змінити else change horizontal angle
  {
  if (avl > avr)
  {
    servoh = --servoh;
    if (servoh < servohLimitLow)
    {
    servoh = servohLimitLow;
    }
  }
  else if (avl < avr)
  {
    servoh = ++servoh;
     if (servoh > servohLimitHigh)
     {
     servoh = servohLimitHigh;
     }
  }
  else if (avl = avr)
  {
    // nothing
  }
  horizontal.write(servoh);
  }
   delay(dtime);

}




