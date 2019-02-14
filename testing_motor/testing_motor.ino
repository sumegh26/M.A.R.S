
#define B_LF 22              
#define D_LF 22              
#define P_LF 22              

void setup() {
  // put your setup code here, to run once:
 pinMode(B_LF, OUTPUT);
 pinMode(D_LF, OUTPUT);
 pinMode(P_LF, OUTPUT);
 pinMode(3, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(3)==HIGH)
  {
  digitalWrite(D_LF,HIGH);
  analogWrite(P_LF,150);
  digitalWrite(B_LF,LOW);
  }
else
 {
  digitalWrite(D_LF,LOW);
  analogWrite(P_LF,0);
  digitalWrite(B_LF,HIGH);
  }
}
