int counter=0;
void my_isr()
{
 switch (digitalRead(23))
  {
 case HIGH: Serial.println("clockwise");
            break;
 
 
 case LOW: Serial.println("anticlock");
           break;

 default:  break;
  }
}
void setup() {
  Serial.begin(115200);

  pinMode(23,INPUT);
  //Attaching interrupt from encoder 1 channel A
  pinMode(3,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3),my_isr,RISING);


}

void loop() {
  
}
