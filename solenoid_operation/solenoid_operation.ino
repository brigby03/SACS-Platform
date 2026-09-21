
const int solenoid1_pin = 9;

bool valve_state = false; //false == closed

int openGap = 50;
int closeGap = 50;


void setup() {

  pinMode(solenoid1_pin, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting..."); 
  Serial.println();
  Serial.println();delay(3000);
}

void loop() {

  if((Serial.available() > 0))
  {
    char inByte = Serial.read();
    if (inByte == 'o')
    {
      valve_state = true;
    }
    else if (inByte == 'c')
    {
      valve_state = false;
    }
  }


  if (valve_state == true)
  {
    digitalWrite(solenoid1_pin,HIGH);
    delay(openGap);
    digitalWrite(solenoid1_pin,LOW);
    delay(closeGap);
  }
  else
  {
    digitalWrite(solenoid1_pin,LOW);
  }


  if(valve_state == false)
  {
    Serial.println("Valve State: Closed" );
  }
  else if(valve_state == true)
  {
    Serial.println("Valve State: Open");
  }


}
