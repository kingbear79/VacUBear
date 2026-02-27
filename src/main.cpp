#include <Arduino.h>
#include <JC_Button.h>

#define PUMPE1 D5
#define PUMPE2 D6
#define VENTIL D7
#define TASTER D2

#define SHOW_LENGTH 50000
#define SHOW_NACHLAUF 50000

void startShow(void);
void stopShow(void);
void handleShow(void);

Button button(TASTER,25,true);

class ShowStatus
{
public:
  bool isRunning;
  unsigned long endAt;
  unsigned long openValveAt;
  bool shouldStart;
  unsigned long showDuration;
  unsigned long showNachlauf;
  ShowStatus() : isRunning(false), endAt(0), openValveAt(0), shouldStart(false), showDuration(SHOW_LENGTH), showNachlauf(SHOW_NACHLAUF) {}
};

ShowStatus showStatus = ShowStatus();

void setup()
{
  //pinMode(TASTER, INPUT_PULLUP);
  pinMode(PUMPE1, OUTPUT);
  pinMode(PUMPE2, OUTPUT);
  pinMode(VENTIL, OUTPUT);
  digitalWrite(PUMPE1, LOW);
  digitalWrite(PUMPE2, LOW);
  digitalWrite(VENTIL, LOW);
  delay(200);
  button.begin();
  /*

  digitalWrite(D5, HIGH);
  delay(5000);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  delay(5000);
  digitalWrite(D6, LOW);
  digitalWrite(D7, HIGH);
  delay(5000);
  digitalWrite(D7, LOW);
*/
}

void loop()
{
  
  button.read();
  if (button.wasPressed()) {
    if (showStatus.isRunning){
      stopShow();
    } else {
      startShow();
    }
  }
  handleShow();
}

void startShow()
{
  if (showStatus.isRunning)
  {
    return;
  }
  showStatus.shouldStart = true;
}

void stopShow()
{
  if (!showStatus.isRunning)
    return;
  showStatus.endAt = millis()+10;
  showStatus.openValveAt = millis()+10;
}

void handleShow()
{
  if (showStatus.shouldStart)
  {
    showStatus.endAt = millis() + showStatus.showDuration;
    showStatus.openValveAt = showStatus.endAt + showStatus.showNachlauf;
    showStatus.shouldStart = false;
    showStatus.isRunning = true;
  }

  if (showStatus.isRunning)
  {
    if (millis() < showStatus.endAt)
    {
      digitalWrite(PUMPE1, HIGH);
      digitalWrite(PUMPE2, HIGH);
      digitalWrite(VENTIL, HIGH);
    }
    else
    {
      if (millis() < showStatus.openValveAt)
      {
        digitalWrite(PUMPE1, LOW);
        digitalWrite(PUMPE2, LOW);
        digitalWrite(VENTIL, HIGH);
      }
      else
      {
        digitalWrite(PUMPE1, LOW);
        digitalWrite(PUMPE2, LOW);
        digitalWrite(VENTIL, LOW);
        showStatus.isRunning = false;
      }
    }
  }
  else
  {
    digitalWrite(PUMPE1, LOW);
    digitalWrite(PUMPE2, LOW);
    digitalWrite(VENTIL, LOW);
    showStatus.isRunning = false;
  }
}