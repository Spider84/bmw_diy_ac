#include <arduino-timer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define MIN_TEMP   220 //1.00V
#define WORK_TEMP  260 //1.20V

#define MIN_PRESS  220 //3.5V
#define WORK_PRESS 750
#define MAX_PRESS  760 //3.5V

#define FAM_TH     10  //10%

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define AC_RELAY  A0
#define FAN_RELAY A1
#define AC_LED    A2
#define BUTTON    A3

#define TEMP     A6
#define PRESS    A7

bool led_blink(void *);
bool every_second(void *);

auto timer = timer_create_default();
auto t_led_blink = timer_create_default();

uint8_t pwm_duty = 0;
unsigned int RunningAverageTemperature = 0;
unsigned int RunningAveragePress = 0;
unsigned int RunningAverageFAN = 0;
bool ac_is_on = false;
bool good_press = true;
bool good_temp = true;

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(AC_RELAY, HIGH);
  digitalWrite(FAN_RELAY, HIGH);
  digitalWrite(AC_LED, LOW);
  digitalWrite(BUTTON, HIGH);
  pinMode(AC_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(AC_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);  
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  t_led_blink.every(500, led_blink);
  timer.every(1000, every_second);

  //Таймер для генерации собственного ШИМ
  pinMode(3, OUTPUT);
  TCNT2 = 0;
  OCR2A = 250;
  OCR2B = 220;
  TCCR2A =(1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<< WGM20);
  TCCR2B = (1<<WGM22) | (1<<CS22);
  TIFR2 = (1<<TOV2);
  TIMSK2 = 0;
  //Таймер для рассчёта частоты с ЕКУ
  TCNT1 = 0;
  TCCR1A = 0;
  TIFR1 = (1<<ICF1);    /* clear input capture flag */
  TIMSK1 |= (1<<ICIE1);
  TCCR1B = 0x41;       /* capture on rising edge */ 

  sei(); // interrupts on.

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    // Clear the buffer
    display.clearDisplay();    
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text    
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    // Draw a single pixel in white
    display.drawPixel(10, 10, SSD1306_WHITE);
    display.display();
  }
}

void loop() {
  timer.tick();
  t_led_blink.tick();
  ProcessADC();  
  CheckBUTTON();
  checkFANrelay();
  checkTemp();
  checkPress();
  
  if (ac_is_on && good_press && good_temp) {
    digitalWrite(AC_RELAY, LOW);
  } else {
    digitalWrite(AC_RELAY, HIGH);
  }

  if (Serial.available()>0) {
    byte ch = Serial.read();
    if (ch == '1'){
      ac_is_on = true;
    } else
    if (ch == '0') {
      ac_is_on = false;
    }
  }
}

void CheckBUTTON(void)
{
  static byte b_state = 0;
  static unsigned long b_time;
  switch (b_state) {
    default:
      b_state = 0;
    case 0:
      if (digitalRead(BUTTON)==LOW) { //Надали на кнопку
        b_state=1;
        b_time = millis();
      }
      break;
    case 1:
      if (millis()-b_time>=50) {
        if (digitalRead(BUTTON)==LOW) //Всё ещё нажата
        { 
          b_state=2;
        } else 
        {
          b_state=0;
        }
      }
      break;
    case 2: //Событие по нажатию
      ac_is_on = !ac_is_on;
      changeACState();
      b_state = 3;
      break;
    case 3:
      if (digitalRead(BUTTON)!=LOW) //Отпустил
      {
        b_state = 0;
      }
      break;
  }
}

void changeACState(void)
{
  t_led_blink.cancel();
  Serial.print("AC switched ");
  if (ac_is_on) {    
    digitalWrite(AC_LED, HIGH);    
    Serial.println("ON");
    t_led_blink.every(100, led_blink);
  } else {
    digitalWrite(FAN_RELAY, HIGH);
    digitalWrite(AC_RELAY, HIGH);
    digitalWrite(AC_LED, LOW);    
    Serial.println("OFF");    
    t_led_blink.every(500, led_blink);
  }
}

void checkFANrelay(void)
{
  if (ac_is_on) {
    if (pwm_duty>FAM_TH) {
      digitalWrite(FAN_RELAY, HIGH);
    } else
    {
      digitalWrite(FAN_RELAY, LOW);
    }
  }
}

void checkTemp(void)
{    
    if (RunningAverageTemperature>=MIN_TEMP) {
      good_temp = false;
    } else 
    if (RunningAverageTemperature<=WORK_TEMP) {
      good_temp = true;
    }
}

void checkPress(void)
{
    if ((RunningAveragePress>=MAX_PRESS) || (RunningAveragePress<=MIN_PRESS)) {
      good_press = false;
    } else 
    if (RunningAveragePress<=WORK_PRESS) {
      good_press = true;
    }
}

void ProcessADC(void)
{
  const byte t_count = 16, p_count = 16;
  static word buffer_temp[t_count], buffer_press[p_count];
  static byte n_temp=0, n_press=0;
 
  word a_temp = analogRead(TEMP);
  word a_press = analogRead(PRESS);

  buffer_temp[n_temp++] = a_temp;
  if (n_temp >= t_count)
  {
    n_temp = 0; 
  }  
  for(int i=0; i< t_count; ++i)
  {
    RunningAverageTemperature += buffer_temp[i];
  }
  RunningAverageTemperature /= t_count;

  buffer_press[n_press++] = a_press;
  if (n_press >= p_count)
  {
    n_press = 0; 
  }  
  for(int i=0; i< p_count; ++i)
  {
    RunningAveragePress += buffer_press[i];
  }
  RunningAveragePress /= p_count;
}

bool led_blink(void *argument)
{
  static bool led = false;
  if (led) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  led = !led;
}

bool every_second(void *argument /* optional argument given to in/at/every */) {
    display.clearDisplay();    
    display.setCursor(0, 0);     // Start at top-left corner
    display.write((ac_is_on)?"ON":"OFF");
    display.write(" ");
    display.write(RunningAverageTemperature);
    display.write(" ");
    display.write(RunningAveragePress);
    display.write("\n");
    display.write((good_press)?"1":"0");
    display.write(" ");
    display.write((good_temp)?"1":"0");
    display.write(" ");
    display.write(pwm_duty);    
    display.display();
  
    Serial.print("Temp: ");
    Serial.print(RunningAverageTemperature);
    Serial.print(", Press: ");
    Serial.print(RunningAveragePress);
    Serial.print(", ");
    Serial.print((ac_is_on)?"ON":"OFF");
    Serial.print(", ");
    Serial.println(pwm_duty);    
    
    return true; // to repeat the action - false to stop
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = 5;
}

ISR(TIMER1_CAPT_vect) // input capture interrupt
{  
  static uint16_t fallTime, riseTime;
  TCNT1 = 0; // clear the timer counter
  TIFR1 = (1<<ICF1);  
  if (TCCR1B & (1<<ICES1)) // rising edge detected
  {      
      TCCR1B &= ~(1 << ICES1);      
      riseTime = ICR1;
      pwm_duty=((uint32_t)riseTime*100U)/(fallTime+riseTime);
    
    const byte f_count = 16;
    static word buffer_fan[f_count];
    static byte n_fan=0;

    buffer_fan[n_fan++] = pwm_duty;
    if (n_fan >= f_count)
    {
        n_fan = 0;
    }
    for(int i=0; i< f_count; ++i)
    {
        RunningAverageFAN += buffer_fan[i];
    }
    RunningAverageFAN /= f_count;
  }
  else // falling edge detected
  {
      TCCR1B |= (1 << ICES1);
      fallTime = ICR1;
  }
}
