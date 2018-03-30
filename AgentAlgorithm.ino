/*
 * Программа реализации алгоритма движения МАС
 */
/* Используемые библиотеки */
#include <avr/interrupt.h> // Подключение библиотеки прерываний
/* Используемые порты */
// Управление направлением для движения левого мотора
#define pinMLeftFor PIND5
#define pinMLeftBack PIND6
// Управление направлением для движения правого мотора
#define pinMRightFor PIND3
#define pinMRightBack PIND4
// ПИНы ШИМа на моторы
#define pinMRightPWM PINB1
#define pinMLeftPWM PINB2
// ПИНы UART (RX)
#define pinRX PIND0

#define pinOCLeft PINB3 // Левая оптопара
#define pinOCRight PINB4 // Правая оптопара
// Макросы для ИК диодов
#define LEDON PIND |= (1<<PIND7) // Включение светодиодов
#define LEDOFF PIND &= (~(1<<PIND7)) // Выключение светодиодов
/**********************/
#define PWM 800 // Верхний предел для ШИМ
/******************************************/
bool flgReceived = false; // Флаг приёма сигнала по UART
bool flgMOVE = false; // Флаг для начала прохода
/**
 * Параметры особи для движения, находятся из наиболее удачных особей в генетическом алгоритме
 * Характеризуют следующие свойства:
 * 0) Задержка при смене ведущего мотора
 * 1) Скорость ведущего мотора
 * 2) Скорость другого мотора
 */
int value[] = {500, 760, 530}; 
/**
 * Прерывание на приём информации с USART
 */
ISR (USART_RX_vect){
  cli(); // Глобальный запрет прерываний
  while(!(UCSR0A&(1<<RXC0)));
  flgReceived = true;
  sei();
}
/**
 * Функция для настройки ШИМа для моторов
 */
void presetPWM(){
  DDRB |= (1<<pinMRightPWM) | (1<<pinMLeftPWM); // Пины ШИМ каналов на моторы
  /*
   * Режим: Fast PWM
   * Положение TOP: ICR1
   * Mode: none-inverted 
   * Предделитель: 1
   */
  TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10);
  TCCR1B = (1<<CS10)|(0<<CS11)|(0<<CS12)|(1<<WGM13)|(1<<WGM12);
  ICR1 = PWM; // Верхний предел
  /* Установка значений */
  OCR1A = 500;
  OCR1B = 500;
}
/**
 * Функция для настройки ПИНов
 */
void presetPIN(){
  /* Настройка ПИНов моторов и светодиодов на выход */
  DDRD |= (1<<pinMLeftFor) | (1<<pinMLeftBack) | (1<<pinMRightFor) | (1<<pinMRightBack) | (1<<PIND7);
  /* Настройка ПИНов оптопар на вход */
  DDRB &= (~(1<<pinOCLeft) | (1<<pinOCRight));
  /* Обнуляем значения на ПИНах */
  PORTD &= (~(1<<pinMLeftFor) | (1<<pinMLeftBack) | (1<<pinMRightFor) | (1<<pinMRightBack) | (1<<PIND7);
  /* Подтягивающее сопротивление на порты оптопар */
  PORTB |= (1<<pinOCLeft) | (1<<pinOCRight);
}
/**
 * Функция для настройки UART для TSOP
 */
void presetUART(){
  /* Настройка ПИНов для UART */
  DDRD &= (~(1<<pinRX)); 
  DDRD |= (1<<pinTX);
  /* Настройка скорости обмена данными */
  UBRR0L = BAUDRATE;
  UBRR0H = (BAUDRATE >> 8);
  /* Конфигурация UART */
  UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
  UCSR0C = (3<<UCSZ00);
}
/**
 * Две аналогичные функции для установки скорости движения моторов
 * Вспомогательные функции для функций runRight, runLeft
 */
void setSpeedR(int speed){
  if(speed > PWM){
    speed = PWM;
  } else if(speed < 0){
    speed = 0;
  }
  OCR1B = speed;
}
void setSpeedL(int speed){
  if(speed > PWM){
    speed = PWM;
  } else if(speed < 0){
    speed = 0;
  }
  OCR1A = speed;
}

/**
 * Две аналогичные функции для локального управления моторами
 * На вход принимает два значения:
 * 1) Скорость для мотора, максимальное значение ограничено пределом ШИМа
 * 2) Направление движения (1 - вперёд, 0 - назад)
 */
void runRight(int speed, byte way){
  if(way == 1){
    PIND |= (1<<pinMRightFor);
    PIND &= (~(1<<pinMRightBack));
    setSpeedR(speed);
  } else {
    PIND &= (~(1<<pinMRightFor));
    PIND |= (1<<pinMRightBack);
    setSpeedR(speed);
  }
}
void runLeft(int speed, byte way){
  if(way == 1){
    PIND |= (1<<pinMLeftFor);
    PIND &= (~(1<<pinMLeftBack));
    setSpeedL(speed);
  } else {
    PIND &= (~(1<<pinMLeftFor));
    PIND |= (1<<pinMLeftBack);
    setSpeedL(speed);
  }
}
/**
 * Две аналогичные функции для снятия показания с оптопары
 * Возвращаемые значения:
 * 1 - есть препятствие
 * 0 - нет препятствия
 */
bool getOCLeft(){
  if((PINB)&(1<<pinOCLeft)){
    return false;
  } else {
    return true;
  }
}
bool getOCRight(){
  if((PINB)&(1<<pinOCRight)){
    return false;
  } else {
    return true;
  }
}
/**
 * Функция для полной остановки 
 */
void stop(){
  runLeft(0, 1);
  runRight(0, 1);
}
/**
 * Функция для следования за агентом
 * Принимаемые значения:
 * 1) Параметры "особи"
 */
 void go(int val[]){
  /* Параметры каждой "особи" */
  int del = val[0]; // Задержка перед переключением режима 
  int speedD = 0; // Скорость ведущего мотора
  int speedR = 0; // Скорость другого мотора
  if(val[1]<val[2]){
    speedR = val[1];
    speedD = val[2];
  } else {
    speedR = val[2];
    speedD = val[1];
  }
  /* Движение робота */
  while(flgMOVE&&flgReceived){
    do{
      runLeft(speedD, 1);
      runRight(speedR, 1);
    } while(!(getOCLeft())&&flgMOVE&&flgReceived);
    if(!flgMOVE){
      break;
    }
    delay(del);
    do{
      runLeft(speedR, 1);
      runRight(speedD, 1);
    } while(!(getOCRight())&&flgMOVE&&flgReceived);
    delay(del);
  }
}
/**
 * Функция для избегания препятствий
 * Принимаемые значения:
 * 1) Параметры "особи"
 */
void avoid(int val[]){
  /* Параметры каждой "особи" */
  int del = val[0]; // Задержка перед переключением режима 
  int speedD = 0; // Скорость ведущего мотора
  int speedR = 0; // Скорость другого мотора
  if(val[1]<val[2]){
    speedR = val[1];
    speedD = val[2];
  } else {
    speedR = val[2];
    speedD = val[1];
  }
  /* Движение робота */
  while(flgMOVE&&!flgReceived){
    do{
      runLeft(speedR, 1);
      runRight(speedD, 1);
    } while(!(getOCLeft())&&flgMOVE&&!flgReceived);
    if(!flgMOVE){
      break;
    }
    delay(del);
    do{
      runLeft(speedD, 1);
      runRight(speedR, 1);
    } while(!(getOCRight())&&flgMOVE&&!flgReceived);
    delay(del);
  }
}
void setup() {
  /* Изначальные настройки */
  DDRD &=(~(1<<PD0)); // Пин RX - вход
  presetPWM();
  presetPIN();
  presetUART();
  sei();
}

void loop() {
  if(getOCLeft()&&getOCRight()){
    runRight(PWM,0);
    runLeft(PWM,0);
    delay(1500);
  } else if(flgReceived){
    go(value);
  } else {
    avoid(value);
  }
}
