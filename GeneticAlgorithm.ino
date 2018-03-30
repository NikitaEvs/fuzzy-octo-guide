/*
 * Программа реализации генетического алгоритма для оптимизации работы мультиагентной системы
 */
/* Используемые библиотеки */
#include <avr/interrupt.h> // Подключение библиотеки прерываний
#include <time.h> // Подключение библиотеки для псевдослучайных чисел

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
// ПИНы UART (RX/TX)
#define pinRX PIND0
#define pinTX PIND1

#define pinOCLeft PINB3 // Левая оптопара
#define pinOCRight PINB4 // Правая оптопара
/**********************/
#define LEDON PIND |= (1<<PIND7) // Включение светодиодов
#define LEDOFF PIND &= (~(1<<PIND7)) // Выключение светодиодов
/**********************/
#define BAUD 9600 // Скорость обмена данными
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1) // Для регистра настройки UART
#define PWM 800 // Верхний предел для ШИМ
/* Константы для генетического алгоритма */
#define genCount 10 // Количество "особей" в поколении
#define valCount 3 // Количество "характеристик" у каждой "особи"
#define maxOne 700 // Максимальные значения для характеристик особей
#define maxTwo 800
#define maxThree 800
#define chanceForMutation 20 // Шанс мутации (в процентах)
/*****************************************/
unsigned long timeLast = 0; // Переменная для хранения времени момента запуска прохода алгоритма
unsigned long timeNow = 0; // Переменная для хранения времени момента конца прохода алгоритма
bool flgMOVE = false; // Флаг для начала прохода
/* Переменные для генетического алгоритма */
/**
 * Одно поколение, характеризуют следующие свойства:
 * 0) Задержка при смене ведущего мотора
 * 1) Скорость ведущего мотора
 * 2) Скорость другого мотора
 */
int generationNOW[genCount][valCount];
int generationFUTURE[genCount][valCount];
/* Массив для значений fitness каждой "особи" */
int fitVal[genCount];
/* Массив для вероятностей окозаться "родителем" для каждой "особи" */ 
int parVal[genCount];
/* Родители прошлой особи */
int lastParOne = -1;
int lastParTwo = -1; 
/******************************************/
/**
 * Прерывание на приём информации с Bluetooth  
 * Необходимость Bluetooth заключается в упрощении прогона генетического алгоритма
 */
ISR (USART_RX_vect){
  cli(); // Глобальный запрет прерываний
  LEDON;
  while(!(UCSR0A&(1<<RXC0)));
  switch(UDR0){ // Действия в зависимости от принятого байта UDR0
    case 's':{ // Начало прогона для вычисления fitness ("коэффициент приспособленности") каждой "особи"
        /* Начало движения и сохранение времени начала */
        flgMOVE = true;
        timeLast = millis();
        sendUART('s');
    }
    break;
    case 'f':{ // Конец прогона 
        /* Конец движения и сохранение времени конца */
        flgMOVE = false;
        timeNow = millis();
        sendUART('f');
    }
    break;
  }
  LEDOFF;
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
 * Функция для настройки UART для bluetooth
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
 * Функция для отправки данных по Bluetooth
 * Принимаемое значение:
 * 1) Символ для отправки
 */
void sendUART(char data){
  while(!(UCSR0A&(1<<UDRE0)));
  UDR0 = data;
}
/**
 * Функция для отправки строки, используя вспомогательную функцию sendUART
 * Принимаемые значения:
 * 1) Символьный массив для отправки
 * 2) Размер массива
 */
void send(const char *data){
  while(*data != 0x00){
    sendUART(*data);
    data++;
  }
  sendUART('\n');
}
void send(int data){
  char str[10];
  sprintf(str, "%d", data);
  send(str);
}
void send(double data){
  char str[100];
  dtostrf(data, 5, 3, str);
  send(str);
}
/**
 * Функция для отправки характеристики поколения по UART
 * Принимаемые значения:
 * 1) Матрица поколения
 */
void sendGeneration(int generation[genCount][valCount]){
  send("***GENERATION***");
  for(int i = 0; i < genCount; i++){
    send("***START***");
    send(i+1);
    for(int j = 0; j < valCount; j++){
      send(generation[i][j]);
      send("-");
    }
    send("***END***");
  }
  send("***END_OF_GENERATION***");
}
/*********************************/
/*Функции генетического алгоритма*/
/*********************************/
/**
 * Функция для генерации первого поколения
 * Принимаемые значения:
 * 1) Максимальные значения для различных параметров
 */
void generate(int max1, int max2, int max3){
  for(int i = 0; i < genCount; i++){
    for(int j = 0; j < valCount; j++){
      switch(j){ // Блок для выбора максимального значения в зависимости от параметра
        case 0:{
          generationNOW[i][j] = random(max1); 
        }
        break;
        case 1:{
          generationNOW[i][j] = random(max2);
        }
        break;
        case 2:{
          generationNOW[i][j] = random(max3); 
        }
        break;
      }
    }
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
 * Функция для прогона генетического алгоритма
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
  send("3");
  /* Движение робота */
  while(flgMOVE){
    send("SI: ");
    send(speedD);
    send(speedR);
    do{
      runLeft(speedD, 1);
      runRight(speedR, 1);
    } while(!(getOCLeft())&&flgMOVE);
    if(!flgMOVE){
      break;
    }
    delay(del);
    do{
      runLeft(speedR, 1);
      runRight(speedD, 1);
    } while(!(getOCRight())&&flgMOVE);
    delay(del);
  }
}
/**
 * Функция для определения вероятности для каждой особи в поколении стать "родителем"
 * Принимаемые значения:
 * 1) Массив со значениями fitness для каждой особи
 * Возвращаемые значени:
 * 1) Массив со значениями вероятности для каждой особи
 */
double *getParentsProb(int valFit[]){
  double *probParents = new double[genCount]; // Массив с вероятностями "особи" стать родителем
  double revValFit[genCount]; // Массив с обратными значениями fitness
  double sumRevVal = 0; // Сумма обратных значений fitness
  for(int i = 0; i < genCount; i++){
    revValFit[i] = 1.0/(valFit[i]*1.0);
    sumRevVal += revValFit[i];
  }
  for(int i = 0; i < genCount; i++){
    probParents[i] = revValFit[i] / sumRevVal;
  }
  return probParents;
}
/**
 * Функция для определения родителей для новой особи исходя из вероятности
 * Принимаемые значения:
 * 1) Массив с вероятностями
 * Возвращаемое значение:
 * 1) Массив с родителями
 */
int *getParents(double probParents[]){
  int probVal[genCount]; // Массив с расположением вероятностей, переведенные в проценты
  int *parents = new int[2]; // Выбранные родители
  probVal[0] = probParents[0]*100;
  for(int i = 1; i < genCount; i++){
    probVal[i] = probParents[i]*100 + probVal[i-1];
    send(probVal[i]);
  }
  do{
    randomSeed(analogRead(1));
    int a = random(100); // Псевдослучайное число для определения первого родителя
    int b = a;
    while(b==a){
      b = random(100); // Псевдослучайное число для определния второго родителя
    }
    if(a<probVal[0]){
      parents[0] = 0;
    } else if(a<probVal[1]){
      parents[0] = 1;
    } else if(a<probVal[2]){
      parents[0] = 2;
    } else if(a<probVal[3]){
      parents[0] = 3;
    } else {
      parents[0] = 4;
    }
    if(b<probVal[0]){
      parents[1] = 0;
    } else if(b<probVal[1]){
      parents[1] = 1;
    } else if(b<probVal[2]){
      parents[1] = 2;
    } else if(b<probVal[3]){
      parents[1] = 3;
    } else {
      parents[1] = 4;
    }
  } while ((parents[0] == parents[1])||(((parents[0]==lastParOne)||(parents[0]==lastParTwo))&&((parents[1]==lastParOne)||(parents[1]==lastParTwo))));
  lastParOne = parents[0];
  lastParTwo = parents[1];
  send("get");
  return parents;
}
/**
 * Функция для скрещивания двух родителей (кроссинговер)
 * Принимаемые значения:
 * 1) Характеристики двух родителей
 * Возвращаемое значение:
 * 1) Характеристика потомка
 */
int *crossingover(int parentOne[valCount], int parentTwo[valCount]){
  randomSeed(analogRead(1));
  int r = random(2); // Псевдослучайное число для разных вариантов скрещивания
  int *child = new int[valCount]; // Характеристика потомка
  if(r == 1){
    child[0] = parentOne[0];
    child[1] = parentTwo[1];
    child[2] = parentTwo[2];
  } else {
    child[0] = parentTwo[0];
    child[1] = parentOne[1];
    child[2] = parentOne[2];
  }
  return child;
}
/**
 * Функция для мутации особи
 * Принимаемое значение:
 * 1) Характеристики особи
 * Возвращаемое значение:
 * 1) Измененные характеристики особи
 */
int *mutation(int val[valCount]){
  randomSeed(analogRead(1));
  int r = random(valCount); // Псевдослучайное число, которое определяем, какая из характеристик будет мутировать
  switch(r){
    case 0: 
      val[r] = random(maxOne);
    break;
    case 1: 
      val[r] = random(maxTwo);
    break;
    case 2: 
      val[r] = random(maxThree);
    break;
  }
  return val;
}
/*********************************/
void setup() {
  /* Изначальные настройки */
  DDRD &=(~(1<<PD0)); // Пин RX - вход
  presetPWM();
  presetPIN();
  presetUART();
  randomSeed(analogRead(0)); // Настройка генератора псевдослучайных чисел
  /* Глобальное разрешение прерываний */
  sei();
  /* Настройки для генетического алгоритма */
  generate(maxOne, maxTwo, maxThree);
}

void loop() {
  if(flgMOVE){
    send("1");
    /* Проба поколения и получения fitness'а */
    for(int i = 0; i < genCount; i++){
      send("2");
      go(generationNOW[i]);
      send("OK");
      fitVal[i] = timeNow - timeLast;
      while(!flgMOVE){
        stop();
        send("Wait for start again");
        _delay_ms(1000);
      }
    }
    stop();
    double *probParents; 
    send(6);
    probParents = getParentsProb(fitVal);
    send(7);
    /* Скрещивание особей и получение нового поколения */ 
    for(int k = 0; k < genCount; k++){
      send(8);
      int *child; // Характеристики потомка
      int parentOne[valCount]; // Характеристики родителей
      int parentTwo[valCount];
      int *parents; // Родители
      parents = getParents(probParents);
      send("Parents");
      send(parents[0]);
      send(parents[1]);
      send("**********");
      for(int j = 0; j < valCount; j++){
        parentOne[j] = generationNOW[parents[0]][j];
        parentTwo[j] = generationNOW[parents[1]][j];
      }
      child = crossingover(parentOne, parentTwo);
      send(10);
      for(int j = 0; j < valCount; j++){
        generationFUTURE[k][j] = child[j];
      }
    }
    int r = random(100)/chanceForMutation; // Псевдослучайное число, которое определяет возможность мутации
    if(r == 0){
      int numb = random(genCount); // Псевдослучайная особь мутирует
      int *temp;
      temp = mutation(generationFUTURE[numb]);
      for(int i = 0; i < valCount; i++){
        generationFUTURE[numb][i] = temp[i];
      }
    }
    send(11);
    memcpy(generationNOW, generationFUTURE, sizeof(generationNOW));
    sendGeneration(generationNOW);
  } else {
    stop();
    send("Wait");
    _delay_ms(1000);
  }
}
