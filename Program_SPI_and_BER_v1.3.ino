/*
Данная программа реализует 5 операций:
- запись и чтение 8 бит по SPI в режиме Master с 8-битным адресом
- запись и чтение 8 бит по SPI в режиме Master с 16-битным адресом
- генерация PN-последовательности
- BER по принятой PN-последовательности
- управление двумя GPIO

***
Установка GPIO
- по умолчанию GPIO_1 (D5) и GPIO_2 (D6) в 0
- досупны два GPIO: GPIO_1 (D5) и GPIO_2 (D6)
- для задания значения GPIO_1 (D5) неодхиодмо подать 2 команды: 
  * 119 - собственно команда
  * 0 или 1 - значение, которое будет подано на GPIO_1
- для задания значения GPIO_2 (D6) неодхиодмо подать 2 команды: 
  * 136 - собственно команда
  * 0 или 1 - значение, которое будет подано на GPIO_1

***
Генератор PN
- выход данных D8
- выход тактового сигнала D9 (таймер 1, А)
- сами эталоны (периоды) PN последовательностей генерируются отдельными функциями
(можно при желании вывести в монитор порта последовательности) после чего
записываются в массив PNx[], откуда берутся при выдаче (экономия 30 % памяти
по сравнению с глобальными константами)
- для запуска подать 4 команды:
  * 85 - собственно команда
  * <3 или 4 или 5 или 7 или 9> - какую из PN3, PN4, PN5, PN7, PN9 выбираем
  * количество бит для передачи (до 2 000 000)
  * частота тактового сигнала выдачи - до 50 000 (далее МК не тянет)
- если после ввода первой команды (85) вводится отличное от 3, 4, 5, 7 или 9
 то команда сбрасывается и ее надо вводить заново
 
***
SPI запись и чтение
- тактовая частота по умолчанию 5 кГц (не задается)
- используемые выводы:
  * D10 - MOSI
  * D7 - MISO
  * D12 - CS (по умолчанию 1, падает в 0 при начале передачи и уходит в 1 по окончанию)
  * D11 - SCLK (таймер 2, А)
- перед активацией таймера запускается функция addr_data_PNx(), которая
записывает в массив PNx адрес и данные, которые потом будут выгружаться по 
прерыванию от таймера
- возможные команды:
  * 17, 34, 51 и 68
    . 68 - читаем по SPI с 8 бит адреса
    . 51 - записываем по SPI с 8 бит адреса
    . 34 - читаем по SPI с 16 бит адреса
    . 17 - записываем по SPI с 16 бит адреса
  * <8 бит адреса> (если 68 - выдает считанное значение по 8-битному адресу)
  * <8 бит адреса / 8 бит данных> (если  34 - выдает считанное значение по 16-битному адресу;
  если 51 - записывает введенные данные по введенному выше адресу)
  * <8 бит данных> (если 17 - записывает введенные данные по введенному выше адресу)

***
BER тест
- используемые выводы:
  * D4 - вход данных
  * D2 - тактовый сигнал данных (INT 0)
- перед разрешением прерывания запускается одна из функций PN_3, PN_4, PN_5, PN_7
или PN_9, которая в массиы PNx записывает эталон (период) ПСП, с которым будет
проводиться сравнение
- работает при входном тактовом сигнале 10 кГц
- выводится количество правильных и ошибочных бит, количество принятых 
для подсчета BER бит и общее количество бит, по которым сработали прерывания
- возможные команды:
  * 102 - собственно команда
  * <3 или 4 или 5 или 7 или 9> - какую из PN3, PN4, PN5, PN7, PN9 выбираем
  * количество бит для подсчета BER (до 2 000 000), период ПСП не входит, 
  биты считаюстя после захвата PN_

***
В версии v1.1 добавлено:
- дисплей OLED 128×64, на который выводится:
  * название проекта
  * версия проекта
  * команда, кторую надо отправить по UART для получения справки на дисплее
- команда 170 (0хAA) для вывода на дисплей команд  (10 сек)
- команда 153 (0x99) для установки скорости SCLK для SPI (по умолчанию 10000 Гц)
- убран вывод лишней информации по UART
- при BER тесте выводится только количество ошибочных бит и количество принятых 
для подсчета BER бит

***
В версии v1.3 добавлено:
* в команду 85 добавлена дополнительная команда 33
  - отправляется посылка 64 бита (по 16 бит преамбула, синхрослово, адрес и данные)
  - посылка задается в константах
* старший бит адреса при чтении по SPI устанволен в 1 - признак чтения
*/

#include <GyverOLED.h>                           //подключаем библиотеку для работы с дисплеем
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;  //подключаем дисплей без буфера

#include "GyverTimers.h"  //подключаем библиотеку дял работы с таймерами

//глобальный переменные для команд управления
uint32_t uart_cmd[] = { 0, 0, 0, 0, 0 };  //массив для хранения принятых по UART команд
byte uart_cmd_cnt = 0;                    //счетчик команд уарт

//переменные для управления GPIO
#define GPIO_1 5  // присваиваем имя GPIO_1 для пина D5
#define GPIO_2 6  // присваиваем имя GPIO_2 для пина D6

//переменные для генерации PNx
//boolean PN3_seq[] = { 1, 1, 0, 0, 1, 0, 1 };  //эталон периода PN3
const int PER_PN3 = 7;  //период PN3

/*boolean PN4_seq[] = { 1, 0, 0, 0, 1, 0, 0, 1,
                      1, 0, 1, 0, 1, 1, 1 };  //эталон периода PN4*/
const int PER_PN4 = 15;

/*boolean PN5_seq[] = { 1, 0, 0, 0, 1, 1, 0, 1, 1,
                      1, 0, 1, 0, 1, 0, 0, 0, 0,
                      1, 0, 0, 1, 0, 1, 1, 0, 0,
                      1, 1, 1, 1 };  //эталон периода PN5*/
const int PER_PN5 = 31;

/*boolean PN7_seq[] = { 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
                      0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1,
                      1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0,
                      0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1,
                      0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1,
                      0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1,
                      0, 1, 0, 1, 1, 1, 1 };  //эталон периода PN7*/
const int PER_PN7 = 127;

/*boolean PN9_seq[] = { 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0,
                      0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,
                      0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1,
                      1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0,
                      1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1,
                      0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1,
                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,
                      0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1,
                      0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1,
                      0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1,
                      0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0,
                      0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0,
                      1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
                      0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
                      0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1,
                      0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1,
                      1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0,
                      0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0,
                      0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1,
                      0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1,
                      1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                      1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0,
                      1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1,
                      0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1,
                      1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1 };*/
const int PER_PN9 = 511;

const uint16_t SEQ_PREAMBLE = 0xAAAA;   //преамбула
const uint16_t SEQ_SYNC_WORD = 0x91DA;  //синхрослово
const uint16_t SEQ_ADDR_WORD = 0x3487;  //адрес
const uint16_t SEQ_DATA_WORD = 0xCCCC;  //данные
const byte SEQ_BYTE_CNT = 4;            //сколько 16-битных чисел в последовательности

int PER_PN = 511;  //переменная, которая определяет период PN
//делаем пустой массив, из которого будут выводиться биты PN
//длиной как максимальный эталон PN
boolean PNx[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//для формирования my_seq - для отправки пакета, собранного из констант:
int PN_cnt_i = 0;                      //счетчик бит последовательности
int PN_cnt_j = 0;                      //счетчик прерываний
uint32_t PN_total_bits = 2 * 10;       //2 * сколько бит передаем
uint32_t PN_clk_freq_clk_freq = 1000;  //частота клока для выдачи PNx
#define PN_DATA 8                      // присваиваем имя PN_DATA для пина D8 - выход данных PNx

//переменные для SPI
//массив даных PNx[] будет использоваться от генерации PNx
byte SPI_prm;  // переменная для считывания по MISO
boolean SPI_read[] = { 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0 };  //что читаем по SPI (24 бита)

int SPI_cnt_byte;                 //переменная для заполнения массива - счетчик количества байт
uint32_t SPI_total_bits = 16;     //сколько бит передаем
uint32_t SPI_total_isr = 2 * 16;  //2 * сколько бит передаем
int SPI_isr_i = 0;                //счетчик бит последовательности
int SPI_isr_j = 0;                //счетчик прерываний
#define SPI_MOSI 10               // присваиваем имя SPI_MOSI для пина 10
#define SPI_MISO 7                // присваиваем имя SPI_MISO для пина 7
#define SPI_CS 12                 // присваиваем имя SPI_CS для пина 12

//переменные для BER теста

volatile boolean BER_interrupt = 0;  //флаг прерывания
boolean BER_data_read;               //сюда кладется пришедший бит
boolean BER_flag_lock_PN = 0;        //флаг захвата одного периода PN_
uint32_t BER_total_bits = 20;        //сколько бит анализируем после захвата
uint32_t BER_cnt = 0;                //счетчик срабатываний для перемещения по эталону
uint32_t BER_cnt_lock = 0;           //на каком бите произошел захват
uint32_t BER_cnt_all = 0;            //счетчик срабатываний общий
uint32_t BER_cnt_error = 0;          //счетчик срабатываний ошибочных бит
uint32_t BER_cnt_norm = 0;           //счетчик срабатываний корректных бит
boolean BER_shift_reg[PER_PN9];      //сдвиговый регистр размером PN9
//volatile boolean BER_shift_reg[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //эталон периода PN3

#define BER_DATA 4  // присваиваем имя BER_DATA для пина 4 - вход данных

void setup() {
  // выставляем режимы работы пинов GPIO
  pinMode(GPIO_1, OUTPUT);
  pinMode(GPIO_2, OUTPUT);

  // выставляем значения GPIO по умолчанию
  digitalWrite(GPIO_1, 0);
  digitalWrite(GPIO_2, 0);

  // выставляем режимы работы пинов для выдачи PNx
  pinMode(9, OUTPUT);        //pin9 это Channel_A для таймера1
  pinMode(PN_DATA, OUTPUT);  //выход данных

  //настраиваем таймер1 для PNx
  Timer1.setFrequency(100);     //частота в Гц 16 бит тамер1, можно до 50 кГц
  Timer1.enableISR(CHANNEL_A);  //включаем прерывание
  Timer1.outputState(CHANNEL_A, LOW);
  Timer1.stop();  //останавливаем таймер1, ждем команду на старт
  digitalWrite(PN_DATA, LOW);

  // выставляем режимы работы пинов для SPI
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_CS, OUTPUT);
  pinMode(11, OUTPUT);  //pin11 это Channel_A для таймера2 и SPI_SCLK

  //настраиваем таймер2 для SPI
  Timer2.setFrequency(10000);   //частота в Гц 16 бит тамер1, можно до 50 кГц
  Timer2.enableISR(CHANNEL_A);  //включаем прерывание
  Timer2.outputState(CHANNEL_A, LOW);
  Timer2.stop();  //останавливаем таймер2, ждем команду на старт

  //установка выводов SPI в исходные значения
  digitalWrite(SPI_MOSI, 0);           // выход - MOSI
  digitalWrite(SPI_MISO, 0);           // вход  - MISO
  Timer2.outputState(CHANNEL_A, LOW);  // выход - SCLK
  digitalWrite(SPI_CS, 1);             // выход - CS

  //выставляем режим работы пинов для BER теста
  pinMode(BER_DATA, INPUT_PULLUP);  //дотяжка до 1

  //инициализация ПО завершена
  Serial.begin(9600);
  Serial.setTimeout(50);
  //Serial.println("Program_SPI_and_BER_v1");

  //работа с дисплеем
  oled.init();           // инициализация
  oled.clear();          // очистка
  oled.setScale(1);      // масштаб текста (1..4)
  oled.home();           // курсор в 0,0
  oled.setCursor(0, 2);  // курсор на начало 2 строки
  oled.print("Program_SPI_and_BER");
  oled.setCursor(0, 3);  // курсор на начало 3 строки
  oled.print("v1.3");
  oled.setCursor(0, 7);  // курсор на начало 7 строки
  oled.print("0xAA - for help");
  delay(1000);


  attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 (D2) по фронту
  detachInterrupt(0);                         //отключаем прерывание по по INT0 (D2)
}

void loop() {

  if (Serial.available() > 0) {
    String uart_read = Serial.readString();      //сюда считываем
    uart_read.trim();                            //удалили ненужные симвлы в конце строки
    uart_cmd[uart_cmd_cnt] = uart_read.toInt();  //записали преобразованные в int данные из принятого типа string
    //Serial.println(uart_cmd[uart_cmd_cnt]);

    //если команда 119, то ждем 1 или 0 по UART чтобы переключить GPIO_1 (D5)
    if ((uart_cmd[0] == 119) && (uart_cmd_cnt == 1)) {  //0x77 GPIO_1, D5
      if ((uart_cmd[1] == 0) || (uart_cmd[1] == 1)) {
        //Serial.println(uart_cmd[1]);
        digitalWrite(GPIO_1, uart_cmd[1]);
        uart_cmd[0] = 0;  //обнуляем команду
      }
      uart_cmd_cnt = 0;  //обнуляем счетчик
    }
    //если команда 136, то ждем 1 или 0 по UART чтобы переключить GPIO_2 (D6)
    if ((uart_cmd[0] == 136) && (uart_cmd_cnt == 1)) {  //0x88 GPIO_2, D6
      if ((uart_cmd[1] == 0) || (uart_cmd[1] == 1)) {
        //Serial.println(uart_cmd[1]);
        digitalWrite(GPIO_2, uart_cmd[1]);
        uart_cmd[0] = 0;  //обнуляем команду
      }
      uart_cmd_cnt = 0;  //обнуляем счетчик
    }
    //если команда 85, то ждем тип PN, количество бит PN_total_bits и скорость PN_clk_freq
    if ((uart_cmd[0] == 85) && (uart_cmd_cnt == 1)) {  //тип PN
      if ((uart_cmd[1] == 3) || (uart_cmd[1] == 4) || (uart_cmd[1] == 5)
          || (uart_cmd[1] == 7) || (uart_cmd[1] == 9) || (uart_cmd[1] == 33)) {
        //Serial.println(uart_cmd[1]);
      } else {
        uart_cmd[0] = 0;   //обнуляем команду
        uart_cmd_cnt = 0;  //обнуляем счетчик
      }
    }
    if ((uart_cmd[0] == 85) && (uart_cmd_cnt == 2)) {  //количество бит PN_total_bits
      //Serial.println(uart_cmd[2]);
      PN_total_bits = 2 * uart_cmd[2];  //чтобы выдать введенное количество а не в 2 раза меньше
    }
    if ((uart_cmd[0] == 85) && (uart_cmd_cnt == 3)) {  //скорость PN_clk_freq

      if (uart_cmd[1] == 3) {  //выбрана PN3
        PN_3();                //загружает в массив PNx эталон сгенерированной PN3
        PER_PN = PER_PN3;      //период PN3
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN3_seq[k];
        }*/
      }
      if (uart_cmd[1] == 4) {  //выбрана PN4
        PN_4();                //загружает в массив PNx эталон сгенерированной PN4
        PER_PN = PER_PN4;      //период PN4
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN4_seq[k];
        }*/
      }
      if (uart_cmd[1] == 5) {  //выбрана PN5
        PN_5();                //загружает в массив PNx эталон сгенерированной PN5
        PER_PN = PER_PN5;      //период PN5
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN5_seq[k];
        }*/
      }
      if (uart_cmd[1] == 7) {  //выбрана PN7
        PN_7();                //загружает в массив PNx эталон сгенерированной PN7
        PER_PN = PER_PN7;      //период PN7
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN7_seq[k];
        }*/
      }
      if (uart_cmd[1] == 9) {  //выбрана PN9
        PN_9();                //загружает в массив PNx эталон сгенерированной PN9
        PER_PN = PER_PN9;      //период PN9
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN9_seq[k];
        }*/
      }
      if (uart_cmd[1] == 33) {       //выбрана своя посылка (собирается из констант)
        clear_PNx();                 //обнуляем массив PNx
        my_seq();                    //загружает в массив PNx посылку, собранную из констант
        PER_PN = SEQ_BYTE_CNT * 16;  //бит в посылке
        /*for (int k = 0; k < PER_PN; k++) {
          PNx[k] = PN9_seq[k];
        }*/
      }
      //Serial.println(uart_cmd[3]);
      Timer1.setFrequency(uart_cmd[3]);
      uart_cmd[0] = 0;   //обнуляем команду
      uart_cmd_cnt = 0;  //обнуляем счетчик
      Timer1.restart();
    }

    //если команда: 68, то значит читаем по SPI с 8 бит адреса
    if ((uart_cmd[0] == 68) && (uart_cmd_cnt == 1)) {  // uart_cmd_cnt = 1 это  вводится адрес по которому читаем
      //Serial.println(uart_cmd[1]);
      uart_cmd[0] = 0;                     //обнуляем команду
      uart_cmd_cnt = 0;                    //обнуляем счетчик
      uart_cmd[1] = uart_cmd[1] + 128;     //для чтения надо 7й бит адреса в 1 поставить
      uart_cmd[4] = 68;                    //для таймера сохраняем команду
      addr_data_PNx();                     //записываем в массив PNx 16 бит адреса и 8 бит данных
      SPI_total_bits = 16;                 //количество бит
      SPI_total_isr = 2 * SPI_total_bits;  //количество прерываний для SPI_CLK
      Timer2.restart();
    }
    //если команда: 51, то значит записываем по SPI с 8 бит адреса
    if ((uart_cmd[0] == 51) && (uart_cmd_cnt == 2)) {  // uart_cmd_cnt = 2 это адрес и данные или адрес 16 бит
      //Serial.println(uart_cmd[1]);
      //Serial.println(uart_cmd[2]);
      uart_cmd[0] = 0;                     //обнуляем команду
      uart_cmd_cnt = 0;                    //обнуляем счетчик
      uart_cmd[4] = 51;                    //для таймера сохраняем команду
      addr_data_PNx();                     //записываем в массив PNx 16 бит адреса и 8 бит данных
      SPI_total_bits = 16;                 //количество бит
      SPI_total_isr = 2 * SPI_total_bits;  //количество прерываний для SPI_CLK
      Timer2.restart();
    }
    //если команда: 34 - читаем по SPI с 16 бит адреса
    if ((uart_cmd[0] == 34) && (uart_cmd_cnt == 2)) {  // uart_cmd_cnt = 2 это адрес и адрес 16 бит
      //Serial.println(uart_cmd[1]);
      //Serial.println(uart_cmd[2]);
      uart_cmd[0] = 0;                     //обнуляем команду
      uart_cmd_cnt = 0;                    //обнуляем счетчик
      uart_cmd[3] = 0;                     //для чтения MOSI в 0
      uart_cmd[1] = uart_cmd[1] + 128;     //для чтения надо 7й бит адреса в 1 поставить
      uart_cmd[4] = 34;                    //для таймера сохраняем команду
      addr_data_PNx();                     //записываем в массив PNx 16 бит адреса и 8 бит данных
      SPI_total_bits = 24;                 //количество бит
      SPI_total_isr = 2 * SPI_total_bits;  //количество прерываний для SPI_CLK
      Timer2.restart();
    }
    //если команда: 17, то значит записываем по SPI с 16 бит адреса
    if ((uart_cmd[0] == 17) && (uart_cmd_cnt == 3)) {  // uart_cmd_cnt = 3 это адрес 16 бит и данные
      //Serial.println(uart_cmd[1]);
      //Serial.println(uart_cmd[2]);
      //Serial.println(uart_cmd[3]);
      uart_cmd[0] = 0;                     //обнуляем команду
      uart_cmd_cnt = 0;                    //обнуляем счетчик
      uart_cmd[4] = 17;                    //для таймера сохраняем команду
      addr_data_PNx();                     //записываем в массив PNx 16 бит адреса и 8 бит данных
      SPI_total_bits = 24;                 //количество бит
      SPI_total_isr = 2 * SPI_total_bits;  //количество прерываний для SPI_CLK
      Timer2.restart();
    }
    //если команда: 102, то значит запускаем BER тест
    if ((uart_cmd[0] == 102) && (uart_cmd_cnt == 2)) {  // uart_cmd_cnt = 3 это адрес 16 бит и данные
      //Serial.println(uart_cmd[1]);
      //Serial.println(uart_cmd[2]);

      if (uart_cmd[1] == 3) {  //выбрана PN3
        PN_3();                //загружает в массив PNx эталон сгенерированной PN3
        PER_PN = PER_PN3;      //период PN3
        BER_total_bits = uart_cmd[2];
        attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 по фронту
      }
      if (uart_cmd[1] == 4) {  //выбрана PN4
        PN_4();                //загружает в массив PNx эталон сгенерированной PN4
        PER_PN = PER_PN4;      //период PN4
        BER_total_bits = uart_cmd[2];
        attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 по фронту
      }
      if (uart_cmd[1] == 5) {  //выбрана PN5
        PN_5();                //загружает в массив PNx эталон сгенерированной PN5
        PER_PN = PER_PN5;      //период PN5
        BER_total_bits = uart_cmd[2];
        attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 по фронту
      }
      if (uart_cmd[1] == 7) {  //выбрана PN7
        PN_7();                //загружает в массив PNx эталон сгенерированной PN7
        PER_PN = PER_PN7;      //период PN7
        BER_total_bits = uart_cmd[2];
        attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 по фронту
      }
      if (uart_cmd[1] == 9) {  //выбрана PN9
        PN_9();                //загружает в массив PNx эталон сгенерированной PN9
        PER_PN = PER_PN9;      //период PN9
        BER_total_bits = uart_cmd[2];
        attachInterrupt(0, BER_take_data, RISING);  //прерывание по INT0 по фронту
      }

      uart_cmd[0] = 0;   //обнуляем команду
      uart_cmd_cnt = 0;  //обнуляем счетчик
    }

    //если команда 153, то ждем по UART частоту SCLK для SPI
    if ((uart_cmd[0] == 153) && (uart_cmd_cnt == 1)) {  //0x99
      if (uart_cmd[1] < 50000) {
        //Serial.println(uart_cmd[1]);
        digitalWrite(GPIO_2, uart_cmd[1]);
        Timer2.setFrequency(uart_cmd[1]);  //частота в Гц 16 бит тамер1, можно до 50 кГц
        uart_cmd[0] = 0;                   //обнуляем команду
      }
      uart_cmd_cnt = 0;  //обнуляем счетчик
    }

    switch (uart_cmd[0]) {  //проверяем по нулевому индексу
      case 17:              //0х11 SPI 16 бит, запись
        {
          //Serial.println("SPI 16 бит запись");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 34:  //0х22 SPI 16 бит, чтение
        {
          //Serial.println("SPI 16 бит чтение");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 51:  //0х33 SPI 8 бит, запись
        {
          //Serial.println("SPI 8 бит запись");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 68:  //0х44 SPI 8 бит, чтение
        {
          //Serial.println("SPI 8 бит чтение");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 85:  //0х55 выдать PN с тактами
        {
          //Serial.print("Выдача PN ");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          //ждем тип PN, количество бит PN_total_bits и скорость PN_clk_freq
          return;
        }
      case 102:  //0х66 посчитать BER по PN
        {
          //Serial.println("BER тест");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 119:  //0х77 установка GPIO_1 (D5)
        {
          //Serial.print("GPIO_1: ");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 136:  //0х88 установка GPIO_2 (D6)
        {
          //Serial.print("GPIO_2: ");
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 153:  //0х99 установка частоты для SPI
        {
          uart_cmd_cnt = uart_cmd_cnt + 1;  //увеличиваем счетчик если приняли правильную команду
          return;
        }
      case 170:  //0хAA вывод справки на дисплей
        {
          oled_help();
          return;
        }
    }
  }
}

//таймер 1 исользуется для выдачи PNx
ISR(TIMER1_A) {
  if (PN_cnt_j == PN_total_bits) {  //останавливаем таймер1 когда выдали нужное количество бит
    digitalWrite(PN_DATA, LOW);
    Timer1.outputDisable(CHANNEL_A);
    Timer1.stop();
    PN_cnt_i = 0;  //обнуляем счетчики
    PN_cnt_j = 0;
    return;
  }
  Timer1.outputEnable(CHANNEL_A, TOGGLE_PIN);

  if ((PN_cnt_j % 2) == 0) {
    digitalWrite(PN_DATA, PNx[PN_cnt_i]);
    PN_cnt_i = PN_cnt_i + 1;
  }

  PN_cnt_j = PN_cnt_j + 1;

  if (PN_cnt_i == PER_PN) {  //обнуление счетчика
    PN_cnt_i = 0;
  }
}

void PN_3() {  //генерируем PN3 (период 7 бит)
  boolean m_1, m_2, m_3;
  int PN_index = 0;
  uint32_t etalon = 3;                       //стартовое значение для сдвигового регистра (определяет "фазу" PN)
  for (int j = 0; j < (2 * PER_PN3); j++) {  //делаем два периода по 7 бит
    m_2 = bitRead(etalon, 1);
    m_3 = bitRead(etalon, 2);
    m_1 = m_2 xor m_3;
    etalon = etalon << 1;
    bitWrite(etalon, 0, m_1);  //запись в число etalon (бит 0) значения xor
    //Serial.print(m_1);
    if (j > (PER_PN3 - 1)) {
      PNx[PN_index] = m_1;
      PN_index = PN_index + 1;
      if (PN_index == (2 * PER_PN3 - 1)) {
        PN_index = 0;
      }
    }
  }
  /*Serial.println();
  for (int j = 0; j < PER_PN3; j++) {
    Serial.print(PNx[j]);
  }*/
}
void PN_4() {  //генерируем PN4 (период 15 бит)
  boolean m_1, m_2, m_3;
  int PN_index = 0;
  uint32_t etalon = 3;                       //стартовое значение для сдвигового регистра (определяет "фазу" PN)
  for (int j = 0; j < (2 * PER_PN4); j++) {  //делаем два периода
    m_2 = bitRead(etalon, 2);
    m_3 = bitRead(etalon, 3);
    m_1 = m_2 xor m_3;
    etalon = etalon << 1;
    bitWrite(etalon, 0, m_1);  //запись в число etalon (бит 0) значения xor
    //Serial.print(m_1);
    if (j > (PER_PN4 - 1)) {
      PNx[PN_index] = m_1;
      PN_index = PN_index + 1;
      if (PN_index == (2 * PER_PN4 - 1)) {
        PN_index = 0;
      }
    }
  }
  /*Serial.println();
  for (int j = 0; j < PER_PN4; j++) {
    Serial.print(PNx[j]);
  }*/
}

void PN_5() {  //генерируем PN5 (период 31 бит)
  boolean m_1, m_2, m_3;
  int PN_index = 0;
  uint32_t etalon = 3;                       //стартовое значение для сдвигового регистра (определяет "фазу" PN)
  for (int j = 0; j < (2 * PER_PN5); j++) {  //делаем два периода
    m_2 = bitRead(etalon, 2);
    m_3 = bitRead(etalon, 4);
    m_1 = m_2 xor m_3;
    etalon = etalon << 1;
    bitWrite(etalon, 0, m_1);  //запись в число etalon (бит 0) значения xor
    //Serial.print(m_1);
    if (j > (PER_PN5 - 1)) {
      PNx[PN_index] = m_1;
      PN_index = PN_index + 1;
      if (PN_index == (2 * PER_PN5 - 1)) {
        PN_index = 0;
      }
    }
  }
  /*Serial.println();
  for (int j = 0; j < PER_PN5; j++) {
    Serial.print(PNx[j]);
  }*/
}

void PN_7() {  //генерируем PN7 (период 127 бит)
  boolean m_1, m_2, m_3;
  int PN_index = 0;
  uint32_t etalon = 3;                       //стартовое значение для сдвигового регистра (определяет "фазу" PN)
  for (int j = 0; j < (2 * PER_PN7); j++) {  //делаем два периода
    m_2 = bitRead(etalon, 5);
    m_3 = bitRead(etalon, 6);
    m_1 = m_2 xor m_3;
    etalon = etalon << 1;
    bitWrite(etalon, 0, m_1);  //запись в число etalon (бит 0) значения xor
    //Serial.print(m_1);
    if (j > (PER_PN7 - 1)) {
      PNx[PN_index] = m_1;
      PN_index = PN_index + 1;
      if (PN_index == (2 * PER_PN7 - 1)) {
        PN_index = 0;
      }
    }
  }
  /*Serial.println();
  for (int j = 0; j < PER_PN7; j++) {
    Serial.print(PNx[j]);
  }*/
}

void PN_9() {  //генерируем PN9 (период 511 бит)
  boolean m_1, m_2, m_3;
  int PN_index = 0;
  uint32_t etalon = 3;                       //стартовое значение для сдвигового регистра (определяет "фазу" PN)
  for (int j = 0; j < (2 * PER_PN9); j++) {  //делаем два периода
    m_2 = bitRead(etalon, 4);
    m_3 = bitRead(etalon, 8);
    m_1 = m_2 xor m_3;
    etalon = etalon << 1;
    bitWrite(etalon, 0, m_1);  //запись в число etalon (бит 0) значения xor
    //Serial.print(m_1);
    if (j > (PER_PN9 - 1)) {
      PNx[PN_index] = m_1;
      PN_index = PN_index + 1;
      if (PN_index == (2 * PER_PN9 - 1)) {
        PN_index = 0;
      }
    }
  }
  /*Serial.println();
  for (int j = 0; j < PER_PN9; j++) {
    Serial.print(PNx[j]);
  }*/
}

void my_seq() {  //формируем данные для посылки своей последовательности из констант

  for (int i = 0; i < 16; i++) {  //записываем посылку в PNx, остальное - нули
    PNx[i] = bitRead(SEQ_PREAMBLE, (15 - i));
    PNx[i + 16] = bitRead(SEQ_SYNC_WORD, (15 - i));
    PNx[i + 32] = bitRead(SEQ_ADDR_WORD, (15 - i));
    PNx[i + 48] = bitRead(SEQ_DATA_WORD, (15 - i));
  }
  /*for (int i = 0; i < 64; i++) {
    Serial.print(PNx[i]);
  }*/

  /*
const uint16_t SEQ_PREAMBLE = 0xAAAA;   //преамбула
const uint16_t SEQ_SYNC_WORD = 0x91DA;  //синхрослово
const uint16_t SEQ_ADDR_WORD = 0x3487;  //адрес
const uint16_t SEQ_DATA_WORD = 0xCCCC;  //данные
const byte SEQ_BYTE_CNT = 4;            //сколько 16-битных чисел в последовательности
    */
}

void clear_PNx() {  //обнуляем массив PNx
  for (int i = 0; i < PER_PN; i++) {
    PNx[i] = 0;
  }
}

void addr_data_PNx() {  //тут мы в массив PNx записываем адрес и данные для выдачи
  for (int i = 0; i < 8; i++) {
    PNx[i] = bitRead(uart_cmd[1], (7 - i));       //addr1 to PNx
    PNx[i + 8] = bitRead(uart_cmd[2], (7 - i));   //addr2 to PNx
    PNx[i + 16] = bitRead(uart_cmd[3], (7 - i));  //data to PNx
  }
  for (int j = 0; j < 24; j++) {
    //Serial.print(PNx[j]);
  }
  //Serial.println();
}

//таймер 2 исользуется для SPI
ISR(TIMER2_A) {
  if (SPI_isr_j == 0) {       //роняем CS в 0
    digitalWrite(SPI_CS, 0);  // выход - CS
  }
  if (SPI_isr_j == SPI_total_isr) {  //останавливаем таймер2 когда выдали нужное количество бит
    Timer2.stop();
    digitalWrite(SPI_MOSI, 0);           // выход - MOSI
    digitalWrite(SPI_MISO, 0);           // вход  - MISO
    Timer2.outputState(CHANNEL_A, LOW);  // выход - SCLK
    Timer2.outputDisable(CHANNEL_A);
    delayMicroseconds(50);
    digitalWrite(SPI_CS, 1);  // выход - CS
    SPI_isr_i = 0;            //обнуляем счетчики
    SPI_isr_j = 0;

    if ((SPI_total_bits == 16) && ((uart_cmd[4] == 51) || (uart_cmd[4] == 68))) {
      for (int k = 8; k < SPI_total_bits; k++) {  //Вывод считанных данных
        bitWrite(SPI_prm, (SPI_total_bits - k - 1), SPI_read[k]);
      }
      Serial.println(SPI_prm);
    }
    if ((SPI_total_bits == 24) && ((uart_cmd[4] == 17) || (uart_cmd[4] == 34))) {
      for (int k = 16; k < SPI_total_bits; k++) {  //Вывод считанных данных
        bitWrite(SPI_prm, (SPI_total_bits - k - 1), SPI_read[k]);
      }
      Serial.println(SPI_prm);
    }
    uart_cmd[4] = 0;
    return;  //выходим из прерывания и чтобы выход MOSI был в 0
  }

  Timer2.outputEnable(CHANNEL_A, TOGGLE_PIN);

  if ((SPI_isr_j % 2) == 0) {
    digitalWrite(SPI_MOSI, PNx[SPI_isr_i]);
    SPI_read[SPI_isr_i] = digitalRead(SPI_MISO);
    SPI_isr_i = SPI_isr_i + 1;
  }

  SPI_isr_j = SPI_isr_j + 1;
}

void BER_take_data() {  //прерывание для считывания данных для BER
  BER_interrupt = 1;
  BER_data_read = digitalRead(BER_DATA);
  BER_test();
}

void BER_test() {
  for (int i = 0; i < PER_PN; i++) {  //сдвигаем "сдвиговый регистр"
    BER_shift_reg[i] = BER_shift_reg[i + 1];
  }
  //Serial.print(BER_data_read);

  BER_shift_reg[PER_PN - 1] = BER_data_read;  //добавляем новый бит в "сдвиговый регистр"

  /*for (int i = 0; i < PER_PN; i++) {  //сдвигаем "сдвиговый регистр"
    Serial.print(BER_shift_reg[i]);
  }
  //Serial.print(BER_shift_reg[0]);
  Serial.println();*/

  if ((BER_flag_lock_PN == 1) && (BER_data_read == PNx[BER_cnt])) {  //проверяем на соответствие эталонному биту
    BER_cnt_norm = BER_cnt_norm + 1;
    //Serial.print(" NORM bit ");
    //Serial.println(PNx[BER_cnt]);
  }

  if ((BER_flag_lock_PN == 1) && (BER_data_read != PNx[BER_cnt])) {  //проверяем на соответствие эталонному биту
    BER_cnt_error = BER_cnt_error + 1;
    //Serial.print(" error bit ");
    //Serial.println(PNx[BER_cnt]);
  }

  if (BER_flag_lock_PN == 1) {
    BER_cnt = BER_cnt + 1;
    if (BER_cnt == PER_PN) {
      BER_cnt = 0;  //обнуляем счетчик
    }
  }

  int BER_cnt_for_lock = 0;  //локальная переменная для подсчета битов для захвата
  if (BER_flag_lock_PN == 0) {
    for (int i = 0; i < PER_PN; i++) {
      if (BER_shift_reg[i] == PNx[i]) {  //сравниваем "сдвиговый регистр" с эталоном
        BER_cnt_for_lock = BER_cnt_for_lock + 1;
      }
      if (BER_cnt_for_lock == PER_PN) {  //тут при желании можно допустить появление ощибочных бит
        BER_cnt_lock = BER_cnt_all;
        /*Serial.println();
        Serial.print("нашли! захват на ");
        Serial.print(BER_cnt_lock + 1);
        Serial.println(" бите");*/
        BER_flag_lock_PN = 1;  //флаг захвата PN
      }
    }
  }
  BER_cnt_all = BER_cnt_all + 1;  //считаем сколько бит приняли

  if ((BER_cnt_all == (1 + BER_total_bits + BER_cnt_lock)) && (BER_flag_lock_PN == 1)) {  //выводим количество ошибок в принятом пакете фиксированной длины
    //Serial.print("CORRECT BIT: ");
    //Serial.println(BER_cnt_norm);
    //Serial.print("ERROR BITS: ");
    Serial.println(BER_cnt_error);
    //Serial.print("TOTAL BITS: ");
    Serial.println(BER_total_bits);
    //Serial.print("RX BITS: ");
    //Serial.println(BER_cnt_all);
    BER_cnt_norm = 0;
    BER_cnt_error = 0;
    BER_cnt = 0;
    BER_cnt_all = 0;       //обнуляем счетчик бит
    BER_flag_lock_PN = 0;  //обнуляем флаг захвата PN
    detachInterrupt(0);    //отключаем прерывание по по INT0 (D2)
  }
}

void oled_help() {
  oled.clear();          // очистка
  oled.setCursor(0, 1);  // курсор на начало 3 строки
  oled.print("0x11/0x22 - SPI 16 bit");
  oled.setCursor(0, 2);  // курсор на начало 4 строки
  oled.print("0x33/0x44 - SPI 8 bit");
  oled.setCursor(0, 3);  // курсор на начало 5 строки
  oled.print("0x55 - PN gen");
  oled.setCursor(0, 4);  // курсор на начало 6 строки
  oled.print("0x66 - BER test");
  oled.setCursor(0, 5);  // курсор на начало 1 строки
  oled.print("0x77 - GPIO 1 set");
  oled.setCursor(0, 6);  // курсор на начало 2 строки
  oled.print("0x88 - GPIO 2 set");
  oled.setCursor(0, 7);  // курсор на начало 7 строки
  oled.print("0x99 - SPI freq set");
  delay(5000);
  //возврат стартового экрана
  oled.clear();          // очистка
  oled.setCursor(0, 2);  // курсор на начало 2 строки
  oled.print("Program_SPI_and_BER");
  oled.setCursor(0, 3);  // курсор на начало 3 строки
  oled.print("v1.1");
  oled.setCursor(0, 7);  // курсор на начало 7 строки
  oled.print("0xAA - for help");
}