# SPI_and_BER

Данный Arduino проект разработан для:
- записи и чтения по SPI для 8- и 16-разрядных адресов
- генерации PN-последовательностей
- BER теста
- позволяет управлять двумя GPIO

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
- сами эталоны (периоды) PN последовательностей генерируются отдельными функциями (можно при желании вывести в монитор порта последовательности) после чего записываются в массив PNx[],
откуда берутся при выдаче (экономия 30 % памяти по сравнению с глобальными константами)
- для запуска подать 4 команды:
  * 85 - собственно команда
  * <3 или 4 или 5 или 7 или 9> - какую из PN3, PN4, PN5, PN7, PN9 выбираем
  * количество бит для передачи (до 2 000 000)
  * частота тактового сигнала выдачи - до 50 000 (далее МК не тянет)
- если после ввода первой команды (85) вводится отличное от 3, 4, 5, 7 или 9, то команда сбрасывается и ее надо вводить заново
 
***
SPI запись и чтение
- тактовая частота по умолчанию 5 кГц (не задается)
- используемые выводы:
  * D10 - MOSI
  * D7 - MISO
  * D12 - CS (по умолчанию 1, падает в 0 при начале передачи и уходит в 1 по окончанию)
  * D11 - SCLK (таймер 2, А)
- перед активацией таймера запускается функция addr_data_PNx(), которая записывает в массив PNx адрес и данные, которые потом будут выгружаться по прерыванию от таймера
- возможные команды:
  * 17, 34, 51 и 68
    . 68 - читаем по SPI с 8 бит адреса
    . 51 - записываем по SPI с 8 бит адреса
    . 34 - читаем по SPI с 16 бит адреса
    . 17 - записываем по SPI с 16 бит адреса
  * <8 бит адреса> (если 68 - выдает считанное значение по 8-битному адресу)
  * <8 бит адреса / 8 бит данных> (если  34 - выдает считанное значение по 16-битному адресу; если 51 - записывает введенные данные по введенному выше адресу)
  * <8 бит данных> (если 17 - записывает введенные данные по введенному выше адресу)

***
BER тест
- используемые выводы:
  * D4 - вход данных
  * D2 - тактовый сигнал данных (INT 0)
- перед разрешением прерывания запускается одна из функций PN_3, PN_4, PN_5, PN_7 или PN_9, которая в массиы PNx записывает эталон (период) ПСП, с которым будет проводиться сравнение
- работает при входном тактовом сигнале 10 кГц
- выводится количество правильных и ошибочных бит, количество принятых для подсчета BER бит и общее количество бит, по которым сработали прерывания
- возможные команды:
  * 102 - собственно команда
  * <3 или 4 или 5 или 7 или 9> - какую из PN3, PN4, PN5, PN7, PN9 выбираем
  * количество бит для подсчета BER (до 2 000 000), период ПСП не входит,   биты считаюстя после захвата PN_
 
 Распиновка платы Arduino Nano и используемые в проекте выводы:
 
  ![image](https://github.com/user-attachments/assets/9c597e72-5e76-4b17-84f7-8d5f8c3c9992)


***
В версии v1.1 добавлено:
- дисплей OLED 128×64, на который выводится:
  * название проекта
  * версия проекта
  * команда, кторую надо отправить по UART для получения справки на дисплее
- команда 170 (0хAA) для вывода на дисплей команд  (10 сек)
- команда 153 (0x99) для установки скорости SCLK для SPI (по умолчанию 10000 Гц)
- убран вывод лишней информации по UART
- при BER тесте выводится только количество ошибочных бит и количество принятых для подсчета BER бит
