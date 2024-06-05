# Nixie
## Main goal of the project
  The main goal of this project is to create a clock based on four gas discharge lamps.
  The control device is STM32F411E-DISCO.

## What has been done so far
  To date, we have implemented a clock that can count an hour and a minute. We set up the
  clock through 3 buttons, 2 of which are responsible for setting the hours and minutes and
  1 for switching modes between settings and showing the time. The dynamic display works
  and the lamps light up alternately, but it seems that everyone is talking. Temperature
  and humidity measurement is also implemented and shown on the 7-segment display. The
  body is usually made of wood.

## Details we used

### IN-1 lamps
### TLP627 Toshiba Optocouplers
### Decryptor K155ID-1
### Real Time Clock DS3231
### Temperature and humidity sensor DHT22 and Indicator key-board module TM1638
## Scheme of project

![scheme](https://github.com/loginyuk/Nixie/assets/55399864/94657f1a-0a78-4f66-8671-4246b7766c4a)

## How it looks like

![IMG_1015](https://github.com/loginyuk/Nixie/assets/55399864/79971fc3-0088-40c7-acf7-f6e78d55d3eb)

## Summary
This project is a
good example of how to deal with different modules of different complexity and different
types of character
