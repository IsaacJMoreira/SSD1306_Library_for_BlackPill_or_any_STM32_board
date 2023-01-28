# SSD1306 oled display driver for the BlackPill (stm32f411) or any STM32 board

Here I provided a simple game called BOUNCE! implemented with this library.

 This code is really basic, but I tried to comment this code as much as
  * possible. Feel free to modify and add functionalities.
  *
  * The purpose of this simple game implementations is to make use of the
  * ssd1306 oled 64p x 128p 0.96'' module with the STM32F411 (Black Pill) or
  * any STM32 board, provided that you modify the <ssd1306_BlackPill_conf.h> to
  * accomodate the device you are using.
  *
  * Here we use the SPI1 peripheral with 2 as the pre-scaler, but make it slower
  * if you need to. Using jumpers is sketchy and may cause erros.
  * Use PA5 (Clock) and PA7(MOSI) as the SPI1 pins and turn the
  * pull-ups ON.
  *
  * PB0 is attached to the Res Pin of the Display.
  * PB1 is attached to the DC Pin of the Display.
  * PB2 is attached to the CS Pin of the Display.
  * PB9 is attached to the button and is called GAME_BUTTON with the pull-up ON.
  * When the button is pushed, it has to pull the pin down.
  * PA1 is attached to the center tap of the potentiometer.
