#include <iostream>
#include <pigpiod_if2.h>
#include <chrono>
#include <thread>
#include <stdlib.h>
using namespace std;

// つないだピン番号を設定
#define PIN1 22
#define PIN2 17
#define PIN3 27


int pi;

int main(){
  int i;

  // 1) 最初にgpioInitialise()を実行する
  pi=pigpio_start(NULL,NULL);

  // 2) ピンの設定をする。出力したい場合は PI_OUTPUT
  set_mode(pi, PIN1, PI_OUTPUT );   // red
  set_mode(pi, PIN2, PI_OUTPUT );   // yellow
  set_mode(pi, PIN3, PI_OUTPUT );   // green

  // 3) Lチカする。
  int LEDTickPin = PIN1;
  for( i=0; i<5; i++ ){

    // LEDを光らせる
    gpio_write(pi, LEDTickPin, 1 );

    // １秒待機
    std::this_thread::sleep_for( std::chrono::seconds(1) );

    // LEDを暗くする
    gpio_write(pi, LEDTickPin, 0);

    std::this_thread::sleep_for( std::chrono::seconds(1) );

  }
  gpio_write(pi, PIN1, 0);
  gpio_write(pi, PIN2, 0);
  gpio_write(pi, PIN3, 0);
  pigpio_stop(pi);
  return 0;
}