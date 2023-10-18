#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

// シグナルハンドラ関数
int pi;
extern const int LED_Y; // = 17;
extern const int LED_R; // = 22;
extern const int LED_G; // = 27;
extern const int IN1; // = 6;  // Motor driver input 1
extern const int IN2; // = 5;  // Motor driver input 2

void signalHandler(int signal) {
    const char* key;
    if (signal == SIGINT) 
    {
        key = "C";
    } else if(signal == SIGTSTP) 
    {
        key = "Z";
    } else 
    {
        key = "Unknown";
    }

    char message[100];
    std::sprintf(message, "Ctrl+%sが押されました。終了します。", key);
    std::cout << message << std::endl;
    // ここに中断時に実行したい処理を追加

    // ***** motor driver cleanup *****
    // pi = pigpio_start(NULL, NULL);
    set_mode(pi, LED_R, PI_OUTPUT);
    set_mode(pi, LED_Y, PI_OUTPUT);
    set_mode(pi, LED_G, PI_OUTPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);

    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);

    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_Y, 0);
    gpio_write(pi, LED_G, 0);

    sleep(1);
    gpio_write(pi, LED_R, 1);
    sleep(1);
    gpio_write(pi, LED_G, 1);
    sleep(1);
    gpio_write(pi, LED_Y, 1);
    sleep(1);
    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_Y, 0);
    gpio_write(pi, LED_G, 0);   

    pigpio_stop(pi);

    std::exit(signal); // プログラムを終了
}

#endif // SIGNAL_HANDLER_H