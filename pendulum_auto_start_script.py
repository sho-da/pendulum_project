import time
import RPi.GPIO as GPIO
import os
import subprocess
import signal

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

#Set 9pin(GPIO) and 10pin(GPIO) to "input mode" and "pullup setting"(input is 0 when pressing the button, otherwise input is 1)
SW1 = 9        # shutdown
SW2 = 10       # control and shutdown
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # shutdown
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # control and shutdown

# CODE_EXEC_FLAG is a flag variable
# If pendulum program is executing, CODE_EXEC_FLAG = True
CODE_EXEC_FLAG = False

print("\n--------------------")
print("To start pendulum code, press YELLOW button for 2 seconds")
print("To shutdown, press YELLOW and BLACK button for 2 seconds")

while True:
    sw_timer = 0
    sw1_timer = 0
    sw2_timer = 0

    while True:
        sw1_status = GPIO.input(SW1)
        sw2_status = GPIO.input(SW2)
        
        # start control program (SW1 not pressed and SW2 pressed)
        # sw1_status == 1 and sw2_status == 0
        if sw1_status and not sw2_status:
            sw2_timer = sw2_timer + 1
            # If pendulum program is executed already, kill the program and clean up GPIO
            if CODE_EXEC_FLAG:
                exec_process.kill()
                # Prepare to kill for PENDULUM program(C++) and run the code
                KILL_CMD1 = "sudo killall -9 PENDULUM"
                subprocess.run(KILL_CMD1, shell=True)
                
                # Prepare to execute cleanup script and run the codes
                ABORT_CMD2 = "sudo /home/ubuntu/PENDULUM_CLEANUP"
                for i in range(5):
                    subprocess.run(ABORT_CMD2, shell=True)
                
                print("-------Control END------\n")
                CODE_EXEC_FLAG = False
                break
            

            # If sw2 is pressed for 2 sec, Pendulum program is executed
            if sw2_timer >= 20:
                print("-------Control START------\n")
                # C++コードをバックグラウンドで実行。そのためにsubprocess.Popenを使用
                EXEC_CMD = "sudo /home/ubuntu/PENDULUM"
                exec_process = subprocess.Popen(EXEC_CMD, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                GPIO.wait_for_edge(SW2, GPIO.RISING)
                time.sleep(1)
                print("-------Control Executed------\n")
                CODE_EXEC_FLAG = True
                break
                
        # shutdown
        # sw1_status == 0 and sw2_status == 0
        elif not sw1_status and not sw2_status:
            sw1_timer = sw1_timer + 1
            if sw1_timer >= 20:
                print("-------SHUTDOWN------\n")
                subprocess.run("sudo shutdown -h now", shell=True)
                break

        # elif not sw1_status and sw2_status:
        #    sw_timer = sw_timer + 1
        #    if sw1_timer >= 10:
        #        COMPILE_CMD1 = "g++ -o /home/ubuntu/PENDULUM /home/ubuntu/pendulum_project/main_pendulum.cpp -lpigpiod_if2 -lrt -pthread"
        #        COMPILE_CMD2 = "g++ -o /home/ubuntu/PENDULUM_CLEANUP /home/ubuntu/pendulum_project/cleanup.cpp -lpigpiod_if2 -lrt"
        #        subprocess.run(COMPILE_CMD1, shell=True)
        #        subprocess.run(COMPILE_CMD2, shell=True)
        #        break
        
        # (sw1_status==1 and sw2_status==1) or (sw1_status==0 and sw2_status==1)
        else:
            sw_timer = 0
            sw1_timer = 0
            sw2_timer = 0
            GPIO.wait_for_edge(SW2, GPIO.FALLING)
            

        time.sleep(0.1)
