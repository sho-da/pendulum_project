#include <thread>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unistd.h>

std::thread thread1;
std::thread thread2;

int x1 = 1;
int x2 = 1;
int t1 = 0;
int t2 = 0;
int span1 = 1;//25; // usec
int span2 = 1;//5/2; // msec

std::ofstream csvFile; // CSVファイルに書き込むためのファイルストリーム

bool stopThread1 = false;
bool stopThread2 = false;

void count1(!stopThread1) {
    while(){
    x1 = x1 + 1;
    t1 = t1 + span1;
        
    // ファイルへの書き込み
    csvFile << t1 << "," << x1 << std::endl;
    std::cout << "x1の値: " << x1 << std::endl;

    std::chrono::seconds dura1(span1);
    //std::chrono::microseconds dura1(span1);
    std::this_thread::sleep_for(dura1);
    }
}

void count2() {
    while(!stopThread2){
    x2 = 2 * x2;
    t2 = t2 + span2;
        
    // ファイルへの書き込み
    csvFile << t2 << "," << x2 << std::endl;
    std::cout << "x2の値: " << x2 << std::endl;

    std::chrono::seconds dura2(span2);
    //std::chrono::milliseconds dura2(static_cast<int>(span2));
    std::this_thread::sleep_for(dura2);
    }
}

int main() {
    csvFile.open("output.csv"); // ファイルを開く

    thread1 = std::thread(count1);
    thread2 = std::thread(count2);

    // スレッドの実行が終了するのを待つ
    thread1.join();
    thread2.join();
    
    sleep(10);
    stopThread1 = true;
    stopThread2 = true;

    // ファイルを閉じる
    csvFile.close();

    std::cout << "すべてのスレッドが終了し、データがCSVファイルに記録されました。" << std::endl;

    return 0;
}