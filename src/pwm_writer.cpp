#include <wiringPi.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
#include <dirent.h>
#include <unistd.h>

#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>

using namespace std;

const char* gpioDevPath = "/dev/gpiopwm";

void* writePwm(void* data)
{
    int pin = *static_cast<int*>(data);

    stringstream pathss;
    pathss << gpioDevPath << "/" << pin;
    auto path = pathss.str();
    //cout << path << endl;

    while(true)
    {
        ifstream ifs(path);
        int pwm;
        while(ifs >> pwm)
        {
            timeval tv;
            gettimeofday(&tv, nullptr);
            cout << "pin: " << pin << " pwm: "<< pwm << " time: " << tv.tv_sec << " "<< tv.tv_usec << endl;
            pwmWrite(pin, pwm);
        }
    }

    return nullptr;
}

int main(int argc, char** argv)
{
    auto dirp = opendir(gpioDevPath);
    struct dirent* ep;

    vector<int> pwmPins;

    while(ep = readdir(dirp))
    {
        int pin;
        stringstream ss(ep->d_name);
        if(ss >> pin)
        {
            pwmPins.push_back(pin);
        }
    }

    wiringPiSetupGpio();

    for (int& pin : pwmPins)
    {
        pinMode(pin, PWM_OUTPUT);
        pthread_t newThread;
        pthread_create(&newThread,nullptr,writePwm, &pin);
    }

    pause();
    return 0;
}
