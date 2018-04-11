#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <sys/epoll.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <system_error>

#include "gpio.h"

using namespace std;

const DigitalValue DigitalValue::HIGH{"1\n"};
const DigitalValue DigitalValue::LOW{"0\n"};
const Direction Direction::IN{"in"};
const Direction Direction::OUT{"out"};
const Edge Edge::RISING{"rising"};
const Edge Edge::FALLING{"falling"};
const Edge Edge::BOTH{"both"};

void waitForFile(string path)
{
    while (access(path.c_str(), R_OK|W_OK) == -1)
        ;
}

template <typename T>
void writeDataToInterface(T data, string interfacePath)
{
    waitForFile(interfacePath);
    ofstream interface(interfacePath);
    interface << data << endl;
    interface.close();
}

DigitalGpio::DigitalGpio(int pin, const Direction &direction)
{
    this->pin = pin;
    this->direction = &direction;

    writeDataToInterface(pin, "/sys/class/gpio/export");

    stringstream directionss;
    directionss << "/sys/class/gpio/gpio" << pin << "/direction";
    writeDataToInterface(direction.buf, directionss.str());

    stringstream valuess;
    valuess << "/sys/class/gpio/gpio" << pin << "/value";
    waitForFile(valuess.str());
    valueFd = open(valuess.str().c_str(), O_RDWR);

    if(valueFd == -1)
    {
        stringstream ss;
        ss << "open value file failed. " << strerror(errno);
        throw runtime_error(ss.str());
    }
}

void DigitalGpio::Write(const DigitalValue &value)
{
    assert(direction == &Direction::OUT);
    write(valueFd, value.buf, 2);
}

const DigitalValue &DigitalGpio::Read()
{
    assert(direction == &Direction::IN);

    char c;
    lseek(valueFd, 0L, SEEK_SET);
    read(valueFd, &c, 1);
    return c == '0' ? DigitalValue::LOW : DigitalValue::HIGH;
}

void DigitalGpio::EnableISR(const Edge &edge, std::function<void(const DigitalValue &)> isr,
    int timeout, std::function<void()> timeoutRoutine)
{
    assert(direction == &Direction::IN);

    stringstream edgess;
    edgess << "/sys/class/gpio/gpio" << pin << "/edge";
    writeDataToInterface(edge.buf, edgess.str());

    isrThread = thread(&DigitalGpio::interruptHandler, this, isr, timeout, timeoutRoutine);
}

DigitalGpio::~DigitalGpio()
{
    close(this->valueFd);
}

void DigitalGpio::interruptHandler(std::function<void(const DigitalValue &)> isr,
    int timeout, std::function<void()> timeoutRoutine)
{
    // Try to promote priority
    struct sched_param sched;
    sched.sched_priority = 55;
    sched_setscheduler(0, SCHED_RR, &sched);

    int epollfd = epoll_create1(0);
    if (epollfd == -1)
        throw runtime_error("epoll_create1");

    struct epoll_event ev;
    ev.events = EPOLLPRI;
    ev.data.fd = this->valueFd;
    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, this->valueFd, &ev) == -1)
        throw runtime_error("epoll_ctl: this->valueFd");

    constexpr int MAX_EVENTS = 5;
    struct epoll_event events[MAX_EVENTS];
    while (true)
    {
        int nfds = epoll_wait(epollfd, events, MAX_EVENTS, timeout);
        if (nfds == -1)
            cerr << "epoll_wait failed" << endl;
        if(nfds == 0)
        {
            timeoutRoutine();
            continue;
        }
        for (int i = 0; i < nfds; i++)
        {
            isr(this->Read());
        }
    }
}
