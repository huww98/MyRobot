#ifndef GPIO_H
#define GPIO_H

#include <functional>
#include <thread>

struct DigitalValue
{
    const char *const buf;
    bool IsHigh() const { return this == &HIGH; }
    bool IsLow() const { return this == &LOW; }

    static const DigitalValue HIGH;
    static const DigitalValue LOW;
};

struct Direction
{
    const char *const buf;

    static const Direction IN;
    static const Direction OUT;
};

struct Edge
{
    const char *const buf;

    static const Edge RISING;
    static const Edge FALLING;
    static const Edge BOTH;
};

class DigitalGpio
{
  public:
    DigitalGpio(int pin, const Direction &direction);
    void Write(const DigitalValue &value);
    const DigitalValue &Read();
    void EnableISR(const Edge &edge, std::function<void(const DigitalValue &)> isr)
    {
        EnableISR(edge, isr, -1, [] {});
    }
    void EnableISR(const Edge &edge, std::function<void(const DigitalValue &)> isr,
        int timeout, std::function<void()> timeoutRoutine);
    ~DigitalGpio();

    DigitalGpio &operator=(DigitalGpio &&other) = default;

  private:
    int pin;
    const Direction *direction;
    int valueFd;
    std::thread isrThread;

    void interruptHandler(std::function<void(const DigitalValue &)> isr,
        int timeout, std::function<void()> timeoutRoutine);
};

#endif
