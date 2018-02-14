#include <chrono>

namespace encoder{
struct Data
{
    double velocity;
    std::chrono::steady_clock::time_point time;
};
}
