// Mostly copied from https://github.com/mstump/queues/blob/master/include/spsc-bounded-queue.hpp
// SPSC stands for single producer single consumer.

#ifndef __SPSC_BOUNDED_QUEUE_INCLUDED__
#define __SPSC_BOUNDED_QUEUE_INCLUDED__

#include <atomic>
#include <utility>
#include <assert.h>

template <typename T>
class SpscBoundedQueue
{
  public:
    SpscBoundedQueue(const SpscBoundedQueue &) = delete;
    SpscBoundedQueue(SpscBoundedQueue &&other):
        _size(other._size), _mask(other._mask),
        _head(other._head.load()), _tail(other._tail.load()),
        _buffer(other._buffer)
    {
        other._buffer = nullptr;
    }
    void operator=(const SpscBoundedQueue &) = delete;

    SpscBoundedQueue(size_t size) : _size(size), _mask(size - 1),
        _buffer(new aligned_t[_size]),
        _head(0), _tail(0)
    {
        // make sure it's a power of 2
        assert((_size != 0) && ((_size & _mask) == 0));
    }

    ~SpscBoundedQueue()
    {
        if(_buffer == nullptr)
            return;
        delete[] _buffer;
    }

    bool enqueue(const T &input)
    {
        const size_t head = _head.load(std::memory_order_relaxed);

        if (((_tail.load(std::memory_order_acquire) - (head + 1)) & _mask) >= 1)
        {
            *reinterpret_cast<T*>(&_buffer[head & _mask]) = input;
            _head.store(head + 1, std::memory_order_release);
            return true;
        }
        return false;
    }

    bool dequeue(T* &output)
    {
        const size_t tail = _tail.load(std::memory_order_relaxed);

        if (((_head.load(std::memory_order_acquire) - tail) & _mask) >= 1)
        {
            output = reinterpret_cast<T*>(&_buffer[tail & _mask]);
            _tail.store(tail + 1, std::memory_order_release);
            return true;
        }
        return false;
    }

  private:
    using aligned_t = std::aligned_storage_t<sizeof(T), alignof(T)>;

    static constexpr int hardware_destructive_interference_size = 64; // This should be in c++17.
    alignas(hardware_destructive_interference_size) const size_t _size;
    const size_t _mask;
    aligned_t * _buffer;

    alignas(hardware_destructive_interference_size) std::atomic<size_t> _head;
    alignas(hardware_destructive_interference_size) std::atomic<size_t> _tail;
};

#endif
