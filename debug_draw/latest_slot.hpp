#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <utility>

namespace auto_aim
{

//最新帧槽：只保最新一份，被覆盖写入。序号变化即代表有新数据（和相机的 LatestImagesOnly 一致）
template <typename T>
struct LatestSlot
{
    std::mutex mutex;
    std::condition_variable ready;
    T payload;
    std::uint64_t seq = 0;
    std::atomic_bool running{true};

    //生产者：覆盖 payload，序号 +1，唤醒一个消费者
    void publish(T value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            payload = std::move(value);
            ++seq;
        }
        ready.notify_one();
    }

    //消费者：等到有比 consumed 更新的数据或停止；返回 false 表示该退出了
    bool wait(std::uint64_t& consumed, T& out)
    {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock, [&] { return seq != consumed || !running; });
        if (!running)
        {
            return false;
        }
        out = payload;
        consumed = seq;
        return true;
    }

    //置停止并唤醒所有等待者
    void stop()
    {
        running = false;
        ready.notify_all();
    }
};

} // namespace auto_aim
