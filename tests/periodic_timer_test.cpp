#include <periodic_timer_thread.hpp>
#include <mutex>
#include <vector>

class DelayedLoop : public PeriodicTimerThread {
public:
    DelayedLoop() : PeriodicTimerThread(100.0) {}
    std::vector<std::chrono::steady_clock::time_point> starts;
    std::thread::id hook_thread;
    std::thread::id timer_thread;
protected:
    void before_start() override { hook_thread = std::this_thread::get_id(); }
    void on_timer() override {
        timer_thread = std::this_thread::get_id();
        starts.push_back(std::chrono::steady_clock::now());
        if (starts.size() == 1) std::this_thread::sleep_for(std::chrono::milliseconds(55));
    }
};

int main() {
    DelayedLoop loop;
    loop.start_thread();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    loop.stop_thread();
    if (loop.starts.size() < 3) return 1;
    for (size_t i = 2; i < loop.starts.size(); ++i) {
        if (loop.starts[i] - loop.starts[i - 1] < std::chrono::milliseconds(5)) {
            std::cerr << "FAIL: missed deadlines caused back-to-back command bursts\n";
            return 2;
        }
    }
    if (loop.hook_thread != loop.timer_thread) {
        std::cerr << "FAIL: startup hook must run in worker thread\n";
        return 3;
    }
}
