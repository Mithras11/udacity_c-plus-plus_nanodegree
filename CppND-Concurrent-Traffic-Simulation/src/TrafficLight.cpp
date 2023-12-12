#include <iostream>
#include <random>
#include "TrafficLight.h"


template<typename T>
T MessageQueue<T>::receive() {

    std::unique_lock <std::mutex> lock(_mutex);
    _condition.wait(lock);
    T recieved = std::move(_queue.back());
    _queue.pop_back();
    return recieved;
}

template<typename T>
void MessageQueue<T>::send(T &&msg) {
    std::lock_guard <std::mutex> lock(_mutex);
    _queue.clear();
    _queue.push_back(msg);
    _condition.notify_one();
}


TrafficLight::TrafficLight() {
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen() {
    while (true) {
        if (_message_queue.receive() == TrafficLightPhase::green) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return;
}

TrafficLightPhase TrafficLight::getCurrentPhase() {
    return _currentPhase;
}

void TrafficLight::simulate() {
    TrafficObject::threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));

}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(4, 6);

    auto t_start = std::chrono::high_resolution_clock::now();
    auto toggle_sec = uni(rng);

    while (true) {
        auto t_curr = std::chrono::high_resolution_clock::now();
        double elapsed_time_sec = std::chrono::duration<double>(t_curr - t_start).count();

        if (elapsed_time_sec >= toggle_sec) {
            _currentPhase = (_currentPhase == red) ? green : red;
            toggle_sec = uni(rng);
            t_start = std::chrono::high_resolution_clock::now();

            _message_queue.send(std::move(_currentPhase));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
