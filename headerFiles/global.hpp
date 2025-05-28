#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <random>
#include <iostream>
#include <cmath>

const float EPSILON = std::numeric_limits<float>::epsilon();

/*by Matthew Reynolds u6949604*/
inline float randomFloat() {
    static std::mt19937 generator(std::random_device{}());
    static std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
    return distribution(generator);
}

/*by Matthew Reynolds u6949604*/
inline void updateProgress(float progress) {
    int barWidth = 50;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; i++) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0f) << " %\r";
    std::cout.flush();
}

#endif // GLOBAL_HPP