#pragma once
#include <vector>
#include <cstdlib>

class Filter {
public:
    virtual double filter(double newData) = 0;
};

class EMAFilter : public Filter {
private:
    double alpha;
    double lastEMA;

public:
    EMAFilter(double initialEMA, double alpha) : lastEMA(initialEMA), alpha(alpha) {}

    double filter(double newData) override {
        lastEMA = alpha * newData + (1 - alpha) * lastEMA;
        return lastEMA;
    }
};

class MAFilter : public Filter {
private:
    size_t windowSize;
    std::vector<double> window;
    double sum;

public:
    MAFilter(size_t windowSize) : windowSize(windowSize), sum(0) {
        window.reserve(windowSize);
    }

    double filter(double newData) override {
        if (window.size() == windowSize) {
            sum -= window[0];
            window.erase(window.begin());
        }
        window.push_back(newData);
        sum += newData;
        return sum / window.size();
    }
};

class FIRFilter : public Filter {
private:
    std::vector<double> coefficients;
    std::vector<double> buffer;

public:
    FIRFilter(const std::vector<double>& coefficients) : coefficients(coefficients) {
        buffer.resize(coefficients.size(), 0);
    }

    double filter(double newData) override {
        buffer.erase(buffer.begin());
        buffer.push_back(newData);

        double result = 0.0;
        for (size_t i = 0; i < coefficients.size(); ++i) {
            result += coefficients[i] * buffer[i];
        }
        return result;
    }
};

class IIRFilter : public Filter {
private:
    double alpha;
    double lastOutput;

public:
    IIRFilter(double initialOutput, double alpha) : lastOutput(initialOutput), alpha(alpha) {}

    double filter(double newData) override {
        lastOutput = alpha * newData + (1 - alpha) * lastOutput;
        return lastOutput;
    }
};