#pragma once
#include <vector>
#include <array>
#include <deque>
#include <cstdlib>
#include <algorithm>

class Filter {
public:
    virtual void reset() = 0;
    virtual double step(double newData) = 0;
protected:
    bool shouldReset = false;
};

class IIRFilter : public Filter {
protected:
    std::vector<double> b; // Numerator coefficients
    std::vector<double> a; // Denominator coefficients
    std::vector<double> inputs; // Input buffer
    std::vector<double> outputs; // Output buffer

    void fill_arrays(double value){
        inputs.resize(b.size(), 0);
        outputs.resize(a.size() - 1, 0);
        std::fill(inputs.begin(), inputs.end(), value);
        std::fill(outputs.begin(), outputs.end(), value);
    }

public:
    IIRFilter(const std::vector<double>& b_coeffs, const std::vector<double>& a_coeffs) : b(b_coeffs), a(a_coeffs) {
        // Initialize input and output buffers

        reset();
    }
    
    void reset() override {
        shouldReset = true;
    }

    double step(double newData) override {
        if (shouldReset) {
            fill_arrays(newData);
            shouldReset = false;
        }

        // Shift input buffer to the right and add new input to index zero
        std::rotate(inputs.rbegin(), inputs.rbegin() + 1, inputs.rend());
        inputs[0] = newData;

        // Accumulate b term
        double bcc = 0.0;
        for (size_t i = 0; i < b.size(); ++i) {
            bcc += b[i] * inputs[i];
        }
        
        // Accumulate a term
        double acc = 0.0;
        for (size_t i = 1; i < a.size(); ++i) {
            acc += a[i] * outputs[i - 1];
        }

        // Shift output buffer
        std::rotate(outputs.rbegin(), outputs.rbegin() + 1, outputs.rend());
        outputs[0] = bcc - acc;

        return outputs[0];
    }
};

// class FIRFilter : public IIRFilter {
// public:
//     FIRFilter(const std::vector<double>& b_coeffs) : IIRFilter(b_coeffs, {1.0}) {}
// };