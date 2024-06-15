#ifndef SYSTEM_H
#define SYSTEM_H

namespace nowmath {

    template <class StateVectorType>
    class DynamicalSystemBase {
    private:
    public:
        virtual StateVectorType step(const StateVectorType& state) = 0;
    };
}

#endif //SYSTEM_H