#ifndef PAIR_HPP_
#define PAIR_HPP_

namespace Maav
{

template <typename T1, typename T2>
struct Pair
{
    T1 first;
    T2 second;
    
    Pair() {}
    Pair(const T1& first_, const T2& second_) : first(first_), second(second_) {}
};

}

#endif /* RCCONTROLLER_HPP_ */
