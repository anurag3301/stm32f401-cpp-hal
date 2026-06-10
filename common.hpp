#ifndef __HAL_COMMON__
#define __HAL_COMMON__
extern uint32_t SystemCoreClock;

struct Callback{
    void (*fn)(void*) = nullptr;
    void* ctx = nullptr;
    void invoke() const {if(fn) fn(ctx);}
};

template<typename T>
T&& move(T& x) { return static_cast<T&&>(x); }

#endif // __HAL_COMMON__
