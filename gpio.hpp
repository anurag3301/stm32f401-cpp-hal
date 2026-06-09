#ifndef __GPIO_HAL__
#define __GPIO_HAL__
#include <stm32f4xx.h>
#include "common.hpp"

extern "C" void EXTI_Handler(void);

class GPIO {
public:
    enum class Mode : uint8_t {
        Input     = 0,
        Output    = 1,
        Alternate = 2,
        Analog    = 3,
        None      = 0xFF
    };
    enum class OutputType : uint8_t {
        PushPull  = 0,
        OpenDrain = 1,
        None      = 0xFF
    };
    enum class Speed : uint8_t {
        Low      = 0,
        Medium   = 1,
        High     = 2,
        VeryHigh = 3,
        None     = 0xFF
    };
    enum class Pull : uint8_t {
        NoPUD    = 0,
        PullUp   = 1,
        PullDown = 2,
        None     = 0xFF
    };
    enum class AlternateFunction : uint8_t {
        AF0  = 0,  AF1  = 1,  AF2  = 2,  AF3  = 3,
        AF4  = 4,  AF5  = 5,  AF6  = 6,  AF7  = 7,
        AF8  = 8,  AF9  = 9,  AF10 = 10, AF11 = 11,
        AF12 = 12, AF13 = 13, AF14 = 14, AF15 = 15,
        None = 0xFF
    };

    enum class Edge: uint8_t{
        Fall,
        Rise,
        FallRise
    };

    GPIO(GPIO_TypeDef *port, uint8_t pin,
         Mode mode = Mode::Input, OutputType otype = OutputType::PushPull, 
         Speed speed = Speed::Low, Pull pull = Pull::NoPUD, 
         AlternateFunction af = AlternateFunction::None);
    GPIO(const GPIO&) = delete;
    GPIO& operator=(const GPIO&) = delete;
    GPIO(GPIO&& other) noexcept;
    GPIO& operator=(GPIO&& other) noexcept;

    ~GPIO();

    void setMode(Mode mode);
    void setOutputType(OutputType otype);
    void setSpeed(Speed speed);
    void setPull(Pull pull);
    void setAlternateFunction(AlternateFunction af);
    bool setInterruptCallback(Edge edge, void (*fn)(void*), void* param);

    void set();
    void reset();
    void toggle();
    bool get();
    friend void EXTI_Handler(void);

private:
    GPIO_TypeDef   *_port;
    uint8_t         _pin;
    int8_t          _inuseIdx;

    static int8_t  portToIdx(GPIO_TypeDef *port);
    static void    enableClock(GPIO_TypeDef *port);
    static bool    inuse[5][16];
    static Callback interrupt_callbacks[16];
};

#endif
