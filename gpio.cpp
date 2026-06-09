#include "gpio.hpp"

bool GPIO::inuse[5][16] = {};
Callback GPIO::interrupt_callbacks[16] = {};

GPIO::GPIO(GPIO_TypeDef *port, uint8_t pin,
           Mode mode, OutputType otype, Speed speed,
           Pull pull, AlternateFunction af)
    : _port(port), _pin(pin), _inuseIdx(-1)
{
    _inuseIdx = portToIdx(port);
    if (_inuseIdx == -1 || inuse[_inuseIdx][_pin]) return;

    enableClock(port);
    setMode(mode);
    setOutputType(otype);
    setSpeed(speed);
    setPull(pull);
    setAlternateFunction(af);

    inuse[_inuseIdx][_pin] = true;
}

GPIO::GPIO(GPIO&& other) noexcept
    : _port(other._port), _pin(other._pin), _inuseIdx(other._inuseIdx)
{
    other._inuseIdx = -1;
}

GPIO& GPIO::operator=(GPIO&& other) noexcept {
    if (this != &other) {
        this->~GPIO();
        _port  = other._port;
        _pin       = other._pin;
        _inuseIdx  = other._inuseIdx;
        other._inuseIdx = -1;
    }
    return *this;
}

GPIO::~GPIO() {
    if (_inuseIdx != -1) {
        inuse[_inuseIdx][_pin] = false;

        if (interrupt_callbacks[_pin].fn != nullptr) {
            EXTI->IMR  &= ~(1U << _pin);
            EXTI->RTSR &= ~(1U << _pin);
            EXTI->FTSR &= ~(1U << _pin);
            EXTI->PR    =  (1U << _pin);

            // Disable NVIC only if no other pin sharing this IRQ is active
            // EXTI9_5 and EXTI15_10 are shared lines
            auto anyActive = [](int lo, int hi) {
                for (int i = lo; i <= hi; i++)
                    if (interrupt_callbacks[i].fn != nullptr) return true;
                return false;
            };

            if (_pin <= 4) {
                const IRQn_Type irqs[] = {
                    EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn
                };
                NVIC_DisableIRQ(irqs[_pin]);
            } else if (_pin <= 9) {
                if (!anyActive(5, 9)) NVIC_DisableIRQ(EXTI9_5_IRQn);
            } else {
                if (!anyActive(10, 15)) NVIC_DisableIRQ(EXTI15_10_IRQn);
            }

            interrupt_callbacks[_pin] = {};
        }
    }
}
void GPIO::setMode(Mode mode) {
    if (mode == Mode::None) return;
    _port->MODER &= ~(0b11u << _pin * 2);
    _port->MODER |=  (static_cast<uint32_t>(mode) << _pin * 2);
}

void GPIO::setOutputType(OutputType otype) {
    if (otype == OutputType::None) return;
    _port->OTYPER &= ~(1u << _pin);
    _port->OTYPER |=  (static_cast<uint32_t>(otype) << _pin);
}

void GPIO::setSpeed(Speed speed) {
    if (speed == Speed::None) return;
    _port->OSPEEDR &= ~(0b11u << _pin * 2);
    _port->OSPEEDR |=  (static_cast<uint32_t>(speed) << _pin * 2);
}

void GPIO::setPull(Pull pull) {
    if (pull == Pull::None) return;
    _port->PUPDR &= ~(0b11u << _pin * 2);
    _port->PUPDR |=  (static_cast<uint32_t>(pull) << _pin * 2);
}

void GPIO::setAlternateFunction(AlternateFunction af) {
    if (af == AlternateFunction::None) return;
    if (_pin <= 7) {
        _port->AFR[0] &= ~(0b1111u << _pin * 4);
        _port->AFR[0] |=  (static_cast<uint32_t>(af) << _pin * 4);
    } else {
        _port->AFR[1] &= ~(0b1111u << (_pin - 8) * 4);
        _port->AFR[1] |=  (static_cast<uint32_t>(af) << (_pin - 8) * 4);
    }
}

void GPIO::set() {
    _port->BSRR = (1u << _pin);
}

void GPIO::reset() {
    _port->BSRR = (1u << (_pin + 16));
}

void GPIO::toggle() {
    _port->ODR ^= (1u << _pin);
}

bool GPIO::get() {
    return (_port->IDR & (1u << _pin)) != 0;
}


bool GPIO::setInterruptCallback(Edge edge, void (*fn)(void*), void* param){
    if(interrupt_callbacks[_pin].fn != nullptr){
        return false;
    }

    if(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN)){
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }

    // SYSCFG setup
    uint32_t exticr_val;
    if      (_port == GPIOA) exticr_val = 0;
    else if (_port == GPIOB) exticr_val = 1;
    else if (_port == GPIOC) exticr_val = 2;
    else if (_port == GPIOD) exticr_val = 3;
    else if (_port == GPIOE) exticr_val = 4;
    else return false;

    uint32_t exticr_idx = _pin / 4;
    uint32_t shift      = (_pin % 4) * 4;
    SYSCFG->EXTICR[exticr_idx] &= ~(0xFu << shift);
    SYSCFG->EXTICR[exticr_idx] |=  (exticr_val << shift);

    // EXTI setup
    EXTI->IMR |= 1U<<_pin;
    if(edge == Edge::Fall){
        EXTI->RTSR &= ~(1U<<_pin);
        EXTI->FTSR |= 1U<<_pin;
    }
    else if(edge == Edge::Rise){
        EXTI->FTSR &= ~(1U<<_pin);
        EXTI->RTSR |= 1U<<_pin;
    }
    else{
        EXTI->FTSR |= 1U<<_pin;
        EXTI->RTSR |= 1U<<_pin;
    }

    // Clear pending before enabling NVIC Otherwise an old pending bit could immediately fire.
    EXTI->PR = (1U << _pin); 
    switch(_pin){
        case 0: NVIC_EnableIRQ(EXTI0_IRQn); break;
        case 1: NVIC_EnableIRQ(EXTI1_IRQn); break;
        case 2: NVIC_EnableIRQ(EXTI2_IRQn); break;
        case 3: NVIC_EnableIRQ(EXTI3_IRQn); break;
        case 4: NVIC_EnableIRQ(EXTI4_IRQn); break;
        case 5: NVIC_EnableIRQ(EXTI9_5_IRQn); break;
        case 6: NVIC_EnableIRQ(EXTI9_5_IRQn); break;
        case 7: NVIC_EnableIRQ(EXTI9_5_IRQn); break;
        case 8: NVIC_EnableIRQ(EXTI9_5_IRQn); break;
        case 9: NVIC_EnableIRQ(EXTI9_5_IRQn); break;
        case 10: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        case 11: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        case 12: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        case 13: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        case 14: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        case 15: NVIC_EnableIRQ(EXTI15_10_IRQn); break;
        default: return false;
    }

    interrupt_callbacks[_pin].fn = fn;
    interrupt_callbacks[_pin].ctx = param;
    return true;
}

int8_t GPIO::portToIdx(GPIO_TypeDef *port) {
    if      (port == GPIOA) return 0;
    else if (port == GPIOB) return 1;
    else if (port == GPIOC) return 2;
    else if (port == GPIOD) return 3;
    else if (port == GPIOE) return 4;
    else                    return -1;
}

void GPIO::enableClock(GPIO_TypeDef *port) {
    if      (port == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (port == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (port == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (port == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (port == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
}

extern "C" void EXTI_Handler(void){
    uint32_t pending = EXTI->PR;
    for (int pin = 0; pin < 16; pin++) {
        if (pending & (1U << pin)) {
            EXTI->PR = (1U << pin);
            GPIO::interrupt_callbacks[pin].invoke();
        }
    }
}
