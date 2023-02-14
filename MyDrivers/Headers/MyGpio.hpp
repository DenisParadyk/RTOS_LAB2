/**
********************************************************************************
* @file     MyGpio.hpp
* @date     10-October-2019
* @authors  Denys Paradiyk <denys.paradiyk@droid-technologies.com>
* @brief    Implementation of GPIO port driver for STM32F411.
********************************************************************************
**/

#ifndef MY_GPIO_HPP
#define MY_GPIO_HPP

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace MyDrivers {

    // Global variables.
    constexpr uint32_t kGpioPinCount = 16;

    /**
     * @brief GPIO mode enumeration definition.
     */
    enum class GpioMode {    
        kGpioModeInput     = GPIO_MODE_INPUT,
        kGpioModeOutputPP  = GPIO_MODE_OUTPUT_PP,
        kGpioModeOutputOD  = GPIO_MODE_OUTPUT_OD,
        kGpioModeItRising  = GPIO_MODE_IT_RISING,
        kGpioModeItFalling = GPIO_MODE_IT_FALLING,
        kGpioModeItBoth    = GPIO_MODE_IT_RISING_FALLING
    };

    /**
     * @brief GPIO pull enumeration definition.
     */
    enum class GpioPull {
        kGpioNoPull = GPIO_NOPULL,     // No Pull-up or Pull-down activation.
        kGpioPullUp = GPIO_PULLUP,     // Pull-up activation.
        kGpioPullDown = GPIO_PULLDOWN  // Pull-down activation.
    };

    enum class GpioSpeed {
        kGpioSpeedLow,      // IO works at 2 MHz.
        kGpioSpeedMedium,   // Range 12,5 MHz to 50 MHz.
        kGpioSpeedHigh,     // Range 25 MHz to 100 MHz.
        kGpioSpeedVeryHigh  // Range 50 MHz to 200 MHz.
    };

    /**
     * @brief GPIO port enumeration definition.
     */
    enum class GpioPort {
        kGpioPortA = GPIOA_BASE,
        kGpioPortB = GPIOB_BASE,
        kGpioPortC = GPIOC_BASE,
        kGpioPortD = GPIOD_BASE,
        kGpioPortE = GPIOE_BASE
    };

    /**
     * @brief GPIO pin enumeration definition.
     */
    enum class GpioPin {
        kGpioPin0  = ((uint16_t)0x0001),  // Pin 0 selected.
        kGpioPin1  = ((uint16_t)0x0002),  // Pin 1 selected.
        kGpioPin2  = ((uint16_t)0x0004),  // Pin 2 selected.
        kGpioPin3  = ((uint16_t)0x0008),  // Pin 3 selected.
        kGpioPin4  = ((uint16_t)0x0010),  // Pin 4 selected.
        kGpioPin5  = ((uint16_t)0x0020),  // Pin 5 selected.
        kGpioPin6  = ((uint16_t)0x0040),  // Pin 6 selected.
        kGpioPin7  = ((uint16_t)0x0080),  // Pin 7 selected.
        kGpioPin8  = ((uint16_t)0x0100),  // Pin 8 selected.
        kGpioPin9  = ((uint16_t)0x0200),  // Pin 9 selected.
        kGpioPin10 = ((uint16_t)0x0400),  // Pin 10 selected.
        kGpioPin11 = ((uint16_t)0x0800),  // Pin 11 selected.
        kGpioPin12 = ((uint16_t)0x1000),  // Pin 12 selected.
        kGpioPin13 = ((uint16_t)0x2000),  // Pin 13 selected.
        kGpioPin14 = ((uint16_t)0x4000),  // Pin 14 selected.
        kGpioPin15 = ((uint16_t)0x8000),  // Pin 15 selected.
        kGpioPinAll = ((uint16_t)0xFFFF)  // All pins selected.
    };

    /**
     * @brief GPIO pin state enumeration definition.
     */
    enum class GpioState {
        kGpioStateLow  = 0x00000000,
        kGpioStateHigh = 0x00000001
    };

    typedef struct {
        GpioPin Pin;       // Specifies the GPIO pins to be configured.
        GpioMode Mode;     // Specifies the operating mode for the selected pins.
        GpioPull Pull;     // Specifies the Pull-up or Pull-Down activation for the selected pins.
        GpioSpeed Speed;   // Specifies the speed for the selected pins.
    } GpioInit_t;

    /**
     * @brief  Base gpio class.
     * @note   None.
     * @bug    None.
     */
    class GpioBase
    {
    public:
        GpioBase(GpioPort port, GpioPin pin);
    protected:
        GpioPin m_pinNumber;
        GpioPort m_portName;
        GPIO_TypeDef* m_pGPIOx;
        
        bool init(GPIO_TypeDef *pGPIOx, GpioInit_t *initStruct);
        bool write(GPIO_TypeDef *pGPIOx, GpioPin pin, GpioState state);
        GpioState read(GPIO_TypeDef *pGPIOx, GpioPin pin);
        bool toggle(GPIO_TypeDef *pGPIOx, GpioPin pin);
    };

    /**
     * @brief  GPI class (Input port only).
     * @note   None.
     * @bug    None.
     */
    class Gpi: public GpioBase
    {
    public:
        Gpi(GpioPort port, GpioPin pin, GpioPull pull = GpioPull::kGpioNoPull);
        virtual ~Gpi() {};

        void operator>>(GpioState &value);
    private:
        virtual GpioState getBit(void);
    };

    /**
     * @brief  GPO class (Outputs port only).
     * @note   None.
     * @bug    None.
     */
    class Gpo: public GpioBase
    {
    public:
        Gpo(GpioPort port, GpioPin pin, GpioPull pull = GpioPull::kGpioNoPull);
        virtual ~Gpo() {};

        void toggleBit(void);
        void operator<<(GpioState value);
    private:
        virtual void setBit(GpioState value);
    };

    /**
     * @brief  GPIO class (Mixed Input/Output ports).
     * @note   None.
     * @bug    None.
     */
    class Gpio: public GpioBase
    {
    public:
        Gpio(GpioPort port, GpioPin pin, GpioPull pull = GpioPull::kGpioNoPull);
        virtual ~Gpio() {};

        void operator<<(GpioState value);
        void operator>>(GpioState &value);
    private:
        bool m_pinIsInput;
        GpioPull m_pull;

        void setBit(GpioState value);
        GpioState getBit(void);

        void setTypePinIn(void);
        void setTypePinOut(void);
    };

} // namespace MyDrivers

#ifdef __cplusplus
}
#endif

#endif // MY_GPIO_HPP