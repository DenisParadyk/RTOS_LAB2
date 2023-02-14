/**
 ******************************************************************************
 * @file    MyGpio.cpp
 * @date    10-October-2019
 * @brief   The file contains descriptions of GpioBase, Gpo, Gpi and Gpio classes.
 ******************************************************************************
 **/

/* Includes ------------------------------------------------------------------*/
#include "MyGpio.hpp"

using namespace MyDrivers;

/* ---------------------------------------------------------------------------*/
/* ---------------------------- GpioBase -------------------------------------*/
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Class constructor.
 * @param  port: name of the port (kGpioPortA ... kGpioPortH).
 * @param  pin:  pin number (kGpioPin0 ... kGpioPin15).
 * @retval none.
 */
GpioBase::GpioBase (GpioPort port, GpioPin pin)
{
    this->m_pinNumber = pin;
    this->m_portName = port;

    switch (this->m_portName)
    {
    case GpioPort::kGpioPortA:
        this->m_pGPIOx = GPIOA;
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case GpioPort::kGpioPortB:
        this->m_pGPIOx = GPIOB;
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case GpioPort::kGpioPortC:
        this->m_pGPIOx = GPIOC;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case GpioPort::kGpioPortD:
        this->m_pGPIOx = GPIOD;
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case GpioPort::kGpioPortE:
        this->m_pGPIOx = GPIOE;
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;
    default:
        break;
    }
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

bool GpioBase::init(GPIO_TypeDef *pGPIOx, GpioInit_t *initStruct)
{
    if (pGPIOx == nullptr || initStruct == nullptr) {
        return false;
    }
    uint32_t position;
    uint32_t ioPosition = 0x00U;
    uint32_t ioCurrent  = 0x00U;
    uint32_t temp       = 0x00U;

    for (position = 0U; position < kGpioPinCount; position++) {
        ioPosition = 0x01U << position;
        ioCurrent = (uint32_t)(initStruct->Pin) & ioPosition;

        if (ioCurrent == ioPosition) {
            if (((uint16_t)initStruct->Mode & GPIO_MODE) == MODE_OUTPUT) {
                temp = pGPIOx->OSPEEDR;
                temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
                temp |= ((uint16_t)initStruct->Speed << (position * 2U));
                pGPIOx->OSPEEDR = temp;

                temp = pGPIOx->OTYPER;
                temp &= ~(GPIO_OTYPER_OT_0 << position) ;
                temp |= ((((uint16_t)initStruct->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
                pGPIOx->OTYPER = temp;
            }
            if(((uint16_t)initStruct->Mode & GPIO_MODE) != MODE_ANALOG)
            {
                temp = pGPIOx->PUPDR;
                temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
                temp |= (((uint16_t)initStruct->Pull) << (position * 2U));
                pGPIOx->PUPDR = temp;
            }

            temp = pGPIOx->MODER;
            temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
            temp |= (((uint16_t)initStruct->Mode & GPIO_MODE) << (position * 2U));
            pGPIOx->MODER = temp;
        }
    }
    return true;
}

bool GpioBase::write(GPIO_TypeDef *pGPIOx, GpioPin pin, GpioState state)
{
    if (pGPIOx == nullptr) {
        return false;
    }

    if(state != GpioState::kGpioStateLow) {
        pGPIOx->BSRR = (uint16_t)pin;
    } else {
        pGPIOx->BSRR = (uint32_t)pin << 16U;
    }

    return true;
}

GpioState GpioBase::read(GPIO_TypeDef *pGPIOx, GpioPin pin)
{
    GpioState bitStatus;

    if((pGPIOx->IDR & (uint16_t)pin) != (uint32_t)GpioState::kGpioStateLow) {
        bitStatus = GpioState::kGpioStateHigh;
    } else {
        bitStatus = GpioState::kGpioStateLow;
    }
    return bitStatus;
}

bool GpioBase::toggle(GPIO_TypeDef *pGPIOx, GpioPin pin)
{
    if (pGPIOx == nullptr) {
        return false;
    }

    uint32_t odrData = pGPIOx->ODR;;

    pGPIOx->BSRR = ((odrData & (uint16_t)pin) << kGpioPinCount) | \
                                    (~odrData & (uint16_t)pin);

    return true;
}

/* ---------------------------------------------------------------------------*/
/* ------------------------------- Gpi ---------------------------------------*/
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Class constructor. (for Input only)
 * @param  port: name of the port (kGpioPortA ... kGpioPortH).
 * @param  pin:  pin number (kGpioPin0 ... kGpioPin15).
 * @param  pull: specifies the pull-up or pull-down activation for the pin.
 * @retval none.
 */
Gpi::Gpi (GpioPort port, GpioPin pin, GpioPull pull) :
        GpioBase(port, pin)
{
    GpioInit_t initStruct;

    this->write(this->m_pGPIOx, this->m_pinNumber, GpioState::kGpioStateLow);

    initStruct.Pin = pin;
    initStruct.Mode = GpioMode::kGpioModeInput;
    initStruct.Pull = pull;
    initStruct.Speed = GpioSpeed::kGpioSpeedMedium;
    this->init(this->m_pGPIOx, &initStruct);
}

/**
 * @brief  The function reads the pin state.
 * @param  none.
 * @retval true: in the case of pin is set.
 */
GpioState Gpi::getBit(void)
{
    return this->read(this->m_pGPIOx, this->m_pinNumber);
}

/**
 * @brief  The operator reads the pin state.
 * @param  value: the variable to save the pin state.
 * @retval none.
 */
void Gpi::operator>>(GpioState &value)
{
    value = this->getBit();
}

/* ---------------------------------------------------------------------------*/
/* ------------------------------- Gpo ---------------------------------------*/
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Class constructor. (for Output only)
 * @param  port: name of the port (kGpioPortA ... kGpioPortH).
 * @param  pin:  pin number (kGpioPin0 ... kGpioPin15).
 * @param  pull: specifies the pull-up or pull-down activation for the pin.
 * @retval none.
 */
Gpo::Gpo (GpioPort port, GpioPin pin, GpioPull pull) :
        GpioBase(port, pin)
{
    GpioInit_t initStruct;

    this->write(this->m_pGPIOx, this->m_pinNumber, GpioState::kGpioStateLow);

    initStruct.Pin = pin;
    initStruct.Mode = GpioMode::kGpioModeOutputPP;
    initStruct.Pull = pull;
    initStruct.Speed = GpioSpeed::kGpioSpeedMedium;
    this->init(this->m_pGPIOx, &initStruct);
}

/**
 * @brief  The function set or clear pin state.
 * @param  value: status of the pin 0 or 1 (false .. true).
 * @retval none.
 */
void Gpo::setBit(GpioState value)
{
    this->write(this->m_pGPIOx, this->m_pinNumber, \
        (GpioState)((bool)value ? (bool)GpioState::kGpioStateHigh : \
                    (bool)GpioState::kGpioStateLow));
}

/**
 * @brief   Toggle pin state.
 * @param   none.
 * @retval  none.
 */
void Gpo::toggleBit (void)
{
    this->toggle(this->m_pGPIOx, this->m_pinNumber);
}

/**
 * @brief The operator for set or clear pin state.
 * @param  value: status of the pin 0 or 1 (false .. true).
 * @retval none.
 */
void Gpo::operator<<(GpioState value)
{
    this->setBit(value);
}

/* ---------------------------------------------------------------------------*/
/* ------------------------------ Gpio ---------------------------------------*/
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Class constructor. (Mixed Input/Output)
 * @param  port: name of the port (kGpioPortA ... kGpioPortH).
 * @param  pin:  pin number (kGpioPin0 ... kGpioPin15).
 * @param  pull: specifies the pull-up or pull-down activation for the pin.
 * @retval none.
 */
Gpio::Gpio(GpioPort port, GpioPin pin, GpioPull pull) :
        GpioBase(port, pin)
{
    this->m_pinIsInput = true;
    this->m_pull = pull;

    this->setTypePinIn();
}

/**
 * @brief  The function  set or clear pin state
 * @param  value: status of the pin 0 or 1 (false .. true)
 * @retval none
 */
void Gpio::setBit (GpioState value)
{
    if (this->m_pinIsInput) {
        this->setTypePinOut();
    }

    this->write(this->m_pGPIOx, this->m_pinNumber, \
        (GpioState)((bool)value ? (bool)GpioState::kGpioStateHigh : \
                    (bool)GpioState::kGpioStateLow));
}

/**
 * @brief  The function read pin state
 * @param  none
 * @retval bool - false or true
 */
GpioState Gpio::getBit()
{
    if (!this->m_pinIsInput) {
        this->setTypePinIn();
    }

    return this->read(this->m_pGPIOx, this->m_pinNumber);
}

/**
 * @brief  The operator set or clear pin state
 * @param  value: status of the pin 0 or 1 (false .. true)
 * @retval none
 */
void Gpio::operator<<(GpioState value)
{
    this->setBit(value);
}

/**
 * @brief  Operator read pin state.
 * @param  value: the variable to save the pin state.
 * @retval none.
 */
void Gpio::operator>>(GpioState &value)
{
    value = this->getBit();
}

/**
 * @brief  Internal function - switch pin to input.
 * @param  none.
 * @retval none.
 */
void Gpio::setTypePinIn ()
{
    GpioInit_t initStruct;

    initStruct.Pin = this->m_pinNumber;
    initStruct.Mode = GpioMode::kGpioModeOutputPP;
    initStruct.Pull = this->m_pull;
    initStruct.Speed = GpioSpeed::kGpioSpeedMedium;
    this->init(this->m_pGPIOx, &initStruct);

    this->m_pinIsInput = true;
}

/**
 * @brief  Internal function - switch pin to output.
 * @param  none.
 * @retval none.
 */
void Gpio::setTypePinOut ()
{
    GpioInit_t initStruct;

    initStruct.Pin   = this->m_pinNumber;
    initStruct.Mode  = GpioMode::kGpioModeOutputPP;;
    initStruct.Pull  = this->m_pull;
    initStruct.Speed = GpioSpeed::kGpioSpeedMedium;
    this->init(this->m_pGPIOx, &initStruct);

    this->m_pinIsInput = false;
}