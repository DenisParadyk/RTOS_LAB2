#define CPP_IMPLEMENTATION 1

  #include "main.h"

#if (CPP_IMPLEMENTATION == 0)
    #include "gpio.h"
#else
    #include "MyGpio.hpp"
#endif

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

#if (CPP_IMPLEMENTATION == 0)
    MX_GPIO_Init();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

    while (true) {}

#else
    // Green LED configuration.
    MyDrivers::GpioPin pwrLedPin = MyDrivers::GpioPin::kGpioPin12;
    MyDrivers::GpioPort pwrLedPort = MyDrivers::GpioPort::kGpioPortD;

    // Button configuration.
    MyDrivers::GpioPin btnPin = MyDrivers::GpioPin::kGpioPin0;
    MyDrivers::GpioPort btnPort = MyDrivers::GpioPort::kGpioPortA;

    // Pin is default pulled-up to 3.3V bus line.
    MyDrivers::GpioState btnState = MyDrivers::GpioState::kGpioStateHigh;

    // Pin to generate signal.
    MyDrivers::GpioPin signalPin = MyDrivers::GpioPin::kGpioPin12;
    MyDrivers::GpioPort signalPort = MyDrivers::GpioPort::kGpioPortD;

    // Object declaration.
    MyDrivers::Gpo signalObj(signalPort, signalPin);
    MyDrivers::Gpo pwrLedObj(pwrLedPort, pwrLedPin);
    MyDrivers::Gpi buttonObj(btnPort, btnPin);

    // Is device powered ON.
    pwrLedObj << MyDrivers::GpioState::kGpioStateHigh;

    // Timing settings.
    uint32_t impulse_t = 700;
    uint32_t impulse_d = 130;

    while (true) {

        buttonObj >> btnState;

        if (btnState == MyDrivers::GpioState::kGpioStateHigh) {
            HAL_Delay(impulse_t);
            signalObj << MyDrivers::GpioState::kGpioStateHigh;
            HAL_Delay(impulse_d);
            signalObj << MyDrivers::GpioState::kGpioStateLow;
        } else {
            signalObj << MyDrivers::GpioState::kGpioStateLow;
        }
    }

#endif

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
	Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */