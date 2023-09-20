/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//һ����ת��

unsigned char rx_Data;		//#define(���滻)   u8    unsigned char   <====>   typedef��ȡ������ unsigned char      u8
char rx_str[256];
char rx_num = 0;	//�������ݵ��ֽ���
char clock = 0;
char s = 1;

char xunji = 0;
char distance = 0;
char ting = 0;
char zhuan = 0;
char avoid = 0;
char hui = 0;

int csb_value;  	//���������ų���������

//111
void left1()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 800); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 800); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_SET);	
}

void right1()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 800); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 800); 	  
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_SET);
}

void straight1()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 800); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 800); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);
}

void back1()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 800); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 800); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_SET);
}

void stop1()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 800); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 800); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);	
}

//222
void left2()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 650); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 650); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_SET);	
}

void right2()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 650); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 650); 	  
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_SET);
}

void straight2()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 650); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 650); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);
}

void back2()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 650); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 650); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_SET);
}

void stop2()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 650); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 650); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);	
}

//333
void left3()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_SET);	
}

void right3()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500); 	  
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN4_Pin, GPIO_PIN_SET);
}

void straight3()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);
}

void back3()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_SET);
}

void stop3()
{
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500); 	
	  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500); 	  
	  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);	
}


// �ض���printf����
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart1,temp,1,2);
	return 0;
}



//�����жϷ�������Ӧ�Ļص�������������������ķ���Ա����Ҫ�ڸú����ｻ��������񼴿ɡ�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance == USART1)//�ж������ĸ����ڴ������ж�
	{
		//����һ���ֽ����ݺ������������±�+1
		rx_str[rx_num++] = rx_Data;
		
		
	}
	
	//ͨ���������Ľ�����־�����ж������Ƿ��������

	if( rx_str[rx_num-1] == '#')
	{
		if(strstr( rx_str, "1234#")  != NULL)
		{
			clock = 1;
		}
		
		if(strstr( rx_str, "s#")  != NULL)
		{
			s++;
			if(s>3)
			{
				s = 1;
			}
		}
		
	
		
		if(clock == 1)
		{
			
			if(strstr( rx_str, "xunji#")  != NULL)
			{
				xunji = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}
			
			if(strstr( rx_str, "distance#")  != NULL)
			{
				distance = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}				
			
			if(strstr( rx_str, "ting#")  != NULL)
			{
				ting = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}	
			
			if(strstr( rx_str, "zhuan#")  != NULL)
			{
				zhuan = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}	
			
			if(strstr( rx_str, "avoid#")  != NULL)
			{
				avoid = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}	
			
			if(strstr( rx_str, "hui#")  != NULL)
			{
				hui = 1;
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;		
			}	
			
			if(s == 1)
			{
				if( strstr( rx_str, "straight#")  != NULL )
				{
					//�������
					straight1();		
				}
				
				if( strstr( rx_str, "back#")  != NULL )
				{
					//�������
					back1();		
				}

				if( strstr( rx_str, "left#")  != NULL )
				{
					//�������
					left1();		
				}

				if( strstr( rx_str, "right#")  != NULL )
				{
					//�������
					right1();		
				}

				if( strstr( rx_str, "stop#")  != NULL )
				{
					//�������
					stop1();		
				}		
				
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;			
			}

			if(s == 2)
			{
				if( strstr( rx_str, "straight#")  != NULL )
				{
					//�������
					straight2();		
				}
				
				if( strstr( rx_str, "back#")  != NULL )
				{
					//�������
					back2();		
				}

				if( strstr( rx_str, "left#")  != NULL )
				{
					//�������
					left2();		
				}

				if( strstr( rx_str, "right#")  != NULL )
				{
					//�������
					right2();		
				}

				if( strstr( rx_str, "stop#")  != NULL )
				{
					//�������
					stop2();		
				}		
				
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;			
			}

			if(s == 3)
			{
				if( strstr( rx_str, "straight#")  != NULL )
				{
					//�������
					straight3();		
				}
				
				if( strstr( rx_str, "back#")  != NULL )
				{
					//�������
					back3();		
				}

				if( strstr( rx_str, "left#")  != NULL )
				{
					//�������
					left3();		
				}

				if( strstr( rx_str, "right#")  != NULL )
				{
					//�������
					right3();		
				}

				if( strstr( rx_str, "stop#")  != NULL )
				{
					//�������
					stop3();		
				}		
				
				//�������
				memset(rx_str, 0, sizeof(rx_str));
				rx_num = 0;			
			}			
		}

				
		//�������
		memset(rx_str, 0, sizeof(rx_str));
		rx_num = 0;	

	}		
	
//����1�Ĵ���	
	//�ѽ��յ��������·�������������   0xffΪ��ʱʱ��
	HAL_UART_Transmit(&huart1, &rx_Data, 1, 0x0f);	
	
	//����ʹ�ܴ���1�����ж�
	HAL_UART_Receive_IT(&huart1, &rx_Data, 1);		

}




//��ʱ����ʱ��������ʱ������
void TIM_Delayus(int times)
{
	//�趨��ʱ���ĳ�ʼֵΪ0
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	
	//������ʱ������
	__HAL_TIM_ENABLE(&htim2);
	
	//�ȴ��������
	while( __HAL_TIM_GET_COUNTER(&htim2) < ( 1 * times) );
	
	//������ʱ������
	__HAL_TIM_DISABLE(&htim2);
}


//��ȡ����������
int csb_get_distance(void)
{
	//��������������
	int value = 0;
	
	//�趨��ʱ���ĳ�ʼֵΪ0
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	
	//оƬPC4---Trig���һ���ߵ�ƽ
	HAL_GPIO_WritePin(GPIOC, Trig_Pin, GPIO_PIN_SET);
	
	//���10us���ϣ���ʱ10us
	TIM_Delayus(15);

	//оƬPC4---Trig���һ���͵�ƽ
	HAL_GPIO_WritePin(GPIOC, Trig_Pin, GPIO_PIN_RESET);
	
	//���ϵ��ж�оƬ��PC5---Echo�����Ƿ�������һ���ߵ�ƽ������ԭ���ĵ͵�ƽ��ɸߵ�ƽ;
	while( 0 == HAL_GPIO_ReadPin(GPIOC, Echo_Pin) );
	
	//���PC5---Echo���ű���˸ߵ�ƽ��������ʱ����ʱ/����
	__HAL_TIM_ENABLE(&htim2);
	
	//���ϵ��ж�оƬ��PC5---Echo�����Ƿ����±�ɵ͵�ƽ
	while( 1 == HAL_GPIO_ReadPin(GPIOC, Echo_Pin) );
	
	//������ʱ���ļ���
	__HAL_TIM_DISABLE(&htim2);
	
	//��ȡ��ʱ���ļ���ֵ����ʱ��������ʱ�����÷�ƵΪ72����Ϊϵͳ������ʱ������Ƶ��Ϊ72MHz��	   \
	���趨��ƵΪ72�����Զ�ʱ����ʵ�ʹ���Ƶ��Ϊ��72M/72 = 1MHz����Ϊ��ʽ��T = 1/f�� T = 1/1M��   \
	���Եõ���ʱ��������Ϊ 1 us ������ʱ������һ�Σ�Ƶ�ʣ���Ҫ1usʱ�䣨���ڣ� \
	�ϰ������ = ʱ�� * �ٶȣ� �������Ծ���=(�ߵ�ƽʱ��*����(340M/S))/2;
	value = __HAL_TIM_GET_COUNTER(&htim2);
	
	//��ӡ ͨ������ֵ�͹�ʽ�����ϰ��� �ľ��� 1000us == 1ms == 0.001s
	printf("csb_get_distance == %dmm\r\n", value*340/1000/2 );
	
	//�����ϰ������
	return value*340/1000/2;
}

/*
//�û���д�Ļص����������ڴ���ѭ��ģ��ĵ�����ƣ�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if( EXTI->PR == right_Pin ) 	
	if(GPIO_Pin == left_Pin)//�����ĸ��ⲿ�жϴ�����
	{
		//�������ǰ��
		right1();

	}
	
	if(GPIO_Pin == right_Pin)
	{
		//�������ǰ��
		left1();

	}
}
*/

void bizhang()
{
	while(1)
	{
		csb_value = csb_get_distance();
		printf("csb_value == %dmm\r\n", csb_value);	
		
		straight3();
		
		if(csb_value/10 > 15 && csb_value/10 < 25)
		{
			right3();
			HAL_Delay(1000);

		}
		
		if(hui == 1)
		{
			stop3();
			avoid = 0;
			hui = 0;
			break;
		}
	}
}

void zouxian()
{
	while(1)
	{
		straight1();
		
		if(HAL_GPIO_ReadPin(GPIOC,right_Pin)|| HAL_GPIO_ReadPin(GPIOC,right_Pin) == 0)
		{
			straight1();
		}	
		
		if(rx_str[rx_num-1] == '#')
		{
			
			if(strstr( rx_str, "break#")  != NULL)
			{
				break;
			}
			
			//�������
			memset(rx_str, 0, sizeof(rx_str));
			rx_num = 0;
		}
	}

}



void ceju()
{
	while(1)
	{
	  csb_value = csb_get_distance();
	  printf("csb_value == %dmm\r\n", csb_value);	
	  
	  if(csb_value/10 > 15)
	  {
		  straight1();
	  }
	  else if(csb_value/10 < 10)
	  {
		  back1();
	  }	
	  else
	  {
		  stop1();
	  }	

			
	  if(hui == 1)
	  {
		  distance = 0;
		  hui = 0;
		  break;
	  }
			

	}

}

void diaotou()
{
	right3();
	HAL_Delay(1500);
	zhuan = 0;
	stop3();
	
}

void tingche()
{
	while(1)
	{
		csb_value = csb_get_distance();
		printf("csb_value == %dmm\r\n", csb_value);
		
		straight3();
		
		if(csb_value/10 > 7 && csb_value/10 < 15)
		{
			stop1();

			ting = 0;
			break;

		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //��������2���жϣ����ý��յ����ݴ����rx_Data�����ռ��ˣ�ֻ�ǲ����ռ��ַ����ָ�룩���������ݴ�СΪ1�ֽڣ���8bit
  //UART_Start_Receive_IT(&huart2, &rx_Data, 1);	
  HAL_UART_Receive_IT(&huart2, &rx_Data, 1);
  HAL_UART_Receive_IT(&huart1, &rx_Data, 1);
  
  //����PWM
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  
  //������ʱ��		   TIM_HandleTypeDef htim5;
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim2);
	 
  //������ʾ����֤����ͨ����OK
  printf("������Դ���Ѿ����ú���\r\n");
  
  
  stop1();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  { 
	  if(zhuan == 1)	
	  {
		  diaotou();
	  }
	  
	  if(ting == 1)
	  {
		  tingche();
	  }
	  
	  if(distance == 1)
	  {
		  ceju();
	  }

	  if(avoid == 1)
	  {
		  bizhang();
	  }
  }	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
