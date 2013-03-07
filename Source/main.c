/**
 ******************************************************************************
 * @file    Audio_playback_and_record/src/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    28-October-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <arm_math.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
enum bufstatus
{
	UPDATING,
	UPDATED,
	PROCESSING,
	PROCESSED
};

/* Private define ------------------------------------------------------------*/
/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2

#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

#define PDM_BUFF_SIZE      64
#define PCM_DECIMATION_SIZE	16
#define PCM_BUFF_SIZE            PCM_DECIMATION_SIZE * 20

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;

uint16_t CCR_Val = 1;

static uint16_t PDMBuffer[2][PDM_BUFF_SIZE];
static enum bufstatus PDMBufStatus[2];
static uint8_t PDMBufToWrite, PDMBufToProcess;

static uint32_t PDMBufPos = 0;

PDMFilter_InitStruct Filter;

float32_t PCMBuffer[PCM_BUFF_SIZE];

uint16_t PCMBufPos = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void WaveCapture_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* Connect SPI pins to AF5 */
	GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF );

	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF );
}

static void WaveCapture_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3 );
	/* Configure the SPI interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void WaveCapture_SPI_Init()
{
	I2S_InitTypeDef I2S_InitStructure;

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* SPI configuration */
	SPI_I2S_DeInit(SPI2 );
	I2S_InitStructure.I2S_AudioFreq = 32000;
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	/* Initialize the I2S peripheral with the structure above */
	I2S_Init(SPI2, &I2S_InitStructure);

	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

static void WaveCaptureInit(void)
{
	/* Enable CRC module */
	RCC ->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	/* Filter LP & HP Init */
	Filter.LP_HZ = 8000;
	Filter.HP_HZ = 10;
	Filter.Fs = 16000;
	Filter.Out_MicChannels = 1;
	Filter.In_MicChannels = 1;

	PDM_Filter_Init((PDMFilter_InitStruct *) &Filter);

	WaveCapture_GPIO_Init();
	WaveCapture_NVIC_Init();
	WaveCapture_SPI_Init();
}

static void WaveCaptureStart(void)
{
	PDMBufStatus[0] = UPDATING;
	PDMBufStatus[1] = PROCESSED;
	PDMBufPos = 0;
	PDMBufToWrite = PDMBufToProcess = 0;
	PCMBufPos = 0;
	I2S_Cmd(SPI2, ENABLE);
}

static void WaveCaptureStop(void)
{
	I2S_Cmd(SPI2, DISABLE);
}

static void ConvertPDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
	uint16_t gain;

	gain = 50;
	PDM_Filter_64_LSB((uint8_t *) PDMBuf, PCMBuf, gain, (PDMFilter_InitStruct *) &Filter);
}

uint32_t fftSize = 256;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

static void AnalyzePCMData(void)
{
	arm_status status;
	arm_cfft_radix4_instance_f32 S;
	float32_t maxValue;
	float32_t fPCMBuffer[PCM_BUFF_SIZE];
	float32_t PCMFFT[PCM_BUFF_SIZE/2];

	status = ARM_MATH_SUCCESS;
	arm_q15_to_float(PCMBuffer, fPCMBuffer, PCM_BUFF_SIZE);

	/* Initialize the CFFT/CIFFT module */
	status = arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, fPCMBuffer);


	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fPCMBuffer, PCMFFT, fftSize);

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(PCMFFT, fftSize, &maxValue, &testIndex);
}

int main(void)
{
	/* Initialize LEDS */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	/* Green Led On: start of application */
	STM_EVAL_LEDOn(LED4);
	STM_EVAL_LEDOn(LED5);

	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

	/* Initialize User Button */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	WaveCaptureInit();
	WaveCaptureStart();

	while (1)
	{
		if (PDMBufStatus[PDMBufToProcess] == UPDATED)
		{
			uint16_t PCMBuf[PCM_DECIMATION_SIZE];

			STM_EVAL_LEDToggle(LED4);

			PDMBufStatus[PDMBufToProcess] = PROCESSING;

			//convert to PCM
			ConvertPDMToPCM(PDMBuffer[PDMBufToProcess], PCMBuf);
			PDMBufStatus[PDMBufToProcess] = PROCESSED;
			PDMBufToProcess = (PDMBufToProcess + 1) % 2;

			memcpy(&PCMBuffer[PCMBufPos], PCMBuf, sizeof(PCMBuf));
			PCMBufPos += sizeof(PCMBuf);
			if (PCMBufPos >= PCM_BUFF_SIZE - 1)
			{
				AnalyzePCMData();
				PCMBufPos = 0;
			}

		}

		__WFI();
	}

	return 0;
}

void SPI2_IRQHandler(void)
{
	u16 app;

	/* Check if data are available in SPI Data register */
	if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE ) != RESET)
	{
		app = SPI_I2S_ReceiveData(SPI2 );

		PDMBuffer[PDMBufToWrite][PDMBufPos++] = HTONS(app);

		if (PDMBufPos == PDM_BUFF_SIZE)
		{
			uint8_t nextbuf;

			PDMBufPos = 0;
			nextbuf = (PDMBufToWrite + 1) % 2;
			if (PDMBufStatus[nextbuf] == PROCESSED)
			{
				PDMBufStatus[PDMBufToWrite] = UPDATED;
				PDMBufStatus[nextbuf] = UPDATING;
				PDMBufToWrite = nextbuf;
			}
			else
			{	//overflow

			}
		}
	}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
