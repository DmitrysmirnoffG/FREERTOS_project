#include "Filter.h"
#include "main.h"
#include "FastMath.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

void clear_array(char* array)
{
	while(*array != '\0')
		*array++ = '\0';
}

void print_to_USART(char* str)
{
	while(*str != '\0')
	{
		while((USART2->ISR & USART_ISR_TC) == 0) {}
		USART2->TDR = (uint8_t)*str;
		str++;
	}
//	while((USART2->ISR & USART_ISR_TC) == 0) {}
//		USART2->TDR = 0x0A;
}

double normal_distribution(void)
{
  double x = (double)rand() / RAND_MAX;
  double y = (double)rand() / RAND_MAX;
  double z = FastSqrt(-2*logf(x))*FastCos(2*PI*y)/30;
  return z;
}

double uniform_distribution(void)
{
	double x = (rand()%1001 - 500)/10000.0f;
	return x;
}

void make_noise(float* signal_noise,int N)
{
	for(uint16_t i = 0; i < N; i++)
	{
		if(NOISE == 1)
			signal_noise[i] = (2.0f + FastSin(i*PI/180) + NOISE_AMP*normal_distribution())*COEF;
		else
			signal_noise[i] = (2.0f + FastSin(i*PI/180) + NOISE_AMP*uniform_distribution())*COEF;
	}
}

void Filter(float* input,float* output,int N,int window)
{
	int i,j,z,k1,k2,hw;
	float tmp;
	if(fmod(window,2) == 0) 
		window++;
	hw = (window - 1)/2;
	output[0] = 1638.0;
	
	for (i = 1;i < N - 1;i++)
	{
	 tmp = 0;
	 if(i < hw)			
	 {
		 k1 = 0;
		 k2 = 2*i;
		 z = k2 + 1;
	 }
	 else if((i + hw)>(N - 1))
	 {
		 k1 = i - N + i + 1;
		 k2 = N - 1;
		 z = k2 - k1 + 1;
	 }
	 else
	 {
		 k1 = i - hw;
		 k2 = i + hw;
		 z = window;
	 }

	 for (j = k1;j <= k2;j++)
	   tmp = tmp + input[j];
	 output[i] = tmp/z;
	}
	output[N - 1] = 1638.0;
}

void transmit_to_USART(float* signal_noise,float* signal_filtered,int N)
{
	char signal_n[15],signal_f[15];
	float Sum_n = 0,Sum_f = 0;
	for(uint16_t i = 0; i < N; i++)
	{
		Sum_n += signal_noise[i];
		Sum_f += signal_filtered[i];
	}
	sprintf(signal_n,"%f",Sum_n);
	sprintf(signal_f,"%f",Sum_f);
	print_to_USART("\nСигнал с шумом: ");
	print_to_USART(signal_n);
	print_to_USART(" Отфильтрованный сигнал: ");
	print_to_USART(signal_f);
}
