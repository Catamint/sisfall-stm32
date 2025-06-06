#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "stm32f10x_tim.h"

#include "beep.h"
#include "math.h"
#include "svm_model.h"

#define WINDOW_SIZE 150
#define EFFECTIVE_WINDOW_SIZE 100

// ȫ�ֱ��� - ��main������������Щ����ʹ���ǿ������ж��з���

volatile u8 key_report = 1;             // Ĭ�Ͽ����ϱ�
volatile u8 key_predict = 1;            // Ĭ�Ͽ���Ԥ��
volatile u8 display_flag = 0;           // ��Ļˢ�±�־
volatile u8 predict_flag = 0;           // Ԥ���־
volatile u8 alarm_flag = 1;             // ������־
volatile u8 alarm_peoceed = 0;          // ����ʱ��������x2��

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} SensorData;

SensorData window[WINDOW_SIZE];			// ʱ�䴰������
volatile uint32_t window_head = 0;		// ����ͷ����

volatile int window_index = 0; 			// ��������
float features[14] = {0};      			// ��������
float prediction;              			// Ԥ��ֵ

volatile uint32_t tim3_frames = 0;    	// Ԥ��֡��
volatile uint32_t tim2_frames = 0;    	// ʵ�ʽ���֡��

//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1,c);  
} 

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ����� 
}

//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}

// ��ʼ����ʱ��2 - ÿ20ms����һ��
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // ʹ�ܶ�ʱ��2ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // ���ö�ʱ������ - 20ms
    // 72MHz/7200=10kHz, 10kHz������200Ϊ20ms
    TIM_TimeBaseStructure.TIM_Period = 199;
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // ʹ��TIM2�ж�
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // ����NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ������ʱ��
    TIM_Cmd(TIM2, ENABLE);
}

// ��ʼ����ʱ��3 - ÿ500ms����һ��
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // ʹ�ܶ�ʱ��3ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // ���ö�ʱ������ - 500ms
    // 72MHz/7200=10kHz, 10kHz������5000Ϊ500ms
    TIM_TimeBaseStructure.TIM_Period = 4999;
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // ʹ��TIM3�ж�
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // ����NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ������ʱ��
    TIM_Cmd(TIM3, ENABLE);
}

// TIM2�жϷ����� - ÿ20msִ��һ�����ݲɼ�
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		SensorData current_data;
		short aacx, aacy, aacz, gyrox, gyroy, gyroz; 
		// ��ȡ���ٶȺ�����������
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
		// �洢��ʱ�䴰��
		current_data.ax = (float)aacx / 8.0f;
		current_data.ay = (float)aacy / 8.0f;
		current_data.az = (float)aacz / 8.0f;
		current_data.gx = (float)gyrox;
		current_data.gy = (float)gyroy;
		current_data.gz = (float)gyroz;

        window[window_head] = current_data;
        // Increment head pointer circularly
        window_head = (window_head + 1) % WINDOW_SIZE;

		// �ϱ�
		if(key_report)
		{
			mpu6050_send_data(
				aacx, aacy, aacz, gyrox, gyroy, gyroz);
		}

		// ������ʾ��־����ѭ���д�����ʾ
		display_flag = 1;
		
		// ����©֡����
		if (tim2_frames > 25564) {
			tim2_frames = 0; // ���ü�����
		} tim2_frames++;
    }
}

// TIM3�жϷ����� - ÿ500msִ��һ��������ȡ��Ԥ��
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
		if(key_predict) {
        	predict_flag = 1; // ����Ԥ���־����ѭ���д���Ԥ��;����ж�
		}

		// ����֡����
		if (tim3_frames > 25564) {
			tim3_frames = 0; // ���ü�����
		} tim3_frames++;
		LCD_ShowString(30, 280, 130, 16, 25, "FRAMES tim3:tim2:low");
		LCD_ShowNum(30, 300, tim3_frames, 5, 16);
		LCD_ShowNum(80, 300, tim2_frames, 5, 16);
		LCD_ShowNum(130, 300, (tim3_frames * 25 - tim2_frames), 5, 16);

		if (alarm_peoceed > 0 && alarm_flag) {
			BEEP = !BEEP; // ����������
			alarm_peoceed--;
			if(alarm_peoceed < 1) {
				BEEP = 0; // �رշ�����
			}
		}

        // LED״̬��ת
        LED0 = !LED0;
    }
}

// ��ʾ���������ݵ�LCD
void display_sensor_data(void)
{
    int display_temp;
    SensorData current_data = window[window_head];
    // ��ʾaccx
    display_temp = (int)(current_data.ax * 10);
    if(display_temp < 0)
    {
        LCD_ShowChar(30+48, 220, '-', 16, 0);
        display_temp = -display_temp;
    }
    else 
        LCD_ShowChar(30+48, 220, ' ', 16, 0);
    
    LCD_ShowNum(30+48+8, 220, display_temp/10, 5, 16);
    LCD_ShowNum(30+48+42, 220, display_temp%10, 1, 16);
    
    // ��ʾaccy
    display_temp = (int)(current_data.ay * 10);
    if(display_temp < 0)
    {
        LCD_ShowChar(30+48, 240, '-', 16, 0);
        display_temp = -display_temp;
    }
    else 
        LCD_ShowChar(30+48, 240, ' ', 16, 0);
    
    LCD_ShowNum(30+48+8, 240, display_temp/10, 5, 16);
    LCD_ShowNum(30+48+40, 240, display_temp%10, 1, 16);
    
    // ��ʾaccz
    display_temp = (int)(current_data.az * 10);
    if(display_temp < 0)
    {
        LCD_ShowChar(30+48, 260, '-', 16, 0);
        display_temp = -display_temp;
    }
    else 
        LCD_ShowChar(30+48, 260, ' ', 16, 0);
    
    LCD_ShowNum(30+48+8, 260, display_temp/10, 5, 16);
    LCD_ShowNum(30+48+40, 260, display_temp%10, 1, 16);
}


// ��������-����ٶȷ��ȵ�RMS (C3����)
float calculate_c3(float acc_x_max, float acc_x_min, float acc_y_max, float acc_y_min, 
				  float acc_z_max, float acc_z_min)
{
	float x_peak_to_peak = acc_x_max - acc_x_min;
	float y_peak_to_peak = acc_y_max - acc_y_min;
	float z_peak_to_peak = acc_z_max - acc_z_min;

	// ����ƽ����
	float sum_squares = x_peak_to_peak * x_peak_to_peak + 
						y_peak_to_peak * y_peak_to_peak + 
						z_peak_to_peak * z_peak_to_peak;

	// ����ƽ����
	return sqrt(sum_squares);
}

// ����ˮƽ�����ϵķ���仯 (C6����)
// float calculate_c6(float window[][6], int window_index)
// {
// 	float mean_acc_x = 0;
// 	float mean_acc_x_prev = 0;
// 	float sum_acc_x = 0;
// 	float sum_acc_x_prev = 0;
// 	int count = 0;
// 	int i;
	
// 	// ���ȼ��㴰��������acc_x�ľ�ֵ
// 	for (i = 0; i < EFFECTIVE_WINDOW_SIZE; i++)
// 	{
// 		int idx = (window_index + i) % EFFECTIVE_WINDOW_SIZE;
// 		sum_acc_x += window[idx][0]; // acc_x�ڵ�һ��
// 		count++;
// 	}
	
// 	// �������Ϊ�գ�����0
// 	if (count == 0)
// 		return 0;
	
// 	mean_acc_x = sum_acc_x / count;
	
// 	// ����ǰһ�����ڵ�acc_x��ֵ (�Ѵ�������ƶ�һλ)
// 	count = 0;
// 	for (i = 0; i < EFFECTIVE_WINDOW_SIZE; i++)
// 	{
// 		int idx = (window_index + i - 1 + EFFECTIVE_WINDOW_SIZE) % EFFECTIVE_WINDOW_SIZE;
// 		sum_acc_x_prev += window[idx][0];
// 		count++;
// 	}
	
// 	if (count == 0)
// 		return 0;
	
// 	mean_acc_x_prev = sum_acc_x_prev / count;
	
// 	// C6����: ǰһʱ�̾�ֵ���Ե�ǰʱ�̾�ֵ
// 	return mean_acc_x_prev * mean_acc_x;
// }

int get_features(SensorData* window, int window_index, float* features)
{
	int p, i;
	SensorData* window_ptr;
	// ��ʼ������ֵ

	float acc_x_max = -32768, acc_x_min = 32767;
	float acc_y_max = -32768, acc_y_min = 32767;
	float acc_z_max = -32768, acc_z_min = 32767;
	float rot_x_max = -32768, rot_x_min = 32767;
	float rot_y_max = -32768, rot_y_min = 32767;
	float rot_z_max = -32768, rot_z_min = 32767;

	for (p = 0; p < EFFECTIVE_WINDOW_SIZE; p++)
	{
		i = (window_index + p + WINDOW_SIZE - EFFECTIVE_WINDOW_SIZE) % WINDOW_SIZE; // ȷ�������ڴ��ڷ�Χ��
		window_ptr = &window[i];
		if (window_ptr->ax > acc_x_max) acc_x_max = window_ptr->ax;
		if (window_ptr->ax < acc_x_min) acc_x_min = window_ptr->ax;
		if (window_ptr->ay > acc_y_max) acc_y_max = window_ptr->ay;
		if (window_ptr->ay < acc_y_min) acc_y_min = window_ptr->ay;
		if (window_ptr->az > acc_z_max) acc_z_max = window_ptr->az;
		if (window_ptr->az < acc_z_min) acc_z_min = window_ptr->az;
		if (window_ptr->gx > rot_x_max) rot_x_max = window_ptr->gx;
		if (window_ptr->gx < rot_x_min) rot_x_min = window_ptr->gx;
		if (window_ptr->gy > rot_y_max) rot_y_max = window_ptr->gy;
		if (window_ptr->gy < rot_y_min) rot_y_min = window_ptr->gy;
		if (window_ptr->gz > rot_z_max) rot_z_max = window_ptr->gz;
		if (window_ptr->gz < rot_z_min) rot_z_min = window_ptr->gz;
	}

	features[0] = calculate_c3(acc_x_max, acc_x_min, acc_y_max, acc_y_min, acc_z_max, acc_z_min);
	features[1] = 0; // C6����δʹ��
	features[2] = acc_x_max;
	features[3] = acc_x_min;
	features[4] = acc_y_max;
	features[5] = acc_y_min;
	features[6] = acc_z_max;
	features[7] = acc_z_min;
	features[8] = rot_x_max;
	features[9] = rot_x_min;
	features[10] = rot_y_max;
	features[11] = rot_y_min;
	features[12] = rot_z_max;
	features[13] = rot_z_min;
	return 0;
}

// ����ָ������С���ֵ��һ������
int standard(float* features, int size)
{
	// ��һ��������˳����������˳��һ��
	// features[0]: C3��������ֵRMS����[0, 65535]
	// features[1]: C6������δ�ã���[0, 1]
	// features[2]: acc_x_max, [-32768, 32767]
	// features[3]: acc_x_min, [-32768, 32767]
	// features[4]: acc_y_max, [-32768, 32767]
	// features[5]: acc_y_min, [-32768, 32767]
	// features[6]: acc_z_max, [-32768, 32767]
	// features[7]: acc_z_min, [-32768, 32767]
	// features[8]: rot_x_max, [-7271, 3804]
	// features[9]: rot_x_min, [-7271, 3804]
	// features[10]: rot_y_max, [-4876, 6314]
	// features[11]: rot_y_min, [-4876, 6314]
	// features[12]: rot_z_max, [-10024, 3002]
	// features[13]: rot_z_min, [-10024, 3002]

	const float min_vals[14] = {
		0.0f,      // C3
		0.0f,      // C6
		-2768.0f, // acc_x_max
		-32768.0f, // acc_x_min
		-2768.0f, // acc_y_max
		-32768.0f, // acc_y_min
		-2768.0f, // acc_z_max
		-32768.0f, // acc_z_min
		-32768.0f,  // rot_x_max
		-32768.0f,  // rot_x_min
		-32768.0f,  // rot_y_max
		-32768.0f,  // rot_y_min
		-32768.0f, // rot_z_max
		-32768.0f  // rot_z_min
	};
	const float max_vals[14] = {
		32767.0f,  // C3
		1.0f,      // C6
		32767.0f,  // acc_x_max
		2767.0f,  // acc_x_min
		32767.0f,  // acc_y_max
		2767.0f,  // acc_y_min
		32767.0f,  // acc_z_max
		2767.0f,  // acc_z_min
		32767.0f,   // rot_x_max
		32767.0f,   // rot_x_min
		32767.0f,   // rot_y_max
		32767.0f,   // rot_y_min
		32767.0f,   // rot_z_max
		32767.0f    // rot_z_min
	};

	int i;
	for (i = 0; i < size && i < 14; i++)
	{
		float minv = min_vals[i];
		float maxv = max_vals[i];
		float denom = maxv - minv;
		if (denom == 0) {
			features[i] = 0.0f;
		} else {
			features[i] = (features[i] - minv) / denom;
			if (features[i] < 0.0f) features[i] = 0.0f;
			if (features[i] > 1.0f) features[i] = 1.0f;
		}
	}
	return 0;
}

int main(void)
{
    u8 key;
	Model svm_model;               // SVMģ��

    init_svm_params(&svm_model);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(500000);
    delay_init();
    usmart_dev.init(72);
    LED_Init();
    KEY_Init();
    LCD_Init();
    MPU_Init();
    BEEP_Init();
    
    POINT_COLOR = RED;
    LCD_ShowString(30, 50, 200, 16, 16, "a program");
    LCD_ShowString(30, 70, 200, 16, 16, "I'm tired");
    
    while(mpu_dmp_init())
    {
        LCD_ShowString(30, 130, 200, 16, 16, "MPU6050 Error");
        delay_ms(200);
        LCD_Fill(30, 130, 239, 130+16, WHITE);
        delay_ms(200);
    }
    
    LCD_ShowString(30, 130, 200, 16, 16, "MPU6050 OK");
	LCD_ShowString(30, 150, 200, 16, 16, "UPLOAD ON ");
	LCD_ShowString(30, 170, 200, 16, 16, "PREDICT ON ");
    POINT_COLOR = BLUE;
    LCD_ShowString(30, 220, 200, 16, 16, "accx: ");
    LCD_ShowString(30, 240, 200, 16, 16, "accy: ");
    LCD_ShowString(30, 260, 200, 16, 16, "accz: ");
    
    // ��ʼ����ʱ��
    TIM2_Configuration();
    TIM3_Configuration();
    
    while(1)
    {
        // ����ɨ��
        key = KEY_Scan(0);

        if(key == KEY0_PRES)
        {
            key_report = !key_report;
            if(key_report)
                LCD_ShowString(30, 150, 200, 16, 16, "UPLOAD ON ");
            else
                LCD_ShowString(30, 150, 200, 16, 16, "UPLOAD OFF");
        } else if(key == KEY1_PRES)
		{
			key_predict = !key_predict; // ����Ԥ���־
			if(key_predict)
				LCD_ShowString(30, 170, 200, 16, 16, "PREDICT ON ");
			else
				LCD_ShowString(30, 170, 200, 16, 16, "PREDICT OFF");
		} else if(key == WKUP_PRES)
		{
			// �л�����״̬
			alarm_flag = !alarm_flag;
			if(alarm_flag) 
				LCD_ShowString(30, 190, 200, 16, 16, "ALARM ON ");
			else 
				LCD_ShowString(30, 190, 200, 16, 16, "ALARM OFF");
		}

        // ��ʾ����
        if(display_flag)
        {
            display_flag = 0;
            display_sensor_data();
        }
        
        // ������ȡ��Ԥ��
        if(predict_flag)
        {
            predict_flag = 0;
            int current_head = window_head; // ��ǰ���ڵ���ʼλ��
            // // ���㴥����������ֵ
            // alarm_threshold = (short)((aacx)/100 + abs(aacy)/100 + abs(aacz)/100);
            // // ��ʾ��ֵ
            // LCD_ShowNum(30, 16, alarm_threshold, 5, 16);
            // if(alarm_threshold > 300)
            {
                // ����Ԥ��
				LCD_ShowString(30, 0, 120, 16, 16, "predicting... ");
				get_features(window, current_head, features);
				LCD_ShowString(30, 20, 100, 16, 16,"features:");
				standard(features, FEATURE_SIZE);
				for(int i = 0; i < FEATURE_SIZE; i++)
				{
					if (features[i] < 0){
						LCD_ShowChar(180-20, 20 + i * 20, '-', 16, 0);
						features[i] = -features[i]; // ȷ������ֵΪ��
					} else 
						LCD_ShowChar(180-20, 20 + i * 20, ' ', 16, 0);
					LCD_ShowNum(180, 20 + i * 20, (u32)(features[i]*100), 5, 16);
				}
                prediction = predict(&svm_model, features, FEATURE_SIZE);

				LCD_ShowString(30, 0, 120, 16, 16, "prediction: ");
				if(prediction < 0) {
						LCD_ShowChar(180-20, 0, '-', 16, 0);
						prediction = -prediction;
					} else 
						LCD_ShowChar(180-20, 0, ' ', 16, 0);
                LCD_ShowNum(180, 0, (u32)(prediction*100), 5, 16);
                
				if(prediction > 100)
					// ��������
					for(short i = 0; i < 5; i++)
					{
						alarm_peoceed = 5; // ���þ�������ʱ��
					}
            }
        }
    }
}



