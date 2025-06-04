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

#define WINDOW_SIZE 100

// ȫ�ֱ��� - ��main������������Щ����ʹ���ǿ������ж��з���
// volatile float pitch, roll, yaw;    
// volatile short temp;                  
volatile short aacx, aacy, aacz;        // ���ٶȴ�����ԭʼ����
volatile short gyrox, gyroy, gyroz;     // ������ԭʼ����

volatile u8 key_report = 1;             // Ĭ�Ͽ����ϱ�
volatile u8 key_predict = 1;            // Ĭ�Ͽ���Ԥ��
volatile u8 display_flag = 0;           // ��Ļˢ�±�־
volatile u8 predict_flag = 0;           // Ԥ���־
volatile u8 alarm_flag = 1;             // ������־
volatile u8 alarm_peoceed = 0;          // ����ʱ��������x2��

volatile float window[WINDOW_SIZE][6] = {0};  	// ʱ�䴰������
volatile int window_index = 0; 					// ��������
float features[14] = {0};      					// ��������
float prediction;              					// Ԥ��ֵ

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

//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
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
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
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

int put_window(float window[][6], int window_index, short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
/* return: next window_index */
{
	if (window_index >= WINDOW_SIZE) {
		// �������������������ɵ�һ��
		window_index = 0;
	}
	float* windowitem = window[window_index];
	windowitem[0] = (float)aacx;
	windowitem[1] = (float)aacy;
	windowitem[2] = (float)aacz;
	windowitem[3] = (float)gyrox;
	windowitem[4] = (float)gyroy;
	windowitem[5] = (float)gyroz;
	return window_index + 1;
}

// TIM2�жϷ����� - ÿ20msִ��һ�����ݲɼ�
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		volatile float* windowitem = window[window_index];
		// // ��ȡMPU6050����
		// MPU_Get_Accelerometer(windowitem, windowitem + 1, windowitem + 2);
		// MPU_Get_Gyroscope(windowitem + 3, windowitem + 4, windowitem + 5);

		// ��ȡ���ٶȺ�����������
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

		// �洢��ʱ�䴰��
		window_index = put_window(
			(float(*)[6])window, window_index, aacx, aacy, aacz, gyrox, gyroy, gyroz);

		// // �ϱ�����
		// if(report)
		// {
		// 	mpu6050_send_data(aacx, aacy, aacz, gyrox, gyroy, gyroz);
		// 	// usart1_report_imu(aacx, aacy, aacz, gyrox, gyroy, gyroz);
		// }
		
		// �ϱ�����
		if(key_report)
		{
			mpu6050_send_data(
				windowitem[0], windowitem[1], windowitem[2],
				windowitem[3], windowitem[4], windowitem[5]);
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
		LCD_ShowString(30, 280, 200, 16, 25, "FRAMES tim3:tim2:low");
		LCD_ShowNum(30, 300, tim3_frames, 5, 16);
		LCD_ShowNum(80, 300, tim2_frames, 5, 16);
		LCD_ShowNum(130, 300, (u32)(tim3_frames * 25 - tim2_frames), 5, 16);

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
    short display_temp;
    
    // ��ʾaccx
    display_temp = aacx * 10;
    if(display_temp < 0)
    {
        LCD_ShowChar(30+48, 220, '-', 16, 0);
        display_temp = -display_temp;
    }
    else 
        LCD_ShowChar(30+48, 220, ' ', 16, 0);
    
    LCD_ShowNum(30+48+8, 220, display_temp/10, 5, 16);
    LCD_ShowNum(30+48+40, 220, display_temp%10, 1, 16);
    
    // ��ʾaccy
    display_temp = aacy * 10;
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
    display_temp = aacz * 10;
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

int get_features(float window[][6], int window_index, float* features)
{
	int p, i;
	float* window_ptr;
	// ��ʼ������ֵ
	float acc_x_max = -32768, acc_x_min = 32767;
	float acc_y_max = -32768, acc_y_min = 32767;
	float acc_z_max = -32768, acc_z_min = 32767;
	float rot_x_max = -32768, rot_x_min = 32767;
	float rot_y_max = -32768, rot_y_min = 32767;
	float rot_z_max = -32768, rot_z_min = 32767;

	for (p = 0; p < WINDOW_SIZE; p++)
	{
		i = (window_index + p) % WINDOW_SIZE; // ȷ�������ڴ��ڷ�Χ��
		window_ptr = window[i];
		if (window_ptr[0] > acc_x_max) acc_x_max = window_ptr[0];
		if (window_ptr[0] < acc_x_min) acc_x_min = window_ptr[0];
		if (window_ptr[1] > acc_y_max) acc_y_max = window_ptr[1];
		if (window_ptr[1] < acc_y_min) acc_y_min = window_ptr[1];
		if (window_ptr[2] > acc_z_max) acc_z_max = window_ptr[2];
		if (window_ptr[2] < acc_z_min) acc_z_min = window_ptr[2];
		if (window_ptr[3] > rot_x_max) rot_x_max = window_ptr[3];
		if (window_ptr[3] < rot_x_min) rot_x_min = window_ptr[3];
		if (window_ptr[4] > rot_y_max) rot_y_max = window_ptr[4];
		if (window_ptr[4] < rot_y_min) rot_y_min = window_ptr[4];
		if (window_ptr[5] > rot_z_max) rot_z_max = window_ptr[5];
		if (window_ptr[5] < rot_z_min) rot_z_min = window_ptr[5];
	}

	features[0] = 0; //'C3';
	features[1] = 0; //'C6';
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

int standard(float* features, int size)
{
	int i;
	float max = 32767.0f; // �������ֵΪ32767
	for (i = 0; i < size; i++)
	{
		if (features[i] > max) max = features[i];
	}
	if (max == 0) return -1; // ��ֹ������
	for (i = 0; i < size; i++)
	{
		features[i] /= max;
	}
	return 0;
}

int main(void)
{
    u8 key;
    short alarm_threshold;
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
            
            // // ���㴥����������ֵ
            // alarm_threshold = (short)((aacx)/100 + abs(aacy)/100 + abs(aacz)/100);
            // // ��ʾ��ֵ
            // LCD_ShowNum(30, 16, alarm_threshold, 5, 16);
            // if(alarm_threshold > 300)
            {
                // ����Ԥ��
				LCD_ShowString(30, 0, 200, 16, 16, "predicting... ");
				get_features((float(*)[6])window, window_index, features);

				LCD_ShowString(150, 240, 100, 16, 16,"acc_x_max");
				LCD_ShowNum(150, 260, (u32)features[2], 5, 16);
				standard(features, FEATURE_SIZE);
                prediction = predict(&svm_model, features, FEATURE_SIZE);
				LCD_ShowString(30, 0, 200, 16, 16, "prediction: ");
                LCD_ShowNum(150, 0, (u32)prediction, 5, 16);
                
				if(prediction > 110)
					// ��������
					for(short i = 0; i < 5; i++)
					{
						alarm_peoceed = 5; // ���þ�������ʱ��
					}
            }
        }
    }
}



