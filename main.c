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
#include "features.h"
#include "beep.h"
#include "math.h"
#include "svm_model.h"

// #define WINDOW_SIZE 150
// #define EFFECTIVE_WINDOW_SIZE 100
#define SEGMENT_DURASTION 2 // 窗口大小，单位为s

// 全局变量 - 在main函数外声明这些变量使它们可以在中断中访问

volatile u8 key_report = 1;             // 默认开启上报
volatile u8 key_predict = 1;            // 默认开启预测
volatile u8 display_flag = 0;           // 屏幕刷新标志
volatile u8 predict_flag = 0;           // 预测标志
volatile u8 alarm_flag = 1;             // 警报标志
volatile u8 alarm_peoceed = 0;          // 警报时长（次数x2）

SensorData window[WINDOW_SIZE];			// 时间窗口数据
volatile uint32_t window_head = 0;		// 窗口头索引

volatile int window_index = 0; 			// 窗口索引
float features[FEATURE_COUNT] = {0};    // 特征数组
float prediction;              			// 预测值

volatile uint32_t tim3_frames = 0;    	// 预期帧数
volatile uint32_t tim2_frames = 0;    	// 实际接收帧数

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口 
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
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
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}

// 初始化定时器2 - 每20ms触发一次
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能定时器2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置定时器参数 - 20ms
    // 72MHz/7200=10kHz, 10kHz计数到200为20ms
    TIM_TimeBaseStructure.TIM_Period = 199;
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

// 初始化定时器3 - 每500ms触发一次
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能定时器3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 配置定时器参数 - 500ms
    // 72MHz/7200=10kHz, 10kHz计数到5000为500ms
    TIM_TimeBaseStructure.TIM_Period = 4999;
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 使能TIM3中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

// TIM2中断服务函数 - 每20ms执行一次数据采集
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		SensorData current_data;
		short aacx, aacy, aacz, gyrox, gyroy, gyroz; 
		// 获取加速度和陀螺仪数据
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
		// 存储到时间窗口
		current_data.ax = (float)aacx * 0.01526f;
		current_data.ay = (float)aacy * 0.01526f;
		current_data.az = (float)aacz * 0.01526f;
		current_data.gx = (float)gyrox;
		current_data.gy = (float)gyroy;
		current_data.gz = (float)gyroz;

        window[window_head] = current_data;
        // Increment head pointer circularly
        window_head = (window_head + 1) % WINDOW_SIZE;

		// 上报
		if(key_report)
		{
			mpu6050_send_data(
				(short)current_data.ax, (short)current_data.ay, (short)current_data.az,
				(short)current_data.gx, (short)current_data.gy, (short)current_data.gz);
		}

		// 设置显示标志，主循环中处理显示
		display_flag = 1;
		
		// 更新漏帧计数
		if (tim2_frames > 25564) {
			tim2_frames = 0; // 重置计数器
		} tim2_frames++;
    }
}

// TIM3中断服务函数 - 每500ms执行一次特征提取和预测
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
		if(key_predict) {
        	predict_flag = 1; // 设置预测标志，主循环中处理预测和警报判断
		}

		// 更新帧计数
		if (tim3_frames > 25564) {
			tim3_frames = 0; // 重置计数器
		} tim3_frames++;
		LCD_ShowString(30, 280, 130, 16, 25, "FRAMES tim3:tim2:low");
		LCD_ShowNum(30, 300, tim3_frames, 5, 16);
		LCD_ShowNum(80, 300, tim2_frames, 5, 16);
		LCD_ShowNum(130, 300, (tim3_frames * 25 - tim2_frames), 5, 16);

		if (alarm_peoceed > 0 && alarm_flag) {
			BEEP = !BEEP; // 开启蜂鸣器
			LED0 = !LED0;
			alarm_peoceed--;
			if(alarm_peoceed < 1) {
				BEEP = 0; // 关闭蜂鸣器
				LED0 = 0; // 关闭LED
			}
		}

        // LED状态翻转

    }
}

// 显示传感器数据到LCD
void display_sensor_data(void)
{
    int display_temp;
    SensorData current_data = window[window_head];
    // 显示accx
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
    
    // 显示accy
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
    
    // 显示accz
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


// 计算最大峰-峰加速度幅度的RMS (C3特征)
float calculate_c3(float acc_x_max, float acc_x_min, float acc_y_max, float acc_y_min, 
				  float acc_z_max, float acc_z_min)
{
	float x_peak_to_peak = acc_x_max - acc_x_min;
	float y_peak_to_peak = acc_y_max - acc_y_min;
	float z_peak_to_peak = acc_z_max - acc_z_min;

	// 计算平方和
	float sum_squares = x_peak_to_peak * x_peak_to_peak + 
						y_peak_to_peak * y_peak_to_peak + 
						z_peak_to_peak * z_peak_to_peak;

	// 返回平方根
	return sqrt(sum_squares);
}

int get_features(SensorData* window, int window_index, float* features)
{
	int p, i;
	SensorData* window_ptr;
	// 初始化特征值

	float acc_x_max = -32768, acc_x_min = 32767;
	float acc_y_max = -32768, acc_y_min = 32767;
	float acc_z_max = -32768, acc_z_min = 32767;
	float rot_x_max = -32768, rot_x_min = 32767;
	float rot_y_max = -32768, rot_y_min = 32767;
	float rot_z_max = -32768, rot_z_min = 32767;

	for (p = 0; p < EFFECTIVE_WINDOW_SIZE; p++)
	{
		i = (window_index + p + WINDOW_SIZE - EFFECTIVE_WINDOW_SIZE) % WINDOW_SIZE; // 确保索引在窗口范围内
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
	features[1] = 0; // C6特征未使用
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

// 按照指定的最小最大值归一化特征
int standard(float* features, int size)
{
	// 归一化参数，顺序需与特征顺序一致
	const float min_vals[FEATURE_COUNT] = {
		5.0990195135927845f,      // acc_mag_min
		0.0048009578673858f,      // C5
		0.0117173029330312f,      // C4
		1.0669582934679311f,      // rot_z_std
		3.045089835194452f,       // C2
		3.0092784468936093f,      // C13
		0.0182726571685674f,      // C14
		0.0766f,                  // C11
		-301.0f,                  // acc_y_mean
		-1.4696113013998708f,     // acc_mag_kurtosis
		1.4302447343024898f,      // rot_y_std
		2.549252217550644f,       // C9
		-316.6f,                  // acc_z_mean
		0.0f,                     // acc_x_zero_cross
		// -1.94979797979798f,       // C6
		// 0.9630680142129112f,      // acc_x_std
		// 1.157826793132302f,       // acc_mag_std
		// 2.0110301897889125f,      // C8
		// 0.0f,                     // rot_y_zero_cross
		// 11.575836902790224f       // C3
	};
	const float max_vals[FEATURE_COUNT] = {
		276.2498868778049f,       // acc_mag_min
		1.5561000272115857f,      // C5
		2.235869092574518f,       // C4
		1808.2899269752072f,      // rot_z_std
		370.77581740312905f,      // C2
		368.7868252132574f,       // C13
		6.533341351865829f,       // C14
		9.5614f,                  // C11
		172.48f,                  // acc_y_mean
		85.38269105926449f,       // acc_mag_kurtosis
		2474.115633918512f,       // rot_y_std
		786.7371798135704f,       // C9
		262.1f,                   // acc_z_mean
		44.0f,                    // acc_x_zero_cross
		// 71678.3996969697f,        // C6
		// 521.8357197432923f,       // acc_x_std
		// 695.9808423309154f,       // acc_mag_std
		// 673.7468393509193f,       // C8
		// 34.0f,                    // rot_y_zero_cross
		// 7774.635039665849f        // C3
	};

	int i;
	for (i = 0; i < size && i < FEATURE_COUNT; i++)
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
	Model svm_model;               // SVM模型

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
    
    // 初始化定时器
    TIM2_Configuration();
    TIM3_Configuration();
    
    while(1)
    {
        // 按键扫描
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
			key_predict = !key_predict; // 设置预测标志
			if(key_predict)
				LCD_ShowString(30, 170, 200, 16, 16, "PREDICT ON ");
			else
				LCD_ShowString(30, 170, 200, 16, 16, "PREDICT OFF");
		} else if(key == WKUP_PRES)
		{
			// 切换警报状态
			alarm_flag = !alarm_flag;
			if(alarm_flag) 
				LCD_ShowString(30, 190, 200, 16, 16, "ALARM ON ");
			else 
				LCD_ShowString(30, 190, 200, 16, 16, "ALARM OFF");
		}

        // 显示数据
        if(display_flag)
        {
            display_flag = 0;
            display_sensor_data();
        }
        
        // 特征提取和预测
        if(predict_flag)
        {
            predict_flag = 0;
            // int current_head = window_head; // 当前窗口的起始位置
            // // 计算触发警报的阈值
            // alarm_threshold = (short)((aacx)/100 + abs(aacy)/100 + abs(aacz)/100);
            // // 显示阈值
            // LCD_ShowNum(30, 16, alarm_threshold, 5, 16);
            // if(alarm_threshold > 300)
            {
                // 进行预测
				LCD_ShowString(30, 0, 120, 16, 16, "predicting... ");
				// get_features(window, window_head, features);
				get_feature(window, window_head, SEGMENT_DURASTION, features);
				LCD_ShowString(30, 20, 100, 16, 16,"features:");
				standard(features, FEATURE_SIZE);
				u32 temp;
				for(int i = 0; i < FEATURE_SIZE; i++)
				{
					if (features[i] < 0){
						LCD_ShowChar(180-20, 20 + i * 20, '-', 16, 0);
						temp = -features[i]*100;
					} else {
						temp = features[i]*100; 
						LCD_ShowChar(180-20, 20 + i * 20, ' ', 16, 0);
					}
					LCD_ShowNum(180, 20 + i * 20, (u32)(temp), 5, 16);
				}
                prediction = predict(&svm_model, features, FEATURE_SIZE);

				LCD_ShowString(30, 0, 120, 16, 16, "prediction: ");
				// LCD_ShowChar(180-20, 0, '-', 16, 0);
				if(prediction < 0) {
						LCD_ShowChar(180-20, 0, '-', 16, 0);
						temp = -prediction*100;
					} else {
						temp = prediction*100;
						LCD_ShowChar(180-20, 0, ' ', 16, 0);
					}
                	LCD_ShowNum(180, 0, (u32)(temp), 5, 16);
                
				if(prediction > 1)
					// 触发警报
					for(short i = 0; i < 5; i++)
					{
						alarm_peoceed = 5; // 设置警报处理时间
					}
            }
        }
    }
}



