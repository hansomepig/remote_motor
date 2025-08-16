/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "lwip/dns.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/apps/mqtt.h"
#include <lwip/sockets.h>
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void pid_thread(void *param);
static void MqttTestTask(void *pvParameter);
static void httpserver_thread(void *param);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	
	//xTaskCreate(pid_thread, "pid_thread", 512, NULL, 0, NULL);
	xTaskCreate(MqttTestTask, "MqttTestTask", 512, NULL, 0, NULL);
	xTaskCreate(httpserver_thread, "httpserver_thread", 512, NULL, 0, NULL);
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* PID Task */
#include "motor/motor.h"
#include "pid/pid.h"
#include "pid/pid_protocol.h"
#include "tim.h"

pid_t pid;		// ����һ��pid���������Ҫ�໷������������

static void pid_thread(void *param)
{
	// PID��ʼ��
//	PID_param_init(&pid, 0.21, 0.07, 0.32);
//	PID_param_init(&pid, 0.2, 0.08, 0.01);
	PID_param_init(&pid, 0.65, 0.55, 0.01);
	protocol_init();
	
	int target = 150;		// ����Ŀ��ֵ
	PID_set_target(&pid, target);
	set_computer_value(SEND_TARGET_CMD, 1, &target, 1);
	
	unsigned int period = 1;		// ���ø������ڣ�1s��
	set_computer_value(SEND_PERIOD_CMD, 1, &period, 1);
	
	float buf[3];
	PID_get_pid(&pid, buf);			// ��ʾ��ʼPID����
	set_computer_value(SEND_P_I_D_CMD, 1, buf, 3*sizeof(float));
	
	// ������ʱ��1������
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	
	// ������ʱ��2������pwm���
	HAL_TIM_Base_Start(&htim2);
	htim1.Instance->CNT = 0x7fff;
	
	// ������ʱ��3�����ڶ�ʱ
	HAL_TIM_Base_Start_IT(&htim3);
	
	// �������
	set_computer_value(SEND_START_CMD, 1, NULL, 0);
	motor_start();
	
	while (1)
  {
		if(RecvFlag) {
			RecvFlag = 0;
			
			uint16_t buf_len = 0;
			for(buf_len=sizeof(RecvBuffer)-1; (buf_len>0) && (!RecvBuffer[buf_len]); buf_len--);
			buf_len++;
			
			protocol_data_recv(RecvBuffer, buf_len);
			receiving_process();
			
			while(buf_len--) RecvBuffer[buf_len] = 0;
		}
		
		// ���ڹ۲�PID�㷨ִ�й���
		int temp = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
		set_computer_value(SEND_PERIOD_CMD, 1, &temp, 1);
		
		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}


/* MQTT Task */
#define PORT            18830
#define HOST_NAME       "mq.tongxinmao.com"  		// ͨ��è����������
#define IP_ADDR         "120.76.100.197"			// �ֶ���ȡ��IP��ַ

char *ClientId = "MqttTest";
char *ClientUser = NULL;
char *ClientPass = NULL;

char SubscriTopic[] = "set_target";		// Ҫ���ĵ�����
char PublishTopic[] = "reply";			// Ҫ������Ϣ������
char payload[] = "currrent speed: %f rpm";					// Ҫ��������Ϣ

static uint8_t topic_flag = 0xff;
#define TOPIC_SPEED	1
#define TOPIC_SS	2

// �ͻ������ӵ�����״̬
void Cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    // Դ��������״̬������
    switch((uint16_t)status)
    {
        case MQTT_CONNECT_ACCEPTED:;
        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:;
        case MQTT_CONNECT_REFUSED_IDENTIFIER:;
        case MQTT_CONNECT_REFUSED_SERVER:;
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:;
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:;
        case MQTT_CONNECT_DISCONNECTED:;
        case MQTT_CONNECT_TIMEOUT:;
    }
}

// ���Ļ��߷���ʱ�Ľ���ص�
void cb(void *arg, err_t err)
{
    char* p = arg;
    if(strcmp(p,"publish") == 0) {		// �ɹ�������һ����Ϣ
        return;
    }	else if(strcmp(p,"subscribe") == 0) {		// �ɹ�������һ������
        return;
    }
}

// ��������󣬿ͻ����յ���������͵�����ص�
void mipcb(void *arg, const char *topic, u32_t tot_len)
{
		if(strcmp(topic,"set_target") == 0) {		// ���õ��ת��
				topic_flag = TOPIC_SPEED;
		} else if(strcmp(topic,"set_ss") == 0) {		// ���õ����ͣ
				topic_flag = TOPIC_SS;
		}
		
    return;
}

// �������⣬�ͻ����յ���������͵��������Ϣ�ص�
void midcb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
		int num = atoi((const char *)data);
	
		switch(topic_flag) {
			case TOPIC_SPEED:		// �����ٶ�Ŀ��ֵ
				PID_set_target(&pid, num);
				set_computer_value(SEND_TARGET_CMD, 1, &num, 1);
				break;
			case TOPIC_SS:		// ���õ����ͣ
				if(num == 0) motor_stop();
				else if(num == 1) motor_start();
				break;
			default:
		}
    return;
}

static void MqttTestTask(void *pvParameter)
{
    // ������ DNS ������
    ip_addr_t dns_server;
    IP4_ADDR(&dns_server, 8, 8, 8, 8);
    dns_setserver(0, &dns_server); // �� 1 �� DNS
    
    // ��ȡ MQTT ��������ַ
    char* host_ip = IP_ADDR;
    ip4_addr_t dns_ip;
    netconn_gethostbyname(HOST_NAME, &dns_ip);
    host_ip = ip_ntoa(&dns_ip);
    printf("host name : %s , host_ip : %s\n", HOST_NAME, host_ip);
    ip4_addr_t MqttServer = { inet_addr(host_ip) };
    
    while(1)
    {
        // ����һ��MQTT�ͻ���
        mqtt_client_t* MqttClient = mqtt_client_new();
        
        // �½�һ���ͻ������ӵ���������Ҫ����Ϣ�飬�Լ������Ӧ����Ϣ
        // �ͻ���ID �û��� �û����� ����ʱ�� ��Ը���� ��Ը��Ϣ ��Ը��Ϣ�ȼ� ������Ը
        struct mqtt_connect_client_info_t Ci = {ClientId, ClientUser, ClientPass, 60, NULL, NULL, 0, 0};
        // ���ӵ�������
        mqtt_client_connect(MqttClient, &MqttServer, PORT, Cb, NULL, &Ci);
        
        // ����ֻΪ���ԣ���Ϊ��RTOS��Ҫ���ȣ�Ҫ��Э��ջ���� �ϱߵ����ӷ����������������� ���������ʱ
        vTaskDelay(1000);
        
        // ע�ᶩ����Ϣ���յ����ݵĻص�
        mqtt_set_inpub_callback(MqttClient, mipcb, midcb, NULL);
        // �������⣬��һ���������õ��ת�٣��ڶ����������õ����ͣ
        //mqtt_subscribe(MqttClient, SubscriTopic, 0, cb, "subscribe");
				mqtt_subscribe(MqttClient, "set_target", 0, cb, "subscribe");
				mqtt_subscribe(MqttClient, "set_ss", 0, cb, "subscribe");
        
        // ��ѯ�Ƿ����ӵ�������
        while( mqtt_client_is_connected(MqttClient) )
        {
						char msg[100];
            snprintf(msg, 100, payload, speed);
						msg[strlen(msg)] = 0;
					
            vTaskDelay(1000);
            // ������Ϣ������Ϊ�����ǰת��
            mqtt_publish(MqttClient, PublishTopic, msg, strlen(msg), 0, 0, cb, "publish");
        }
        
        // �Ͽ��ͻ��˵������������� �����ͻ���
        mqtt_disconnect(MqttClient);
        // �ͷŴ����Ŀͻ�����ռ�ڴ�
        mqtt_client_free(MqttClient);
        
        vTaskDelay(1000);
    }
}

/* HTTP Task */
const static char http_html_hdr[] =
    "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\nConnection: close\r\n"
"<!DOCTYPE html>"
"<html>";

const unsigned char Led1On_Data[] =
    "<HTML> \
<head><title>HTTP MOTOR Control</title></head> \
<center> \
<p> \
<font size=\"6\">MOTOR<font style = \"color:red\">�Ѵ򿪣�</font> \
<form method=post action=\"off\" name=\"motorform\"> \
<input type=\"submit\" value=\"�ر�\" style=\"width:80px;height:30px;\"></form> ";

//��MOTOR��ʱ�����������������html��Ϣ����ʾ�������ͼ15-7��ʾ
const unsigned char Led1Off_Data[] =
    "<HTML> \
<head><title>HTTP MOTOR Control</title></head> \
<center> \
<p> \
<font size=\"6\">MOTOR<font style = \"color:red\">�ѹرգ�</font> \
<form method=post action=\"on\" name=\"motorform\"> \
<input type=\"submit\" value=\"��\" style=\"width:80px;height:30px;\"></form> ";

//const unsigned char Speed_Data[] = 
//	"<p> \
//<font size=\"6\">cur speed: <font style = \"color:red\">%d</font> \
//<p> \
//<input type=\"number\" min=0 id=\"speed\" name=\"speed\" placeholder=\"set motor speed\" style=\"width:150px;height:30px;\"></form> \
//<form method=post action=\"set\" name=\"motorform\"> \
//<input type=\"submit\" value=\"����Ŀ���ٶ�\" style=\"width:100px;height:30px;\"></form> \
//<p> \
//</center> \
//</HTML> ";

const unsigned char Speed_Data[] = 
"<form method=post action=\"set\">"
"  <label for='number'>���Ŀ���ٶ�:</label>"
"  <input type='number' id='number' name='number' min='0'>"
"  <input type='submit' value='����'>"
"</form>";

static const char http_index_html[] =
    "<html><head><title>Congrats!</title></head>\
<body><h2 align=\"center\">Welcome to HandSomePig lwIP HTTP Server!</h2>\
<p align=\"center\">This is a small test page : http control motor.</p>\
<p align=\"center\"><a href=\"http://www.baidu.com/\">\
<font size=\"6\"> ˧�� </font> </a></p>\
</body></html>";

#define MOTOR_OFF	0
#define MOTOR_ON	1

// ��¼MOTOR״̬
static uint8_t motor_on = MOTOR_OFF;

/*������ҳ����*/
void httpserver_send_html(struct netconn *conn, uint8_t motor_status)
{
    //��������ͷ
    netconn_write(conn, http_html_hdr,
                sizeof(http_html_hdr)-1, NETCONN_NOCOPY);

    /* ����MOTOR״̬�����Ͳ�ͬ��MOTOR���� */
    if (motor_status == MOTOR_ON)
        netconn_write(conn, Led1On_Data,
                    sizeof(Led1On_Data)-1, NETCONN_NOCOPY);
    else
        netconn_write(conn, Led1Off_Data,
                    sizeof(Led1Off_Data)-1, NETCONN_NOCOPY);

		netconn_write(conn, Speed_Data,
                sizeof(Speed_Data)-1, NETCONN_NOCOPY);
		
    netconn_write(conn, http_index_html,
                sizeof(http_index_html)-1, NETCONN_NOCOPY);

}

static void httpserver_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    /* �ȴ��ͻ��˵��������� */
    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK)
    {
        netbuf_data(inbuf, (void**)&buf, &buflen);
        
        if (buflen>=5 &&
                buf[0]=='G' &&
                buf[1]=='E' &&
                buf[2]=='T' &&
                buf[3]==' ' &&
                buf[4]=='/' )		// ��GET������
        {
            /* �������� */
            httpserver_send_html(conn, motor_on);
        }
        
        else if (buflen>=8&&buf[0]=='P'&&buf[1]=='O'
                &&buf[2]=='S'&&buf[3]=='T')		//��POST�� ����
        {
            if (buf[6]=='o'&&buf[7]=='n')
            {
                //�����MOTOR
                motor_on = MOTOR_ON;
			                
				        // �������
								motor_start();
                
            }
            else if (buf[6]=='o'&&buf[7]=='f'&&buf[8]=='f')
            {
                //����ر�MOTOR
                motor_on = MOTOR_OFF;
			                
				        // �������
                motor_stop();
            }
						else if (buf[6]=='s'&&buf[7]=='e'&&buf[8]=='t')
						{
								printf("%s\n", buf);
							
								// ���������忪ʼλ��
								char *post_data = strstr(buf, "number=");
								if (post_data != NULL) {
										printf("%s\n", post_data);
										
										// ���������� (��ʽΪ number=123)
										int number = 0;
										if (sscanf(post_data, "number=%d", &number) == 1) {
												// ������յ�������
												printf("���յ�����: %d\n", number);
												
												// ������Ӧҳ��
												char response[256];
												snprintf(response, sizeof(response),
														"HTTP/1.1 200 OK\r\n"
														"Content-Type: text/html\r\n"
														"Connection: close\r\n"
														"\r\n"
														"<html><body>"
														"<h1>�ɹ���������</h1>"
														"<p>���յ�������: %d</p>"
														"<a href='/'>����</a>"
														"</body></html>", number);
												
												netconn_write(conn, response, strlen(response), NETCONN_COPY);
										}
								}
						}
						
            //��������
            httpserver_send_html(conn, motor_on);
        }

        netbuf_delete(inbuf);
    }
        
    /* �ر����� */
    netconn_close(conn);
}

static void  httpserver_thread(void *arg)
{
    struct netconn *conn, *newconn;
    err_t err;
    LWIP_UNUSED_ARG(arg);

    /* �������ӽṹ */
    conn = netconn_new(NETCONN_TCP);
    LWIP_ERROR("http_server: invalid conn", (conn != NULL), return;);
    
		/* Ĭ�Ͽ������ */
    motor_on = MOTOR_ON;

    /* ��IP��ַ��˿ں�*/
    netconn_bind(conn, NULL, 80);

    /* ���� */
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK)
        {
            httpserver_serve(newconn);
            netconn_delete(newconn);
        }
				
				vTaskDelay(1000);
    } while (err == ERR_OK);

    netconn_close(conn);
    netconn_delete(conn);
}


/* USER CODE END Application */
