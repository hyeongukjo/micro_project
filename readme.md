# **마이크로 프로세서 프로젝트 진행 사항 발표**
### 반려동물 자동 급식기 Smart Feeder System
<br>

# 1. 초기 계획 대비 변경 사항
> 기존 계획된 센서

    MPU-6050
* 복잡한 데이터: true/false의 단순 신호가 아닌, 6축의 자이로/가속도의 데이터 스트림만 출력. 
* 알고리즘 복잡성: "섭취가 무엇인가?"를 코드로 직접 정의해야 함. (예: 5초간 0.5G 이상의 진동이 10회 이상 감지)
* 디버깅 난이도 (고스트 현상): 문제가 발생 시 TWI 통신 오류인지, 알고리즘의 **임계값(Threshold)** 설정 오류인지 파악하기 어려움.

<br>

> 교체한 센서

    IR 적외선 근접 센서

* 작동 원리:

    * 센서가 적외선을 쏘고, 물체(반려동물 머리)에 반사되어 돌아오는 빛을 감지함.


* MPU-6050 대비 장점:

    * 쉬운 통신: I2C(TWI) 통신이 필요 없음.

    * 간단한 연결: Vcc(5V), GND, 그리고 'Analog Out' 핀 1개만 ATmega128A의 ADC 포트에 연결하면 끝.

    * 간단한 코드: CodeVisionAVR에서 ADC를 활성화하고, read_adc(0) 함수 한 줄로 센서 값을 읽을 수 있음.

> 세그먼트 축소
    
    4열 7세그먼트 -> 1열 7세그먼트
* feedcount의 남은 횟수를 보여주기 위해 세그먼트를 사용하는데, 하루에 9회를 초과하여 급식하는 경우는 없는 것으로 가정하여 세그먼트 축소.

<br>

# 2. 최종 센서

|용도 | 센서|
|:--:|:--:|
|시간 (When)| RTC 모듈|
|동작 (How)| 5V TT 모터 + 1채널 릴레이 모듈|
|감지 (Check)| IR 적외선 근접 센서|
|표시 (Status)| 1열 7세그먼트 + RGB LED 모듈|

<br>

# 3. 부품 단위 테스트

> **릴레이 + 5V TT 모터** 테스트 (Digital Out)
```c
#include <mega128a.h>
#include <delay.h>

void main(void) {
    // PORTA.0 핀을 출력으로 설정
    DDRA.0 = 1; 
    
    while (1) {
        // 릴레이 켜기 (모터 작동)
        // (릴레이 모듈이 Low-Trigger 방식이면 PORTA.0 = 0;)
        PORTA.0 = 1; 
        delay_ms(4000); // 4초 대기
        
        // 릴레이 끄기 (모터 정지)
        // (릴레이 모듈이 Low-Trigger 방식이면 PORTA.0 = 1;)
        PORTA.0 = 0; 
        delay_ms(4000); // 4초 대기
    }
}
```

<br>
<br>

> **IR 적외선 근접 센서** 테스트 (Digital In)

```c
#include <mega128a.h>

void main(void) {
    // PINF.0 (IR 센서)를 입력으로 설정
    DDRF.0 = 0; 
    PORTF.0 = 0; // 풀업 저항 비활성화
    
    // PORTD.0 (LED0)을 출력으로 설정
    DDRD.0 = 1; 
    
    while (1) {
        // [가정] 감지 시 0(LOW) 신호가 들어오는 경우
        if (PINF.0 == 0) {
            PORTD.0 = 1; // LED 켜기
        } else {
            PORTD.0 = 0; // LED 끄기
        }
    }
}
```
<br>
<br>

> **RGB LED 모듈** 테스트 (Digital Out x3)

```c
#include <mega128a.h>
#include <delay.h>

void main(void) {
    // PORTA의 1, 2, 3번 핀을 출력으로 설정 (포트 전체로 설정)
    DDRA = 0b00001110; 
    
    while (1) {
        // Red
        PORTA = 0b00000010; // R=1, G=0, B=0
        delay_ms(1000);
        
        // Green
        PORTA = 0b00000100; // R=0, G=1, B=0
        delay_ms(1000);
        
        // Blue
        PORTA = 0b00001000; // R=0, G=0, B=1
        delay_ms(1000);
        
        // White
        PORTA = 0b00001110; // R=1, G=1, B=1
        delay_ms(1000);
    }
}
```
<br>
<br>

> **1열 7세그먼트** 테스트 (Digital Out x7)

```c
#include <mega128a.h>
#include <delay.h>

// 7세그먼트 숫자 패턴 (0~9) - Common Cathode
// 비트 순서: g f e d c b a (LSB)
// 0x3F = 0011 1111 (0 표시: a,b,c,d,e,f 켜짐)
unsigned char seg_digits[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x27, // 7
    0x7F, // 8
    0x6F  // 9
};

void main(void) {
    int i;

    // 1. 포트 설정
    // PORTE 전체를 출력으로 설정 (세그먼트 연결)
    DDRE = 0xFF; 

    while (1) {
        // 0부터 9까지 숫자 반복 표시
        for (i = 0; i < 10; i++) {
            
            PORTE = seg_digits[i]; 
            
            delay_ms(1000); // 1초 대기
        }
    }
}
```
<br>
<br>

> **RTC 모듈** 테스트 (I2C/TWI)

```c
//12주차 RTC 실습 예제 참고
// 현재시간 수정
DS1302_write(DS1302_DATE_WRITE, dec_to_bcd(20));     // 일
DS1302_write(DS1302_MONTH_WRITE, dec_to_bcd(11));    // 11월
DS1302_write(DS1302_DAY_WRITE, dec_to_bcd(5));       // 요일
DS1302_write(DS1302_HOUR_WRITE, dec_to_bcd(14));     // 현재 시
DS1302_write(DS1302_MIN_WRITE, dec_to_bcd(30));      // 30분

DS1302_init();  // RTC 모듈 핀 초기화
    delay_ms(100);

    printf("DS1302 RTC Start\r\n");

    // =====================================================================
    // 전원을 껐다 켜도 시간이 초기화되지 않고 계속 흘러가도록 주석처리 후 다시 업로드.
    // =====================================================================
    
    /* <-- 주석 시작
    printf("Setting initial time...\r\n");
    DS1302_write(DS1302_WP_WRITE, 0x00); // 쓰기 방지 해제

    
    sec_bcd = dec_to_bcd(0);
    sec_bcd &= ~(1 << 7);
    DS1302_write(DS1302_SEC_WRITE, sec_bcd); 
    
    // ... (중략) ...
    
    DS1302_write(DS1302_YEAR_WRITE, dec_to_bcd(25));

    DS1302_write(DS1302_WP_WRITE, 0x80); // 쓰기 방지 설정
    printf("Initial time set.\r\n\r\n");
    */                                   // <-- 주석 끝
    
    delay_ms(2000);

```
<br>
<br>

부품 테스트 결과
---
|센서 및 모듈|테스트 결과|
|:--:|:--:|
|릴레이 + 5V TT 모터| 정상 작동|
|IR 적외선 근접 센서|정상 작동|
|RGB LED 모듈|정상 작동|
|1열 7세그먼트|정상 작동| 
|RTC 모듈|정상 작동|

<br>
<br>

# 4. 최종 코드 작업 중  [codeVisionAVR Evalution]
```c
#include <mega128a.h>
#include <delay.h>
#include <stdio.h>
#include <stdint.h>

// =====================================================
//                  핀 설정 및 상수 정의
// =====================================================

// --- 구동부 및 센서 ---
#define MOTOR_PIN       PORTA.0
#define LED_R           PORTA.1
#define LED_G           PORTA.2
#define LED_B           PORTA.3
// IR 센서 (입력): PINF.0 (0:감지됨, 1:미감지)
#define IR_PIN_MASK     0x01 

// --- 7세그먼트 (출력) ---
#define SEG_PORT        PORTE

// --- DS1302 RTC 핀 ---
#define DS1302_PORT     PORTB
#define DS1302_DDR      DDRB
#define DS1302_PIN      PINB
#define DS1302_SCLK_PIN 0   // PB0
#define DS1302_IO_PIN   1   // PB1
#define DS1302_RST_PIN  2   // PB2

// --- 모터 회전 시간 ---
#define OPEN_TIME_MS    1000
#define CLOSE_TIME_MS   1000

// =====================================================
//                 전역 변수 및 상태 정의
// =====================================================

enum SystemState { 
    STATE_IDLE,       // 대기 모드
    STATE_DISPENSING, // 사료 배출
    STATE_WAIT_EAT,   // 급식 대기
    STATE_DONE,       // 급식 완료
    STATE_ERROR       // 에러 
};

volatile enum SystemState current_state = STATE_IDLE;
volatile uint8_t feed_count = 3;
volatile uint8_t last_min = 60;

// 시간 관련 변수
uint8_t cur_sec, cur_min, cur_hour;
uint8_t target_hour[3], target_min[3]; 
uint16_t wait_timer = 0;

// 7세그먼트 패턴 (Common Cathode: 1=ON)
uint8_t seg_digits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x27, 0x7F, 0x6F};

// =====================================================
//                    하드웨어 제어 함수 
// =====================================================

// --- DS1302 RTC 드라이버 ---
#define DS1302_SEC_READ 0x81
#define DS1302_MIN_READ 0x83
#define DS1302_HOUR_READ 0x85
// 현재 시간 설정 -> 실습예제 참고 
uint8_t bcd_to_dec(uint8_t v) { return (v / 16 * 10) + (v % 16); }
// 초기화, 바이트 전송 후 읽기
void DS1302_init(void) {
    DS1302_DDR |= (1 << DS1302_RST_PIN) | (1 << DS1302_SCLK_PIN);
    DS1302_PORT &= ~((1 << DS1302_RST_PIN) | (1 << DS1302_SCLK_PIN));
}

void DS1302_write_byte(uint8_t byte) {
    uint8_t i;
    DS1302_DDR |= (1 << DS1302_IO_PIN); 
    for (i = 0; i < 8; i++) {
        if (byte & 0x01) DS1302_PORT |= (1 << DS1302_IO_PIN);
        else             DS1302_PORT &= ~(1 << DS1302_IO_PIN);
        delay_us(5); DS1302_PORT |= (1 << DS1302_SCLK_PIN); delay_us(5);
        DS1302_PORT &= ~(1 << DS1302_SCLK_PIN); delay_us(2);
        byte >>= 1;
    }
}

uint8_t DS1302_read_byte(void) {
    uint8_t i, byte = 0;
    DS1302_DDR &= ~(1 << DS1302_IO_PIN); DS1302_PORT &= ~(1 << DS1302_IO_PIN);
    for (i = 0; i < 8; i++) {
        delay_us(2);
        if (DS1302_PIN & (1 << DS1302_IO_PIN)) byte |= (1 << i);
        DS1302_PORT |= (1 << DS1302_SCLK_PIN); delay_us(5);
        DS1302_PORT &= ~(1 << DS1302_SCLK_PIN); delay_us(5);
    }
    return byte;
}

uint8_t DS1302_read(uint8_t addr) {
    uint8_t data;
    #asm("cli") // 인터럽트 차단 (노이즈 방지)
    DS1302_PORT &= ~(1 << DS1302_RST_PIN); delay_us(5);
    DS1302_PORT |= (1 << DS1302_RST_PIN);  delay_us(5);
    DS1302_write_byte(addr);
    data = DS1302_read_byte();
    DS1302_PORT &= ~(1 << DS1302_RST_PIN); delay_us(5);
    #asm("sei") // 인터럽트 재개
    return data;
}

// --- UI 제어 (LED, 7Seg) ---
void set_led(uint8_t r, uint8_t g, uint8_t b) {
    LED_R = r; LED_G = g; LED_B = b;
}

void update_7seg(uint8_t num) {
    if(num > 9) num = 9;
    SEG_PORT = seg_digits[num]; 
    
}

// =====================================================
//                    핵심 로직 함수 
// =====================================================

// 시스템 초기화 (모든 하드웨어 설정)
void System_Init(void) {
    // 1. 포트 설정
    DDRA = 0xFF;   // 모터, LED 출력
    
    // IR 센서 입력 설정
    DDRF &= ~IR_PIN_MASK; 
    PORTF &= ~IR_PIN_MASK; // 풀업 해제
    
    DDRE = 0xFF;   // 7세그먼트 출력
    
    // 2. UART 설정 (9600bps)
    UCSR0A=0x00; UCSR0B=0x08; UCSR0C=0x06; UBRR0L=103; 
    
    // 3. RTC 초기화
    DS1302_init();
    delay_ms(100);
    
    printf("\r\n=== Smart Pet Feeder System Initialized ===\r\n");
    
    // 4. 초기 상태 표시
    update_7seg(feed_count);
    set_led(0, 0, 1); // 대기(파랑)
}

// 시연용 스케줄 자동 생성 (+2분, +4분, +5분)
void Create_Demo_Schedule(void) {
    uint8_t start_min, start_hour;
    uint16_t temp_min;
    int i;
    uint8_t offsets[3] = {2, 4, 5}; 

    // 현재 시간 읽기
    start_min  = bcd_to_dec(DS1302_read(DS1302_MIN_READ));
    start_hour = bcd_to_dec(DS1302_read(DS1302_HOUR_READ) & 0x3F);
    
    printf("Start Time: %02d:%02d\r\n", start_hour, start_min);
    
    for(i=0; i<3; i++) {
        temp_min = start_min + offsets[i];
        target_hour[i] = start_hour;
        
        // 분이 60을 넘어가면 시간 증가 처리
        if(temp_min >= 60) {
            target_hour[i] = (start_hour + 1) % 24;
            temp_min -= 60;
        }
        target_min[i] = (uint8_t)temp_min;
        
        printf("Schedule #%d: %02d:%02d\r\n", i+1, target_hour[i], target_min[i]);
    }
}

// 시간 읽기 및 상태 체크
void Update_Time(void) {
    cur_sec  = bcd_to_dec(DS1302_read(DS1302_SEC_READ) & 0x7F);
    cur_min  = bcd_to_dec(DS1302_read(DS1302_MIN_READ));
    cur_hour = bcd_to_dec(DS1302_read(DS1302_HOUR_READ) & 0x3F);

    // 5초마다 디버깅 출력
    if(cur_sec % 5 == 0) { 
         printf("Time: %02d:%02d:%02d | Cnt: %d | St: %d\r\n", cur_hour, cur_min, cur_sec, feed_count, current_state);
    }
    
    // 자정 리셋 (00:00)
    if (cur_hour == 0 && cur_min == 0 && feed_count < 3) {
        feed_count = 3;
        update_7seg(feed_count);
        printf("Midnight Reset! Count -> 3\r\n");
        last_min = cur_min; 
    }
}
//================================================================================
//                           상태 머신 실행 (메인 로직)
//================================================================================
void Run_State_Machine(void) {
    switch (current_state) {
        
        case STATE_IDLE: // [대기]
            set_led(0, 0, 1); // 파랑
            
            // 스케줄 시간 확인
            if (feed_count > 0 && cur_min != last_min) {
                // 예약된 3개의 시간과 비교
                if ( (cur_hour == target_hour[0] && cur_min == target_min[0]) ||
                     (cur_hour == target_hour[1] && cur_min == target_min[1]) ||
                     (cur_hour == target_hour[2] && cur_min == target_min[2]) )
                {
                    last_min = cur_min;
                    printf(">> Time Match! Start Dispensing...\r\n");
                    current_state = STATE_DISPENSING;
                }
            }
            break;

        case STATE_DISPENSING: // [급식] 
            set_led(0, 1, 0); // 초록
            
            printf(">> Open Lid\r\n");
            MOTOR_PIN = 1; delay_ms(OPEN_TIME_MS); // 열기
            
            printf(">> Pouring...\r\n");
            MOTOR_PIN = 0; delay_ms(5000);         // 배출 대기
            
            printf(">> Close Lid\r\n");
            MOTOR_PIN = 1; delay_ms(CLOSE_TIME_MS); // 닫기
            
            MOTOR_PIN = 0; 
            wait_timer = 0;
            current_state = STATE_WAIT_EAT;
            break;

        case STATE_WAIT_EAT: // [섭취 감지]
            set_led(0, 1, 0); // 초록
            
            // IR 센서 감지 (0일 때 감지)
            if ( (PINF & IR_PIN_MASK) == 0 ) {
                delay_ms(500); // 노이즈 방지
                if ( (PINF & IR_PIN_MASK) == 0 ) {
                    printf(">> Pet Eaten! Count decreased.\r\n");
                    if(feed_count > 0) feed_count--;
                    update_7seg(feed_count);
                    current_state = STATE_DONE;
                }
            }
            
            // 타임아웃 (약 60초)
            wait_timer++;
            if (wait_timer > 60) {
                printf(">> Timeout! Pet didn't eat.\r\n");
                current_state = STATE_ERROR;
            }
            break;

        case STATE_DONE: // [완료]
            set_led(1, 1, 1); // 흰색
            delay_ms(5000);   // 5초간 완료 표시
            current_state = STATE_IDLE; // 다시 대기로
            break;

        case STATE_ERROR: // [오류]
            set_led(1, 0, 0); // 빨강
            delay_ms(5000); 
            current_state = STATE_IDLE; // 5초 후 대기로 복귀
            break;
    }
}

// =====================================================
//                        메인 함수 
// =====================================================

void main(void) {
    // 1. 시스템 초기화
    System_Init();
    
    // 2. 시연 스케줄 생성
    Create_Demo_Schedule();

    // 3. 무한 루프
    while (1) {
        Update_Time();       // 시간 업데이트 & 리셋 체크
        Run_State_Machine(); // 상태 머신 실행
        
        delay_ms(1000);      
    }
}
```
# 5. 현재 작업중인 단계
>## 하드웨어

**1. 사료 배출기 (Feeder Mechanism)**
* **구조 및 작동 원리**:
    * 사료 호퍼 : 상단에 위치하여 사료를 저장.
    * 배출 통로 : 모터 하단에 원통형 통로를 배치하여 사료가 일정하게 낙하하도록 유도. 통로 하단에는 사료 배출을 위한 모터와 뚜껑이 존재. 
    * 사료 그릇 : 통로 하단에 위치하여 배출된 사료를 받음.
* **비산 방지 가드** : 배출 통로와 사료 그릇 사이의 낙차로 인한 사료 흩어짐을 방지하기 위해 양쪽에 가드를 설치하여 위생적이고 안정적인 급식을 보장.

<br>

**2. 컨트롤 타워 (Control Tower)**
* **기능 및 구성** : 시스템의 핵심 제어부로, 메인보드(MCU), RTC모듈, 7세그먼트, rgb led 모듈 등 주요 전자 부품을 탑재.
* **상태 표시 인터페이스**:
    * 상태 LED : 급식 대기, 급식 중, 급식 완료 등 시스템의 현재 상태를 색상으로 직관적으로 표시.
    * 잔여 횟수 표시기 (1열 7-Segment) : 당일 남은 급식 횟수를 숫자로 시각화하여 사용자에게 정보를 제공.
* **연결 및 배선** : 측면의 배선구를 통해 본체(사료 배출기)와 연결되며, 점퍼 케이블을 이용하여 제어 신호 및 전력을 공급. 

<br>

**3. 분리형 설계의 목적 (노이즈 및 진동 격리):**
* **전기적 안정성** : 릴레이 및 모터 구동 시 발생하는 스파이크 노이즈가 메인보드와 RTC 등 민감한 제어 회로에 간섭을 일으켜 오작동을 유발하는 것을 방지.
* **기계적 안정성** : 모터 구동 시 발생하는 물리적 진동을 제어부와 격리하여 시스템의 내구성을 확보하고 센서 오류를 최소화.

**[Preview by Ansys Workbench]** 


<img src='https://i.ifh.cc/0QotFx.png' width="280" height="225"> <img src='https://i.ifh.cc/2sYWJm.png' width="280" height="225">



>## **Firebase**의 **Realtime Database**을 이용한 웹 인터페이스 구현
    세그먼트와 led로 상태를 확인할 수 있으나, 결국 프로젝트의 목표는 사용자의 부재에도 급식이 되어야함.
    사용자가 반려동물의 급식 상태를 실시간으로 확인하고 제어할 수 있는 직관적인 인터페이스를 제공하는 것을 목표로 함. 
    임베디드 하드웨어 <-> PC 소프트웨어 <-> 클라우드/웹 플랫폼이 유기적으로 결합된 IoT(사물인터넷) 아키텍처를 기반.

  *  **임베디드 시스템** (codeVision/ATmega128): 센서 데이터 수집 및 급식 모터 제어 등 **물리적인 동작**, 상태 정보를 **시리얼 통신**으로 전송.
  *  **PC SW** (Python): 임베디드 시스템과 클라우드 사이의 중계 역할을 수행. 시리얼 통신으로 받은 데이터를 파싱하여 **Firebase Realtime Database**에 실시간으로 업데이트.
  *  **클라우드/웹** (Firebase/HTML/JS): 사용자에게 시각적인 인터페이스를 제공. **Firebase Realtime Database**에 저장된 데이터를 실시간으로 동기화하여 웹 브라우저(앱)를 통해 언제 어디서든 급식 내역과 현재 상태를 모니터링.

<br>

**[Preview by javaScript  &  React]**
<br>
<img src='https://i.ifh.cc/KkQWXm.png' width="300" height="500">

<br>
<br>

# 6. 작업 예정 단계

> ## 하드웨어 

  * 소재 확정 및 설계 구체화, 센서 & 모듈 설치할 환경 최적화

> ## 소프트웨어 

  *  **모터 제어 구체화** : 최대한 일정한 양을 배출할 수 있게 모터 구동 시간 세밀하게 제어
  *  **전체 시스템 연동 테스트** : 임베디드 처리 -> PC 중계 -> 클라우드 업데이트 -> 웹 화면 반영까지의 전 과정 테스트
  *  **오류 로그 분석 및 디버깅** : 각 단계에서 발생하는 오류 메시지를 putty로 확인하고 수집, 분석하여 원인을 파악하고 수정 (유지보수)
  *  **사용자 인터페이스 디자인** : 남은 급식 횟수, 마지막 급식 시간 등 핵심 정보를 한눈에 파악할 수 있는 UI 개선 -> Tailwind css 사용 예정

<br>

# 7. 하드웨어 제작과정

* ### 하드웨어 전체 소재
    * 커터칼로 가공이 용이한 **포맥스(Foamex)**

* ### 센서 위치 
    * **본체** : 모터, 적외선 센서 
    * **컨트롤 타워** : 메인보드를 포함한 나머지 센서

* ### 사료 그릇
    * 반려동물의 **수염 스트레스 저하**와 **사료 비산 방지**를 위해 **넓은 그릇** 사용.
* ### 기타 사항 
    * 본체와 컨트롤 타워를 잇는 배선은 받침대 밑으로 **매립**함.

* ### 교체된 센서
    * 1채널 릴레이 모듈 -> L9110S 모터 드라이버
         * 릴레이 모듈은 모터 속도 및 방향 설정 불가능.
         * 릴레이의 기능은 유지하면서 모터 속도와 방향을 제어하기 위해 모터 드라이버로 변경   

# 8. 하드웨어와 소프트웨어를 연결하는 작업
> 웹 화면의 전체적인 프레임은 Demo 버전을 유지하면서 세부 UI 개선
```java
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Pet Care</title>
    <script src="https://cdn.tailwindcss.com"></script>
    
    <!-- 아이콘 라이브러리 -->
    <link rel='stylesheet' href='https://cdn-uicons.flaticon.com/uicons-regular-rounded/css/uicons-regular-rounded.css'>
    <link rel='stylesheet' href='https://cdn-uicons.flaticon.com/uicons-regular-straight/css/uicons-regular-straight.css'>
    <link rel='stylesheet' href='https://cdn-uicons.flaticon.com/uicons-bold-rounded/css/uicons-bold-rounded.css'>
    
    <script crossorigin src="https://unpkg.com/react@18/umd/react.production.min.js"></script>
    <script crossorigin src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>
    <script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
    
    <script type="module">
        import { initializeApp } from "https://www.gstatic.com/firebasejs/10.7.1/firebase-app.js";
        import { getDatabase, ref, onValue, set } from "https://www.gstatic.com/firebasejs/10.7.1/firebase-database.js";

       // api key from firebase
       const firebaseConfig = {
  apiKey: "AIzaSyB_DxBE1e2ABj0YBX1TCv7jotlgOXom4Vw",
  authDomain: "petfeeder-260f5.firebaseapp.com",
  databaseURL: "https://petfeeder-260f5-default-rtdb.firebaseio.com",
  projectId: "petfeeder-260f5",
  storageBucket: "petfeeder-260f5.firebasestorage.app",
  messagingSenderId: "148512290432",
  appId: "1:148512290432:web:4270dc80b59cfb9d4b1865",
  measurementId: "G-HP26RYVX97"
};


        const app = initializeApp(firebaseConfig);
        window.db = getDatabase(app);
        window.Firebase = { ref, onValue };
    </script>
    <style>
        body { background-color: #FFF7ED; font-family: 'Apple SD Gothic Neo', sans-serif; }
    </style>
</head>
<body>
    <div id="root"></div>

    <script type="text/babel">
        const { useState, useEffect } = React;

        function App() {
            const [status, setStatus] = useState("대기 중");
            const [lastFedTime, setLastFedTime] = useState("-");
            const [feedCount, setFeedCount] = useState(3);

            useEffect(() => {
                if (window.db && window.Firebase) {
                    try {
                        const statusRef = window.Firebase.ref(window.db, 'feeder_status');
                        window.Firebase.onValue(statusRef, (snapshot) => {
                            const data = snapshot.val();
                            if (data) {
                                setStatus(data.status || "대기 중");
                                setLastFedTime(data.last_time || "-");
                                setFeedCount(data.remain_count !== undefined ? data.remain_count : 3);
                            }
                        });
                    } catch(e) {}
                }
            }, []);

            return (
                <div className="max-w-md mx-auto min-h-screen bg-orange-50 flex flex-col items-center pt-10 px-6">
                    {/* 상단 타이틀 */}
                    <div className="w-full flex justify-between items-center mb-8">
                        <div>
                            <span className="text-orange-400 font-bold text-sm tracking-widest">SMART CARE</span>
                            <h1 className="text-3xl font-extrabold text-gray-800">User-123</h1>
                        </div>
                        
                   
                        <div className="w-12 h-12 bg-white rounded-2xl shadow-sm flex items-center justify-center text-xl border-2 border-orange-100 text-orange-400">
                            <i className="fi fi-rs-users"></i>
                        </div>
                    </div>

                    {/* 메인 상태 카드 */}
                    <div className="w-full bg-white rounded-[2rem] shadow-xl p-8 mb-6 border border-orange-100 relative overflow-hidden">
                        <div className="absolute top-0 right-0 w-32 h-32 bg-orange-100 rounded-full -mr-10 -mt-10 opacity-50"></div>
                        
                        <div className="flex flex-col items-center relative z-10">
                            <div className={`w-40 h-40 rounded-full flex items-center justify-center text-6xl mb-4 transition-all duration-500 ${
                                status === '급식 중' ? 'bg-orange-400 animate-bounce shadow-orange-200 shadow-lg text-white' : 'bg-gray-100 text-gray-400'
                            }`}>
                                {/* 2. 대기 중 집 아이콘 -> 모래시계 아이콘 변경 */}
                                {status === '급식 중' ?
				<i className="fi fi-rr-restaurant text-5xl"></i> : 
				status === '섭취 완료' ?
				<i className="fi fi-br-check text-5xl"></i> : 
				<i className="fi fi-rr-hourglass-end text-5xl"></i>}
                            </div>
                            <h2 className="text-2xl font-bold text-gray-800 mb-1">{status}</h2>
                            <p className="text-gray-400 text-sm">
                                {status === '급식 중' ? "맛있는 밥이 나오고 있어요!" : "다음 식사 시간을 기다려요"}
                            </p>
                        </div>
                    </div>

                    {/* 정보 카드 그리드 */}
                    <div className="w-full grid grid-cols-2 gap-4">
                        <div className="bg-white p-5 rounded-3xl shadow-lg border-b-4 border-orange-200">
                            {/* 3. 남은 횟수 아이콘 -> 편집 아이콘 변경 */}
                            <div className="w-10 h-10 bg-[#E7E5E4] rounded-full flex items-center justify-center mb-4 text-gray-600">
                                <i className="fi fi-rr-edit text-xl"></i>
                            </div>
                            <p className="text-gray-400 text-xs font-bold mb-1">남은 횟수</p>
                            <p className="text-3xl font-black text-gray-800">{feedCount}<span className="text-lg font-medium text-gray-400 ml-1">회</span></p>
                        </div>
                        <div className="bg-white p-5 rounded-3xl shadow-lg border-b-4 border-blue-200">
                            <div className="w-10 h-10 bg-[#E7E5E4] rounded-full flex items-center justify-center mb-4 text-gray-600">
                                <i className="fi fi-rr-time-past text-xl text-blue-500"></i>
                            </div>
                            <p className="text-gray-400 text-xs font-bold mb-1">마지막 식사</p>
                            <p className="text-lg font-bold text-gray-800 leading-tight mt-1">{lastFedTime}</p>
                        </div>
                    </div>

                    <div className="mt-8 text-center">
                        <p className="text-orange-300 text-xs font-medium">IoT Pet Care System</p>
                    </div>
                </div>
            );
        }
        const root = ReactDOM.createRoot(document.getElementById('root'));
        root.render(<App />);
    </script>
</body>
</html>
```
* ### 프론트엔드 (UI/UX)
    * 모바일 환경에서의 사용자 경험을 위해 **React 라이브러리**를 기반으로 구축하였으며, **Tailwind CSS**를 활용해 모바일 환경에서도 깔끔하게 보이도록 직관적인 카드 형태의 디자인을 적용.
* ### 백엔드 및 데이터 연동
    * 프로젝트의 핵심 : **‘실시간 데이터 동기화’** -> Firebase 내부의 Realtime Database를 사용하여 연동. 생성된 database가 제공하는 API key 값을 통해 인식 및 동기화.

<br>

> 하드웨어(ATmega)와 웹(Firebase)을 연결하는 미들웨어
```python
import serial
import time
import requests
import json


COM_PORT = 'COM4'  
BAUD_RATE = 9600


DB_URL = "https://petfeeder-260f5-default-rtdb.firebaseio.com/feeder_status.json"


def update_firebase(data):
    try:
        requests.patch(DB_URL, json=data)
        print(f"   >>> 클라우드 업데이트 완료: {data}")
    except Exception as e:
        print(f"   >>> 전송 실패: {e}")


print("시스템 시작: 웹 화면을 초기화합니다...")
init_data = {
    'status': '대기 중',
    'remain_count': 3,   
    'last_time': '-'    
}
update_firebase(init_data)
print("초기화 완료. 이제 ATmega 데이터를 대기합니다.\n")
# ---------------------------------------------------------

print(f"Connecting to {COM_PORT}...")

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {COM_PORT}! Listening...")
    print("------------------------------------------------")

    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except:
                continue
            
            if not line:
                continue

            print(f"[ATmega] {line}")

            # (1) 급식 시작
            if "Dispensing" in line or "Open Lid" in line:
                update_firebase({'status': '급식 중'})

            # (2) 섭취 감지
            elif "Pet Eaten" in line:
                now_str = time.strftime('%Y-%m-%d %H:%M:%S')
                update_firebase({
                    'status': '섭취 완료',
                    'last_time': now_str
                })

            # (3) 대기 상태 복귀
            elif "System Initialized" in line or "State: Idle" in line:
                update_firebase({'status': '대기 중'})

            # (4) 남은 횟수 동기화
            elif "Cnt:" in line:
                parts = line.split('|')
                for p in parts:
                    if "Cnt:" in p:
                        try:
                            # 보드에서 오는 실제 횟수 값을 읽어서 업데이트
                            cnt_val = int(p.split(':')[1].strip())
                            update_firebase({'remain_count': cnt_val})
                        except:
                            pass

except serial.SerialException:
    print(f"오류: {COM_PORT} 포트를 열 수 없습니다.")
    print("   1. PuTTY가 켜져 있다면 꺼주세요.")
    print("   2. 포트 번호가 맞는지 확인하세요.")
    
except KeyboardInterrupt:
    print("\n프로그램 종료")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
``` 
* ### In: Serial Communication (UART) - from ATmega -> Receive Data
* ### Out: HTTP, REST API (PATCH) - to Firebase -> Exchange Data

<br>

# 9. Demo Test
* ### 시연 절차 
    * 빠른 시간 내의 시연을 위해 전원 연결로부터 45초, 70초, 95초 후로 시간을 설정.
    * 하드웨어 전원 연결 -> 터미널에서 로그 확인 -> 클라우드 업데이트 -> 하드웨어 실행 -> 웹 반영


<br>

# 10. 개선 사항
* ### 사료의 양을 척도하는 방법으로 모터 동작의 시간이 아닌 무게로 측정하도록 설계
    * 현재 그릇의 무게로 정량을 배출할 수 있다면 반려동물이 다 먹지 않더라도 다음 급식 때 정량을 맞출 수 있음.
* ### 카메라 모듈
    * 적외선 센서가 감지한 순간부터 카메라 모듈을 동작시켜 실제로 급식하고 있는 상태를 사용자가 눈으로 확인할 수 있음.


