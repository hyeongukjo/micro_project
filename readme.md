# 마이크로 프로세서 프로젝트 진행 사항 발표

# 1. 초기 계획 대비 변경 사항
> 기존 계획된 센서

    MPU-6050
* 복잡한 데이터: true/false의 단순 신호가 아닌, 6축의 가속도/자이로 데이터 스트림만 출력한다. 
* 알고리즘 복잡성: "섭취가 무엇인가?"를 코드로 직접 정의해야 한다. (예: 5초간 0.5G 이상의 진동이 10회 이상 감지)
* 디버깅 난이도 (고스트 현상): 문제가 발생 시 TWI 통신 오류인지, 알고리즘의 **임계값(Threshold)** 설정 오류인지 파악하기 어렵다. 이는 **데이터 분석**의 영역에 속한다.

<br>

> 교체한 센서

    IR 적외선 근접 센서

* 작동 원리:

    * 센서가 적외선을 쏘고, 물체(반려동물 머리)에 반사되어 돌아오는 빛을 감지함.

    * 밥그릇 바로 위에 설치해서, 반려동물이 밥을 먹기 위해 머리를 밥그릇에 넣는 순간을 감지한다.

* MPU-6050 대비 장점:

    * 쉬운 통신: I2C(TWI) 통신이 필요 없다.

    * 간단한 연결: Vcc(5V), GND, 그리고 'Analog Out' 핀 1개만 ATmega128A의 ADC 포트에 연결하면 끝.

    * 간단한 코드: CodeVisionAVR에서 ADC를 활성화하고, read_adc(0) 함수 한 줄로 센서 값을 읽을 수 있다.
  
# 2. 최종 센서

|용도 | 센서|
|:--:|:--:|
|시간 (When)| RTC 모듈|
|동작 (How)| 5V TT 모터 + 1채널 릴레이 모듈|
|감지 (Check)| IR 적외선 근접 센서|
|표시 (Status)| 7세그먼트 + RGB LED 모듈|

<br>

# 3. 부품 단위 테스트

> **릴레이 + 5V TT 모터** 테스트 (Digital Out)
```c
#include <mega128.h>
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
#include <mega128.h>

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
#include <mega128.h>
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
#include <mega128.h>
#include <delay.h>

// 7세그먼트 숫자 0~9 (Common Cathode 기준)
// {g, f, e, d, c, b, a} -> 0b_gfedcba
unsigned char seven_seg_digits[] = {
    0x3F, // 0 (0b00111111)
    0x06, // 1 (0b00000110)
    0x5B, // 2 (0b01011011)
    0x4F, // 3 (0b01001111)
    0x66, // 4 (0b01100110)
    0x6D, // 5 (0b01101101)
    0x7D, // 6 (0b01111101)
    0x27, // 7 (0b00100111)
    0x7F, // 8 (0b01111111)
    0x6F  // 9 (0b01101111)
};

void main(void) {
    int i;
    
    // PORTC 전체를 출력으로 설정
    DDRC = 0xFF; 
    
    while (1) {
        for (i = 0; i < 10; i++) {
            PORTC = seven_seg_digits[i];
            delay_ms(1000); // 1초 대기
        }
    }
}
```
<br>
<br>

> **RTC 모듈** 테스트 (I2C/TWI)

```c
#include <mega128.h>
#include <delay.h>
#include <i2c.h> // CodeVisionAVR TWI 라이브러리

// DS1307/DS3231 I2C 주소
#define RTC_ADDRESS 0xD0 // (0x68 << 1)

// BCD(2진화 10진수)를 10진수로 변환
unsigned char bcd_to_dec(unsigned char val) {
    return (val & 0x0F) + ((val >> 4) * 10);
}

void main(void) {
    unsigned char current_second = 0;
    unsigned char last_second = 60; // 초기값을 다르게 설정
    
    // PORTD.1 (LED1)을 출력으로 설정
    DDRD.1 = 1; 
    
    // I2C(TWI) 초기화
    // (CodeWizardAVR에서 TWI 활성화, 비트레이트 100kHz 설정 필요)
    i2c_init();
    
    while (1) {
        // 1. RTC로부터 0번지(초) 레지스터 읽기 준비
        i2c_start(); // I2C 시작
        i2c_write(RTC_ADDRESS | 0x00); // RTC 주소 (Write 모드)
        i2c_write(0x00); // 0번지(초)부터 읽겠다고 주소 포인터 설정
        i2c_stop(); // I2C 정지
        
        // 2. RTC로부터 데이터 읽기
        i2c_start(); // I2C 시작
        i2c_write(RTC_ADDRESS | 0x01); // RTC 주소 (Read 모드)
        
        // 0번지(초) 레지스터 값 1바이트 읽기
        current_second = i2c_read(0); // 1바이트 읽고 NACK 전송(읽기 종료)
        
        i2c_stop(); // I2C 정지
        
        // BCD 값을 10진수로 변환
        current_second = bcd_to_dec(current_second & 0x7F);
        
        // 3. 1초가 변했는지 확인
        if (current_second != last_second) {
            PORTD.1 = !PORTD.1; // LED1 토글 (깜빡임)
            last_second = current_second; // 마지막 초 업데이트
        }
        
        delay_ms(100); // 너무 빨리 I2C를 읽지 않도록 잠시 대기
    }
}
```
<br>
<br>

부품 테스트 결과
---
|센서 및 모듈|테스트 결과|
|:--:|:--:|
|릴레이 + 5V TT 모터|-|
|IR 적외선 근접 센서|-|
|RGB LED 모듈|-|
|1열 7세그먼트|-| 
|RTC 모듈|-|


