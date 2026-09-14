/*
 * tmc5160_diagnostics.cpp - TMC5160 하드웨어/SPI 동작 진단 테스트
 *
 * 목적: "TMC5160이 정말 살아있는가?"를 단계별로 확인한다.
 *   1단계  SPI 링크    : IOIN.VERSION == 0x30 인지 (모드 0/3 모두 시도)
 *   2단계  레지스터    : 쓰기 -> 읽기 왕복이 일치하는지
 *   3단계  전원/오류   : GSTAT, DRV_STATUS 해석 (전원, 단락, 과열, 코일 단선)
 *   4단계  코일 감지   : 전류 인가 시 open-load 플래그가 사라지는지
 *   5단계  모션       : 내부 램프로 회전, XACTUAL/VACTUAL 이 따라오는지
 *
 * 사용법:
 *   pio run -e tmc5160_diagnostics -t upload -t monitor
 *   (5단계 모터 회전은 모니터에서 'm' 입력 시에만 실행 - 안전상 수동)
 */
#include <Arduino.h>
#include <SPI.h>

// ---- 핀맵 (include/MyData.h 와 동일) ----
constexpr uint8_t PIN_EN   = 13; // Active LOW
constexpr uint8_t PIN_STEP = 14;
constexpr uint8_t PIN_DIR  = 27;
constexpr uint8_t PIN_CS   = 5;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_SCK  = 18;

// ---- 레지스터 ----
constexpr uint8_t REG_GCONF        = 0x00;
constexpr uint8_t REG_GSTAT        = 0x01;
constexpr uint8_t REG_IOIN         = 0x04;
constexpr uint8_t REG_GLOBALSCALER = 0x0B;
constexpr uint8_t REG_IHOLD_IRUN   = 0x10;
constexpr uint8_t REG_TPOWERDOWN   = 0x11;
constexpr uint8_t REG_TPWMTHRS     = 0x13;
constexpr uint8_t REG_RAMPMODE     = 0x20;
constexpr uint8_t REG_XACTUAL      = 0x21;
constexpr uint8_t REG_VACTUAL      = 0x22;
constexpr uint8_t REG_VSTART       = 0x23;
constexpr uint8_t REG_A1           = 0x24;
constexpr uint8_t REG_V1           = 0x25;
constexpr uint8_t REG_AMAX         = 0x26;
constexpr uint8_t REG_VMAX         = 0x27;
constexpr uint8_t REG_DMAX         = 0x28;
constexpr uint8_t REG_D1           = 0x2A;
constexpr uint8_t REG_VSTOP        = 0x2B;
constexpr uint8_t REG_XTARGET      = 0x2D;
constexpr uint8_t REG_CHOPCONF     = 0x6C;
constexpr uint8_t REG_DRV_STATUS   = 0x6F;
constexpr uint8_t WRITE_FLAG       = 0x80;

// 전류/쵸퍼 설정은 동작이 확인된 tmc5160_oscillation.cpp 와 동일하게 맞춘다.
// cs_actual 이 이 값들과 일치하는지로 전류단을 판정한다.
constexpr uint32_t CFG_IHOLD = 6;
constexpr uint32_t CFG_IRUN = 15;
constexpr uint32_t CFG_IHOLDDELAY = 6;
constexpr uint32_t CFG_GLOBALSCALER = 192;

// CHOPCONF.MRES=4 -> 1/16 마이크로스텝 (oscillation 테스트와 동일)
constexpr uint32_t CFG_MRES = 4;
constexpr int32_t MICROSTEPS_PER_REV = 200L * 16L; // 1.8도 모터 1회전

// 쵸퍼 모드. spreadCycle 이 기본값 - stealthChop 은 튜닝 전에는 토크가 거의
// 안 나와서 "스텝은 세는데 축은 안 도는" 증상이 나온다.
bool g_useStealthChop = false;

// 속도 단위 변환: v[마이크로스텝/s] = VACTUAL * f_CLK / 2^24  (f_CLK = 12MHz)
constexpr uint32_t velocityToInternal(uint32_t stepsPerSec) {
    return static_cast<uint32_t>(static_cast<uint64_t>(stepsPerSec) * 16777216ULL / 12000000ULL);
}
constexpr uint32_t VEL_TEST = velocityToInternal(8000);   // 2.5 rev/s
constexpr uint32_t VEL_POSITION = velocityToInternal(16000); // 5 rev/s

// 마지막 SPI 트랜잭션이 돌려준 SPI_STATUS 바이트
uint8_t g_spiStatus = 0;
uint8_t g_spiMode = SPI_MODE3; // 1단계에서 확정됨

static SPISettings spiCfg() { return SPISettings(1000000, MSBFIRST, g_spiMode); }

void writeRegister(uint8_t address, uint32_t value) {
    SPI.beginTransaction(spiCfg());
    digitalWrite(PIN_CS, LOW);
    g_spiStatus = SPI.transfer(address | WRITE_FLAG);
    SPI.transfer(static_cast<uint8_t>(value >> 24));
    SPI.transfer(static_cast<uint8_t>(value >> 16));
    SPI.transfer(static_cast<uint8_t>(value >> 8));
    SPI.transfer(static_cast<uint8_t>(value));
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
}

// TMC5160 은 파이프라인 방식: 요청한 값은 "다음" 트랜잭션에서 나온다.
uint32_t readRegister(uint8_t address) {
    uint32_t v = 0;
    for (uint8_t pass = 0; pass < 2; ++pass) {
        SPI.beginTransaction(spiCfg());
        digitalWrite(PIN_CS, LOW);
        g_spiStatus = SPI.transfer(address & 0x7F);
        v  = static_cast<uint32_t>(SPI.transfer(0)) << 24;
        v |= static_cast<uint32_t>(SPI.transfer(0)) << 16;
        v |= static_cast<uint32_t>(SPI.transfer(0)) << 8;
        v |= static_cast<uint32_t>(SPI.transfer(0));
        digitalWrite(PIN_CS, HIGH);
        SPI.endTransaction();
        delayMicroseconds(10);
    }
    return v;
}

int32_t readSigned(uint8_t address) { return static_cast<int32_t>(readRegister(address)); }

void hr(const char *title) {
    Serial.println();
    Serial.println(F("------------------------------------------------------------"));
    Serial.print(F("  "));
    Serial.println(title);
    Serial.println(F("------------------------------------------------------------"));
}

void printHex32(const char *name, uint32_t v) {
    Serial.print(F("  "));
    Serial.print(name);
    Serial.print(F(" = 0x"));
    for (int8_t i = 28; i >= 0; i -= 4) Serial.print((v >> i) & 0xF, HEX);
    Serial.println();
}

// IOIN (0x04) 입력핀 상태. 0 REFL_STEP, 1 REFR_DIR, 4 DRV_ENN, 6 SD_MODE,
// 24:31 VERSION.  SD_MODE 는 모듈 기판에서 결정되며 SPI 로 바꿀 수 없다.
constexpr uint32_t IOIN_REFL_STEP = 1UL << 0;
constexpr uint32_t IOIN_REFR_DIR  = 1UL << 1;
constexpr uint32_t IOIN_DRV_ENN   = 1UL << 4;
constexpr uint32_t IOIN_SD_MODE   = 1UL << 6;

bool g_sdModePin = false; // true 면 STEP/DIR 모드 - 내부 램프로는 모터가 안 돈다

void decodeIoin(uint32_t ioin) {
    printHex32("IOIN      ", ioin);
    Serial.print(F("    STEP 핀="));
    Serial.print((ioin & IOIN_REFL_STEP) ? 1 : 0);
    Serial.print(F("  DIR 핀="));
    Serial.print((ioin & IOIN_REFR_DIR) ? 1 : 0);
    Serial.print(F("  DRV_ENN="));
    Serial.print((ioin & IOIN_DRV_ENN) ? 1 : 0);
    Serial.println((ioin & IOIN_DRV_ENN) ? F(" (출력 비활성)") : F(" (출력 활성)"));

    g_sdModePin = (ioin & IOIN_SD_MODE) != 0;
    Serial.print(F("    SD_MODE = "));
    Serial.println(g_sdModePin ? 1 : 0);
    if (g_sdModePin) {
        Serial.println(F("    >> !! STEP/DIR 모드 !! 내부 램프 제너레이터는 모터를 구동하지 못한다."));
        Serial.println(F("       XACTUAL 은 계속 세지만 축은 멈춰 있고, 드라이버는 정지 상태로 보여"));
        Serial.println(F("       전류가 IHOLD 에 머문다. 모터를 돌리려면 둘 중 하나:"));
        Serial.println(F("         (a) 모듈 기판의 SD_MODE 를 GND 로 (점퍼/트레이스 컷) -> SPI 램프 사용"));
        Serial.println(F("         (b) 펌웨어를 STEP/DIR 펄스 생성 방식으로 변경"));
    } else {
        Serial.println(F("    >> 내부 모션 컨트롤러 모드. SPI 램프로 구동 가능."));
    }
}

// ============================================================
// 1단계: SPI 링크 확인 (모드 자동 판별)
// ============================================================
bool step1_spiLink() {
    hr("1. SPI LINK / CHIP ID");
    const uint8_t modes[2] = {SPI_MODE3, SPI_MODE0};
    const char *names[2] = {"SPI_MODE3", "SPI_MODE0"};
    int8_t good = -1;

    for (uint8_t i = 0; i < 2; ++i) {
        g_spiMode = modes[i];
        uint32_t ioin = readRegister(REG_IOIN);
        uint8_t version = (ioin >> 24) & 0xFF;
        Serial.print(F("  "));
        Serial.print(names[i]);
        Serial.print(F(" -> IOIN=0x"));
        for (int8_t b = 28; b >= 0; b -= 4) Serial.print((ioin >> b) & 0xF, HEX);
        Serial.print(F("  VERSION=0x"));
        Serial.print(version, HEX);
        if (version == 0x30) {
            Serial.println(F("   [OK]"));
            if (good < 0) good = i;
        } else {
            Serial.println(F("   [NG]"));
        }
    }

    if (good < 0) {
        g_spiMode = SPI_MODE3;
        Serial.println();
        Serial.println(F("  >> FAIL: 어느 모드에서도 VERSION=0x30 이 안 나옴."));
        Serial.println(F("     0x00 또는 0xFF 만 보이면 대개 다음 중 하나:"));
        Serial.println(F("       - VM(모터 전원)이 안 들어옴  <-- 가장 흔함"));
        Serial.println(F("       - SDO/MISO, SDI/MOSI 뒤바뀜 또는 단선"));
        Serial.println(F("       - CS 핀이 다른 핀에 물림 / CSN 이 계속 HIGH"));
        Serial.println(F("       - 3.3V 로직 전원(VCC_IO) 미공급"));
        return false;
    }

    g_spiMode = modes[good];
    Serial.print(F("  >> 통신 OK. 사용할 모드: "));
    Serial.println(names[good]);
    Serial.println(F("     (둘 다 OK 인 것은 정상: MODE0/MODE3 모두 상승엣지에서 샘플링하므로"));
    Serial.println(F("      비트 정렬이 같다. 차이는 SCK 유휴 레벨뿐이며 스펙은 MODE3.)"));

    decodeIoin(readRegister(REG_IOIN));
    return true;
}

// ============================================================
// 2단계: 레지스터 왕복 쓰기/읽기
// ============================================================
bool step2_registerRoundTrip() {
    hr("2. REGISTER WRITE/READ ROUND-TRIP");
    // RW 레지스터만 검사한다. VMAX/AMAX/IHOLD_IRUN/GLOBALSCALER 등은 쓰기 전용
    // (W) 이라 되읽으면 항상 0 이 나온다 - 고장이 아니라 정상 동작이다.
    struct Case { uint8_t reg; const char *name; uint32_t pattern; };
    const Case tests[] = {
        {REG_XTARGET, "XTARGET", 0x0012ABCD},
        {REG_XTARGET, "XTARGET", 0xFFEDCBA9}, // 음수 패턴
        {REG_XACTUAL, "XACTUAL", 0x5A5A5A5A},
        {REG_XACTUAL, "XACTUAL", 0xA5A5A5A5},
    };
    bool ok = true;

    // hold 모드로 두고 테스트 -> 값만 써도 모터가 움직이지 않음
    writeRegister(REG_RAMPMODE, 3);

    for (const Case &t : tests) {
        writeRegister(t.reg, t.pattern);
        uint32_t back = readRegister(t.reg);
        bool match = (back == t.pattern);
        Serial.print(F("  "));
        Serial.print(t.name);
        Serial.print(F("  write=0x"));
        Serial.print(t.pattern, HEX);
        Serial.print(F("  read=0x"));
        Serial.print(back, HEX);
        Serial.println(match ? F("   [OK]") : F("   [MISMATCH]"));
        if (!match) ok = false;
    }

    writeRegister(REG_XTARGET, 0);
    writeRegister(REG_XACTUAL, 0);

    if (!ok) {
        Serial.println(F("  >> 값이 안 맞음: SPI 타이밍/노이즈 의심. SCK 속도를 낮추거나 배선을 짧게."));
    } else {
        Serial.println(F("  >> 32bit 읽기/쓰기 정상 (양수/음수 패턴 모두). SPI 데이터 경로 이상 없음."));
    }
    return ok;
}

// ============================================================
// 3단계: 상태 플래그 해석
// ============================================================
void printSpiStatus() {
    Serial.print(F("  SPI_STATUS = 0x"));
    Serial.print(g_spiStatus, HEX);
    Serial.print(F("  ("));
    if (g_spiStatus & 0x01) Serial.print(F("reset_flag "));
    if (g_spiStatus & 0x02) Serial.print(F("driver_error "));
    if (g_spiStatus & 0x04) Serial.print(F("sg2 "));
    if (g_spiStatus & 0x08) Serial.print(F("standstill "));
    if (g_spiStatus & 0x10) Serial.print(F("velocity_reached "));
    if (g_spiStatus & 0x20) Serial.print(F("position_reached "));
    if (g_spiStatus & 0x40) Serial.print(F("stop_l "));
    if (g_spiStatus & 0x80) Serial.print(F("stop_r "));
    Serial.println(F(")"));
}

// DRV_STATUS (0x6F) 비트 배치 - TMC5160 datasheet rev 1.18 표 기준.
// 9:0 SG_RESULT, 12 s2vsa, 13 s2vsb, 14 stealth, 15 fsactive,
// 20:16 CS_ACTUAL, 24 stallGuard, 25 ot, 26 otpw, 27 s2ga, 28 s2gb,
// 29 ola, 30 olb, 31 stst
constexpr uint32_t DRV_S2VSA = 1UL << 12;
constexpr uint32_t DRV_S2VSB = 1UL << 13;
constexpr uint32_t DRV_STEALTH = 1UL << 14;
constexpr uint32_t DRV_FSACTIVE = 1UL << 15;
constexpr uint32_t DRV_STALLGUARD = 1UL << 24;
constexpr uint32_t DRV_OT = 1UL << 25;
constexpr uint32_t DRV_OTPW = 1UL << 26;
constexpr uint32_t DRV_S2GA = 1UL << 27;
constexpr uint32_t DRV_S2GB = 1UL << 28;
constexpr uint32_t DRV_OLA = 1UL << 29;
constexpr uint32_t DRV_OLB = 1UL << 30;
constexpr uint32_t DRV_STST = 1UL << 31;
// 즉시 셧다운으로 이어지는 비트만 결함으로 센다 (open load, otpw 는 경고)
constexpr uint32_t DRV_FAULT_MASK =
    DRV_S2VSA | DRV_S2VSB | DRV_S2GA | DRV_S2GB | DRV_OT;

uint8_t csActualOf(uint32_t drv) { return (drv >> 16) & 0x1F; }

void decodeDrvStatus(uint32_t drv) {
    printHex32("DRV_STATUS", drv);
    if (drv & DRV_S2VSA) Serial.println(F("    s2vsa  : !! 코일 A 전원측 단락"));
    if (drv & DRV_S2VSB) Serial.println(F("    s2vsb  : !! 코일 B 전원측 단락"));
    if (drv & DRV_S2GA)  Serial.println(F("    s2ga   : !! 코일 A GND 단락"));
    if (drv & DRV_S2GB)  Serial.println(F("    s2gb   : !! 코일 B GND 단락"));
    if (drv & DRV_OT)    Serial.println(F("    ot     : !! 과열 셧다운"));
    if (drv & DRV_OTPW)  Serial.println(F("    otpw   : 과열 경고 (셧다운 전 단계)"));
    if (drv & DRV_OLA)   Serial.println(F("    ola    : 코일 A open load (저속/정지 중이면 오탐 가능)"));
    if (drv & DRV_OLB)   Serial.println(F("    olb    : 코일 B open load (저속/정지 중이면 오탐 가능)"));
    if (drv & DRV_STST)  Serial.println(F("    stst   : 모터 정지 상태"));
    if (drv & DRV_STEALTH)  Serial.println(F("    stealth: stealthChop 동작 중"));
    if (drv & DRV_FSACTIVE) Serial.println(F("    fsactive: fullstep 모드 동작 중"));
    if (drv & DRV_STALLGUARD) Serial.println(F("    stallGuard: 스톨 감지"));

    Serial.print(F("    cs_actual = "));
    Serial.print(csActualOf(drv));
    Serial.println(F("   (정지 중엔 IHOLD, 이동 중엔 IRUN 값이어야 정상)"));
}

bool step3_statusFlags() {
    hr("3. POWER / FAULT FLAGS");
    // GSTAT 은 래치 레지스터다. 읽기만으로 클리어되는지는 구현에 따라 다르므로
    // 그 가정에 기대지 않는다. 한 번 읽어 현재 값을 본 뒤 1 을 써서 명시적으로
    // 지우고(write-1-to-clear), 잠시 뒤 다시 읽는다. 이렇게 해야 "전원 투입 시
    // 한 번 래치된 흔적"과 "지금도 계속 발생 중"을 확실히 구분할 수 있다.
    uint32_t gstat = readRegister(REG_GSTAT);
    printHex32("GSTAT (1st)", gstat);
    if (gstat & 0x01) Serial.println(F("    reset  : 리셋 후 첫 읽기 (정상, 읽으면 클리어)"));
    if (gstat & 0x02) Serial.println(F("    drv_err: 드라이버 셧다운 이력 -> DRV_STATUS 확인"));
    if (gstat & 0x04) Serial.println(F("    uv_cp  : 차지펌프 저전압 이력 -> VM 전압 부족/불안정"));

    writeRegister(REG_GSTAT, 0x07); // 세 비트 모두 명시적으로 클리어
    delay(200);
    uint32_t gstat2 = readRegister(REG_GSTAT);
    printHex32("GSTAT (클리어 후)", gstat2);
    if (gstat2 == 0) {
        Serial.println(F("    >> 클리어 후 0: 위 플래그는 전원 투입 시 래치된 흔적일 뿐."));
        Serial.println(F("       현재 전원단은 정상이다."));
    } else {
        if (gstat2 & 0x02) Serial.println(F("    >> drv_err 지속 !! 실제 드라이버 고장/과열/단락"));
        if (gstat2 & 0x04) Serial.println(F("    >> uv_cp 지속 !! 차지펌프가 계속 저전압"));
        // 명시적으로 지운 뒤에도 reset 이 다시 섰다면 그 사이에 칩이 또
        // 리셋된 것 - VM 브라운아웃의 전형적인 패턴이다.
        if (gstat2 & 0x01) {
            Serial.println(F("    >> 클리어 후에도 reset 재설정 = 칩이 실제로 반복 리셋 중."));
            Serial.println(F("       VM 이 순간적으로 무너지고 있다는 뜻이다."));
        }
        Serial.println(F("       확인: (1) VM 전원이 실제로 인가되어 있는지 - USB 만 연결한 상태는 아닌지"));
        Serial.println(F("             (2) VM >= 8V 인지 (테스터로 모듈 VM-GND 직접 측정)"));
        Serial.println(F("             (3) 전원 용량과 VM 벌크 전해콘덴서(100uF 이상) 유무"));
        Serial.println(F("       VM 이 없으면 로직/SPI 는 3.3V 로 멀쩡히 동작하지만 모터는 절대 안 돈다."));
    }

    uint32_t drv = readRegister(REG_DRV_STATUS);
    decodeDrvStatus(drv);
    printSpiStatus();

    bool fault = (drv & DRV_FAULT_MASK) != 0;
    if (!fault) Serial.println(F("  >> 치명적 결함 플래그 없음."));
    return !fault;
}

// ============================================================
// 드라이버 기본 설정 (진단용 저전류)
// ============================================================
void configureDriver() {
    // en_pwm_mode 를 켜면 stealthChop. 기본은 spreadCycle(0) - 토크가 확실하다.
    writeRegister(REG_GCONF, g_useStealthChop ? 0x00000004UL : 0x00000000UL);
    // stealthChop 을 쓸 때도 고속 구간에서는 spreadCycle 로 넘어가도록 임계값을
    // 준다. TPWMTHRS=0 이면 전 속도에서 stealthChop 이라 토크가 죽는다.
    writeRegister(REG_TPWMTHRS, g_useStealthChop ? 500UL : 0UL);

    // TOFF=3 HSTRT=4 HEND=1 TBL=2, MRES 로 마이크로스텝 지정
    const uint32_t chopconf =
        (CFG_MRES << 24) | (2UL << 15) | (1UL << 7) | (4UL << 4) | 3UL;
    writeRegister(REG_CHOPCONF, chopconf);

    writeRegister(REG_GLOBALSCALER, CFG_GLOBALSCALER);
    writeRegister(REG_IHOLD_IRUN,
                  (CFG_IHOLDDELAY << 16) | (CFG_IRUN << 8) | CFG_IHOLD);
    writeRegister(REG_TPOWERDOWN, 10);

    writeRegister(REG_VSTART, 1);
    writeRegister(REG_A1, 1000);
    writeRegister(REG_V1, 0);        // V1=0 -> A1/D1 구간 없이 AMAX/DMAX 만 사용
    writeRegister(REG_AMAX, 1000);
    writeRegister(REG_VMAX, VEL_TEST);
    writeRegister(REG_DMAX, 1000);
    writeRegister(REG_D1, 1000);
    writeRegister(REG_VSTOP, 10);

    writeRegister(REG_RAMPMODE, 0);
    writeRegister(REG_XACTUAL, 0);
    writeRegister(REG_XTARGET, 0);
}

// ============================================================
// 4단계: 코일 연결 확인
// ============================================================
void step4_coilCheck() {
    hr("4. COIL / OUTPUT CHECK");
    configureDriver();
    digitalWrite(PIN_EN, LOW); // 출력 인에이블
    delay(200);

    uint32_t drv = readRegister(REG_DRV_STATUS);
    decodeDrvStatus(drv);
    uint8_t cs = csActualOf(drv);

    if (cs == 0) {
        Serial.println(F("  >> 전류 스케일이 0. GLOBALSCALER / IHOLD_IRUN 쓰기가 안 먹었음."));
    } else if (drv & (DRV_OLA | DRV_OLB)) {
        Serial.println(F("  >> open load 감지. 모터 결선(코일 A/B)과 커넥터를 확인."));
        Serial.println(F("     단, 정지 상태에서는 오탐이 잦으니 5단계 회전 중 값도 같이 보세요."));
    } else {
        Serial.print(F("  >> 코일 연결 정상으로 보임 (cs_actual="));
        Serial.print(cs);
        Serial.print(F(" = 설정한 IHOLD="));
        Serial.print(CFG_IHOLD);
        Serial.println(F(")."));
        Serial.println(F("     모터축을 손으로 돌려보세요. 홀딩 토크가 전혀 없으면 VM 전력단 문제."));
    }
}

// ============================================================
// 5단계: 실제 모션 (내부 램프 제너레이터)
// ============================================================
uint8_t g_maxCsInMotion = 0; // 이동 중 관측된 CS_ACTUAL 최대값

void reportMotion(const char *tag) {
    int32_t x = readSigned(REG_XACTUAL);
    int32_t v = readSigned(REG_VACTUAL);
    if (v & 0x00800000) v |= 0xFF000000; // VACTUAL 은 24bit signed
    uint32_t drv = readRegister(REG_DRV_STATUS);
    uint8_t cs = csActualOf(drv);
    if (v != 0 && cs > g_maxCsInMotion) g_maxCsInMotion = cs;

    Serial.print(F("    "));
    Serial.print(tag);
    Serial.print(F("  XACTUAL="));
    Serial.print(x);
    Serial.print(F("  VACTUAL="));
    Serial.print(v);
    Serial.print(F("  cs="));
    Serial.print(cs);
    Serial.print(F("  ola/olb="));
    Serial.print((drv & DRV_OLA) ? 1 : 0);
    Serial.print('/');
    Serial.print((drv & DRV_OLB) ? 1 : 0);
    Serial.print(F("  stealth="));
    Serial.println((drv & DRV_STEALTH) ? 1 : 0);
}

void step5_motion() {
    hr("5. MOTION TEST (velocity + position mode)");
    Serial.println(F("  >> 모터가 실제로 회전합니다. 축을 비워두세요."));
    g_maxCsInMotion = 0;
    configureDriver();
    digitalWrite(PIN_EN, LOW);

    Serial.print(F("  쵸퍼 모드: "));
    Serial.println(g_useStealthChop ? F("stealthChop") : F("spreadCycle"));

    Serial.println(F("  [velocity +] 8000 마이크로스텝/s (2.5 rev/s), 2초 정방향"));
    writeRegister(REG_VMAX, VEL_TEST);
    writeRegister(REG_RAMPMODE, 1);
    for (uint8_t i = 0; i < 4; ++i) { delay(500); reportMotion("t"); }

    Serial.println(F("  [velocity -] 2초 역방향"));
    writeRegister(REG_RAMPMODE, 2);
    for (uint8_t i = 0; i < 4; ++i) { delay(500); reportMotion("t"); }

    writeRegister(REG_VMAX, 0);
    writeRegister(REG_RAMPMODE, 1);
    delay(500);
    reportMotion("stop");

    int32_t start = readSigned(REG_XACTUAL);
    Serial.print(F("  [position] 현재 "));
    Serial.print(start);
    Serial.print(F(" 에서 +"));
    Serial.print(MICROSTEPS_PER_REV);
    Serial.println(F(" (1/16 마이크로스텝, 200step 모터 정확히 1회전) 이동"));
    writeRegister(REG_VMAX, VEL_POSITION);
    writeRegister(REG_RAMPMODE, 0);
    writeRegister(REG_XTARGET, start + MICROSTEPS_PER_REV);

    uint32_t t0 = millis();
    while (millis() - t0 < 4000) {
        delay(500);
        reportMotion("t");
        if ((g_spiStatus & 0x20) && (millis() - t0) > 600) {
            Serial.println(F("    position_reached"));
            break;
        }
    }
    int32_t endPos = readSigned(REG_XACTUAL);
    Serial.print(F("  이동량 = "));
    Serial.println(endPos - start);

    // 원위치 복귀
    writeRegister(REG_XTARGET, start);
    delay(3000);
    reportMotion("home");

    // 안전 정지. 여기서 출력을 끄므로 테스트가 끝난 뒤 전류계는 무부하값을
    // 가리키고 축도 헐거워진다 - 고장이 아니라 의도된 동작이다.
    writeRegister(REG_RAMPMODE, 1);
    writeRegister(REG_VMAX, 0);
    delay(300);
    digitalWrite(PIN_EN, HIGH);
    Serial.println(F("    [출력 비활성] 테스트 종료 - 이제 전류는 무부하, 축은 헐거운 것이 정상."));
    Serial.println(F("    홀딩 토크를 다시 걸려면 'r' (드라이버 재설정) 을 누르세요."));

    Serial.println();
    if (abs(endPos - start) < MICROSTEPS_PER_REV / 4) {
        Serial.println(F("  >> XACTUAL 이 거의 안 움직임: 램프 제너레이터가 동작하지 않음."));
        Serial.println(F("     RAMPMODE/VMAX/AMAX 쓰기 실패 또는 VM 미인가를 의심."));
        return;
    }

    Serial.println(F("  >> 램프 제너레이터 정상 동작 (SPI/로직 OK)."));

    // 전류단 판정: 이동 중 CS_ACTUAL 은 IRUN 까지 올라가야 한다.
    Serial.print(F("  이동 중 관측된 cs_actual 최대값 = "));
    Serial.print(g_maxCsInMotion);
    Serial.print(F("  (기대값 IRUN="));
    Serial.print(CFG_IRUN);
    Serial.println(F(")"));

    if (g_maxCsInMotion <= CFG_IHOLD) {
        Serial.println(F("  >> !! 이동 중에도 전류가 IHOLD 수준에 머무름 = 드라이버는 자신이"));
        Serial.println(F("     정지해 있다고 판단하고 있다."));
        if (g_sdModePin) {
            Serial.println(F("     원인 확정: SD_MODE=1 (STEP/DIR 모드)."));
            Serial.println(F("     내부 램프는 XACTUAL 만 갱신할 뿐 출력단과 연결되어 있지 않다."));
            Serial.println(F("     -> 모듈의 SD_MODE 를 GND 로 내리거나, STEP/DIR 구동으로 전환할 것."));
        } else {
            Serial.println(F("     확인 순서: VM 전압/용량 -> GSTAT 의 uv_cp 지속 여부 -> 센스저항/GLOBALSCALER"));
        }
    } else {
        Serial.println(F("  >> 전류도 IRUN 까지 정상 상승. 드라이버는 살아있음."));
        Serial.println(F("     그래도 축이 안 돌면 모터 결선 또는 기구부(구속/과부하)를 확인."));
    }
}

// ============================================================
// 6단계: STEP/DIR 구동 테스트
// SD_MODE=1 에서는 이것이 모터를 돌릴 수 있는 유일한 경로다. 동시에
// XACTUAL 이 외부 스텝 펄스를 따라가는지도 확인한다 - 따라간다면 기존
// getSPIPosition() 기반 위치/속도 코드를 그대로 재사용할 수 있다.
// ============================================================
void pulseSteps(bool dir, uint32_t count, uint32_t startPeriodUs, uint32_t fastPeriodUs) {
    digitalWrite(PIN_DIR, dir ? HIGH : LOW);
    delayMicroseconds(20);

    const uint32_t rampSteps = count / 4; // 앞 1/4 구간에서 가속
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t period = fastPeriodUs;
        if (i < rampSteps && rampSteps > 0) {
            period = startPeriodUs -
                     ((startPeriodUs - fastPeriodUs) * i / rampSteps);
        } else if (i > count - rampSteps && rampSteps > 0) {
            const uint32_t left = count - i;
            period = fastPeriodUs +
                     ((startPeriodUs - fastPeriodUs) * (rampSteps - left) / rampSteps);
        }
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(period - period / 2);
    }
}

void step6_stepDir() {
    hr("6. STEP/DIR 구동 테스트");
    if (!g_sdModePin) {
        Serial.println(F("  참고: SD_MODE=0 이라 내부 램프가 주 경로다. 이 테스트는 보조 확인용."));
    }
    configureDriver();
    digitalWrite(PIN_EN, LOW);
    delay(100);

    // 램프 제너레이터가 XACTUAL 을 0 으로 붙들고 있으면 스텝 카운트가 묻힌다.
    // hold 모드로 두어 램프를 떼어낸 뒤 순수하게 STEP 입력만 관찰한다.
    writeRegister(REG_RAMPMODE, 3);
    writeRegister(REG_XACTUAL, 0);
    delay(50);
    const int32_t before = readSigned(REG_XACTUAL);
    Serial.print(F("  펄스 전 XACTUAL = "));
    Serial.println(before);

    // 펄스를 나눠 쏘면서 중간에 DRV_STATUS 를 본다. 드라이버가 STEP 을 실제로
    // 받고 있다면 stst(정지) 비트가 풀리고 cs_actual 이 IRUN 으로 올라간다.
    // 이것이 "펄스가 드라이버에 도달하는가"에 대한 소프트웨어 증거다.
    Serial.print(F("  정방향 "));
    Serial.print(MICROSTEPS_PER_REV);
    Serial.println(F(" 펄스 (1회전) 송출 - 중간에 드라이버 상태 관찰..."));
    const uint32_t chunk = MICROSTEPS_PER_REV / 8;
    bool sawMotion = false;
    for (uint8_t i = 0; i < 8; ++i) {
        pulseSteps(false, chunk, 1500, 150);
        uint32_t drv = readRegister(REG_DRV_STATUS);
        bool stst = (drv & DRV_STST) != 0;
        if (!stst) sawMotion = true;
        Serial.print(F("    chunk "));
        Serial.print(i + 1);
        Serial.print(F("  stst="));
        Serial.print(stst ? 1 : 0);
        Serial.print(F("  cs="));
        Serial.print(csActualOf(drv));
        Serial.print(F("  XACTUAL="));
        Serial.println(readSigned(REG_XACTUAL));
    }
    Serial.print(F("  >> 펄스 중 stst 가 풀린 적: "));
    Serial.println(sawMotion ? F("있음 = 드라이버가 STEP 을 수신 중")
                             : F("없음 = STEP 이 드라이버에 도달하지 않음"));
    delay(200);
    const int32_t afterFwd = readSigned(REG_XACTUAL);
    Serial.print(F("  펄스 후 XACTUAL = "));
    Serial.print(afterFwd);
    Serial.print(F("   (변화량 "));
    Serial.print(afterFwd - before);
    Serial.println(F(")"));

    Serial.println(F("  역방향 동일 펄스 송출..."));
    pulseSteps(true, MICROSTEPS_PER_REV, 1500, 150);
    delay(200);
    const int32_t afterRev = readSigned(REG_XACTUAL);
    Serial.print(F("  복귀 후 XACTUAL = "));
    Serial.print(afterRev);
    Serial.print(F("   (변화량 "));
    Serial.print(afterRev - afterFwd);
    Serial.println(F(")"));

    digitalWrite(PIN_EN, HIGH);
    Serial.println(F("  [출력 비활성] 종료 - 전류 무부하/축 헐거움이 정상. 'r' 로 다시 켤 수 있음."));

    Serial.println();
    const int32_t moved = afterFwd - before;
    if (abs(moved) >= MICROSTEPS_PER_REV * 9 / 10) {
        Serial.println(F("  >> XACTUAL 이 STEP 펄스를 그대로 카운트한다."));
        Serial.println(F("     => STEP/DIR 로 전환해도 getSPIPosition() 기반 위치/속도 코드를"));
        Serial.println(F("        그대로 재사용할 수 있다. 펄스 생성기만 새로 만들면 된다."));
    } else if (moved != 0) {
        Serial.println(F("  >> XACTUAL 이 변하긴 하는데 펄스 수와 일치하지 않는다."));
        Serial.println(F("     마이크로스텝 설정(MRES)과 카운트 단위를 다시 맞춰볼 것."));
    } else {
        Serial.println(F("  >> XACTUAL 이 전혀 변하지 않았다."));
        Serial.println(F("     => 위치는 ESP32 쪽에서 펄스를 세어 직접 관리해야 한다."));
    }
    Serial.println(F("  * 모터가 실제로 돌았는지, 전류가 0.5A 부근까지 올랐는지 함께 확인하세요."));
}

void printMenu() {
    Serial.println();
    Serial.println(F("============================================================"));
    Serial.println(F("  d = 1~4단계 진단 (모터 안 돎)"));
    Serial.println(F("  m = 5단계 모션 테스트 - 내부 램프 (모터 회전! 축을 비워두세요)"));
    Serial.println(F("  p = 6단계 STEP/DIR 펄스 테스트 (모터 회전!)"));
    Serial.println(F("  s = 상태 플래그만 다시 읽기"));
    Serial.println(F("  r = 드라이버 재설정"));
    Serial.print(F("  c = 쵸퍼 모드 전환 (현재: "));
    Serial.print(g_useStealthChop ? F("stealthChop") : F("spreadCycle"));
    Serial.println(F(")"));
    Serial.println(F("============================================================"));
}

void runDiagnostics() {
    if (!step1_spiLink()) {
        Serial.println();
        Serial.println(F("  SPI 링크부터 실패. 이후 단계는 의미 없음 - 배선/전원 먼저 확인."));
        return;
    }
    step2_registerRoundTrip();
    step3_statusFlags();
    step4_coilCheck();
    Serial.println();
    Serial.println(F("  1~4단계 완료. 모터 회전까지 보려면 'm' 입력."));
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_EN, HIGH); // 진단 끝날 때까지 출력 비활성

    // STEP/DIR 을 띄워두면 노이즈로 원치 않는 스텝이 들어가고 IOIN 판독도
    // 지저분해진다. 내부 램프를 쓰더라도 명시적으로 LOW 로 고정한다.
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    delay(50);

    Serial.println();
    Serial.println(F("############################################################"));
    Serial.println(F("#                 TMC5160 DIAGNOSTICS                      #"));
    Serial.println(F("############################################################"));
    Serial.println(F("  EN=13  CS=5  SCK=18  MISO=19  MOSI=23"));

    runDiagnostics();
    printMenu();
}

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c) {
        case 'd': runDiagnostics(); printMenu(); break;
        case 'm': step5_motion();   printMenu(); break;
        case 'p': step6_stepDir();  printMenu(); break;
        case 's': step3_statusFlags(); break;
        case 'r':
            configureDriver();
            digitalWrite(PIN_EN, LOW); // 재설정 후 출력까지 다시 켜야 홀딩 토크가 돌아온다
            Serial.println(F("  드라이버 재설정 + 출력 활성. 축에 홀딩 토크가 걸려야 정상."));
            break;
        case 'c':
            g_useStealthChop = !g_useStealthChop;
            configureDriver();
            Serial.print(F("  쵸퍼 모드 -> "));
            Serial.println(g_useStealthChop ? F("stealthChop") : F("spreadCycle"));
            printMenu();
            break;
        default: break;
    }
}
