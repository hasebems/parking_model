//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

// 配線の重要事項！
//  LCD SDA/SCL のプルアップは必ず3.3V側に接続すること！！

use bsp::entry;
use defmt_rtt as _;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_0_2::PwmPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        bank0, FunctionI2C, FunctionPwm, FunctionSio, Pin, PullDown, PullUp, SioInput, SioOutput,
    },
    i2c::{Controller, I2C},
    pac,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
    Timer,
};
use fugit::RateExtU32;

use hd44780_embedded_hal::{
    interface::pcf8574::{Pcf8574EncoderDefault, Pcf8574Interface},
    types::DisplayTypeFont5x8,
    FnsetLines, Hd44780, Hd44780Trait,
};

// 最大計測距離
const MAX_DISTANCE_CM: u64 = 100;
// 音速(m/sec)
const VELOCITY_OF_SOUND: u64 = 340;
// タイムアウト時間（最大待機時間） 10000 = 0.01(m) * 1_000_000(usec)
const ULTRASONIC_TIMEOUT_USEC: u64 = (2 * 10000 * MAX_DISTANCE_CM) / VELOCITY_OF_SOUND;

type I2cPins = (
    Pin<bank0::Gpio4, FunctionI2C, PullUp>,
    Pin<bank0::Gpio5, FunctionI2C, PullUp>,
);
type I2cBus = I2C<pac::I2C0, I2cPins, Controller>;

//================================================
//  Main
//================================================
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Timer implements embedded-hal 1.0 DelayNs in rp2040-hal 0.10
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let lcd_delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // I2C0 on GP4(SDA) / GP5(SCL)
    let sda = pins
        .gpio4
        .into_pull_up_input()
        .into_function::<rp_pico::hal::gpio::FunctionI2C>();
    let scl = pins
        .gpio5
        .into_pull_up_input()
        .into_function::<rp_pico::hal::gpio::FunctionI2C>();

    let i2c = I2C::i2c0(
        pac.I2C0,
        sda,
        scl,
        100.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    let mut app = ParkingApp::new(
        bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS), // PWM sliceの作成
        i2c,
        lcd_delay,
        pins.gpio16.into_pull_down_input(),
        pins.gpio18.into_pull_down_input(),
        pins.gpio19.into_push_pull_output(),
    );

    // push button
    let mut button_pin = pins.gpio15.into_pull_up_input();

    // on-board LED
    let mut led_pin = pins.led.into_push_pull_output();

    let mut periodic_5ms: u64 = 0;
    let mut periodic_200ms: u64 = 0;

    loop {
        let distance = app.sensor.detect_distance(&timer);

        let crnt_time = timer.get_counter().ticks();
        if crnt_time - periodic_5ms > 5000 {
            let open = button_pin.is_low().ok().unwrap() | app.sensor.is_open();
            app.svc.periodic(open);
            periodic_5ms = crnt_time;
        }

        // Heartbeat LED
        let crnt_time = timer.get_counter().ticks();
        if crnt_time - periodic_200ms < 100 * 1000 {
            led_pin.set_high().unwrap();
        } else if crnt_time - periodic_200ms < 200 * 1000 {
            led_pin.set_low().unwrap();
        } else {
            periodic_200ms = crnt_time;
        }

        app.display.display_distance(distance);
    }
}

//================================================
//  Parking App
//================================================
struct ParkingApp {
    pub svc: ServoController,
    pub sensor: SuperSonicSensor,
    pub display: DisplayController,
}
impl ParkingApp {
    fn new(
        pwm_slices: Slices,
        i2c: I2cBus,
        lcd_delay: cortex_m::delay::Delay,
        pin16: Pin<bank0::Gpio16, FunctionSio<SioInput>, PullDown>,
        pin18: Pin<bank0::Gpio18, FunctionSio<SioInput>, PullDown>,
        pin19: Pin<bank0::Gpio19, FunctionSio<SioOutput>, PullDown>,
    ) -> Self {
        ParkingApp {
            svc: ServoController::new(pwm_slices, pin16),
            sensor: SuperSonicSensor::new(pin18, pin19),
            display: DisplayController::new(i2c, lcd_delay),
        }
    }
}
//================================================
struct ServoController {
    duty_max: i64,
    //pwm_channel: bsp::hal::pwm::PwmChannel<bsp::hal::pac::PWM, bsp::hal::pwm::C0, bsp::hal::pwm::A>,
    pwm_channel: bsp::hal::pwm::Channel<
        bsp::hal::pwm::Slice<bsp::hal::pwm::Pwm0, bsp::hal::pwm::FreeRunning>,
        bsp::hal::pwm::A,
    >,
    open_state: bool,
    last_duty: i64,
}
impl ServoController {
    fn new(pwm_slices: Slices, pin16: Pin<bank0::Gpio16, FunctionSio<SioInput>, PullDown>) -> Self {
        // PWM0の有効化
        let mut pwm0 = pwm_slices.pwm0;
        pwm0.set_ph_correct();
        pwm0.enable();

        // PWMの周波数を50Hzに設定する
        // (PWMの周波数) = (125MHz) / ((top + 1) * div)
        // div = div_int + div_frac / 16
        pwm0.set_top(24999);
        pwm0.set_div_int(100);
        pwm0.set_div_frac(0);

        // チャンネルの割当
        // GPIOピンによって使えるPWM番号とチャンネルが決まっている
        // https://ushitora.net/archives/2617 などを参照
        //let pwm_channel = &mut pwm0.channel_a;
        let pin16 = pin16.into_function::<FunctionPwm>();

        let mut channel_a = pwm0.channel_a;
        channel_a.output_to(pin16);

        // Duty比の調整
        // set_duty() は 0..=duty_max の範囲で指定する（u16）。
        // サーボ用途では 50Hz でおおむね 1ms〜2ms(= 5%〜10%) のパルス幅を想定。

        ServoController {
            duty_max: channel_a.get_max_duty() as i64,
            pwm_channel: channel_a,
            open_state: false,
            last_duty: 0,
        }
    }
    fn periodic(&mut self, open: bool) {
        // 5ms周期で呼び出される
        if !self.open_state && open {
            self.open_state = true;
        } else if self.open_state && !open {
            self.open_state = false;
        }
        if !self.open_state {
            self.last_duty += 20;
            if self.last_duty > (self.duty_max / 27) {
                // 90度相当
                self.last_duty = self.duty_max / 27;
            }
        } else {
            self.last_duty -= 20;
            if self.last_duty < 0 {
                self.last_duty = 0;
            }
        }
        self.pwm_channel.set_duty(self.last_duty as u16);
    }
}
//================================================
struct SuperSonicSensor {
    trig_pin: Pin<bank0::Gpio19, FunctionSio<SioOutput>, PullDown>,
    echo_pin: Pin<bank0::Gpio18, FunctionSio<SioInput>, PullDown>,
    distance: u64,
}
impl SuperSonicSensor {
    fn new(
        pin18: Pin<bank0::Gpio18, FunctionSio<SioInput>, PullDown>,
        pin19: Pin<bank0::Gpio19, FunctionSio<SioOutput>, PullDown>,
    ) -> Self {
        SuperSonicSensor {
            trig_pin: pin19,
            echo_pin: pin18,
            distance: 0,
        }
    }
    fn delay_us(timer: &Timer, us: u64) {
        let start = timer.get_counter().ticks();
        while timer.get_counter().ticks().wrapping_sub(start) < us {}
    }
    fn detect_distance(&mut self, timer: &Timer) -> u64 {
        // トリガーから10マイクロ秒のパルスをぶちこむ
        self.trig_pin.set_low().unwrap();
        Self::delay_us(timer, 2);
        self.trig_pin.set_high().unwrap();
        Self::delay_us(timer, 10);
        self.trig_pin.set_low().unwrap();

        // 超音波が発射されたときのクロックティックを取得（発射されるまでlowを維持）
        let start_time = timer.get_counter().ticks();
        let mut low_time = start_time;
        while self.echo_pin.is_low().ok().unwrap() {
            low_time = timer.get_counter().ticks();
            if (low_time - start_time) > ULTRASONIC_TIMEOUT_USEC {
                break;
            }
        }
        // 超音波を受信したときのクロックティックを取得（反射波を捉えるまでhighを維持）
        let mut high_time = low_time;
        while self.echo_pin.is_high().ok().unwrap() {
            high_time = timer.get_counter().ticks();
            if (high_time - low_time) > ULTRASONIC_TIMEOUT_USEC {
                break;
            }
        }

        // 超音波が発射されて帰ってくるまでの経過時間から距離を算出（mm）
        let raw_distance = ((high_time - low_time) * VELOCITY_OF_SOUND) / (2 * 1000);

        // 平均距離を指数移動平均で更新
        self.distance = (95 * self.distance + 5 * raw_distance) / 100;

        self.distance
    }
    fn is_open(&mut self) -> bool {
        self.distance < 100
    }
}
//================================================
struct LcdDelay {
    inner: cortex_m::delay::Delay,
}
impl embedded_hal::delay::DelayNs for LcdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let us = ns.div_ceil(1_000); //(ns + 999) / 1_000;
        if us > 0 {
            self.inner.delay_us(us);
        }
    }
}
//================================================
type LcdInterface = Pcf8574Interface<I2cBus, LcdDelay, Pcf8574EncoderDefault>;
type Lcd = Hd44780<LcdInterface, DisplayTypeFont5x8>;

struct DisplayController {
    lcd: Lcd,
}
impl DisplayController {
    fn new(i2c: I2cBus, delay: cortex_m::delay::Delay) -> Self {
        let addr = 0x27;
        // Typical backpack mapping:
        // P0=RS, P1=RW, P2=E, P3=BL, P4..P7=D4..D7
        let encoder = Pcf8574EncoderDefault::new();
        let delay = LcdDelay { inner: delay };
        let interface = Pcf8574Interface::new(i2c, addr, delay, encoder);
        let mut lcd = Hd44780::new(interface, DisplayTypeFont5x8::new(2, 16, FnsetLines::Two)); // 16 columns x 2 lines, 5x8 font
        lcd.init().unwrap();
        lcd.clear().unwrap();
        DisplayController { lcd }
    }
    fn display_distance(&mut self, distance: u64) {
        self.lcd.position(0, 0).unwrap();
        self.lcd.print_str("Dist.:  ").unwrap();
        let mut buf = [32u8; 6];
        let cnt_str = Self::i16_to_str(distance as i16, &mut buf);
        self.lcd.print_str(cnt_str).unwrap();
        self.lcd.print_str("mm").unwrap();
    }
    fn i16_to_str(n: i16, out: &mut [u8; 6]) -> &str {
        // Max length of i16 is "-32768" (6 bytes)
        let mut i = out.len();

        let mut v: i32 = n as i32;
        let negative = v < 0;
        if negative {
            v = -v; // safe in i32 even for i16::MIN
        }

        // write at least one digit
        loop {
            let digit = (v % 10) as u8;
            v /= 10;
            i -= 1;
            out[i] = b'0' + digit;
            if v == 0 {
                break;
            }
        }

        if negative {
            i -= 1;
            out[i] = b'-';
        }

        core::str::from_utf8(out).unwrap()
    }
}
// End of file
