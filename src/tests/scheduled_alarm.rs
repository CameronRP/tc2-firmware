//! Unit tests for `MainI2C::get_scheduled_alarm`.
//!
//! The PCF8563 alarm block only stores minutes, hours, day-of-month and weekday — there are no
//! month or year alarm registers on the part. `get_scheduled_alarm` therefore has to reconstruct
//! the full date by borrowing the year and month from the current time, and inferring which month
//! the alarm belongs to from the day and the already-triggered flag.
//!
//! These tests drive that reconstruction directly, by planting alarm register values in the mock
//! RTC and asking for them back.

use crate::attiny_rtc_i2c::{AlarmMode, MainI2C, encode_bcd};
use crate::re_exports::bsp::hal::clocks::ClocksManager;
use crate::re_exports::bsp::hal::gpio::bank0::{Gpio3, Gpio6, Gpio7};
use crate::re_exports::bsp::hal::gpio::{FunctionI2C, FunctionSio, Pin, PullUp, SioInput};
use crate::re_exports::bsp::hal::{I2C, Timer};
use crate::re_exports::bsp::pac::{CLOCKS, I2C1, RESETS, TIMER};
use crate::synced_date_time::SyncedDateTime;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::{DateTime, NaiveDate, TimeZone, Utc};
use fugit::RateExtU32;

/// Alarm control register: alarm interrupt enabled, not yet fired.
const ALARM_PENDING: u8 = 0b0000_0010;
/// Alarm control register: alarm interrupt enabled, and the alarm flag has fired.
const ALARM_TRIGGERED: u8 = 0b0000_1010;
/// Weekday register: weekday matching disabled (bit 7), mode bits clear == `AlarmMode::Thermal`.
const WEEKDAY_THERMAL: u8 = 0b1000_0000;

fn timer() -> Timer {
    Timer::new(TIMER, &RESETS, &ClocksManager::new(CLOCKS))
}

/// Build a `MainI2C` backed by the mock I2C peripheral, exactly as [`crate::entry`] does.
fn mock_i2c() -> MainI2C {
    let sda: Pin<Gpio6, FunctionI2C, PullUp> = Pin::new(Gpio6, FunctionI2C, PullUp);
    let scl: Pin<Gpio7, FunctionI2C, PullUp> = Pin::new(Gpio7, FunctionI2C, PullUp);
    let unlocked_pin: Pin<Gpio3, FunctionSio<SioInput>, PullUp> =
        Pin::new(Gpio3, FunctionSio::new(SioInput), PullUp);
    MainI2C::new(
        I2C::i2c1(
            I2C1,
            sda,
            scl,
            400.kHz(),
            &mut RESETS,
            125_000_000u32.Hz(),
        ),
        unlocked_pin,
        timer(),
    )
    .expect("mock attiny should report the expected firmware version")
}

fn utc(year: i32, month: u32, day: u32, hour: u32, min: u32) -> DateTime<Utc> {
    Utc.from_utc_datetime(
        &NaiveDate::from_ymd_opt(year, month, day)
            .unwrap()
            .and_hms_opt(hour, min, 0)
            .unwrap(),
    )
}

/// Plant the raw alarm registers the way the RTC would hold them, and set the current time.
fn given_alarm(day: u8, hours: u8, minutes: u8, control: u8, now: DateTime<Utc>) {
    TEST_SIM_STATE.with(|s| {
        let mut s = s.borrow_mut();
        s.current_time = now;
        s.rtc_alarm_state.minutes = encode_bcd(minutes);
        s.rtc_alarm_state.hours = encode_bcd(hours);
        s.rtc_alarm_state.day = encode_bcd(day);
        s.rtc_alarm_state.weekday_alarm_mode = WEEKDAY_THERMAL;
        s.rtc_alarm_state.enabled = control;
    });
}

// ---------------------------------------------------------------------------
// Control cases: these pass both before and after the fix, and exist to show
// that the harness itself agrees with the current behaviour.
// ---------------------------------------------------------------------------

#[test]
fn pending_alarm_later_today_is_read_back_unchanged() {
    let now = utc(2026, 4, 10, 18, 0);
    given_alarm(10, 22, 30, ALARM_PENDING, now);

    let alarm = mock_i2c()
        .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
        .expect("alarm should be readable");

    assert_eq!(alarm.date_time(), utc(2026, 4, 10, 22, 30));
    assert_eq!(alarm.mode, AlarmMode::Thermal);
    assert!(!alarm.has_triggered());
}

#[test]
fn pending_alarm_early_next_month_rolls_the_month_forward() {
    // It's the 28th; the alarm day is the 1st, so it must belong to next month.
    let now = utc(2026, 4, 28, 23, 0);
    given_alarm(1, 5, 0, ALARM_PENDING, now);

    let alarm = mock_i2c()
        .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
        .expect("alarm should be readable");

    assert_eq!(alarm.date_time(), utc(2026, 5, 1, 5, 0));
}

#[test]
fn triggered_alarm_late_last_month_rolls_the_month_back() {
    // It's the 2nd; the alarm day is the 28th and it has fired, so it was last month.
    let now = utc(2026, 4, 2, 1, 0);
    given_alarm(28, 22, 0, ALARM_TRIGGERED, now);

    let alarm = mock_i2c()
        .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
        .expect("alarm should be readable");

    assert_eq!(alarm.date_time(), utc(2026, 3, 28, 22, 0));
    assert!(alarm.has_triggered());
}

// ---------------------------------------------------------------------------
// Failing cases: the alarm is a real, valid date, but the current code builds
// an intermediate date in the *current* month before correcting the month, and
// panics when the alarm's day doesn't exist in that intermediate month.
// ---------------------------------------------------------------------------

#[test]
fn triggered_alarm_on_the_31st_read_after_midnight_into_a_30_day_month() {
    // Alarm fired at 22:00 on 31 March. The camera is still awake ten seconds into 1 April,
    // and bookkeeping re-reads the alarm. April has 30 days, so the intermediate
    // NaiveDate::from_ymd_opt(2026, 4, 31) is None and the current code unwraps it.
    let now = utc(2026, 4, 1, 0, 0);
    given_alarm(31, 22, 0, ALARM_TRIGGERED, now);

    let alarm = mock_i2c()
        .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
        .expect("a 31 March alarm is a real date and should be readable on 1 April");

    assert_eq!(alarm.date_time(), utc(2026, 3, 31, 22, 0));
}

#[test]
fn triggered_alarm_on_the_31st_read_on_the_1st_of_february() {
    // Same shape, but the worst case: 31 January read on 1 February.
    let now = utc(2026, 2, 1, 0, 30);
    given_alarm(31, 23, 45, ALARM_TRIGGERED, now);

    let alarm = mock_i2c()
        .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
        .expect("a 31 January alarm is a real date and should be readable on 1 February");

    assert_eq!(alarm.date_time(), utc(2026, 1, 31, 23, 45));
}

#[test]
fn triggered_alarm_on_the_30th_read_on_the_1st_of_a_31_day_month() {
    // Here the intermediate date (2026-05-30) *is* valid, so there's no panic — but
    // checked_sub_months clamps 2026-04-30... which is right. Use March instead, where
    // subtracting a month from a valid 2026-03-30 lands on a February that has no 30th
    // and silently clamps to the 28th.
    let now = utc(2026, 3, 1, 2, 0);
    given_alarm(30, 21, 0, ALARM_TRIGGERED, now);

    // 30 February never existed, so there is no correct date to report here; the alarm
    // registers are unresolvable and the read should fail rather than invent one.
    let alarm = mock_i2c().get_scheduled_alarm(&SyncedDateTime::new(now, timer()));

    assert!(
        alarm.is_none(),
        "an unresolvable alarm day should not be silently clamped to a different date, got {:?}",
        alarm.map(|a| a.date_time())
    );
}

// ---------------------------------------------------------------------------
// Failing cases: garbage register values. decode_bcd does no range checking, so
// a corrupt read produces out-of-range fields that are unwrapped straight into
// NaiveDate/NaiveTime.
// ---------------------------------------------------------------------------

#[test]
fn out_of_range_day_is_an_error_not_a_panic() {
    // The day field is masked to 6 bits, so decode_bcd can yield up to 45.
    let now = utc(2026, 4, 10, 18, 0);
    given_alarm(10, 22, 30, ALARM_PENDING, now);
    TEST_SIM_STATE.with(|s| s.borrow_mut().rtc_alarm_state.day = 0b0011_1111);

    assert!(
        mock_i2c()
            .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
            .is_none(),
        "a corrupt day register should read back as no alarm"
    );
}

#[test]
fn zero_day_is_an_error_not_a_panic() {
    // chrono days are 1-based; a zeroed day register with the alarm still flagged as
    // enabled has no valid interpretation.
    let now = utc(2026, 4, 10, 18, 0);
    given_alarm(0, 22, 30, ALARM_PENDING, now);

    assert!(
        mock_i2c()
            .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
            .is_none(),
        "a zeroed day register should read back as no alarm"
    );
}

#[test]
fn out_of_range_hours_is_an_error_not_a_panic() {
    // The hours field is masked to 6 bits, so decode_bcd can yield up to 45.
    let now = utc(2026, 4, 10, 18, 0);
    given_alarm(10, 35, 30, ALARM_PENDING, now);

    assert!(
        mock_i2c()
            .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
            .is_none(),
        "a corrupt hours register should read back as no alarm"
    );
}

#[test]
fn out_of_range_minutes_is_an_error_not_a_panic() {
    // The minutes field is masked to 7 bits, so decode_bcd can yield up to 85.
    let now = utc(2026, 4, 10, 18, 0);
    given_alarm(10, 22, 75, ALARM_PENDING, now);

    assert!(
        mock_i2c()
            .get_scheduled_alarm(&SyncedDateTime::new(now, timer()))
            .is_none(),
        "a corrupt minutes register should read back as no alarm"
    );
}
