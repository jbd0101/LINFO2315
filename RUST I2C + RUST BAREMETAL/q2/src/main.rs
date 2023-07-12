#![allow(warnings, unused)]

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::delay::{FreeRtos,BLOCK};
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::i2c::*;
use esp_idf_hal::prelude::*;


use std::sync::mpsc;
use std::thread;

const I2C_BMP: u8= 0x77;
const BMP180_START_MEASURMENT_REG: u8 = 0xF4;
const BMP180_MEASURMENT_TEMP: u8 = 0x2E;
const BMP180_READ_ADC_MSB_REG: u8=0xF6;
const BMP180_GET_ID_REG: u8=0xD0;
const BMP180_CAL_AC5_REG: u8=0xB2;
const BMP180_CAL_AC6_REG: u8=0xB4;
const BMP180_CAL_MC_REG: u8=0xBC;
const BMP180_CAL_MD_REG: u8=0xBE;

fn computeB5(ut: u16, ac5: u16, ac6: u16, mc: i16, md: i16) -> i32 {
    let x1: i32 = ((ut as i32 - ac6 as i32) * (ac5 as i32)) >> 15;
    let x2: i32 = ((mc as i32) << 11) / (x1 + md as i32);
    return x1 + x2;
}


fn main(){
    let mut bmpAC6: u16 = 0;
    let mut bmpAC5: u16 = 0;
    let mut bmpMC: i16= 0;
    let mut bmpMD: i16 = 0;
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    println!("Hello, world!");

    //create a mpc channel
    let (tx, rx) = mpsc::channel();
    tx.send(42).unwrap();

    let peripherals = Peripherals::take().unwrap();
    let mut led =  PinDriver::output(peripherals.pins.gpio35).unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio45;
    let scl = peripherals.pins.gpio46;
    let config = I2cConfig::new().baudrate(100.kHz().into());
    let mut i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();

    // read coefficients
    let mut data = [0u8; 2];
    i2c.write_read(I2C_BMP, &[BMP180_CAL_AC5_REG], &mut data,BLOCK).unwrap();
    bmpAC5 = data[0] as u16 * 256 + data[1] as u16;
    println!("bmpAC5: {:?}", bmpAC5);
    thread::sleep(std::time::Duration::from_millis(20));


    let mut data = [0u8; 2];
    i2c.write_read(I2C_BMP, &[BMP180_CAL_AC6_REG], &mut data,BLOCK).unwrap();
    bmpAC6 = data[0] as u16 * 256 + data[1] as u16;
    thread::sleep(std::time::Duration::from_millis(20));
    println!("bmpAC6: {:?}", bmpAC6);


    let mut data = [0u8; 2];
    i2c.write_read(I2C_BMP, &[BMP180_CAL_MC_REG], &mut data,BLOCK).unwrap();
    bmpMC = (u16::from_be_bytes(data) as u16) as i16;
    thread::sleep(std::time::Duration::from_millis(20));
    println!("bmpMC: {:?}", bmpMC);


    let mut data = [0u8; 2];
    i2c.write_read(I2C_BMP, &[BMP180_CAL_MD_REG], &mut data,BLOCK).unwrap();
    bmpMD = (data[0] as u16 * 256 + data[1] as u16) as i16;
    thread::sleep(std::time::Duration::from_millis(20));

    println!("bmpMD: {:?}", bmpMD);




    loop {
        //start BMP180 I2C
        i2c.write(I2C_BMP, &[BMP180_START_MEASURMENT_REG, BMP180_MEASURMENT_TEMP],BLOCK).unwrap();

        thread::sleep(std::time::Duration::from_millis(5));
        let mut data = [0u8; 2];
        i2c.write_read(I2C_BMP, &[BMP180_READ_ADC_MSB_REG], &mut data,BLOCK).unwrap();
        println!("data: {:?}", data);
        let temp = data[0] as u16 * 256 + data[1] as u16;
        let tf: f32 = 0.1*(((computeB5(temp, bmpAC5, bmpAC6, bmpMC, bmpMD)+8) >> 4 )as f32);
        // read chip id
        println!("temp: {:?}", tf);
        let mut data = [0u8; 1];
        i2c.write_read(I2C_BMP, &[BMP180_GET_ID_REG], &mut data,BLOCK).unwrap();
        println!("chip id: {:?}", data[0]);

        thread::sleep(std::time::Duration::from_millis(2000));


    }





}
