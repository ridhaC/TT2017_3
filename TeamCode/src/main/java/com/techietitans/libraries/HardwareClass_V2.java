package com.techietitans.libraries;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.I2cController;

//import com.qualcomm.robotcore.hardware.

/**
 * Created by vinayjagan on 10/3/15.
 */
public abstract class HardwareClass_V2 extends OpMode {

    protected static DcMotor front_left;
    protected static DcMotor front_right;
    protected static DcMotor lift;
    protected static DcMotor shooter;
    protected static OpticalDistanceSensor ods_front;
    protected static ModernRoboticsI2cGyro gyro;
    public static Servo pusher_left;
    public static Servo pusher_right;
    protected static Servo latch;
    protected static Servo release;
    protected static TouchSensor touch_left;
    protected static TouchSensor touch_right;

    // Declare Color Sensor objects (and other items required to enable/disable)
    protected static ModernRoboticsI2cColorSensor  mrcolor_under = null;
    protected static ModernRoboticsI2cColorSensor  mrcolor_front = null;

    protected static I2cAddr underColorAddress  = I2cAddr.create8bit(0x4c);
    protected static I2cAddr frontColorAddress = I2cAddr.create8bit(0x3c);

    protected static I2cController   underColorController;
    protected static I2cController   frontColorController;

    protected static I2cController.I2cPortReadyCallback underColorCallback;
    protected static I2cController.I2cPortReadyCallback frontColorCallback;




    @Override
    public void init() {


    }

    @Override
    public void loop() {

    }

}
