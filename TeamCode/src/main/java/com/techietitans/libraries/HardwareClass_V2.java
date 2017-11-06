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

    protected static DcMotor left_front_motor = null;
    protected static DcMotor right_front_motor = null;
    protected static DcMotor left_back_motor = null;
    protected static DcMotor right_back_motor = null;

    protected static DcMotor lift_motor = null;
    protected static DcMotor relic_motor = null;

    protected static Servo leftGlyphHolder  = null;
    protected static Servo rightGlyphHolder  = null;

    public Servo jewelPusherArm   = null;
    public Servo relicGrabber_hand   = null;
    public Servo relicGrabber_base   = null;
    protected static ModernRoboticsI2cGyro gyro  = null;
    protected static ModernRoboticsI2cColorSensor Color_jewel  = null;




    @Override
    public void init() {

        left_front_motor = hardwareMap.dcMotor.get("leftFront");
        right_front_motor = hardwareMap.dcMotor.get("rightFront");
        left_back_motor = hardwareMap.dcMotor.get("leftBack");
        right_back_motor = hardwareMap.dcMotor.get("rightBack");


        lift_motor = hardwareMap.dcMotor.get("lift");
        relic_motor = hardwareMap.dcMotor.get("relicGrabber");

        rightGlyphHolder = hardwareMap.servo.get("right_hand");
        leftGlyphHolder = hardwareMap.servo.get("left_hand");
       jewelPusherArm = hardwareMap.servo.get("jewel_arm");
//        relicGrabber_hand = hardwareMap.servo.get("relicGrabber_hs");
//        relicGrabber_base = hardwareMap.servo.get("relicGrabber_bs");

        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        left_back_motor.setDirection(DcMotor.Direction.REVERSE);


        try {
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        } catch (Exception p_exeception) {
            gyro = null;
        }
        try {
            Color_jewel   = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "Color_jewel");
        } catch (Exception p_exeception) {
            Color_jewel = null;
        }
    }

    @Override
    public void loop() {

    }

}
