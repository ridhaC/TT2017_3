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

    protected static Servo bottom_right_hand  = null;
    protected static Servo bottom_left_hand  = null;
    protected static Servo glyph_rotator  = null;
    protected static Servo jewel_pusher  = null;
    protected static Servo top_right_hand  = null;
    protected static Servo top_left_hand  = null;
    protected static Servo jewel_pusher_arm  = null;


    //public Servo jewelPusherArm   = null;
    public Servo relicGrabber_claw  = null;
    public Servo relicGrabber_base   = null;


    protected static ModernRoboticsI2cGyro gyro  = null;
    protected static ModernRoboticsI2cColorSensor Color_jewel  = null;

    //Init Parameters

    public static final double GLYPH_BOTTOM_RIGHT_SERVO_OPEN       =  0.20 ;
    public static final double GLYPH_BOTTOM_RIGHT_SERVO_CLOSE      =  0.60 ;
    public static final double GLYPH_BOTTOM_LEFT_SERVO_OPEN       =  0.55 ;
    public static final double GLYPH_BOTTOM_LEFT_SERVO_CLOSE      =  0.15 ;

    public static final double GLYPH_TOP_LEFT_SERVO_INIT = 185.0/256.0;
    public static final double GLYPH_TOP_RIGHT_SERVO_INIT = 10.0/256.0;


    public static final double GLYPH_ROTATOR_POSITION_A = 39/256.0;
    public static final double GLYPH_ROTATOR_POSITION_B = 18/256.0;


    public static final double JEWEL_PUSHER_REST = 256/256.0;    //******
    public static final double JEWEL_PUSHER_ARM_REST= 5/256.0;
    public static final double JEWEL_PUSHER_ARM_ENGAGE= 18/256.0;

    //******

    //185
    //10






    @Override
    public void init() {

        left_front_motor = hardwareMap.dcMotor.get("leftFront");
        right_front_motor = hardwareMap.dcMotor.get("rightFront");
        left_back_motor = hardwareMap.dcMotor.get("leftBack");
        right_back_motor = hardwareMap.dcMotor.get("rightBack");


        lift_motor = hardwareMap.dcMotor.get("lift");
        relic_motor = hardwareMap.dcMotor.get("relicGrabber");

        bottom_left_hand = hardwareMap.servo.get("bottom_left_hand");
        bottom_right_hand = hardwareMap.servo.get("bottom_right_hand");
        top_left_hand = hardwareMap.servo.get("top_left_hand");
        top_right_hand = hardwareMap.servo.get("top_right_hand");

        glyph_rotator = hardwareMap.servo.get("glyph_rotator");
        jewel_pusher = hardwareMap.servo.get("jewel_pusher");

        jewel_pusher_arm = hardwareMap.servo.get("jewel_pusher_arm");


        relicGrabber_claw = hardwareMap.servo.get("relicGrabber_claw");
        relicGrabber_base = hardwareMap.servo.get("relicGrabber_base");

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
