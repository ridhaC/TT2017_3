/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.techietitans.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

import static java.lang.Math.abs;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * =================================================================================
 * JOYSTICK ASSIGNMENTS
 * =================================================================================
 * gamepad1.a : regular speed
 * gamepad1.b : low speed (one-fifth power)

 * gamepad1.left_stick_y: forward-backward move
 * gamepad1.left_stick_x: sideways move


 * gamepad2.left_stick_y: Lift motor
 * gamepad2.right_stick_y: Relic motor

 * gamepad2.a : glyph holder mechanism standard position
 * gamepad2.b : glyph holder mechanism 180 degree rotated

 * gamepad2.right_trigger: GLYPH BOTTOM SERVOs OPEN
 * gamepad2.left_trigger: GLYPH BOTTOM SERVOs CLOSE

 * gamepad2.left_bumper: GLYPH TOP SERVOs CLOSE
 * gamepad2.right_bumper: GLYPH TOP SERVOs OPEN
 * ================================================================================
 */

@TeleOp(name="Faraaz Mecanum opmode", group="Faraaz")
//@Disabled
public class PushbotTeleopTank_Mecanum extends OpMode{

    /* Declare OpMode members. */
    // HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    // double          clawOffset  = 0.0 ;                  // Servo mid position
    // final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_front_motor;
    private DcMotor right_front_motor;
    private DcMotor left_back_motor;
    private DcMotor right_back_motor;

    private DcMotor lift_motor;
    private DcMotor relic_motor;

    public Servo topLeftGlyphHolder    = null;
    public Servo topRightGlyphHolder   = null;
    public Servo bottomLeftGlyphHolder    = null;
    public Servo bottomRightGlyphHolder   = null;
    public Servo glyphHolderRotator    = null;

    public Servo jewelPusherArm   = null;
    public Servo jewel_pusher  = null;
    public Servo relicGrabber_claw   = null;
    public Servo relicGrabber_base   = null;

    public static final double GLYPH_TOP_RIGHT_SERVO_OPEN       =  130/256.0 ;  // was 0.25
    public static final double GLYPH_TOP_RIGHT_SERVO_CLOSE      =  35.0/255.0 ;  // was 0.6
    public static final double GLYPH_TOP_LEFT_SERVO_OPEN       =  70.0/256.0 ;   // was 0.7 // opposite values than Right Servo
    public static final double GLYPH_TOP_LEFT_SERVO_CLOSE      =   150.0/255.0;  // was 0.375
    public static final double GLYPH_BOTTOM_RIGHT_SERVO_OPEN       =  50/256.0 ;
    public static final double GLYPH_BOTTOM_RIGHT_SERVO_CLOSE      =  137/256.0 ;
    public static final double GLYPH_BOTTOM_LEFT_SERVO_OPEN       =  137/256.0 ; // 0.59   // opposite values than Right Servo
    public static final double GLYPH_BOTTOM_LEFT_SERVO_CLOSE      =  50/256.0 ; // 0.275
    public static final double GLYPH_ROTATOR_POSITION_A = 39/256.0;
    public static final double GLYPH_ROTATOR_POSITION_B = 18/256.0;

    private boolean lowSpeed = false;
    private boolean positionA = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        left_front_motor = hardwareMap.dcMotor.get("leftFront");
        right_front_motor = hardwareMap.dcMotor.get("rightFront");
        left_back_motor = hardwareMap.dcMotor.get("leftBack");
        right_back_motor = hardwareMap.dcMotor.get("rightBack");

        lift_motor = hardwareMap.dcMotor.get("lift");
        relic_motor = hardwareMap.dcMotor.get("relicGrabber");

        topRightGlyphHolder = hardwareMap.servo.get("top_right_hand");
        topLeftGlyphHolder = hardwareMap.servo.get("top_left_hand");
        bottomRightGlyphHolder = hardwareMap.servo.get("bottom_right_hand");
        bottomLeftGlyphHolder = hardwareMap.servo.get("bottom_left_hand");
        glyphHolderRotator = hardwareMap.servo.get("glyph_rotator");

//        jewelPusherArm = hardwareMap.servo.get("jewel_arm");
        jewel_pusher = hardwareMap.servo.get("jewel_pusher");
        relicGrabber_claw = hardwareMap.servo.get("relicGrabber_claw");
        relicGrabber_base = hardwareMap.servo.get("relicGrabber_base");

        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        left_back_motor.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Program initialized");    //

        // init the rotating glyph holder
        glyphHolderRotator.setPosition(GLYPH_ROTATOR_POSITION_A);
        positionA = true;

        topRightGlyphHolder.setPosition(GLYPH_TOP_RIGHT_SERVO_OPEN);
        topLeftGlyphHolder.setPosition(GLYPH_TOP_LEFT_SERVO_OPEN);
        bottomRightGlyphHolder.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_OPEN);
        bottomLeftGlyphHolder.setPosition(GLYPH_BOTTOM_LEFT_SERVO_OPEN);
        jewel_pusher.setPosition(256/256.0);
        relicGrabber_base.setPosition(33.0/255.0);
        relicGrabber_claw.setPosition(0.0/255.0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //double left;
        //double right;
        telemetry.addData("Status", "Running: " + runtime.toString());
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        float LFspeed = 0;
        float LBspeed = 0;
        float RFspeed = 0;
        float RBspeed = 0;

        int threshold = 10;

        telemetry.addData("Status", "Gamepad L: " + gamepad1.left_stick_x + " " + gamepad1.left_stick_y + " R: " + gamepad1.right_stick_x + " " + gamepad1.right_stick_y);

        if (gamepad1.b)
            lowSpeed = true;

        if (gamepad1.a)
            lowSpeed = false;

        // check for forward-backward move - gamepad1.left_stick_y
        if (abs(gamepad1.left_stick_y)*100 >= threshold) {
            LFspeed = -gamepad1.left_stick_y ;  // ignore x component
            LBspeed = -gamepad1.left_stick_y ;
            RFspeed = -gamepad1.left_stick_y ;
            RBspeed = -gamepad1.left_stick_y ;

            telemetry.addData("Status", "***** forward/backward *****");
        }


        // check for sideways move - gamepad1.left_stick_x. Will take precedence over vertical move
        if (abs(gamepad1.left_stick_x)*100 >= threshold) {
            LFspeed = gamepad1.left_stick_x;  // ignore y component  // flipping from original
            LBspeed = -gamepad1.left_stick_x;
            RFspeed = -gamepad1.left_stick_x;
            RBspeed = gamepad1.left_stick_x;

            telemetry.addData("Status", "***** sideways *****");
        }

        telemetry.addData("Status", "*****3*****");


        // ========================================================
        // Lift motor - GAMEPAD2 LEFT STICK Y
        // check if it's going opposite direction (up vs down)

        double lift_motor_power = 0;
        if (abs(gamepad2.left_stick_y)*100 >= threshold) {  // no-op
            telemetry.addData("Status", "Lift motor ");

            lift_motor_power = gamepad2.left_stick_y;


            if (gamepad2.left_stick_y > 0)// -ve going opposite?
                lift_motor_power = 0.2;

        }

        telemetry.addData("Status", "***** Lift motor set power *****");
        lift_motor_power = Range.clip(lift_motor_power, -1, 1);
        lift_motor.setPower(lift_motor_power);

        // ========================================================
        // Relic motor - GAMEPAD2 RIGHT STICK Y
        // check if it's going opposite direction (up vs down)

        double relic_motor_power = 0;
        if (abs(gamepad2.right_stick_y)*100 >= threshold) {  // no-op
            telemetry.addData("Status", "Relic motor ");

            relic_motor_power = gamepad2.right_stick_y;

            /*
            if (gamepad2.right_stick_y > 0)
                relic_motor_power = 1.0;         // -ve going opposite?
            else
                relic_motor_power = -0.5;
               */
        }

        telemetry.addData("Status", "***** Relic motor set power *****");
        relic_motor_power = Range.clip(relic_motor_power, -1, 1);
        relic_motor.setPower(relic_motor_power);


        // ========================================================
        // GLYPH SERVOs - using GAMEPAD2
        // right servo works with these values. Left servo needs to be reversed

        // Rotate glyph backbone servo - using A and B button in GAMEPAD2
        if (gamepad2.a) { // if (gamepad2.a && !positionA) {
            glyphHolderRotator.setPosition(GLYPH_ROTATOR_POSITION_A);
            positionA = !positionA;
            telemetry.addData("Status", "***** GLYPH ROTATOR - POSITION A *****");
        }
        else if (gamepad2.b) { //if (gamepad2.b && positionA) {
            glyphHolderRotator.setPosition(GLYPH_ROTATOR_POSITION_B);
            positionA = !positionA;
            telemetry.addData("Status", "***** GLYPH ROTATOR - POSITION B *****");
        }

        // using JOYSTICK BUMPERS for top glyph
        if (gamepad2.left_bumper || gamepad2.right_bumper) {

            telemetry.addData("Status", "TOP Glyph Servos ");

            if (gamepad2.left_bumper) { //left_stick_y * 100 >= threshold)
                topRightGlyphHolder.setPosition(GLYPH_TOP_RIGHT_SERVO_CLOSE);
                topLeftGlyphHolder.setPosition(GLYPH_TOP_LEFT_SERVO_CLOSE);
            }
            else { //right_bumper
                topRightGlyphHolder.setPosition(GLYPH_TOP_RIGHT_SERVO_OPEN);
                topLeftGlyphHolder.setPosition(GLYPH_TOP_LEFT_SERVO_OPEN);
            }

            telemetry.addData("Status", "***** TOP GLYPH CONTROL *****");
        }




        // using JOYSTICK TRIGGERS for bottom glyph
        if (gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5) {

            telemetry.addData("Status", "BOTTOM Glyph Servo ");

            if (gamepad2.left_trigger > 0.5) { //left_stick_y * 100 >= threshold)
                bottomRightGlyphHolder.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_CLOSE);
                bottomLeftGlyphHolder.setPosition(GLYPH_BOTTOM_LEFT_SERVO_CLOSE);
            }
            else { //right_trigger
                bottomRightGlyphHolder.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_OPEN);
                bottomLeftGlyphHolder.setPosition(GLYPH_BOTTOM_LEFT_SERVO_OPEN);
            }


            telemetry.addData("Status", "***** BOTTOM GLYPH CONTROL *****");
        }


        if (gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5) {

            telemetry.addData("Status", "BOTTOM Glyph Servo ");

            if (gamepad1.left_trigger > 0.5) { //left_stick_y * 100 >= threshold)
                bottomRightGlyphHolder.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_CLOSE);
                bottomLeftGlyphHolder.setPosition(GLYPH_BOTTOM_LEFT_SERVO_CLOSE);
            }
            else { //right_trigger
                bottomRightGlyphHolder.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_OPEN);
                bottomLeftGlyphHolder.setPosition(GLYPH_BOTTOM_LEFT_SERVO_OPEN);
            }

            telemetry.addData("Status", "***** BOTTOM GLYPH CONTROL *****");
        }
/*
        // ==========================================================
        // JEWEL Pusher
        if (gamepad1.left_bumper) {
            jewelPusherArm.setPosition(0);
            telemetry.addData("Status", "***** JEWEL Pusher Left bumper *****");
        }

        if (gamepad1.right_bumper) {
            jewelPusherArm.setPosition(.5);
            telemetry.addData("Status", "***** JEWEL Pusher Right bumper *****");
        }
*/
        // ==========================================================
        // relic grabber servo



        // servo control
        if (gamepad2.dpad_up) {
            //old value = 21
            if (relicGrabber_base.getPosition()<(20.0/256.0)){
                relicGrabber_base.setPosition(relicGrabber_base.getPosition()+0.005);
            }
            //relicGrabber_base.setPosition(21.0/256);    // standing position   // 31 - back flat
            telemetry.addData("Status", "***** relicGrabber_base up *****");
        }
        if (gamepad2.dpad_down) {
            relicGrabber_base.setPosition(7.0/256.0);        // grab position
            telemetry.addData("Status", "***** relicGrabber_base down *****");
        }
        if (gamepad2.dpad_left) {   // close claw
            relicGrabber_claw.setPosition(0.0);
            telemetry.addData("Status", "***** relicGrabber_claw left / close *****");
        }
        if (gamepad2.dpad_right) {  // open claw
            relicGrabber_claw.setPosition(0.5);
            telemetry.addData("Status", "***** relicGrabber_claw right /open *****");
        }



        /*
        //===========================================================================================
        // check for diagonal movement - right button up & down. Will take preference over previous 2 cases
        if (Math.abs(gamepad1.right_stick_y)*100 >= threshold) {
                LFspeed = 0;
                RBspeed = 0; // gamepad1.right_stick_y ;
                LBspeed = gamepad1.right_stick_y ;
                RFspeed = gamepad1.right_stick_y ;

            // LBspeed = gamepad1.right_stick_y ;
                // RFspeed = gamepad1.right_stick_y ;
        }

        if (Math.abs(gamepad1.right_stick_x)*100 >= threshold) {
            LFspeed = gamepad1.right_stick_x;
            RBspeed = gamepad1.right_stick_x; // gamepad1.right_stick_y ;
            LBspeed = 0 ;
            RFspeed = 0;
        }
        */

        // In-place turning - GAMEPAD1 RIGHT stick
        if (abs(gamepad1.right_stick_x) * 100 >= threshold) {
            LFspeed = gamepad1.right_stick_x;  // Turning right - Left motors forward, right backward
            LBspeed = gamepad1.right_stick_x;
            RBspeed = -gamepad1.right_stick_x;
            RFspeed = -gamepad1.right_stick_x;
        }

        LFspeed = Range.clip(LFspeed, -1, 1);
        LBspeed = Range.clip(LBspeed, -1, 1);
        RFspeed = Range.clip(RFspeed, -1, 1);
        RBspeed = Range.clip(RBspeed, -1, 1);

        // lowe the speed
        if (lowSpeed) {
            LFspeed = LFspeed / 5;
            LBspeed = LBspeed / 5;
            RFspeed = RFspeed / 5;
            RBspeed = RBspeed / 5;
        }

        right_front_motor.setPower(RFspeed);
        left_front_motor.setPower(LFspeed);
        left_back_motor.setPower(LBspeed);
        right_back_motor.setPower(RBspeed);
        telemetry.addData("Status", "Motors LF: " + LFspeed + " RF:" + RFspeed + " LB: " + LBspeed + " RB:" + RBspeed);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


//PushbotTe ... anum.java

//        Open with Drive Notepad




