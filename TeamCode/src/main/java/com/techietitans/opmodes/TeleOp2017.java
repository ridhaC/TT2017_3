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
 */

@TeleOp(group="TeleOp")
//@Disabled
public class TeleOp2017 extends OpMode{

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

    public Servo leftGlyphHolder    = null;
    public Servo rightGlyphHolder   = null;

    public Servo jewelPusherArm   = null;
    public Servo relicGrabber_hand   = null;
    public Servo relicGrabber_base   = null;


    public static final double GLYPH_RIGHT_SERVO_OPEN       =  0.86 ;
    public static final double GLYPH_RIGHT_SERVO_CLOSE      =  0.647 ;
    public static final double GLYPH_LEFT_SERVO_OPEN       =  0.313 ;   // opposite values than Right Servo
    public static final double GLYPH_LEFT_SERVO_CLOSE      =  0.549 ;

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

        rightGlyphHolder = hardwareMap.servo.get("right_hand");
        leftGlyphHolder = hardwareMap.servo.get("left_hand");
//        jewelPusherArm = hardwareMap.servo.get("jewel_arm");
//        relicGrabber_hand = hardwareMap.servo.get("relicGrabber_hs");
//        relicGrabber_base = hardwareMap.servo.get("relicGrabber_bs");

        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        left_back_motor.setDirection(DcMotor.Direction.REVERSE);

       /* left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);
        */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Bueno Mecanum");    //

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

       /* right_front_motor.setPower(gamepad1.right_stick_y);
        left_front_motor.setPower(gamepad1.right_stick_y);

        left_back_motor.setPower(gamepad1.left_stick_y);
        right_back_motor.setPower(gamepad1.left_stick_y);
        */

        float LFspeed = 0;
        float LBspeed = 0;
        float RFspeed = 0;
        float RBspeed = 0;

        int threshold = 20;

        telemetry.addData("Status", "Gamepad L: " + gamepad1.left_stick_x + " " + gamepad1.left_stick_y + " R: " + gamepad1.right_stick_x + " " + gamepad1.right_stick_y);

        /*
        if (Math.abs(gamepad1.left_stick_x)*100 >= threshold || Math.abs(gamepad1.left_stick_y)*100 >= threshold) {
            LFspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;  // y up is negative in joystick
            LBspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            RFspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            RBspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        }*/

        // check for forward move
        if (abs(gamepad1.left_stick_y)*100 >= threshold) {
            LFspeed = gamepad1.left_stick_y ;  // ignore x component
            LBspeed = gamepad1.left_stick_y ;
            RFspeed = gamepad1.left_stick_y ;
            RBspeed = gamepad1.left_stick_y ;

            telemetry.addData("Status", "***** forward *****");
        }


        // check for sideways move. Will take precedence over vertical move
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
        // problem: motor was connected with wrong port (M1 vs M2) *****

        lift_motor.setPower(0);
        if (abs(gamepad2.left_stick_y)*100 >= threshold) {  // no-op

            telemetry.addData("Status", "Lift motor ");
            double motor_power;

            if (gamepad2.left_stick_y > 0)
                motor_power = -0.5;
            else
                motor_power = 0.5;

            telemetry.addData("Status", "***** Lift motor set power *****");
            lift_motor.setPower(motor_power);
        }

        // ========================================================
        // GLYPH SERVO - GAMEPAD2 LEFT STICK Y
        // right servo works with these values. Left servo needs to be reversed

        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            // if (abs(gamepad2.left_stick_y)*100 >= threshold) {

            telemetry.addData("Status", "Glyph Servo ");

            if (gamepad2.left_bumper) { //left_stick_y * 100 >= threshold)
                rightGlyphHolder.setPosition(GLYPH_RIGHT_SERVO_CLOSE);
                leftGlyphHolder.setPosition(GLYPH_LEFT_SERVO_CLOSE);
            }
            else { //right_bumper
                rightGlyphHolder.setPosition(GLYPH_RIGHT_SERVO_OPEN);
                leftGlyphHolder.setPosition(GLYPH_LEFT_SERVO_OPEN);
            }

            telemetry.addData("Status", "***** GLYPH CONTROL *****");
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

        // ==========================================================
        // relic grabber

        // motor control
        if (gamepad2.right_stick_button)
            relic_motor.setPower(0);

        if (abs(gamepad2.right_stick_y) > 0) {
            float relic_motor_power = gamepad2.right_stick_y / 4;  // slowdown motor power
            relic_motor.setPower(relic_motor_power);
            telemetry.addData("Status", "***** relicGrabber motor *****");
        }

        // servo control
        if (gamepad2.dpad_up) {
            relicGrabber_base.setPosition(0.10);
            telemetry.addData("Status", "***** relicGrabber_base up *****");
        }
        if (gamepad2.dpad_down) {
            relicGrabber_base.setPosition(0.15);
            telemetry.addData("Status", "***** relicGrabber_base down *****");
        }
        if (gamepad2.dpad_left) {
            relicGrabber_hand.setPosition(0.10);
            telemetry.addData("Status", "***** relicGrabber_hand left *****");
        }
        if (gamepad2.dpad_right) {
            relicGrabber_hand.setPosition(0.15);
            telemetry.addData("Status", "***** relicGrabber_hand right *****");
        }

*/

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

