
package com.techietitans.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.techietitans.libraries.DataLogger;
import com.techietitans.libraries.HardwareClass_V2;
import com.techietitans.libraries.TTAutoCommon;
import com.techietitans.libraries.TTCrypto;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

//---VueForia-Specific-Imports---

@Autonomous(group = "TechieTitans")
//@Disabled

public class TTAutoTestWIthCommon extends TTAutoCommon{

    int currentState = 0;
    int previousState = 0;
    //boolean isRunning = false;
    //boolean isResetRunning = false;
    DataLogger dl;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //Colors allianceColor = Colors.RED;
    //Colors jewelColor;
    boolean isRobotLost = false;
    boolean logEnabled = false;
    long logTime = System.currentTimeMillis();
    int loopCounter =0;
    double allianceSpecific;
    int allianceSpecificDistance;
    boolean shortVersion = false;
    boolean jewelEnabled = false;
    int moveDistance = 0;
    int offPlate = 680;

    //---VueForia-Specific-Variables---
    //TODO:Move these to another class like hardware class so that we don't have to declare in all code.
    private TTCrypto vu;
    private RelicRecoveryVuMark vm;
    int collumn = 0; //0 is left 1 is middle 2 is right default to 0 so that it defaults to left row the row with least margin for error


    /**
     * Construct the class.
     * The system calls this member when the class is instantiated.
     */
    public TTAutoTestWIthCommon() {
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
    }


    @Override
    public void init() {
        //Get initialization..mainly servos.
        super.init();

    }

    //*****************************************************************************
    @Override
    public void init_loop() {
        super.init_loop();

    }

    //*****************************************************************************
    @Override
    public void start() {

        currentState = 0;
        //TODO: Change for 2017 data points
        if (logEnabled) {
            //Set a new data logger and header of the file
//            dl = new DataLogger("Dl_TT_Auto_V4");
//            dl.addField("LoopCounter");
//            dl.newLine();
        }
        super.loop();
    }

    //*****************************************************************************
    @Override
    public void loop() {

        //*********Start State Machine
        switch (currentState) {
            //Tasks are broken down to finite STATES. We will increment to to next state after successful
            // completion of each state.

            case 0:
                //First state

                if (CommoncurrentState>6) {
                    currentState++;
                }
                else {
                    super.loop();
                }

                break;

            //************END of Jewel Push
            case 1:
                // Come down and move towards glyph drop zone
                //TODO: Adjust alliance specific parameters

                allianceSpecific = (allianceColor == Colors.RED) ? -0.15 : 0.15 ;
                allianceSpecificDistance = (allianceColor == Colors.RED) ? 680 : 680;

                if (collumn == 0)   {
                    offPlate = 690;
                    moveDistance = 150;
                }
                else if(collumn == 1)   {
                    offPlate = 740;
                    moveDistance = 220;
                }
                else {
                    offPlate = 760;
                    moveDistance = 260;
                }

                if (driveWithEncodersV2(allianceSpecific, allianceSpecific, offPlate, offPlate,5000)) {
                    //if (runtime.milliseconds() > 7000) {
                    currentState++;
                    runtime.reset();
                    //}
                }
                break;


            case 2:
                if (runtime.milliseconds() > 1000) {
                    currentState++;
                    runtime.reset();
                }
                break;


            case 3:
                if (collumn == 2)   {

                    if (gyroPointTurnV2(.15, Sides.LEFT, 180 - 57,5000)) {
                        runtime.reset();
                        currentState++;
                    }
                }

                else if (collumn == 1)  {

                    if (gyroPointTurnV2(.25, Sides.LEFT, 180 - 50,5000)) {
                        runtime.reset();
                        currentState++;
                    }

                    //was 650

                }
                // Column 0 - it will come here from DEFAULT case
                else {

                    if (gyroPointTurnV2(.25, Sides.LEFT, 180 - 25,5000)) {
                        runtime.reset();
                        currentState++;
                    }

                    //was 450
                }

                break;

            case 4:
                // Move front to the drop zone -First move
                telemetry.addData("**** RUN TIME ****  ", runtime.milliseconds());

                if ((driveWithEncodersV2(0.15, 0.15, moveDistance, moveDistance,1000))) {
                    runtime.reset();
                    if (collumn>1) //Need an additional step..so separate
                        currentState++;
                    else currentState =7;
                }

                break;
            //** Only for Column 3 - Start
            case 5:
                // Undo angle
                if (gyroPointTurnV2(.2, Sides.LEFT, 23,3000)) {
                    currentState++;
                    runtime.reset();
                }
                break;

            case 6:
                // Move front to the drop zone - 2nd move
                //left count right count was 150
                if ((driveWithEncodersV2(0.1, 0.1, 60, 60,2000))) {
                    stopMotors();
                    runtime.reset();
                    currentState++;
                }

                break;
            //** Only for Column 3 - End
            case 7:
                // Lower glyph -- Not sure if we need it
                // Lift the glyph to mid height
                lift_motor.setPower(-0.3);
                if (runtime.milliseconds()>195){
                    lift_motor.setPower(0.0);
                    currentState++;
                    runtime.reset();
                    //left count right count was 150
                }

                break;


            case 8:
                // Release glyph
                bottom_right_hand.setPosition(GLYPH_BOTTOM_RIGHT_SERVO_OPEN);
                bottom_left_hand.setPosition(GLYPH_BOTTOM_LEFT_SERVO_OPEN);
                currentState ++;
                runtime.reset();
                break;


            case 9:
                // come back
                //left count right count was 100
                if (driveWithEncodersV2(-0.3, -0.3, 100, 100,2000)) {
                    stopMotors();
                    currentState++;
                    runtime.reset();
                }

                break;

            case 10:
                // push
                //left count right count was 300
                if (driveWithEncodersV2(0.3, 0.3, 110, 110,2000)) {
                    stopMotors();
                    currentState++;
                    runtime.reset();
                }
                break;

            case 11:
                // come back ...final step
                //left count right count was 100
                if (driveWithEncodersV2(-0.3, -0.3, 100, 100,2000)) {
                    stopMotors();
                    currentState++;
                    runtime.reset();
                }

                break;


            case 99:
                // Recovery State. Any known failures will lead the state machine to this state.
                // Display in telemetry and log to the file
                left_front_motor.setPower(0);
                right_front_motor.setPower(0);
                telemetry.addData("**** ROBOT STOPPED at State: ****", previousState);
                break;

            default:
                // The autonomous actions have been accomplished (i.e. the state machine has
                // transitioned into its final state.
                break;
        }

        previousState = currentState;
        loopCounter++;

        telemetry.addData("state: ", currentState);
        telemetry.addData("Jewel : ", jewelColor);
        telemetry.addData("Red : ",Color_jewel.red());
        telemetry.addData("Blue : ",Color_jewel.blue());
        telemetry.addData("Alliance : ",allianceColor);
        telemetry.addData("Placing in ",collumn);


        // Write data to log file..if enabled and log duration has reached
        //TODO: Change for 2017 data points
        if ((logEnabled) && ((System.currentTimeMillis()- logTime)>100)){
//            dl.addField(String.valueOf(loopCounter));
//            dl.newLine();
            //Reset counter
            logTime = System.currentTimeMillis();
        }
    }

    @Override
    public void stop() {
        //Close data logger and Adafruit
        if (logEnabled){
            dl.closeDataLogger();
        }

    }

     /*
     * ************** Helper Methods*************************
     */


}

