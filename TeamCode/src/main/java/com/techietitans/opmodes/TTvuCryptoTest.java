package com.techietitans.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.techietitans.libraries.TTCrypto;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * Created by ruhul on 2/4/16.
 */
@Autonomous(group ="TT_Test")
//@Disabled
public class TTvuCryptoTest extends OpMode {

    private TTCrypto vu ;
    private RelicRecoveryVuMark vm;


    @Override
    public void init() {
        vu = new TTCrypto(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
    }

    @Override
    public void start() {
     vu.activateView();
    }

    @Override
    public void loop() {
            vm=vu.getViewResult();

            telemetry.addData(">> ", vm);

            if(vm==RelicRecoveryVuMark.LEFT)    {

                telemetry.addLine("this is left");

            }

    }

    public void stop() {

    }


}
