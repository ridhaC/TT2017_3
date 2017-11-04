package com.techietitans.libraries;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by ruhul on 2/4/16.
 */
public class TTCrypto {


    // --------------------------------- CLASS VARIABLES -------------------------------------------
    public static final String TAG = "Vuforia VuMark";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;


    public TTCrypto(int cameraMonitorViewId) {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUocCzn/////AAAAGWdQoCuqAUJqsJjVazzQK59YgDcNBmOSosm/6OAfyQUXOpCG/Q6Dokb5yzlVe9xLuLfCYe1z2P0/zs0sYRSktLWVLFNqpWkO5zZ/N+l26vc/b6TnolSH8rj66x1iEU4w80bS2ps8jDeJegduT2vPlDsQ5w0faLqfxhlSKxSihvpcJoRb504cVUj96lOEwV8f2Y+pKBfQCm3KbDSHCXm2JAcM/InU+03WaDi0gz6R/eLBFi/O9/COB2rnYQIOMElbzpkBPmHQK/Dj8hdMpmxhAK21ZmIrHv4Ms2mQmJWOTX01JCdCM6VBWrkMmI5R+kreFBEkHGWp6nlYZo/AIIi407OOqUl8OAzV/Q9kQwJ1xWhj";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }
    public void activateView() {

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark getViewResult() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
    public void close() {

    }

}