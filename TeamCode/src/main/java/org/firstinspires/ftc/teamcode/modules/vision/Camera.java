package org.firstinspires.ftc.teamcode.modules.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.core.Module;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera extends Module {
    private final OpenCvWebcam webcam;
    private final HardwareMap hardwareMap;
    private final SamplePipeline p1;
    private final TagPipeline p2;
    private boolean cameraInit = false;

    public Camera(OpMode registrar) {
        super(registrar);

        p1 = new SamplePipeline();
        p2 = new TagPipeline();

        this.hardwareMap = registrar.hardwareMap;

        int cameraMonitorViewId =
            hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        webcam.setPipeline(p1);

        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        cameraInit = true;
                    }

                    @Override
                    public void onError(int errorCode) {
                        RobotLog.e("Cannot access camera - error code: %u", errorCode);
                        cameraInit = false;
                    }
                }
        );
    }

    public void switchToSecondPipeline(){
        webcam.setPipeline(p2);
    }

    public void switchToFirstPipeline(){
        webcam.setPipeline(p1);
    }


    @Override
    public void ensureSafety() {
        //camera, no output to ensure the safety of
    }

    @Override
    public boolean isConnected() {
        return cameraInit;
    }

    @Override
    public void log() {
        //nothing to log yet
    }
}
