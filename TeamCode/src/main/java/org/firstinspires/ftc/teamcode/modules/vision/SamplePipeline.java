package org.firstinspires.ftc.teamcode.modules.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
    private String output = "nothing";

    public SamplePipeline() {

    }

    //mat
    @Override
    public Mat processFrame(Mat input) {
        output = "Sample Pipeline Is Running";
        return input;
    }

    public String getOutput() {
        return output;
    }
}
