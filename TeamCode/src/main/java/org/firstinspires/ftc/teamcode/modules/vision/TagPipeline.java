package org.firstinspires.ftc.teamcode.modules.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class TagPipeline extends OpenCvPipeline {
    private String output = "nothing";

    public TagPipeline() {

    }

    //mat
    @Override
    public Mat processFrame(Mat input) {
        output = "Tag Pipeline Is Running";
        return input;
    }

    public String getOutput() {
        return output;
    }
}
