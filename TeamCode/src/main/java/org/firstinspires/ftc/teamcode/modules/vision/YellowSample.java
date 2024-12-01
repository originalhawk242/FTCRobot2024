package org.firstinspires.ftc.teamcode.modules.vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowSample extends OpenCvPipeline {

    private String location = "nothing";
    private final Mat hsvMat = new Mat();

    public Scalar lowerHSV = new Scalar(16.0, 82.0, 50.0, 0.0);
    public Scalar upperHSV = new Scalar(38.0, 255.0, 255.0, 0.0);
    private final Mat hsvBinaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    private final ArrayList<Rect> contoursRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(0.0, 0.0, 0.0, 0.0);
    public int lineThickness = 3;

    private final Mat inputRects = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursRects.clear();
        for(MatOfPoint points : contours) {
            contoursRects.add(Imgproc.boundingRect(points));
        }

        input.copyTo(inputRects);
        for(Rect rect : contoursRects) {
            Imgproc.rectangle(inputRects, rect, lineColor, lineThickness);
        }

        return inputRects;
    }

    public String getLocation() {
        return location;
    }
}
