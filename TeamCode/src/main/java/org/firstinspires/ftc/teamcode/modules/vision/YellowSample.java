package org.firstinspires.ftc.teamcode.modules.vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowSample extends OpenCvPipeline {

    private static String status = "nothing";
    private static Rect thisRect = null;
    private final Mat hsvMat = new Mat();

    public Scalar lowerYellowHSV = new Scalar(16.0, 82.0, 50.0, 0.0);
    public Scalar upperYellowHSV = new Scalar(38.0, 255.0, 255.0, 0.0);
    private static final Mat hsvBinaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    private static ArrayList<Rect> contoursRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(0.0, 0.0, 0.0, 0.0);
    public int lineThickness = 3;

    private final Mat inputRects = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, lowerYellowHSV, upperYellowHSV, hsvBinaryMat);

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

        status = "Yellow Sample Pipeline Is Running :)";
        return inputRects;
    }

    public static String getStatus() {
        return status;
    }

    public static double getDistance() {
        double maxWidth = 0;

        for (int i = 0; i < (contoursRects.toArray().length); i++) {
            thisRect = contoursRects.get(i);
            if (thisRect.width > maxWidth) {
                maxWidth = thisRect.width;
                break;
            }
        }

        double distance;
        double distanceWidth;
        double distanceHeight;
        double c720FocalLength = 1430; //logitech c720 camera
        double sampleWidthPixels = thisRect.width;
        double sampleHeightPixels = thisRect.height;
        double sampleWidthCm = 3.8;
        double sampleHeightCm = 8.9;

        distanceWidth = sampleWidthCm * c720FocalLength / sampleWidthPixels;
        distanceHeight = sampleHeightCm * c720FocalLength / sampleHeightPixels;
        distance = (distanceWidth + distanceHeight) / 2;
        return distance;
    }
}
