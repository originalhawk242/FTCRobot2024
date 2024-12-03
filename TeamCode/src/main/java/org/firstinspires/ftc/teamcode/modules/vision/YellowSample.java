package org.firstinspires.ftc.teamcode.modules.vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowSample extends OpenCvPipeline {

    private static String status = "nothing";
    private static int location = 0;
    private final Mat hsvMat = new Mat();

    public Scalar lowerYellowHSV = new Scalar(16.0, 82.0, 50.0, 0.0);
    public Scalar upperYellowHSV = new Scalar(38.0, 255.0, 255.0, 0.0);
    private static final Mat hsvBinaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    private final ArrayList<Rect> contoursRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(0.0, 0.0, 0.0, 0.0);
    public int lineThickness = 3;

    private final Mat inputRects = new Mat();

    private static final Point topLeft1 = new Point(0, 0);
    private static final Point bottomRight1 = new Point(319, 479);
    private static final Point topLeft2 = new Point(320, 0);
    private static final Point bottomRight2 = new Point(639, 479);



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

    public static int getLocation() {

        double w1 = 0, w2 = 0;
        //process rectangles (255=W,0=B)
        for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
            for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                if (hsvBinaryMat.get(i, j)[0] == 255) {
                    w1++;
                }
            }
        }

        for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
            for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                if (hsvBinaryMat.get(i, j)[0] == 255) {
                    w2++;
                }
            }
        }

        //get location
        if (w1 > w2) {
            location = 1;
        } else if (w1 < w2) {
            location = 2;
        }

        return location;
    }

    public static String getStatus() {
        return status;
    }

    public static double getDistance() {
        double distance;
        double distanceWidth;
        double distanceHeight;
        double c720FocalLength = 1430; //logitech c720 camera
        double sampleWidthPixels;
        double sampleHeightPixels;
        double sampleWidthCm;
        double sampleHeightCm;

        distanceWidth = (sampleWidthCm * c720FocalLength) / sampleWidthPixels;
        distanceHeight = (sampleHeightCm * c720FocalLength) / sampleHeightPixels;
        distance = (distanceWidth + distanceHeight) / 2;
        return distance;
    }    
}
