package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class RedPipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat centerCrop;
    double leftavgfin;
    double rightavgfin;
    double centeravgfin;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

    public enum Position {
        LEFT, RIGHT, CENTER,
    }

    public Position position = Position.LEFT;

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        //telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(1,1,213,359);
        Rect centerRect = new Rect(213,1,213,359);
        Rect rightRect = new Rect(426,1,213,359);

        input.copyTo(outPut);
        Scalar redColor = new Scalar(255.0, 0.0, 0.0);
        Scalar greenColor = new Scalar(0.0, 255.0, 0.0);

        Imgproc.rectangle(outPut, leftRect, position == Position.LEFT ? greenColor : redColor, 2);
        Imgproc.rectangle(outPut, rightRect, position == Position.RIGHT ? greenColor : redColor, 2);
        Imgproc.rectangle(outPut, centerRect, position == Position.CENTER ? greenColor : redColor, 2);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);
        centerCrop = YCbCr.submat(centerRect);

        //change coi to 2 for scanning blue element
        Core.extractChannel(leftCrop,leftCrop,1);
        Core.extractChannel(rightCrop,rightCrop,1);
        Core.extractChannel(centerCrop,centerCrop,1);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);
        Scalar centeravg = Core.mean(centerCrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];
        centeravgfin = centeravg.val[0];

        double max = Math.max(leftavgfin, Math.max(centeravgfin, rightavgfin));

        if (max == leftavgfin) {
            position = Position.LEFT;
        } else if (max == centeravgfin) {
            position = Position.CENTER;
        } else if (max == rightavgfin) {
            position = Position.RIGHT;
        }

        /*if ((leftavgfin > rightavgfin)&&(leftavgfin > centeravgfin)){
            telemetry.addLine("left");
        }
        else if ((rightavgfin > leftavgfin)&&(rightavgfin > centeravgfin)){
            telemetry.addLine("right");
        }

        else if ((centeravgfin > rightavgfin)&&(centeravgfin > leftavgfin)){
            telemetry.addLine("center");
        }*/


        return(outPut);
    }
}