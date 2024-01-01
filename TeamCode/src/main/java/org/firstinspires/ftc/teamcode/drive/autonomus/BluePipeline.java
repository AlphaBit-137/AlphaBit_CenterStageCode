package org.firstinspires.ftc.teamcode.drive.autonomus;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {

    int x1start = 0;
    int y1start = 125;
    int width1 = 57;
    int height1 = 77;

    int x2start = 72;
    int y2start = 103;
    int width2 = 149;
    int height2 = 42;

    int x3start = 264;
    int y3start =113;
    int width3 =55;
    int height3 = 64;
    //y-channel 0
    //cr-channel 1 blue:100 red:180-200
    //cb-channel 2 blue:156 red:100

    Mat YCbCr = new Mat();
    Mat leftSpike;
    Mat middleSpike;
    Mat rightSpike;
    public double leftAvgFinal;
    public double middleAvgFinal;
    public double rightAvgFinal;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        //telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(x1start,y1start,width1,height1);
        Rect middleRect = new Rect(x2start,y2start,width2,height2);
        Rect rightRect = new Rect(x3start,y3start,width3,height3);

        input.copyTo(outPut);
        Imgproc.rectangle(outPut,leftRect,rectColor,1);
        Imgproc.rectangle(outPut,middleRect,rectColor,1);
        Imgproc.rectangle(outPut,rightRect,rectColor,1);

        leftSpike = YCbCr.submat(leftRect);
        middleSpike = YCbCr.submat(middleRect);
        rightSpike = YCbCr.submat(rightRect);

        Core.extractChannel(leftSpike, leftSpike, 2);
        Core.extractChannel(middleSpike, middleSpike, 2);
        Core.extractChannel(rightSpike, rightSpike, 2);

        Scalar leftAvg = Core.mean(leftSpike);
        Scalar middleAvg = Core.mean(middleSpike);
        Scalar rightAvg = Core.mean(rightSpike);

        leftAvgFinal = leftAvg.val[0];
        middleAvgFinal = middleAvg.val[0];
        rightAvgFinal = rightAvg.val[0];

        return(outPut);

    }

}
