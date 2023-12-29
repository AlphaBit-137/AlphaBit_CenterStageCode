package org.firstinspires.ftc.teamcode.drive.autonomus;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SimplePipeline extends OpenCvPipeline {

    int x1start = 90;
    int y1start = 80;
    int width1 = 132;
    int height1 = 116;

    int x2start = 1;
    int y2start = 1;
    int width2 = 1;
    int height2 = 1;

    int x3start = 1;
    int y3start = 1;
    int width3 = 1;
    int height3 = 1;
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
        telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(x1start,y1start,width1,height1);
        Rect middleRect = new Rect(x2start,y2start,width2,height2);
        Rect rightRect = new Rect(x3start,y3start,width3,height3);

        input.copyTo(outPut);
        Imgproc.rectangle(outPut,leftRect,rectColor,2);
        Imgproc.rectangle(outPut,middleRect,rectColor,2);
        Imgproc.rectangle(outPut,rightRect,rectColor,2);

        leftSpike = YCbCr.submat(leftRect);
        middleSpike = YCbCr.submat(middleRect);
        rightSpike = YCbCr.submat(rightRect);

        Core.extractChannel(leftSpike, leftSpike, 0);
        Core.extractChannel(middleSpike, middleSpike, 0);
        Core.extractChannel(rightSpike, rightSpike, 0);

        Scalar leftAvg = Core.mean(leftSpike);
        Scalar middleAvg = Core.mean(middleSpike);
        Scalar rightAvg = Core.mean(rightSpike);

        leftAvgFinal = leftAvg.val[0];
        middleAvgFinal = middleAvg.val[0];
        rightAvgFinal = rightAvg.val[0];

        telemetry.addData("leftAvg", leftAvgFinal);
        telemetry.addData("middleAvg", middleAvgFinal);
        telemetry.addData("rightAvg", rightAvgFinal);

        return(outPut);

    }

}
