package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RedWebCam {
    OpenCvCamera camera = null;
    RedPipeline pipe = new RedPipeline();
    double maxi;

    public double getLeftVal() {
        return pipe.leftAvgFinal;
    }
    public double getMiddleVal() {
        return pipe.middleAvgFinal;
    }
    public double getRightVal() {
        return pipe.rightAvgFinal;
    }

    void init(HardwareMap hwmap) {
        WebcamName webcamName = hwmap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(pipe);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    int getAutoCase() {
        maxi=Math.max(getLeftVal(),getMiddleVal());
        maxi=Math.max(maxi,getRightVal());
        if(maxi==getLeftVal()) {
            return 1;
        }
        else if(maxi==getRightVal()) {
            return 3;
        }
        else {
            return 2;
        }
    }
}
