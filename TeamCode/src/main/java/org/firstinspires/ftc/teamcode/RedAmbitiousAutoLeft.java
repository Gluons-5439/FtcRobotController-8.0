package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@Autonomous(name="RedSideCMYAuto",group="Autonomous")
public class RedAmbitiousAutoLeft extends LinearOpMode{
    private Robot robot=new Robot();

    public void runOpMode() throws InterruptedException {

        final String[] LABELS = {
               "Cyan",
                "Magenta",
                "Yellow"
        };
        robot.init(hardwareMap);
        robot.tfod.loadModelFromAsset("CMYSignalSleeve.tflite", LABELS);

        robot.tfod.activate();
        robot.tfod.setZoom(1, 16.0/9.0);

        boolean found=false;
        boolean failsafe=false;

        waitForStart();



        final ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);
        executor.schedule(new Runnable() {
            @Override
            public void run() {

            }
        }, 29, TimeUnit.SECONDS);





        int minDetections=1; //minimum number of "frames" the robot detects the element for to ensure it is detected properly
        int detections=0;
        String label="";
        long startTime=System.currentTimeMillis();
        while (!found) {
            telemetry.addData("found=", found);
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.update();
                        for (int j = 0; j < 3; j++) {
                            if (recognition.getLabel().equals(LABELS[j])) {
                                detections++;
                                label = recognition.getLabel();
                                if (detections >= minDetections) {
                                    found = true;
                                }
                            }
                        }

                        i++;

                    }
                }
            }
            long elapsedTime=System.currentTimeMillis();
            long num=elapsedTime-startTime;
            if(num>2000) {
                failsafe=true;
                break;
            }
        }
        robot.robotMotors.moveForward(1000,0.8);
        robot.robotMotors.turn(90,'r');
        robot.lift.liftMidLevel();
        robot.s.open();

        if(label.equals(LABELS[0])) {
            Thread.sleep(500);
        }
        else if(label.equals(LABELS[1])) {
            Thread.sleep(500);
            robot.robotMotors.moveForward(1000, 0.8);
        }
        else if(label.equals(LABELS[2])) {
            robot.robotMotors.strafe(800, 'r');
            Thread.sleep(500);
            robot.robotMotors.moveForward(1000, 0.8);
        }
        executor.schedule(new Runnable() {
            @Override
            public void run() {
                robot.robotMotors.setMotorPower(0);
            }
        }, 5, TimeUnit.SECONDS);
    }
}

