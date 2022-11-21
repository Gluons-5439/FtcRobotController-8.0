package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@Autonomous(name="CarouselBlue Auto",group="Autonomous")


public class CarouselBlueAuto extends LinearOpMode{

    private Robot robot=new Robot();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
//        robot.robotMotors.turnOnEncoders();
        robot.tfod.activate();
        robot.tfod.setZoom(1, 16.0/9.0);
        robot.s.close();
        robot.lift.backToBase();

        boolean found=false;
        final String ELEMENT_LABEL="Duck";

        waitForStart();


        final ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);
        executor.schedule(new Runnable() {
            @Override
            public void run() {

            }
        }, 29, TimeUnit.SECONDS);

//Constants for positions of object
        final double X_LEFT=0;
        final double X_MID=530;
        final double X_RIGHT=1060;
        final double Y_DUCK=340;
        float leftPos=0;
        float topPos=0;
        char result;


        // Finding the element
        int minDetections=1; //minimum number of "frames" the robot detects the element for to ensure it is detected properly
        int detections=0;
        while (!found)
        {
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
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.update();
                        if(recognition.getLabel().equals(ELEMENT_LABEL))
                        {
                            detections++;
                            if(detections>=minDetections)
                            {
                                found=true;
                                leftPos=recognition.getLeft();
                                topPos=recognition.getTop();
                            }
                        }

                        i++;

                    }
                }
            }
        }
        telemetry.addData("Left: ",leftPos);
        telemetry.addData("Top: ", topPos);

        //Detecting which position element is at (from left and top)
        if(leftPos>=X_LEFT && leftPos<=X_LEFT+200)
        {
            result='l';
        }
        else if(leftPos>=X_LEFT && leftPos<X_RIGHT-200)
        {
            result='m';
        }
        else
        {
            result='t';
        }
        telemetry.addData("Level: ", result);
        telemetry.update();

        if (result == 'l')
        {
            robot.lift.liftLowerLevel();
        }
        else if (result == 'm')
        {
            robot.lift.liftMidLevel();
        }
        else {
            robot.lift.liftUpperLevel();
        }

        robot.robotMotors.moveForwardEn(8);
//        Thread.sleep(1000);
        robot.robotMotors.strafe(1015,'r');
        Thread.sleep(1000);
        robot.carouselTurn.runBlueOnce();
        Thread.sleep(1000);
        robot.robotMotors.moveForwardEn(30);
        Thread.sleep(1000);
        robot.robotMotors.turn(90,'l');
        Thread.sleep(1000);
        robot.robotMotors.moveForwardEn(24);
        Thread.sleep(1000);
        robot.s.open();
        Thread.sleep(1000);
        robot.robotMotors.strafe(900,'l');
        Thread.sleep(1000);
        robot.robotMotors.moveForwardEn(24);
        Thread.sleep(1000);
//        robot.lift.backToBase();
//        Thread.sleep(1000);

//        robot.robotMotors.strafe(2000,'l'); //plan has different value for speed, value is default
//        robot.carouselTurn.runOnce(); //need to make RunOnce method in carousel RedAuto Class
//        robot.robotMotors.strafe(200,'r');
        robot.lift.liftLowerLevel();






        executor.schedule(new Runnable() {
            @Override
            public void run() {
                robot.robotMotors.setMotorPower(0);
            }
        }, 5, TimeUnit.SECONDS);

    }
}
