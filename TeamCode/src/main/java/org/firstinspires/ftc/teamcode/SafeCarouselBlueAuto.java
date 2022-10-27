package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@Autonomous(name="SafeCarouselBlueAuto",group="Autonomous")
public class SafeCarouselBlueAuto extends LinearOpMode{
    private Robot robot=new Robot();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
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




        executor.schedule(new Runnable() {
            @Override
            public void run() {
                robot.robotMotors.setMotorPower(0);
            }
        }, 5, TimeUnit.SECONDS);
        robot.robotMotors.moveForward(100,0.8);
        Thread.sleep(1000);
        robot.robotMotors.strafe(500,'r');
        Thread.sleep(1000);
        robot.carouselTurn.runBlueOnce();
        Thread.sleep(1000);
        robot.robotMotors.moveForward(500,0.8);
        Thread.sleep(1000);
        robot.robotMotors.strafe(100,'r');
        Thread.sleep(1000);
    }
}

