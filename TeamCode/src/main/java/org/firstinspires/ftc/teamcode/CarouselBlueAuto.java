package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        robot.lift.liftLowerLevel();
        Thread.sleep(1000);
        robot.robotMotors.moveForward(175, 0.8);
        Thread.sleep(1000);
        robot.robotMotors.strafe(380, 'r');
        Thread.sleep(1000);
        robot.carouselTurn.runBlueOnce();
        Thread.sleep(1000);
        robot.robotMotors.moveForward(525, 0.8);
        Thread.sleep(1000);
        robot.robotMotors.strafe(300, 'r');
        robot.lift.backToBase();
        Thread.sleep(1000);

//        robot.robotMotors.strafe(2000,'l'); //plan has different value for speed, value is default
//        robot.carouselTurn.runOnce(); //need to make RunOnce method in carousel RedAuto Class
//        robot.robotMotors.strafe(200,'r');



        executor.schedule(new Runnable() {
            @Override
            public void run() {
                robot.robotMotors.setMotorPower(0);
            }
        }, 5, TimeUnit.SECONDS);

    }
}
