package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.Flywheel;

import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@Autonomous(name="SafeCarouselRedAuto",group="Autonomous")
public class SafeCarouselRedAuto extends LinearOpMode{
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

        robot.robotMotors.strafe(200,'l');
        Thread.sleep(1000);
        robot.robotMotors.moveForward(300,-0.8);
        Thread.sleep(1000);
        robot.carouselTurn.runRedOnce();
        Thread.sleep(1000);
        robot.robotMotors.moveForward(200,0.8);
        Thread.sleep(1000);
        robot.robotMotors.turn(80,'l');
        Thread.sleep(1000);
        robot.robotMotors.moveForward(500,0.8);


        executor.schedule(new Runnable() {
            @Override
            public void run() {
                robot.robotMotors.setMotorPower(0);
            }
        }, 5, TimeUnit.SECONDS);

    }
}
