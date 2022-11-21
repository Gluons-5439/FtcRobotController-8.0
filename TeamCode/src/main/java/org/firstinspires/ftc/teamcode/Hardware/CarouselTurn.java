package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CarouselTurn {
    public DcMotor turnMotor;

    public CarouselTurn(DcMotor t)
    {
        t.setPower(0);
        t.setDirection(DcMotor.Direction.FORWARD);
//        t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        turnMotor=t;
    }

    private final int FULL_ROTATION=1000; //time for the carousel to spin once completely. subject to change.
//    public static double redPower=-0.4;
//    public static double bluePower=0.8;

    public static double maxPower=0.35;

    public void startBlueTurn()
    {
//        turnMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turnMotor.setPower(maxPower);
    }

    public void stopTurn() { turnMotor.setPower(0); }

    public void startRedTurn()
    {
//        turnMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turnMotor.setPower(-maxPower);
    }

    public void runBlueOnce() throws InterruptedException
    {
        startBlueTurn();
        Thread.sleep(FULL_ROTATION);
        stopTurn();
    }

    public void runRedOnce() throws InterruptedException
    {
        startRedTurn();
        Thread.sleep(FULL_ROTATION);
        stopTurn();
    }

}
