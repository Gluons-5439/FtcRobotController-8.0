package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {


    private Servo claw;

    private final double CLAW_OPEN = 0.68; //numbers subject to change
    private final double CLAW_CLOSE = 1; //numbers subject to change


    public Servos(Servo c)
    {
        //servo hardware moments
//        r.setDirection(Servo.Direction.FORWARD);
        c.setDirection(Servo.Direction.FORWARD);
        claw = c;
//        b.setDirection(Servo.Direction.FORWARD);
//        boxDrop=b;

    }

    public void open() {claw.setPosition(CLAW_OPEN); }
//
    public void close() {claw.setPosition(CLAW_CLOSE);}

    public double getClawPosition()
    {
        return claw.getPosition();
    }


//    public void setBoxPosition(Lift lift)
//    {
//        boxOpen=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION-0.667;
//        boxClose=lift.liftMotor.getCurrentPosition()/lift.TICKS_PER_REVOLUTION*boxDrop.MAX_POSITION;
//    }
//
//    public double getBoxPosition()
//    {
//        return boxDrop.getPosition();
//    }
}
