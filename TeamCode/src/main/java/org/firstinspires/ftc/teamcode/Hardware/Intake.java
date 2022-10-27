package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake{
        public DcMotor inMotor;

        public Intake(DcMotor i)
        {
            i.setPower(0);
            i.setDirection(DcMotor.Direction.FORWARD);
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            inMotor=i;
        }

        public static double maxPower=0.8;

    public void intake() {
        inMotor.setPower(-maxPower);
    }

    public void noIntake() {
        inMotor.setPower(0);
    }

    public void reverseIntake() { inMotor.setPower(maxPower);}
}
