/* Copyright (c) 2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.RobotMotorsAuto;

public class DriveTrainVel extends RobotMotorsAuto {
    private final double MAX_VELOCITY = 20;

    public DriveTrainVel(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);

        final double PIDF_F = 223 / MAX_VELOCITY; //32767
        final double PIDF_P = 0.1 * PIDF_F;
        final double PIDF_I = 0.1 * PIDF_P;
        final double PIDF_D = 0;
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PIDF_P,PIDF_I,PIDF_D,PIDF_F);
//        for (int i = 0; i < wheels.size(); i++) {
//            wheels.get(i).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        }
    }

    @Override
    public void move(int inches, double power, int[] dir) {
        if (!checkMotors()) {
            return;
        }

        int dest = getTicks(inches);

        for (int i = 0; i < wheels.size(); i++) {
            ((DcMotorEx)wheels.get(i)).setVelocity(MAX_VELOCITY * power * dir[i]);
        }

        boolean isBusy = true;
        while (isBusy) {
            for (int i = 0; i < wheels.size(); i++) {
                if (Math.abs(wheels.get(i).getCurrentPosition()) > dest) {
                    isBusy = false;
                }
            }
        }
    }

    private boolean checkMotors() {
        boolean instanceOf = true;
        for (int i = 0; i < wheels.size(); i++) {
            if (wheels.get(i) instanceof DcMotorEx) {
                instanceOf = false;
                break;
            }
        }

        return instanceOf;
    }
    public void moveForward(int inches, double power) {
        direction = MoveStyle.FORWARD;
        move(inches, power, getDirs(direction));
    }

    public void moveBackward(int inches, double power) {
        direction = MoveStyle.BACKWARD;
        move(inches, power, getDirs(direction));
    }

    public void turnLeft(int inches, double power) {
        direction = MoveStyle.TURN_LEFT;
        move(inches, power, getDirs(direction));
    }

    public void turnRight(int inches, double power) {
        direction = MoveStyle.TURN_RIGHT;
        move(inches, power, getDirs(direction));
    }

    public void strafeLeft(int inches, double power) {
        direction = MoveStyle.LEFT;
        move(inches, power, getDirs(direction));
    }

    public void strafeRight(int inches, double power) {
        direction = MoveStyle.BACKWARD;
        move(inches, power, getDirs(direction));
    }
}
