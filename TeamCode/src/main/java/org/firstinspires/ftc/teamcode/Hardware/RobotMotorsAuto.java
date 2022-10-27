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

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class RobotMotorsAuto extends RobotMotors {
    private static final double ENCODER_TICKS = 753.2;
    private static final double WHEEL_RADIUS = 2;   // Radius in inches.
    private static final int GEAR_RATIO = 2;

    private static final double IN_PER_REV = 2 * Math.PI * WHEEL_RADIUS;
    private static double TICKS_PER_REV = ENCODER_TICKS * GEAR_RATIO;
    private static double TICKS_PER_IN = TICKS_PER_REV / IN_PER_REV;

    public RobotMotorsAuto(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }

    protected static int getTicks(int inches) {
        return (int)(TICKS_PER_IN * inches);
    }

    public static int[] getDirs(MoveStyle moveStyle) {
        int[] dirs = {1, 1, 1, 1};
        if (moveStyle == MoveStyle.BACKWARD) {
            dirs[0] = -1; dirs[1] = -1; dirs[2] = -1; dirs[3] = -1;
        } else if (moveStyle == MoveStyle.LEFT) {
            dirs[1] = -1; dirs[2] = -1;
        } else if (moveStyle == MoveStyle.RIGHT) {
            dirs[0] = -1; dirs[3] = -1;
        } else if (moveStyle == MoveStyle.TURN_LEFT) {
            dirs[0] = -1; dirs[2] = -1;
        } else if (moveStyle == MoveStyle.TURN_RIGHT) {
            dirs[1] = -1; dirs[3] = -1;
        }
        return dirs;
    }

    public abstract void move(int inches, double power, int[] dir);
}
