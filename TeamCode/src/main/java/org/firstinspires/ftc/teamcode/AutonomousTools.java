package org.firstinspires.ftc.teamcode;

//import android.content.Context;
//import android.support.annotation.NonNull;
//import android.support.annotation.Nullable;

public class AutonomousTools {
//    private final double WHEEL_RADIUS = 2;  // Radius in inches
//    private final double AUTO_POWER = 0.1;
//    private final int GEAR_RATIO = 2;
//    private final double TICKS_PER_REV = 145.6 * GEAR_RATIO;
//    private final double IN_PER_REV = 2 * Math.PI * WHEEL_RADIUS;
//    private final double TICKS_PER_IN = TICKS_PER_REV / IN_PER_REV;
//
//    static final String TFOD_MODEL_ASSET = "Skystone.tflite";
//    static final String LABEL_STONE = "Stone";
//    static final String LABEL_SKYSTONE = "Skystone";
//
//
//
//
//    Orientation lastAngles = new Orientation();
//    double globalAngle, correction;
//
//    VuforiaLocalizer vuforia;
//    TFObjectDetector tfod;
//
//    public AutonomousTools() {
//
//    }
//
// NOTE ===========================================================
// Old encoder stuff!
// ================================================================
//
//
//
//    /**
//     * Public method to move the robot using encoder position.
//     *
//     * @param inches The distance to travel in inches.
//     * @param h The Hardware object that holds the motors.
//     * @param moveStyle An enum determining the style of movement.
//     */
//    public void move(MoveStyle moveStyle, int inches, Hardware h) {
//        int ticks = (int)(TICKS_PER_REV / IN_PER_REV * inches);
//
//        moveDistInTicks(moveStyle, ticks, h);
//    }
//
//    /**
//     * Move forward a specified distance.
//     *
//     * @param inches The distance to move.
//     * @param h The Hardware class with the motors.
//     */
//    void moveStraightForward(int inches, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = getTicks(inches);
//
//        moveDistInTicks('f',ticks, h, telemetry);
//    }
//
//    /**
//     * Move backwards a specified distance.
//     *
//     * @param inches The distance to move.
//     * @param h The Hardware class with the motors.
//     */
//    void moveStraightBack(int inches, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = getTicks(inches);
//
//        moveDistInTicks('b', ticks, h, telemetry);
//    }
//
//    /**
//     * Strafe right a specified distance.
//     *
//     * @param inches The distance to move.
//     * @param h The Hardware class with the motors.
//     */
//    void moveStrafeRight(int inches, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = getTicks(inches);
//
//        moveDistInTicks(MoveStyle.STRAFE_RIGHT, ticks, h, telemetry);
//    }
//
//    /**
//     * Strafe left a specified distance.
//     *
//     * @param inches The distance to move.
//     * @param h The Hardware class with the motors.
//     */
//    void moveStrafeLeft(int inches, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = getTicks(inches);
//
//        moveDistInTicks(MoveStyle.STRAFE_LEFT, ticks, h, telemetry);
//    }
//
//    /**
//     * Point turn to the right.
//     *
//     * @param degrees The angle to turn relative to the robot, in degrees.
//     * @param h The Hardware class with the motors.
//     */
//    void moveTurnRight(int degrees, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = (int)(TICKS_PER_REV / IN_PER_REV * WHEEL_RADIUS / GEAR_RATIO * degrees * Math.PI / 180);
//
//        moveDistInTicks(MoveStyle.POINT_TURN_RIGHT, ticks, h, telemetry);
//    }
//
//    /**
//     * Point turn to the left.
//     *
//     * @param degrees The angle to turn relative to the robot, in degrees.
//     * @param h The Hardware class with the motors.
//     */
//    void moveTurnLeft(int degrees, @NonNull Hardware h, Telemetry telemetry) {
//        int ticks = (int)(TICKS_PER_REV / IN_PER_REV * WHEEL_RADIUS / GEAR_RATIO * degrees * Math.PI / 180);
//
//        moveDistInTicks(MoveStyle.POINT_TURN_LEFT, ticks, h, telemetry);
//    }
//
//    /**
//     * Start the motors to move forwards.
//     *
//     * @param speed The power of the motors.
//     * @param h The Hardware class with the motors.
//     * @param isReverse Whether the robot should instead move backwards.
//     */
//    void startMoveStraight(double speed, @NonNull Hardware h, boolean isReverse) {
//        if (isReverse) {
//            startMotors(speed, MoveStyle.STRAIGHT_BACK, h);
//        } else {
//            startMotors(speed, MoveStyle.STRAIGHT_FORWARD, h);
//        }
//    }
//
//    /**
//     * Start the motors to strafe to the right.
//     *
//     * @param speed The power of the motors.
//     * @param h The Hardware class with the motors.
//     * @param isReverse Whether the robot should instead strafe to the left.
//     */
//    void startMoveStrafe(double speed, @NonNull Hardware h, boolean isReverse) {
//        if (isReverse) {
//            startMotors(speed, MoveStyle.STRAFE_RIGHT, h);
//        } else {
//            startMotors(speed, MoveStyle.STRAFE_LEFT, h);
//        }
//    }
//
//    /**
//     * Start the motors to turn on point to the right.
//     *
//     * @param h The Hardware class with the motors.
//     * @param speed The power of the motors.
//     * @param isReverse Whether the robot should instead turn to the left.
//     */
//    void startMoveTurn(double speed, @NonNull Hardware h, boolean isReverse) {
//        if (isReverse) {
//            startMotors(speed, MoveStyle.POINT_TURN_RIGHT, h);
//        } else {
//            startMotors(speed, MoveStyle.POINT_TURN_LEFT, h);
//        }
//    }
//
//    /**
//     * Stops the motors.
//     *
//     * @param h The Hardware class with the motors.
//     */
//    void resetMotors(@NonNull Hardware h) {
//        for (int i = 0; i < h.wheels.size(); i ++ ) {
//            h.wheels.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            h.wheels.get(i).setPower(0);
//        }
//    }
//
//    public int getTicks(int inches) {
//        return (int)(TICKS_PER_IN * inches);
//
//    }
//    //Distance / Wheel Diameter * PI * Encoder Counts Per Rotation = Encoder Counts Per Distance
//
//    public double checkDirection(Hardware h)
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle(h);
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
//
//        return correction;
//    }
//    public void resetAngle(Hardware h)
//    {
//        lastAngles = h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    public double getAngle(Hardware h)
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = h.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//    public void turnDegrees(int degrees, char dir, double power, Hardware h) throws InterruptedException
//    {
//        h.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        h.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        h.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        h.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // restart imu movement tracking.
//        resetAngle(h);
//        int leftPower = 0 , rightPower = 0;
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        if (dir == 'r')
//        {   // turn right.
//            leftPower = 1;
//            rightPower = -1;
//            degrees = -degrees;
//        }
//        else if (dir == 'l')
//        {   // turn left.
//            leftPower = -1;
//            rightPower = 1;
//        }
//
//
//        // set power to rotate.
//        h.frontLeft.setPower(leftPower * power);
//        h.backLeft.setPower(leftPower * power);
//        h.frontRight.setPower(rightPower * power);
//        h.backRight.setPower(rightPower * power);
//
//        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (getAngle(h) == 0);
//
//            while (getAngle(h) > degrees);
//        }
//        else    // left turn.
//            while (getAngle(h) < degrees);
//
//        // turn the motors off.
//        h.frontRight.setPower(0);
//        h.frontLeft.setPower(0);
//        h.backRight.setPower(0);
//        h.backLeft.setPower(0);
//
//
//        // wait for rotation to stop.
//        Thread.sleep(1000);
//
//        // reset angle tracking on new heading.
//        resetAngle(h);
//        h.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        h.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        h.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        h.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    private ArrayList<Integer> getDirs(@NonNull MoveStyle moveStyle) {
//        ArrayList<Integer> motorDirs = new ArrayList<>();
//
//        switch (moveStyle) {
//            case STRAIGHT_FORWARD:
//                motorDirs.add(1); motorDirs.add(1); motorDirs.add(1); motorDirs.add(1);
//                break;
//            case STRAIGHT_BACK:
//                motorDirs.add(-1); motorDirs.add(-1); motorDirs.add(-1); motorDirs.add(-1);
//                break;
//            case STRAFE_RIGHT:
//                motorDirs.add(-1); motorDirs.add(1); motorDirs.add(1); motorDirs.add(-1);
//                break;
//            case STRAFE_LEFT:
//                motorDirs.add(1); motorDirs.add(-1); motorDirs.add(-1); motorDirs.add(1);
//                break;
//            case POINT_TURN_RIGHT:
//                motorDirs.add(-1); motorDirs.add(1); motorDirs.add(-1); motorDirs.add(1);
//                break;
//            case POINT_TURN_LEFT:
//                motorDirs.add(1); motorDirs.add(-1); motorDirs.add(1); motorDirs.add(-1);
//                break;
//        }
//
//        return motorDirs;
//        }
//
//    private void startMotors(double speed, @NonNull MoveStyle moveStyle, @NonNull Hardware h) {
//        ArrayList<Integer> motorDirs = getDirs(moveStyle);
//
//        for (int i = 0; i < h.wheels.size(); i++) {
//            h.wheels.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            h.wheels.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            h.wheels.get(i).setPower(motorDirs.get(i) * speed);
//        }
//    }
//
//    private void moveDistInTicks(char dir, int ticks, @NonNull Hardware h, Telemetry telemetry) {
//        for (int i = 0; i < h.wheels.size(); i ++ ) {
//            h.wheels.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            h.wheels.get(i).setTargetPosition((dir == 'f' ? 1 : -1) * ticks);
//            h.wheels.get(i).setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            h.wheels.get(i).setPower((dir == 'f' ? 1 : -1) * AUTO_POWER);
//        }
//
//        while (h.frontLeft.isBusy() && h.frontRight.isBusy() && h.backLeft.isBusy() && h.backRight.isBusy()) {
//            for (int j = 0; j < h.wheels.size(); j ++ ) {
//                telemetry.addData("Motor " + j + " power: ", h.wheels.get(j).getPower());
//                telemetry.addData("Motor " + j + " position: ", h.wheels.get(j).getCurrentPosition());
//                telemetry.addData("Motor " + j + " dest. actual: ", h.wheels.get(j).getTargetPosition());
//            }
//            telemetry.update();
//        }
//
//        resetMotors(h);
//    }
//
//    // NOTE ===========================================================
//    // If we are to use encoders, we might be able to change these movement methods to be better
//    // ================================================================
//    // Update: we made the movement methods better
//
//    /**
//     * Sets the motors to not use Encoders.
//     *
//     * @param h The Hardware class with the motors.
//     *
//     */
//
//    private void setToNoEncoder(@NonNull Hardware h) {
//        for (int i = 0; i < h.wheels.size(); i ++ ) {
//            h.wheels.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//    }
//
//    /**
//     * Sets the power of the motors.
//     *
//     * @param speed The speed of the motors.
//     * @param fl Multiplier for the front-left motor.
//     * @param fr Multiplier for the front-right motor.
//     * @param bl Multiplier for the back-left motor.
//     * @param br Multiplier for the back-right motor.
//     * @param h The Hardware class with the motors.
//     *
//     */
//
//    public void setMotorPower(double speed, int fl, int fr, int bl, int br, Hardware h) {
//       // setToNoEncoder(h);
//
//        h.frontLeft.setPower(speed * fl);
//        h.frontRight.setPower(speed * fr);
//        h.backLeft.setPower(speed * bl);
//        h.backRight.setPower(speed * br);
//    }
//
//    /**
//     * Sets the power of the motors.
//     *
//     * @param speed The speed of the motors.
//     * @param h The Hardware class with the motors.
//     *
//     */
//
//    public void setMotorPower(double speed, Hardware h) {
//        //setToNoEncoder(h);
//
//        for (int i = 0; i < h.wheels.size(); i ++ ) {
//            h.wheels.get(i).setPower(speed);
//        }
//    }
//
//    /**
//     * Sets the power of the motors.
//     *
//     * @param h The Hardware class with the motors.
//     *
//     */
//
//   // public void setMotorPower(@NonNull Hardware h) {
//    //    for (int i = 0; i < h.wheels.size(); i ++ ) {
//   //         h.wheels.get(i).setPower(0);
//     //   }
//   // }
//
////    public void setMotorPower(int ticks, double speed, Hardware h) throws InterruptedException {
////        //setToNoEncoder(h);
////
////        setMotorPower(speed, h);
////        Thread.sleep(ticks);
////        setMotorPower(h);
////    }
//
//    /**
//     * Activates the robot motors for a period of time.
//     *
//     * @param moveTime The time in milliseconds to move.
//     * @param speed The speed of the motors.
//     * @param hulk The Hardware class with the motors.
//     *
//     */
//
//    public void moveForward(int moveTime, double speed, Hardware hulk) throws InterruptedException {
//        /*
//        HOW TO USE:
//        MAXSPEED   67.5 in/sec
//        DISTANCE TRAVELED = MAXSPEED * (moveTime / 1000) * speed
//         */
//        hulk.frontRight.setPower(speed);
//        hulk.frontLeft.setPower(speed);
//        hulk.backRight.setPower(speed);
//        hulk.backLeft.setPower(speed);
//        Thread.sleep(moveTime);
//        hulk.frontRight.setPower(0);
//        hulk.frontLeft.setPower(0);
//        hulk.backRight.setPower(0);
//        hulk.backLeft.setPower(0);
//    }
//
//    /**
//     * Activates the motors in a way that the robot will turn.
//     *
//     * @param degree The degree to turn the robot.
//     * @param dir The direction the robot should turn in.
//     * @param hulk The Hardware class with the motors.
//     *
//     *
//     */
//
//    public void turn(int degree, char dir, Hardware hulk) throws InterruptedException {
//        /*
//        HOW TO USE:
//        Enter degree, direction, and type "hulk"
//         */
//        if (dir == 'r') {
//            setMotorPower(0.7, 1, -1, 1, -1, hulk);
//        }
//        else if (dir == 'l') {
//            setMotorPower(0.7, -1, 1, -1, 1, hulk);
//        }
//        Thread.sleep((int)(550 * degree / 90));
//        setMotorPower(0, hulk);
//    }
//
//    /**
//     * Activates the motors in a way that the robot will strafe.
//     *
//     * @param time The time to activate the robot's motors.
//     * @param dir The direction to strafe in.
//     * @param hulk The Hardware class with the motors.
//     *
//     */
//
//    public void strafe(int time, char dir, Hardware hulk) throws InterruptedException {
//
//        if (dir == 'l') {
//
//            hulk.frontRight.setPower(-.8);
//            hulk.frontLeft.setPower(.8);
//            hulk.backRight.setPower(.8);
//            hulk.backLeft.setPower(-.8);
//        }
//        else if (dir == 'r'){
//
//            hulk.frontRight.setPower(.8);
//            hulk.frontLeft.setPower(-.8);
//            hulk.backRight.setPower(-.8);
//            hulk.backLeft.setPower(.8);
//        }
//        Thread.sleep(time);
//        hulk.frontRight.setPower(0);
//        hulk.frontLeft.setPower(0);
//        hulk.backRight.setPower(0);
//        hulk.backLeft.setPower(0);
//    }
//
//    public void initSensors(HardwareMap h) {
//        initVuforia();
//        initTfod(h);
//    }
//
//    private void initVuforia() {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//
//        parameters.camera = new Camera() {
//            @NonNull
//            @Override
//            public CameraName getCameraName() {
//                CameraName sad = new CameraName() {
//                    @Override
//                    public boolean isWebcam() {
//                        return true;
//                    }
//
//                    @Override
//                    public boolean isCameraDirection() {
//                        return false;
//                    }
//
//                    @Override
//                    public boolean isSwitchable() {
//                        return false;
//                    }
//
//                    @Override
//                    public boolean isUnknown() {
//                        return false;
//                    }
//
//                    @Override
//                    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {
//
//                    }
//
//                    @Override
//                    public boolean requestCameraPermission(Deadline deadline) {
//                        return false;
//                    }
//
//                    @Override
//                    public CameraCharacteristics getCameraCharacteristics() {
//                        return null;
//                    }
//                };
//                return sad;
//            }
//
//            @Override
//            public CameraCaptureRequest createCaptureRequest(int androidFormat, Size size, int fps) throws CameraException {
//                return null;
//            }
//
//            @NonNull
//            @Override
//            public CameraCaptureSession createCaptureSession(Continuation<? extends CameraCaptureSession.StateCallback> continuation) throws CameraException {
//                return null;
//            }
//
//            @Override
//            public void close() {
//
//            }
//
//            @Override
//            public Camera dup() {
//                return null;
//            }
//
//            @Nullable
//            @Override
//            public <T extends CameraControl> T getControl(Class<T> controlType) {
//                return null;
//            }
//        };
//        parameters.vuforiaLicenseKey = "AfmBbcz/////AAAAGbLGg++zzk4MiOrcPTc3t9xQj3QHfISJprebOgt5JJ4+83xtFO+ApGlI3GVY/aMgCpoGEIzaJse9sXiYDiLYpJQlGDX765tWJUrqM+pzqLxVXjWA1J6c968/YqYq74Vq5emNxGHj5SF3HP3m43Iq/YYgkSdMv4BR+RThPPnIIzrbAjEAHHtMgH7vVh036+bcw9UqBfSdD/IBqrKpJLERn5+Qi/4Q4EoReCC0CTDfZ+LcY0rUur0QZRkMpxx/9s4eCgIU+qfOcSlBvjoX7QAQ2MImUME1y5yJiyaWueamnhRBOwERGBuDKyGp4eBWp4i3esJcplrWYovjzPg9fL7Thy8v9KnrHy22PUFAYY+1vjKp";
//        parameters.cameraDirection = CameraDirection.BACK;
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
//
//    private void initTfod(@NonNull HardwareMap hardwareMap) {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
//        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
//        tfodParameters.minimumConfidence = 0.6;
//    }

//    public void moveRake(char dir, Hardware hulk) throws  InterruptedException {
//    if(dir == 'd')
//        {
//        hulk.rake.setPosition(.3);
//        }
//    else if(dir == 'u')
//        {
//        hulk.rake.setPosition(0.6);
//        }
//    }
//
//    public void openArms(Hardware hulk) throws InterruptedException
//    {
//        hulk.bArmLeft.setPosition(0);
//        Thread.sleep(700);
//        hulk.bArmRight.setPosition(0);
//        Thread.sleep(500);
//    }
//    public void closeArms(Hardware hulk) throws InterruptedException
//    {
//        hulk.bArmLeft.setPosition(.55);
//        hulk.bArmRight.setPosition(0.3);
//    }


}