package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

@Config
@Autonomous(name = "AUTO BLUE_v12_FastShot")
public class testAuto_v6 extends LinearOpMode {

    public enum BallColor { PURPLE, GREEN, UNKNOWN, NONE }
    // 槽位陣列：0=A, 1=B, 2=C
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    public static int FRONT_SHOOT_TURRET_POS = 0;
    public static double FRONT_SHOOT_RPM = 560;
    public static int BACK_SHOOT_TURRET_POS = -42;
    public static double BACK_SHOOT_RPM = 1400;

    public static boolean isShootingMode = false;
    public static boolean isPreheating = false;
    public static boolean useBackShootingParams = false;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(90, 0, 0, 15);
    private static final double ANGLE_CLOSE = 0.12;
    private static final double ANGLE_FAR = 0.12;
    private static final double RPM_IDLE = 300.0;

    // 伺服馬達位置定義
    public static double FILL_POS_A = 0.0;
    public static double FILL_POS_B = 0.2431;
    public static double FILL_POS_C = 0.5355;

    private static double FIRE_POS_A = 0.5882;
    private static double FIRE_POS_B = 0.08;
    private static double FIRE_POS_C = 0.31;

    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware robot = new SharedHardware(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // 初始化
        isShootingMode = false;
        isPreheating = false;
        useBackShootingParams = false;
        Arrays.fill(actualBallSlots, BallColor.NONE);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(28),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-28, 28);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        telemetry.addLine("Ready: Shortest Path Shooting Mode");
        telemetry.update();

        while (opModeInInit()) {
            telemetry.addData("Limelight Status", robot.limelight.isConnected());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // 初始硬體位
        robot.angleServo.setPosition(ANGLE_CLOSE);
        robot.gateServoL.setPosition(0.6667);
        robot.gateServoR.setPosition(0.6902);
        robot.intakeMotor.setPower(1.0);

        // 預裝球初始化 (A, B, C)
        actualBallSlots[0] = BallColor.PURPLE;
        actualBallSlots[1] = BallColor.PURPLE;
        actualBallSlots[2] = BallColor.GREEN;

        Action cmdPrepareBack = packet -> {
            useBackShootingParams = true;
            isPreheating = true;
            return false;
        };

        Action cmdStopPreheat = packet -> { isPreheating = false; return false; };
        Action cmdStopIntake = packet -> { robot.intakeMotor.setPower(0); return false; };

        Action closeGate = packet -> {
            robot.gateServoL.setPosition(0.6667);
            robot.gateServoR.setPosition(0.6902);
            return false;
        };

        Actions.runBlocking(
                new ParallelAction(
                        new BackgroundSystemAction(robot, telemetry),
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        // === 第一波射擊 (最快射擊) ===
                                        .afterTime(0, cmdPrepareBack)
                                        .waitSeconds(1.5)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        // === 第一波吸球 ===
                                        .strafeTo(new Vector2d(-24.5, -14))
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-24.5, -50.5), slowVel, slowAccel)

                                        // === 第二波射擊 ===
                                        .strafeTo(new Vector2d(0, 0))
                                        .stopAndAdd(closeGate)
                                        .waitSeconds(0.4)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        // === 第二波吸球 ===
                                        .strafeTo(new Vector2d(-49, -14))
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-49, -50.5), slowVel, slowAccel)
                                        .stopAndAdd(cmdStopIntake)

                                        // === 第三波射擊 ===
                                        .strafeTo(new Vector2d(0, 0))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        // === 停車 ===
                                        .strafeTo(new Vector2d(-49, -45))
                                        .build()
                        )
                )
        );
    }

    // =================================================================
    // 背景控制系統
    // =================================================================
    public static class BackgroundSystemAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private double currentCommandedRpm = RPM_IDLE;

        public BackgroundSystemAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int targetPos = useBackShootingParams ? BACK_SHOOT_TURRET_POS : FRONT_SHOOT_TURRET_POS;
            double targetRpm = useBackShootingParams ? BACK_SHOOT_RPM : FRONT_SHOOT_RPM;
            double targetAngleServo = useBackShootingParams ? ANGLE_FAR : ANGLE_CLOSE;

            robot.baseMotor.setTargetPosition(targetPos);
            robot.baseMotor.setPower(1.0);

            double desiredRpm = (isShootingMode || isPreheating) ? targetRpm : RPM_IDLE;

            // 簡易升速邏輯
            if (desiredRpm >= currentCommandedRpm) currentCommandedRpm = desiredRpm;
            else currentCommandedRpm -= 0.8;

            robot.shooterRight.setVelocity(currentCommandedRpm);
            robot.shooterLeft.setPower(robot.shooterRight.getPower());
            robot.angleServo.setPosition(targetAngleServo);

            return true;
        }
    }

    // =================================================================
    // 循序吸球 (A -> B -> C)
    // =================================================================
    public static class AutoIntakeAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;
        private enum State { IDLE, WAIT_SETTLE, ROTATING, FULL }
        private State state = State.IDLE;

        public AutoIntakeAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                robot.gateServoL.setPosition(0.32);
                robot.gateServoR.setPosition(0.32);
                robot.diskServo.setPosition(FILL_POS_A);
                robot.intakeMotor.setPower(1.0);
                Arrays.fill(actualBallSlots, BallColor.NONE);
                currentFillStep = 0;
                initialized = true;
            }
            if (currentFillStep >= 3) return false;

            switch (state) {
                case IDLE:
                    if (getSensorColor() != BallColor.NONE) { timer = System.currentTimeMillis(); state = State.WAIT_SETTLE; }
                    break;
                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > 30) {
                        BallColor confirmed = getSensorColor();
                        if (confirmed != BallColor.NONE) {
                            actualBallSlots[currentFillStep] = confirmed;
                            moveToNextSequence();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        } else state = State.IDLE;
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > 60) state = State.IDLE;
                    break;
            }
            return true;
        }

        private void moveToNextSequence() {
            currentFillStep++;
            if (currentFillStep == 1) robot.diskServo.setPosition(FILL_POS_B);
            else if (currentFillStep == 2) robot.diskServo.setPosition(FILL_POS_C);
        }

        private BallColor getSensorColor() {
            NormalizedRGBA c1 = robot.colorSensor1.getNormalizedColors();
            NormalizedRGBA c2 = robot.colorSensor2.getNormalizedColors();
            if (c1.alpha > 0.7f) return processColor(c1);
            if (c2.alpha > 0.7f) return processColor(c2);
            return BallColor.NONE;
        }

        private BallColor processColor(NormalizedRGBA color) {
            if (color.blue > color.green && color.blue > color.red) return BallColor.PURPLE;
            if (color.green > color.red) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }

    // =================================================================
    // 最短路徑射擊 (Shortest Path Shot)
    // =================================================================
    public static class BackShooterAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsCompleted = 0;
        private String targetSlot = "";

        private enum State { INIT, CHECK_READY, AIM_DISK, KICK, RETRACT, DONE }
        private State state = State.INIT;

        public BackShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = true;

            if (!initialized) { shotsCompleted = 0; state = State.INIT; initialized = true; }

            switch (state) {
                case INIT:
                    if (shotsCompleted >= 3) {
                        state = State.DONE;
                    } else {
                        targetSlot = findNearestOccupiedSlot();
                        if (targetSlot == null) state = State.DONE;
                        else {
                            state = State.CHECK_READY;
                            timer = System.currentTimeMillis();
                        }
                    }
                    break;

                case CHECK_READY:
                    if (System.currentTimeMillis() - timer > 50) { // 減少等待
                        moveServoToFire(targetSlot);
                        state = State.AIM_DISK;
                        timer = System.currentTimeMillis();
                    }
                    break;

                case AIM_DISK:
                    if (System.currentTimeMillis() - timer > 400) { // 轉盤到位時間優化
                        robot.kickerServo.setPosition(0.8);
                        state = State.KICK;
                        timer = System.currentTimeMillis();
                    }
                    break;

                case KICK:
                    if (System.currentTimeMillis() - timer > 250) { // 擊發時間優化
                        robot.kickerServo.setPosition(0.0);
                        clearSlot(targetSlot);
                        shotsCompleted++;
                        state = State.RETRACT;
                        timer = System.currentTimeMillis();
                    }
                    break;

                case RETRACT:
                    if (System.currentTimeMillis() - timer > 250) { // 收回時間優化
                        state = State.INIT;
                    }
                    break;

                case DONE:
                    isShootingMode = false;
                    robot.diskServo.setPosition(0.0);
                    return false;
            }
            return true;
        }

        // 核心邏輯：計算哪一個有球的槽位離當前伺服馬達位置最近
        private String findNearestOccupiedSlot() {
            double currentPos = robot.diskServo.getPosition();
            double minDiff = Double.MAX_VALUE;
            String bestSlot = null;

            // 檢查 A
            if (actualBallSlots[0] != BallColor.NONE) {
                double diff = Math.abs(currentPos - FIRE_POS_A);
                if (diff < minDiff) { minDiff = diff; bestSlot = "A"; }
            }
            // 檢查 B
            if (actualBallSlots[1] != BallColor.NONE) {
                double diff = Math.abs(currentPos - FIRE_POS_B);
                if (diff < minDiff) { minDiff = diff; bestSlot = "B"; }
            }
            // 檢查 C
            if (actualBallSlots[2] != BallColor.NONE) {
                double diff = Math.abs(currentPos - FIRE_POS_C);
                if (diff < minDiff) { minDiff = diff; bestSlot = "C"; }
            }
            return bestSlot;
        }

        private void moveServoToFire(String slot) {
            if (slot.equals("A")) robot.diskServo.setPosition(FIRE_POS_A);
            else if (slot.equals("B")) robot.diskServo.setPosition(FIRE_POS_B);
            else if (slot.equals("C")) robot.diskServo.setPosition(FIRE_POS_C);
        }

        private void clearSlot(String slot) {
            if (slot.equals("A")) actualBallSlots[0] = BallColor.NONE;
            else if (slot.equals("B")) actualBallSlots[1] = BallColor.NONE;
            else if (slot.equals("C")) actualBallSlots[2] = BallColor.NONE;
        }
    }

    // =================================================================
    // 硬體映射
    // =================================================================
    public static class SharedHardware {
        public Limelight3A limelight;
        public DcMotorEx shooterRight, shooterLeft, baseMotor;
        public Servo angleServo, diskServo, kickerServo, gateServoL, gateServoR;
        public DcMotor intakeMotor;
        public NormalizedColorSensor colorSensor1, colorSensor2;

        public SharedHardware(HardwareMap map) {
            limelight = map.get(Limelight3A.class, "limelight");
            shooterRight = map.get(DcMotorEx.class, "motor7");
            shooterLeft = map.get(DcMotorEx.class, "motor5");
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            baseMotor = map.get(DcMotorEx.class, "motor6");
            baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setTargetPosition(0);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            angleServo = map.get(Servo.class, "servo3"); angleServo.setDirection(Servo.Direction.REVERSE);
            diskServo = map.get(Servo.class, "servo2");
            kickerServo = map.get(Servo.class, "servo1"); kickerServo.scaleRange(0.0, 0.5); kickerServo.setPosition(0.0);
            gateServoL = map.get(Servo.class, "servo4"); gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR = map.get(Servo.class, "servo5"); gateServoR.setDirection(Servo.Direction.FORWARD);
            intakeMotor = map.get(DcMotor.class, "motor4");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            colorSensor1 = map.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = map.get(NormalizedColorSensor.class, "colorSensor2");
            colorSensor1.setGain(25.0f); colorSensor2.setGain(25.0f);
        }
    }
}