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
import com.qualcomm.robotcore.hardware.LED;
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
@Autonomous(name = "AUTO BLUE front with LED")
public class testAuto_v3 extends LinearOpMode {

    public enum BallColor { PURPLE, GREEN, UNKNOWN, NONE }
    public static List<BallColor> targetSequence = new ArrayList<>();
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    public static int FRONT_SHOOT_TURRET_POS = 0;
    public static double FRONT_SHOOT_RPM = 560;
    public static int BACK_SHOOT_TURRET_POS = 380;
    public static double BACK_SHOOT_RPM = 1130;

    public static boolean isShootingMode = false;
    public static boolean isPreheating = false;
    public static boolean useBackShootingParams = false;

    public enum TurretState { MANUAL_POSITION, IDLE }
    public static TurretState currentTurretState = TurretState.MANUAL_POSITION;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(90, 0, 0, 15);
    private static final double ANGLE_CLOSE = 0.12;
    private static final double ANGLE_FAR = 0.12;
    private static final double RPM_IDLE = 300.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware robot = new SharedHardware(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // 初始化狀態
        isShootingMode = false;
        isPreheating = false;
        useBackShootingParams = false;
        currentTurretState = TurretState.MANUAL_POSITION;
        Arrays.fill(actualBallSlots, BallColor.NONE);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(28),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-28, 28);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        setSequence(BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE);

        telemetry.addLine("Ready: AUTO with LED Logic");
        telemetry.update();

        while (opModeInInit()) {
            LLResult result = robot.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    int id = (int) tags.get(0).getFiducialId();
                    if (id == 21) setSequence(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE);
                    else if (id == 22) setSequence(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE);
                    else if (id == 23) setSequence(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);
                }
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // 初始硬體設定
        robot.angleServo.setPosition(ANGLE_CLOSE);
        robot.gateServoL.setPosition(0.6667);
        robot.gateServoR.setPosition(0.6902);
        robot.intakeMotor.setPower(1.0);

        // 模擬開局預裝球，同時點亮 LED
        actualBallSlots[0] = BallColor.PURPLE; robot.LED0.on();
        actualBallSlots[1] = BallColor.PURPLE; robot.LED1.on();
        actualBallSlots[2] = BallColor.GREEN;  robot.LED2.on();

        // 定義簡單 Actions
        Action cmdPrepareBack = packet -> {
            useBackShootingParams = true;
            isPreheating = true;
            currentTurretState = TurretState.MANUAL_POSITION;
            return false;
        };

        Action cmdStopPreheat  = packet -> { isPreheating = false; return false; };
        Action cmdStopIntake = packet -> { robot.intakeMotor.setPower(0); return false; };
        Action cmdReverseIntake = packet -> { robot.intakeMotor.setPower(-1); return false; };

        Action closeGate = packet -> {
            robot.gateServoL.setPosition(0.6667);
            robot.gateServoR.setPosition(0.6902);
            return false;
        };

        // 執行 RoadRunner 路線
        Actions.runBlocking(
                new ParallelAction(
                        new BackgroundSystemAction(robot, telemetry),
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-72,0))
                                        .waitSeconds(0.9)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)
                                        .strafeTo(new Vector2d(-72, -14))
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-72, -40.5), slowVel, slowAccel)
                                        .stopAndAdd(cmdStopIntake)
                                        .stopAndAdd(cmdReverseIntake)
                                        .strafeTo(new Vector2d(-72,-20))
                                        .strafeTo(new Vector2d(-72,0))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)
                                        .strafeTo(new Vector2d(-49, -14))
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-49, -50.5), slowVel, slowAccel)
                                        .stopAndAdd(cmdReverseIntake)
                                        .stopAndAdd(cmdStopIntake)
                                        .strafeTo(new Vector2d(-72,0))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)
                                        .strafeTo(new Vector2d(-24.5, -14))
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(-24.5, -50.5), slowVel, slowAccel)
                                        .stopAndAdd(cmdStopIntake)
                                        .stopAndAdd(cmdReverseIntake)
                                        .strafeTo(new Vector2d(-72,0))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)
                                        .strafeTo(new Vector2d(-49,-45))
                                        .build()
                        )
                )
        );
    }

    private void setSequence(BallColor c1, BallColor c2, BallColor c3) {
        targetSequence.clear();
        targetSequence.add(c1); targetSequence.add(c2); targetSequence.add(c3);
    }

    // =================================================================
    // 硬體設定
    // =================================================================
    public static class SharedHardware {
        public Limelight3A limelight;
        public DcMotorEx shooterRight, shooterLeft, baseMotor;
        public Servo angleServo, diskServo, kickerServo, gateServoL, gateServoR;
        public DcMotor intakeMotor;
        public NormalizedColorSensor colorSensor1, colorSensor2;
        public LED LED0, LED1, LED2; // 新增 LED

        public SharedHardware(HardwareMap map) {
            limelight = map.get(Limelight3A.class, "limelight");
            shooterRight = map.get(DcMotorEx.class, "motor7");
            shooterLeft = map.get(DcMotorEx.class, "motor5");
            shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
            shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

            // LED 初始化
            LED0 = map.get(LED.class, "LED0");
            LED1 = map.get(LED.class, "LED1");
            LED2 = map.get(LED.class, "LED2");
            LED0.off(); LED1.off(); LED2.off();
        }
    }

    // =================================================================
    // 後台系統
    // =================================================================
    public static class BackgroundSystemAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private double currentCommandedRpm = RPM_IDLE;
        private static final double RPM_RAMP_DOWN_STEP = 0.4;

        public BackgroundSystemAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int targetPos = useBackShootingParams ? BACK_SHOOT_TURRET_POS : FRONT_SHOOT_TURRET_POS;
            double targetRpm = useBackShootingParams ? BACK_SHOOT_RPM : FRONT_SHOOT_RPM;
            double targetAngleServo = useBackShootingParams ? ANGLE_FAR : ANGLE_CLOSE;

            robot.baseMotor.setTargetPosition(targetPos);
            robot.baseMotor.setPower(1.0);

            double desiredRpm = RPM_IDLE;
            if (isShootingMode || isPreheating) {
                desiredRpm = Math.max(0, Math.min(2800, targetRpm));
                robot.angleServo.setPosition(targetAngleServo);
            }

            if (desiredRpm >= currentCommandedRpm) currentCommandedRpm = desiredRpm;
            else { currentCommandedRpm -= RPM_RAMP_DOWN_STEP; if (currentCommandedRpm < desiredRpm) currentCommandedRpm = desiredRpm; }

            robot.shooterRight.setVelocity(currentCommandedRpm);
            robot.shooterLeft.setPower(robot.shooterRight.getPower());
            return true;
        }
    }

    // =================================================================
    // 吸球動作 (Auto Intake + LED Logic)
    // =================================================================
    public static class AutoIntakeAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;
        private enum State { INIT, IDLE, WAIT_SETTLE, ROTATING, FULL }
        private State state = State.INIT;
        private static double FILL_POS_1 = 0.0;
        private static double FILL_POS_2 = 0.28;
        private static double FILL_POS_3 = 0.5355;

        public AutoIntakeAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                robot.gateServoL.setPosition(0.32);
                robot.gateServoR.setPosition(0.32);
                robot.diskServo.setPosition(FILL_POS_1);
                robot.intakeMotor.setPower(1.0);
                // 進入 AutoIntake 時重新清理狀態與 LED
                Arrays.fill(actualBallSlots, BallColor.NONE);
                robot.LED0.off(); robot.LED1.off(); robot.LED2.off();
                currentFillStep = 0;
                state = State.IDLE;
                initialized = true;
            }
            if (currentFillStep >= 3) state = State.FULL;

            switch (state) {
                case IDLE:
                    BallColor detected = getSensorColor();
                    if (detected != BallColor.NONE) { timer = System.currentTimeMillis(); state = State.WAIT_SETTLE; }
                    break;
                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > 50) {
                        BallColor confirmed = getSensorColor();
                        if (confirmed != BallColor.NONE) {
                            actualBallSlots[currentFillStep] = confirmed;

                            // 根據目前位置點亮 LED
                            if (currentFillStep == 0) robot.LED0.on();
                            else if (currentFillStep == 1) robot.LED1.on();
                            else if (currentFillStep == 2) robot.LED2.on();

                            moveToNextPos();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        } else state = State.IDLE;
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > 60) state = State.IDLE;
                    break;
                case FULL:
                    return false;
            }
            return true;
        }

        private void moveToNextPos() {
            if (currentFillStep == 0) { robot.diskServo.setPosition(FILL_POS_2); currentFillStep = 1; }
            else if (currentFillStep == 1) { robot.diskServo.setPosition(FILL_POS_3); currentFillStep = 2; }
            else currentFillStep = 3;
        }

        private BallColor getSensorColor() {
            BallColor c1 = checkSensor(robot.colorSensor1);
            if (c1 != BallColor.NONE) return c1;
            return checkSensor(robot.colorSensor2);
        }

        private BallColor checkSensor(NormalizedColorSensor sensor) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            if (color.alpha < 0.7f) return BallColor.NONE;
            if (color.blue > color.green && color.blue > color.red && color.blue > (color.green * 1.2f)) return BallColor.PURPLE;
            if (color.green > color.red && (color.green >= color.blue || (color.green > color.blue * 0.85f))) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }

    // =================================================================
    // 連發射擊動作 (Back Shooter + LED Logic)
    // =================================================================
    public static class BackShooterAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsAttempted = 0;
        private String currentSlot = "";

        private static double FIRE_POS_A = 0.5882;
        private static double FIRE_POS_B = 0.08;
        private static double FIRE_POS_C = 0.31;

        private enum State { INIT, CHECK_READY, AIM_DISK, KICK, RETRACT, DONE }
        private State state = State.INIT;

        public BackShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = true;

            if (!initialized) { shotsAttempted = 0; state = State.INIT; initialized = true; }

            switch (state) {
                case INIT:
                    if (shotsAttempted >= 3) { state = State.DONE; }
                    else {
                        BallColor needed = (shotsAttempted < targetSequence.size()) ? targetSequence.get(shotsAttempted) : BallColor.UNKNOWN;
                        currentSlot = findSlotForColor(needed);
                        if (currentSlot == null) currentSlot = findAnyOccupiedSlot();

                        if (currentSlot == null) state = State.DONE;
                        else {
                            state = State.CHECK_READY;
                            timer = System.currentTimeMillis();
                        }
                    }
                    break;
                case CHECK_READY:
                    if (System.currentTimeMillis() - timer > 20) { setupDisk(currentSlot); state = State.AIM_DISK; timer = System.currentTimeMillis(); }
                    break;
                case AIM_DISK:
                    if (System.currentTimeMillis() - timer > 550) { robot.kickerServo.setPosition(0.8); state = State.KICK; timer = System.currentTimeMillis(); }
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 250) {
                        robot.kickerServo.setPosition(0.0);
                        removeBall(currentSlot); // 這裡會關閉對應 LED
                        shotsAttempted++;
                        state = State.RETRACT;
                        timer = System.currentTimeMillis();
                    }
                    break;
                case RETRACT:
                    if (System.currentTimeMillis() - timer > 200) state = State.INIT;
                    break;
                case DONE:
                    isShootingMode = false;
                    robot.diskServo.setPosition(0.0);
                    return false;
            }
            return true;
        }

        private void setupDisk(String slot) {
            if (slot.equals("A")) robot.diskServo.setPosition(FIRE_POS_A);
            else if (slot.equals("B")) robot.diskServo.setPosition(FIRE_POS_B);
            else robot.diskServo.setPosition(FIRE_POS_C);
        }

        private String findSlotForColor(BallColor color) {
            if (actualBallSlots[0] == color) return "A";
            if (actualBallSlots[1] == color) return "B";
            if (actualBallSlots[2] == color) return "C";
            return null;
        }

        private String findAnyOccupiedSlot() {
            if (actualBallSlots[0] != BallColor.NONE) return "A";
            if (actualBallSlots[1] != BallColor.NONE) return "B";
            if (actualBallSlots[2] != BallColor.NONE) return "C";
            return null;
        }

        private void removeBall(String slot) {
            if (slot.equals("A")) { actualBallSlots[0] = BallColor.NONE; robot.LED0.off(); }
            else if (slot.equals("B")) { actualBallSlots[1] = BallColor.NONE; robot.LED1.off(); }
            else if (slot.equals("C")) { actualBallSlots[2] = BallColor.NONE; robot.LED2.off(); }
        }
    }
}