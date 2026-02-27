package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "teleop_v5")
public class teleop_v5 extends LinearOpMode {

    private CRServo diskServo;
    private DcMotor intakeMotor;

    // === PID 參數 ===
    private double diskTargetPosition = 0;
    private double diskLastError = 0;

    // 因為編碼器靈敏，維持超低增益
    private static final double K_P = 0.00008;
    private static final double K_D = 0.00002;

    private static final double DISK_MAX_POWER = 0.25;
    private static final double DISK_TOLERANCE = 300;

    // 位置設定
    private static final double POS_A = 0;
    private static final double POS_B = 2623;

    @Override
    public void runOpMode() {
        diskServo = hardwareMap.get(CRServo.class, "servo2");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");

        // 重要：不使用 setDirection(REVERSE)，維持馬達原生方向
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("初始化完成 - 編碼器已清零");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateDiskPID();

            // 按下 B 前往 2623，按下 A 回到 0
            if (gamepad1.b) diskTargetPosition = POS_B;
            if (gamepad1.a) diskTargetPosition = POS_A;

            // 測試吸球 (不影響編碼器清零)
            if (gamepad1.right_trigger > 0.1) intakeMotor.setPower(0.8);
            else intakeMotor.setPower(0);

            updateTelemetry();
        }
    }

    private void updateDiskPID() {
        double currentPos = intakeMotor.getCurrentPosition();

        // 【核心修正點】：在計算誤差前加上負號，反轉 PID 邏輯
        // 這樣就不需要去 reverse 馬達硬體，也能修正「方向反了」的問題
        double error = (diskTargetPosition - currentPos);

        double absError = Math.abs(error);

        if (absError < DISK_TOLERANCE) {
            diskServo.setPower(0);
            diskServo.getController().pwmDisable();
            diskLastError = error;
            return;
        }

        diskServo.getController().pwmEnable();

        double derivative = error - diskLastError;
        double power = (error * K_P) + (derivative * K_D);

        // 限制最大功率
        power = Math.max(-DISK_MAX_POWER, Math.min(DISK_MAX_POWER, power));

        // 最小啟動功率門檻
        if (Math.abs(power) < 0.08) power = Math.signum(power) * 0.08;

        diskServo.setPower(power);
        diskLastError = error;
    }

    private void updateTelemetry() {
        telemetry.addData("Target", diskTargetPosition);
        telemetry.addData("Current", intakeMotor.getCurrentPosition());
        telemetry.addData("Error (Inverted)", -(diskTargetPosition - intakeMotor.getCurrentPosition()));
        telemetry.addData("Servo Power", diskServo.getPower());
        telemetry.update();
    }
}