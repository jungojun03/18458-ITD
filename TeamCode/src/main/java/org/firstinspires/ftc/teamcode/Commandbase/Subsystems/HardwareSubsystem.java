package org.firstinspires.ftc.teamcode.Commandbase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class HardwareSubsystem {
    public Servo Claw, ZPitch;
    private DcMotor RTPSpec;
    private DcMotor extendoSlide;
    private DcMotor InvertextendoSlide;
    private DcMotorEx specArm;
    private Motor.Encoder encoder;
    public Servo specWrist, specClaw;
    private Servo Wrist;

    public HardwareMap hardwareMap;

    public IMU imu;

    //Claw
    public boolean clawStateGrabbed = true;

    public static double GRAB = 0.74, OPEN = 0.45;
    public static double ZGRAB = 0.74, ZOUTTAKE = 0.37;

    public static double SPECGRAB = 0.315;
    public static double SPECOPEN = 0;

    public static double RESET_POSE = 0.95;
    public static double GRAB_POSE = 0.23;
    public static double SPEC_GRAB = 0.32;
    public double targetPose;

    //RTP SpecArm
    public static int specMin = -5;
    public static int specMax = 2500;

    public static int Intake = 3;
    public static int Mid = 400;
    public static int Outtake = 825;
    public static int Last = 825;
    public int specCurrent = 0;

    //Extendo Slide
    public static int slideMin = -5;
    public static int slideMax = 1450;

    public static int High = 1450;
    public static int Reset = 0;
    public int slideCurrent = 0;

    //Start at 0.001 then go up
    public PIDController armController;
    public static double p = 0.002, i = 0.055, d = 0.000005, f = 0.005;
    public static int armTarget;
    public double pos;
    public static int INTAKE = 0, OUTTAKE = 2800, SPEC = 1700;
    public boolean usingPIDFArm = true;

    //Spec Servos    public static double SPECOPEN = 0, SPECGRAB = 0.5;
    public static double NEUTRAL = 0.92, SCORE = 0.37;

    //Wrist
    public static double NEUTRAL_POSE = 0.94, HORIZONTAL_GRAB_POSE = 0.6;
    public double rotating = 0.15;

    Telemetry telemetry;

    public HardwareSubsystem(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.claw = (Servo) hardwareMap.get("Claw");
        this.zpitch = (Servo) hardwareMap.get("ZPitch");

        this.RTPSpec = (DcMotor) hardwareMap.get("specArm");

        RTPSpec.setDirection(DcMotorSimple.Direction.FORWARD);
        RTPSpec.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTPSpec.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.extendoSlide = (DcMotor) hardwareMap.get("rightSlide");
        this.InvertextendoSlide = (DcMotor) hardwareMap.get("leftSlide");

        extendoSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        InvertextendoSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        extendoSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InvertextendoSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InvertextendoSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendoSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.specArm = (DcMotorEx) hardwareMap.get("specArm");
        encoder = new MotorEx(hardwareMap, "encoder").encoder;
        encoder.setDirection(Motor.Direction.FORWARD);
        specArm.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.reset();
        specArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController = new PIDController(p, i, d);

        this.specClaw = (Servo) hardwareMap.get("sClaw");
        this.specWrist = (Servo) hardwareMap.get("sWrist");
        this.wrist = (Servo) hardwareMap.get("wrist");

        //wrist.setDirection(Servo.Direction.REVERSE);
        specWrist.setDirection(Servo.Direction.REVERSE);
    }

    public void armGrab() {
        Arm.setPosition(GRAB_POSE);
        targetPose = GRAB_POSE;
    }

    public void hover() {
      Arm.setPosition(SPEC_GRAB);
        targetPose = SPEC_GRAB;
    }

    public void armReset() {
      Arm.setPosition(RESET_POSE);
        targetPose = RESET_POSE;
    }

    public void SampClose() {
        Claw.setPosition(GRAB);
        ClawStateGrabbed = true;
    }

    public void SampOpen() {
        Claw.setPosition(OPEN);
        ClawStateGrabbed = false;
    }
    public void Zintake() {
        ZPitch.setPosition(ZGRAB);
    }
    public void Zouttake() {
        ZPitch.setPosition(ZOUTTAKE);
    }

    public void setPos(int pos) {
        if (pos <= specMax && pos >= specMin) specCurrent = pos;
        System.out.println(specCurrent);
        normalize();
    }
    public void normalize() {
        specArm.setTargetPosition(specCurrent);
        specArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        specArm.setPower(1);
        /*if (getSpecPose() < 10) {
            specArm.setPower(0);
        } else {
            specArm.setPower(1);
        }*/
    }

    public int getSpecPose() {
        return specCurrent;
    }

    //RTP Spec Pose
    public void Intake() {
        setPos(Intake);
    }
    public void Mid() {
        setPos(Mid);
    }
    public void Outtake() {
        setPos(Outtake);
    }

    public void Last() {
        setPos(Last);
    }

    public void setSlidePos(int pos) {
        if (pos <= slideMax && pos >= slideMin) slideCurrent = pos;
        System.out.println(slideCurrent);
        setExtendoSlide();
    }

    public int getPos() {
        return slideCurrent;
    }

    public void moveManual(double position) {
        setPos((int) position);
    }

    public void setExtendoSlide() {
        extendoSlide.setTargetPosition(slideCurrent);
        InvertextendoSlide.setTargetPosition(slideCurrent);
        extendoSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        InvertextendoSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendoSlide.setPower(1);
        InvertextendoSlide.setPower(1);
    }

    public void Manual(double position) {
        extendoSlide.setPower(0.75);
        InvertextendoSlide.setPower(0.75);
        if (Math.abs(position) > 0.1) {
            moveManual(getPos() + position * 15);
        }
    }

    //Lift Pose
    public void liftRest() {
        setSlidePos(Reset);
    }

    public void liftHigh() {
        setSlidePos(High);
    }

    public void update() {
        if (usingPIDFArm) {
            armController.setPID(p, i, d);

            specArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid_output = armController.calculate(getArmPos(), armTarget);
            double power = pid_output + f;

            if (getArmPos() < 50 && armTarget < 50) {
                specArm.setPower(0);
            } else {
                specArm.setPower(power);
            }
        }
    }

    //Spec Arm (OG)
    public double getArmPos() {
        pos = encoder.getPosition() / 4;
        return pos;
    }

    public int getTarget() {
        return armTarget;
    }

    public void setTarget(int b) {
        usingPIDFArm = true;
        armTarget = b;
    }

    public void init() {
        armController.setPID(p,i,d);
        setTarget(0);
    }
    public void intake() {
        setTarget(INTAKE);
    }
    public void outtake() {
        setTarget(OUTTAKE);
    }
    public void spec() {
        setTarget(SPEC);
    }

    //Spec Servos
    public void specGrab() {
        specClaw.setPosition(SPECGRAB);
    }

    public void specOpen() {
        specClaw.setPosition(SPECOPEN);
    }
    public void specNuetral() {
        specWrist.setPosition(NEUTRAL);
    }
    public void specScore() {
        specWrist.setPosition(SCORE);
    }

    //Wrist
    public void neutralGrab() {
        wrist.setPosition(NEUTRAL_POSE);
    }

    public void horizontalGrab() {
        wrist.setPosition(HORIZONTAL_GRAB_POSE);
    }

    public void RTPIntake() {
        Intake();
        specScore();
    }

    public void robotInit() {
        neutralGrab();
        specOpen();
        liftRest();
        Zouttake();
        specScore();
        armReset();
        SampClose();
    }
}
