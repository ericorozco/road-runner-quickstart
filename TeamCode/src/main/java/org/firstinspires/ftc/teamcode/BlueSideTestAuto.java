package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@Config
@Autonomous(name = "Right Auto", group = "ITD Auto", preselectTeleOp = "AyMarthaV2")
public class BlueSideTestAuto extends LinearOpMode {
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 1000;
    private int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionIn = 0.2;
    final double OuttakeElbowPositionOut = 0.92;
    final double OuttakeElbowPositionMiddle = 0.55;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private enum IntakeState{
        IN,
        OUT
    }
    final double IntakeSliderPositionOut = 0.60;
    final double IntakeSliderPositionIN = 0.0;
    public class Outtake{

    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-8, 70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //Outtake
        // Sliders Mapping and Setup
        OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);
        // vision here that outputs position
        int visionOutputPosition = 1;


        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        // actions that need to happen on init; for instance, a claw tightening.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(47, new TranslationalVelConstraint(100));
                //.waitSeconds(1);
        /**
         * Push the first sample and line up for second sample
         */
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-8, 47, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-7.04, 37.15, Math.toRadians(90.00)), Math.toRadians(-89.33))
                .splineToLinearHeading(new Pose2d(-33.89, 23.0, Math.toRadians(270.00)), Math.toRadians(270.00))
                //.lineToY(20.0)
                .strafeTo(new Vector2d(-45.0,23.0))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(55.0)
                //.waitSeconds(1.5)
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(23.0)
                .strafeTo(new Vector2d(-58.0,23.0));
                //.setTangent(Math.toRadians(270.0))

        /**
             * Push the second sample
         */
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58, 18.0, Math.toRadians(270.00)))
                .lineToYConstantHeading(53.0, new TranslationalVelConstraint(90))
                .waitSeconds(0.5);
        /**
         * Prepare to grab sample
         */
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-58, 53.0, Math.toRadians(270.00)))

                //.strafeTo(new Vector2d(-40.0,50.0), 1000, 500)
                .strafeTo(new Vector2d(-37.0,58.0))
                .setTangent(Math.toRadians(270.0))
                .lineToY(62.0, new TranslationalVelConstraint(100))
                .waitSeconds(0.5);

        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-30.0, 60.0, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(-8.0, 40, Math.toRadians(90.0)), Math.toRadians(90.0))
                //.lineToY(60.00)
                .waitSeconds(0.5);
        /**
         *Park
         */
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-8.0, 40.0, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(-30.0, 58.0, Math.toRadians(45.0)), Math.toRadians(45.0))
                //.lineToY(60.00)
                .waitSeconds(0.5);

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        OuttakeElbowMove(OuttakeElbowPositionMiddle);

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );
        OuttakeSliders(HIGH_CHAMBER, 0, 0);

        sleep(250);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(500);
        OuttakeSliders(-HIGH_CHAMBER, 0, 0);
        sleep(850);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        tab2.build()
                )
        );
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionOut);

        Actions.runBlocking(
                new SequentialAction(
                        tab3.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        tab4.build()
                )
        );
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(400);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        Actions.runBlocking(
                new SequentialAction(
                        tab5.build()
                )
        );
        sleep(500);
        OuttakeSliders(HIGH_CHAMBER,0,0);
        sleep(600);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(500);
        OuttakeSliders(-HIGH_CHAMBER,0,0);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        Actions.runBlocking(
                new SequentialAction(
                        tab6.build()
                )
        );
        intakeSlidersElbow(IntakeState.OUT);
        sleep(500);


    }

    private void OuttakeSliders(int targetPosition, int velocity, int timeout) {
        // Set run modes for both motors
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        //Run to target position
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        OuttakeSliderRight.setPower(0.85);
        OuttakeSliderLeft.setPower(0.85);
    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
               break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                break;

        }

    }

}