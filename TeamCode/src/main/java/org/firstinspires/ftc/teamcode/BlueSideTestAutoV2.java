package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@Config
@Autonomous(name = "Right Auto V2", group = "ITD Auto", preselectTeleOp = "AyMarthaV2")
public class BlueSideTestAutoV2 extends LinearOpMode {
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 1000;
    private int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionIn = 0.2;
    final double OuttakeElbowPositionOut = 0.95;
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
        //PoseVelocity2d pv = new PoseVelocity2d(new Vector2d(70,70), 50);
        Pose2d initialPose = new Pose2d(-8, 70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //drive.setDrivePowers(pv);

        double Y_Samples = 13.0;
        double Y_Observation_Samples = 53;
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

        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        // vision here that outputs position
        int visionOutputPosition = 1;


        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        // actions that need to happen on init; for instance, a claw tightening.
        TrajectoryActionBuilder driveToHighChamber = drive.actionBuilder(initialPose)
                .lineToY(47, new TranslationalVelConstraint(100));
                //.waitSeconds(1);
        /**
         * Push the first sample and line up for second sample
         */
        TrajectoryActionBuilder firstSample = drive.actionBuilder(new Pose2d(-8, 47, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-7.04, 37.15, Math.toRadians(90.00)), Math.toRadians(-89.33))
                .splineToLinearHeading(new Pose2d(-33.89, Y_Samples, Math.toRadians(270.00)), Math.toRadians(270.00))
                //.lineToY(20.0)
                .strafeTo(new Vector2d(-45.0,Y_Samples), new TranslationalVelConstraint(100))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(Y_Observation_Samples, new TranslationalVelConstraint(100))
                //.waitSeconds(1.5)
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(Y_Samples)
                .strafeTo(new Vector2d(-55.0,Y_Samples));
                //.setTangent(Math.toRadians(270.0))

        /**
             * Push the second sample
         */
        TrajectoryActionBuilder secondSample = drive.actionBuilder(new Pose2d(-55.0, Y_Samples, Math.toRadians(270.00)))
                .lineToYConstantHeading(Y_Observation_Samples)
                .waitSeconds(0.5);
        /**
         * Prepare to grab sample
         */
        TrajectoryActionBuilder lineUpToSecondSample = drive.actionBuilder(new Pose2d(-55.0, Y_Observation_Samples, Math.toRadians(270.00)))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(Y_Samples)
                .strafeTo(new Vector2d(-61.0,Y_Samples))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(Y_Observation_Samples)
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(50.0)
                //.strafeTo(new Vector2d(-40.0,50.0), 1000, 500)
                .strafeTo(new Vector2d(-37.0,50.0))
                .setTangent(Math.toRadians(270.0))
                .lineToY(60, new TranslationalVelConstraint(100));
               // .waitSeconds(0.5);

        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder driveToHighChamberSecondSample = drive.actionBuilder(new Pose2d(-37.0, 50.0, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(-3.0, 37.0, Math.toRadians(90.0)), Math.toRadians(90.0));
                //.lineToY(60.00)
                //.waitSeconds(0.5);
        /**
         *Park
         */
        TrajectoryActionBuilder parkObservationZone = drive.actionBuilder(new Pose2d(-3.0, 37.0, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(-30.0, 58.0, Math.toRadians(270.0)), Math.toRadians(270.0))                //.lineToY(60.00)
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

        if (isStopRequested()) return;
        /**
         *Prepare Specimen to score in the HIGH CHAMBER
         */
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         *Run Trajectory
         */
        Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamber.build()
                )
        );
        /**
         *Sliders Up to HIGH CHAMBER
         * Elbow Moved to place Specimen
         * Sliders Down After Placing Specimen
         * Elbow moved back to Middle Position
         */
        OuttakeSliders(HIGH_CHAMBER, 0, 0);

        sleep(250);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(400);
        OuttakeSliders(-(HIGH_CHAMBER - 100), 0, 0);
        sleep(700);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);

        /**
         *Spline to First Sample
         * Push First Sample to Observation Zone
         * Drive Back to line up for Second Sample
         */
        Actions.runBlocking(
                new SequentialAction(
                        firstSample.build()
                )
        );

        /**
         * Push Second Sample to Observation Zone
         */

        Actions.runBlocking(
                new SequentialAction(
                        secondSample.build()
                )
        );

        /**
         * Prepare Claw to get Specimen from Wall
         */
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        OuttakeSliderLeft.setPower(0);
        OuttakeSliderRight.setPower(0);

        /**
         * Drive to get second Specimen from Wall
         */
        Actions.runBlocking(
                new SequentialAction(
                        lineUpToSecondSample.build()
                )
        );

        /**
         * Grab second Specimen from Wall and Prepare Elbow to Transport
         */
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(300);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         * Drive to align to the HIGH CHAMBER
         */
        Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamberSecondSample.build()
                )
        );
        /**
         * Place second sample on the HIGH CHAMBER
         */
        sleep(500);
        OuttakeSliders(HIGH_CHAMBER,0,0);
        sleep(600);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(500);
        OuttakeSliders(-HIGH_CHAMBER,0,0);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        /**
         * PARK
         */
        Actions.runBlocking(
                new SequentialAction(
                        parkObservationZone.build()
                )
        );
        OuttakeElbowMove(OuttakeElbowPositionOut);
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