package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@Config
@Autonomous(name = "Left Auto", group = "ITD Auto", preselectTeleOp = "AyMarthaV2")
public class LeftSideTestAuto extends LinearOpMode {
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 1000;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.15;
    final double OuttakeElbowPositionIn = 0.85;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeElbowPositionOutSpecimen = 0.1;
    //final double OuttakeElbowPositionMiddle = 0.55;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private enum IntakeState{
        IN,
        OUT
    }
    private Servo IntakeClaw;

    public static double IntakeSliderPositionIN = 0.73;
    final double IntakeSliderPositionOut = 0.35;
    final double IntakeElbowPositionIn = 0.95;
    final double IntakeElbowPositionOut = 0.0;
    private Servo IntakeElbowRight;
    private Servo IntakeElbowLeft;
    private boolean IntakeClawOpen = true;
    private boolean IntakeSliderChanged = false;
    final double IntakeClawPositionClose = 0.0;
    final double IntakeClawPositionOpen = 1.00;
    public class Outtake{

    }
//    public static double x_initial, x_park;
    public static double SliderVelocity = 0.9;
    public static double x_initial = 8;
    public static double y_firstSpecimen = 48;
    public static double y_initial = 70;
    public static double y_firstYSample = 58.25;
    public static double x_firstYSample = 43.0;
    public static double x_toScore = 50.0;
    public static double y_toScore = 63.0;
    public static double x_secondYSample = 50;
//    public static double y_park;
    @Override


    public void runOpMode() {
//        y_firstSpecimen = 43;
//        x_initial = -8;
//        y_initial = 70;
        //PoseVelocity2d pv = new PoseVelocity2d(new Vector2d(70,70), 50);
        Pose2d initialPose = new Pose2d(x_initial, y_initial, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //drive.setDrivePowers(pv);

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
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");

        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");

        // RightWheel.setDirection(CRServo.Direction.FORWARD);
        // LeftWheel.setDirection(CRServo.Direction.REVERSE);
        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        // vision here that outputs position
        int visionOutputPosition = 1;


        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        // actions that need to happen on init; for instance, a claw tightening.
        TrajectoryActionBuilder driveToHighChamber = drive.actionBuilder(initialPose)
                .lineToY(y_firstSpecimen, new TranslationalVelConstraint(90));
                //.waitSeconds(1);
        /**
         * Push the first sample and line up for first yellow sample
         */
        TrajectoryActionBuilder firstSample = drive.actionBuilder(new Pose2d(x_initial, y_firstSpecimen, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-7.04, 37.15, Math.toRadians(90.00)), Math.toRadians(-89.33))
                .splineToLinearHeading(new Pose2d(x_firstYSample, y_firstYSample, Math.toRadians(270.0)), Math.toRadians(180.0), new TranslationalVelConstraint(20));

        /**
             * Push the second sample
         */
        TrajectoryActionBuilder firstSampleScore = drive.actionBuilder(new Pose2d(x_firstYSample, y_firstYSample, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(x_toScore, y_toScore, Math.toRadians(225.0)), Math.toRadians(225.0), new TranslationalVelConstraint(20));

        /**
         * Prepare to grab Specimen
         */
        TrajectoryActionBuilder lineUpToSecondSpecimen = drive.actionBuilder(new Pose2d(-58, 53.0, Math.toRadians(270.00)))
                .setTangent(Math.toRadians(270.0))
                .lineToYConstantHeading(50.0)
                //.strafeTo(new Vector2d(-40.0,50.0), 1000, 500)
                .strafeTo(new Vector2d(-42.0,50.0))
                .setTangent(Math.toRadians(270.0))
                .lineToY(61, new TranslationalVelConstraint(70));
               // .waitSeconds(0.5);

        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder driveToHighChamberSecondSample = drive.actionBuilder(new Pose2d(-42.0, 61, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(-3.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0));
                //.lineToY(60.00)
                //.waitSeconds(0.5);
        /**
         *Park
         */
        TrajectoryActionBuilder parkObservationZone = drive.actionBuilder(new Pose2d(-3.0, 48.0, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(-37.0, 53.0, Math.toRadians(270.0)), Math.toRadians(270.0))
                //.setTangent(270.0)
                .lineToY(60.0)
                .waitSeconds(0.15);

        /**
         * Line up to high chamber
         */
        TrajectoryActionBuilder driveToHighChamberThirdSample = drive.actionBuilder(new Pose2d(-37.0, 61.0, Math.toRadians(270.0)))
                .splineToLinearHeading(new Pose2d(-6.0, 49, Math.toRadians(90.0)), Math.toRadians(90.0));
        /**
         *Park
         */
        TrajectoryActionBuilder parkObservationZone2 = drive.actionBuilder(new Pose2d(-3.0, 48.0, Math.toRadians(90.0)))
                .splineToLinearHeading(new Pose2d(-37.0, 53.0, Math.toRadians(270.0)), Math.toRadians(270.0))
                //.setTangent(270.0)
                .lineToY(61.0)
                .waitSeconds(0.15);

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

        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(400);
        OuttakeSliders(-(HIGH_CHAMBER - 50), 0, 0);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         *Spline to First Yellow Sample
         */
        Actions.runBlocking(
                new SequentialAction(
                        firstSample.build()
                )
        );
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        IntakeClaw.setPosition(IntakeClawPositionOpen);
        intakeSlidersElbow(IntakeState.OUT);
        sleep(2500);
        IntakeClaw.setPosition(IntakeClawPositionClose);
        sleep(500);
        OuttakeElbowMove(OuttakeElbowPositionIn);
        sleep(500);
        intakeSlidersElbow(IntakeState.IN);
        sleep(1000);
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(2050);
        IntakeClaw.setPosition(IntakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        OuttakeSliders(HIGH_BASKET, 0 , 0);
        sleep(1500);



        /**
         * Score Hihg Basket first sample
         */

        Actions.runBlocking(
                new SequentialAction(
                        firstSampleScore.build()
                )
        );
        sleep(2500);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        sleep(2500);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        sleep(2500);
        OuttakeSliders(-(HIGH_BASKET-50),0,0);
        requestOpModeStop();
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
                        lineUpToSecondSpecimen.build()
                )
        );

        /**
         * Grab second Specimen from Wall and Prepare Elbow to Transport
         */
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(350);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         * Drive to align to the HIGH CHAMBER
         */
        OuttakeSliders(HIGH_CHAMBER,0,0);
        Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamberSecondSample.build()
                )
        );
        /**
         * Place second sample on the HIGH CHAMBER
         */
        //sleep(300);
        //OuttakeSliders(HIGH_CHAMBER,0,0);
        sleep(250);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(400);
        OuttakeSliders(-(HIGH_CHAMBER-500),0,0);
        sleep(350);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
        OuttakeElbowMove(OuttakeElbowPositionOutSpecimen);

        // OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        /**
         * Prepare for third specimen
         */
        Actions.runBlocking(
                new SequentialAction(
                        parkObservationZone2.build()
                )
        );
        //OuttakeElbowMove(OuttakeElbowPositionOut);
        /**
         * Grab second Specimen from Wall and Prepare Elbow to Transport
         */
       // OuttakeElbowMove(OuttakeElbowPositionOutSpecimen);

        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        sleep(350);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        /**
         * Drive to align to the HIGH CHAMBER
         */
        OuttakeSliders(HIGH_CHAMBER,0,0);

        Actions.runBlocking(
                new SequentialAction(
                        driveToHighChamberThirdSample.build()
                )
        );
        /**
         * Place second sample on the HIGH CHAMBER
         */

        //sleep(350);
        OuttakeElbowMove(OuttakeElbowPositionOut);
        sleep(400);
        OuttakeSliders(-(HIGH_CHAMBER),0,0);
        sleep(400);
        OuttakeElbowMove(OuttakeElbowPositionMiddle);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);

       // sleep(500);


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
        OuttakeSliderRight.setPower(SliderVelocity);
        OuttakeSliderLeft.setPower(SliderVelocity);
    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                sleep(500);
                IntakeClaw.setPosition(0.4);
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);

                break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                break;

        }

    }

}