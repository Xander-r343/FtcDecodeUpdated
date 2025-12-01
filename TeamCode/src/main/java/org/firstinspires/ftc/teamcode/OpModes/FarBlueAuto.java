package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Configs.RedAutoPaths;
import org.firstinspires.ftc.teamcode.Sensors.OdoPods;
import org.firstinspires.ftc.teamcode.Subsystems.Aimbots;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
@Autonomous(name = "9 ball auto blue")
public class FarBlueAuto extends LinearOpMode {

    Servo rgb;
    ShooterSubsystem robotSubsystem;
    OdoPods pods;
    MecanumDrivetrain drivetrain;
    RedAutoPaths pathDatabase;
    Object Xpos;
    Object Ypos;
    Object Headingpos;
    Object Alliance;
    Config config;
    int AutoState;
    ElapsedTime timer;
    double speed;
    double time;
    Aimbots aimbots;
    private ElapsedTime runtime = new ElapsedTime();
    int tagID;
    int flywheelSpeed;
    double driveSpeed;
    double headingSpeed;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(1, hardwareMap);
        pods = new OdoPods(hardwareMap, drivetrain);
        pathDatabase = new RedAutoPaths();
        config = new Config();
        robotSubsystem = new ShooterSubsystem(hardwareMap);
        //intialize blackboard objects
        Xpos = blackboard.getOrDefault(config.Xkey, 0);
        Ypos = blackboard.getOrDefault(config.Ykey, 0);
        Headingpos = blackboard.getOrDefault(config.HeadingKey, 0);
        Alliance = blackboard.getOrDefault(config.AllianceKey, 0);//isRedValue
        blackboard.put(config.AllianceKey, config.BlueAlliance);
        AutoState = 0;
        //initialize odopods by using the config class
        pods.setPosition(56, 9, 0);
        timer = new ElapsedTime();
        AutoState = 0;
        aimbots = new Aimbots(config.BlueAlliance, pods, hardwareMap);
        rgb = hardwareMap.get(Servo.class, config.RBGName);
        flywheelSpeed = 2650;
        robotSubsystem.setServoPosition(0.5);
        rgb.setPosition(config.Green);
        driveSpeed = 0.4;
        waitForStart();
        headingSpeed = 0.5;
        runtime.reset();
        timer.reset();
        while(opModeIsActive()){
            aimbots.update();
            switch (AutoState){
                case 0:{
                    robotSubsystem.setServoPosition(0);
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    aimbots.startLL();
                    aimbots.switchPipeline(1);
                    robotSubsystem.spinIntake(1);
                    timer.startTime();
                    //go to launch position and use aimbots to align to goal
                    pods.holdPosition(56, 90, 0, 0.8);
                    pods.update();
                    //start flywheel early to get up to speed and keep it
                    if (timer.seconds() < 2) {
                        pods.holdPosition(56, 90, 0, 0.8);
                        pods.update();
                    }
                    if(timer.seconds() > 2){
                        AutoState = 1;
                    }
                }
                break;
                case 1:{
                    //align for 1 secs, ended with 2
                    while(timer.seconds() < 3.2) {
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        /*if (aimbots.LLstatusIsValid()) {
                            targetH = pods.getHeading() - aimbots.getHeadingErrorLL();
                        } else {*/
                            targetH = aimbots.getIdealAngle()-9;
                       //x   }
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 3.1){
                            AutoState = 2;
                        }
                    }

                }
                break;
                case 2:{
                    double speed = 0.4;
                    robotSubsystem.setServoPosition(1);
                    //fire the first 3 artifacts
                    while(timer.seconds() < 7) {
                        robotSubsystem.spinIntake(speed);
                        robotSubsystem.spinBelt(0.45);

                        if(timer.seconds() > 6.8){
                            shutOff();
                            robotSubsystem.setServoPosition(0);
                            AutoState = 3;
                            robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                            robotSubsystem.spinIntake(1);
                        }
                        else if(timer.seconds() > 4 && timer.seconds() < 5){
                            robotSubsystem.setServoPosition(0);
                        }
                        else if(timer.seconds() > 5 && timer.seconds() < 6.7){
                            robotSubsystem.setServoPosition(1);
                        }
                    }
                }
                break;
                case 3:{
                    pods.holdHeading(-90,0.5);
                    pods.update();
                    //strafe to align with pickup balls
                    while(timer.seconds() < 8.5){
                        pods.holdPosition(40,73,-90,1);
                        pods.update();
                    }
                    //speed up flywheel early
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(0.4);
                    timer.reset();
                    timer.startTime();
                    //pickup 2 more balls from far pickup
                    while(timer.seconds() < 2.1){
                        //adjust this 115 x for 3rd ball if necessary
                        pods.holdPosition(5,71.5,-90,0.5);
                        pods.update();
                    }
                    while(timer.seconds() < 2.8) {
                        //move back to fire
                        pods.holdPosition(62, 84, -90, 0.7);
                        pods.update();
                        if(timer.seconds() > 2.7){
                            AutoState = 4;
                        }
                    }
                    robotSubsystem.spinIntake(1);
                }
                break;
                case 4:{
                    //align to goal using limelight
                    while(timer.seconds() < 4) {
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        /*if (aimbots.LLstatusIsValid()) {
                            targetH = pods.getHeading() - aimbots.getHeadingErrorLL();
                        } else {*//*
                        }
                       */
                        targetH = aimbots.getIdealAngle()-5;
                        pods.holdHeading(targetH, 0.8);
                        aimbots.update();
                    }
                    //fire balls
                    while(timer.seconds() < 7) {
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.4);
                        if(timer.seconds() > 6.9){
                            shutOff();
                            AutoState = 5;
                            robotSubsystem.setServoPosition(0);
                        }
                    }
                }
                break;
                case 5:{
                    while(timer.seconds() < 8.2) {
                        pods.holdHeading(-90, 0.8);
                        pods.update();
                    }
                    robotSubsystem.spinIntake(1);
                    //prepare for another pickup
                    while(timer.seconds() < 10){
                        pods.holdPosition(38,46,-90,0.9);
                        pods.update();
                    }
                    AutoState = 6;
                }
                break;
                case 6:{
                    timer.reset();
                    timer.startTime();
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(0.6);
                    while(timer.seconds() < 2){
                        pods.holdPosition(-2, 47, -90, 0.45);
                        pods.update();
                    }
                    AutoState = 7;
                }
                break;
                case 7: {
                    while (timer.seconds() < 3.8) {
                        pods.holdPosition(40, 84, -90, 1);
                        pods.update();
                    }
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    while(timer.seconds() < 4.8){
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        /*if (aimbots.LLstatusIsValid()) {
                            //targetH = pods.getHeading() - aimbots.getHeadingErrorLL();

                        } else {*/
                            targetH = aimbots.getIdealAngle()-8;
                        //}
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 3.9){
                            AutoState = 8;

                        }
                    }
                }
                break;
                case 8:{
                    while(timer.seconds() < 7.4) {
                        robotSubsystem.spinIntake(0.4);
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.4);
                    }
                    pods.holdPosition(15, 80, -90, 1);
                    pods.update();

                }
                break;
            }
            blackboard.put(config.Xkey, pods.getX());
            blackboard.put(config.Ykey, pods.getY());
            blackboard.put(config.HeadingKey, pods.getHeading());
            telemetry.addData("x", pods.getX());
            telemetry.addData("y", pods.getY());
            telemetry.addData("h", pods.getHeading());
            telemetry.addData("I see apriltag: ", aimbots.LLstatusIsValid());
            telemetry.update();
        }
        if(isStopRequested()){
            blackboard.put(config.Xkey, pods.getX());
            blackboard.put(config.Ykey, pods.getY());
            blackboard.put(config.HeadingKey, pods.getHeading());
        }
    }

    public void resetArtifacts(){
        robotSubsystem.setFlywheelVelocity(-300);
        robotSubsystem.spinBelt(-0.2);
        robotSubsystem.spinIntake(1);
    }
    public void shutOff(){
        robotSubsystem.spinBelt(0);
    }
}
