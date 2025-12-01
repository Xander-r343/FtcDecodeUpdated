package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configs.Config;
@TeleOp
public class blackboardReader extends OpMode {
    Config config;
    @Override
    public void init() {
        config = new Config();
    }

    @Override
    public void loop() {
        telemetry.addData("x:", blackboard.get(config.Xkey));
        telemetry.addData("y:", blackboard.get(config.Ykey));
        telemetry.addData("h:", blackboard.get(config.HeadingKey));
        telemetry.addData("a:", blackboard.get(config.AllianceKey));
        telemetry.update();
    }
}
