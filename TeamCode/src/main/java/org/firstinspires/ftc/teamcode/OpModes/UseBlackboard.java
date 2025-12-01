package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Configs.Config;
@Autonomous
public class UseBlackboard extends OpMode {
    Object Xpos;
    Object Ypos;
    Object Headingpos;
    Object Alliance;
    Config config;
    @Override
    public void init() {
        config = new Config();
        Xpos = blackboard.getOrDefault(config.Xkey, 0);
        Ypos = blackboard.getOrDefault(config.Ykey, 0);
        Headingpos = blackboard.getOrDefault(config.HeadingKey, 0);
        Alliance = blackboard.getOrDefault(config.AllianceKey, 0);//isRedValue
        blackboard.put(config.AllianceKey, config.RedAlliance);
        blackboard.put(config.Xkey, 72.0);
        blackboard.put(config.Ykey, 9.0);
        blackboard.put(config.HeadingKey, 0.0);


    }

    @Override
    public void loop() {

    }
}
