import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestBenchColor;

@TeleOp
public class colorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    TestBenchColor color;
    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
    }
}
