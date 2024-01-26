/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
//@Disabled
public class SensorREV2mDistance extends LinearOpMode {

    private DistanceSensor sensorDistancePort;
    private DistanceSensor sensorDistanceStarboard;
    private ColorSensor colorPort;
    private ColorSensor colorStarboard;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorDistancePort = hardwareMap.get(DistanceSensor.class, "distancesensorport");
        sensorDistanceStarboard = hardwareMap.get(DistanceSensor.class, "distancesensorstarboard");

        colorPort = hardwareMap.get(ColorSensor.class, "colorsensorport");
        colorStarboard = hardwareMap.get(ColorSensor.class, "colorsensorport");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistancePort;
        Rev2mDistanceSensor sensorTimeOfFlightStarboard = (Rev2mDistanceSensor) sensorDistanceStarboard;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            //telemetry.addData("deviceName", sensorDistancePort.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", sensorDistancePort.getDistance(DistanceUnit.MM)));
            telemetry.addData("Port range", String.format("%.01f cm", sensorDistancePort.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", sensorDistancePort.getDistance(DistanceUnit.METER)));
            telemetry.addData("Port range", String.format("%.01f in", sensorDistancePort.getDistance(DistanceUnit.INCH)));

            //telemetry.addData("","");

            //telemetry.addData("deviceName", sensorDistanceStarboard.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", sensorDistanceStarboard.getDistance(DistanceUnit.MM)));
            telemetry.addData("Starboard range", String.format("%.01f cm", sensorDistanceStarboard.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", sensorDistanceStarboard.getDistance(DistanceUnit.METER)));
            telemetry.addData("Starboard range", String.format("%.01f in", sensorDistanceStarboard.getDistance(DistanceUnit.INCH)));

            telemetry.addData("","");

            telemetry.addData("Port Red", colorPort.red());
            telemetry.addData("Port Green", colorPort.green());
            telemetry.addData("Port Blue", colorPort.blue());

            telemetry.addData("","");

            telemetry.addData("Starboard Red", colorStarboard.red());
            telemetry.addData("Starboard Green", colorStarboard.green());
            telemetry.addData("Starboard Blue", colorStarboard.blue());

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}
