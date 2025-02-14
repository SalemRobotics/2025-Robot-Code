package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Scanner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BatteryScannerConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class BatteryScanner extends SubsystemBase {
    private String prev;
    private final StringSubscriber subscriber;
    /*
     * - Function that gets the current battery & compares it against the previous
     * found in kBatteryFile
     * - Helper function that gets the ID of the previous battery
     * - If the IDs match, then handle the error somehow (LEDs, notification, etc?)
     */

    public BatteryScanner() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable dataTable = inst.getTable("team6324");
        subscriber = dataTable.getStringTopic("batteryID").subscribe("");
    }

    @Override
    public void periodic() {
        if (!prev.equals(""))
            return;

        String value = subscriber.get();
        if (value.equals(prev)) // using a guard clause here
            return;

        prev = value;
        String previousID = getPreviousID();
        if (value.equals(previousID)) {
            Notification alert = new Notification();

            Elastic.sendNotification(
                    alert.withLevel(NotificationLevel.ERROR)
                            .withTitle("Battery re-use")
                            .withDescription(
                                    "The current battery on the robot (" + prev
                                            + ") is identical to the previously used battery")
                            .withDisplayMilliseconds(10_000));
        }

    }

    private String getPreviousID() {
        try {
            byte[] bytes = Files.readAllBytes(Paths.get(BatteryScannerConstants.kBatteryFile));
            return new String(bytes, Charset.defaultCharset());
        } catch (Exception e) {
            return "";
        }
    }

}
