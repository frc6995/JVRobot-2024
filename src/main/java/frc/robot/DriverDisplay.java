package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class DriverDisplay {
    private DoublePublisher matchTimeEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/matchTime").publish();

    private BooleanPublisher seeNoteEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/seeNote").publish();
    private BooleanSupplier seeNoteSupplier = ()->false;
    public void setSeeNoteSupplier(BooleanSupplier seeNoteSupplier) {
        this.seeNoteSupplier = seeNoteSupplier;
    }
    public DriverDisplay() {}
    
    public void update() {
        matchTimeEntry.accept(DriverStation.getMatchTime());
        seeNoteEntry.accept(seeNoteSupplier.getAsBoolean());
    }
}
