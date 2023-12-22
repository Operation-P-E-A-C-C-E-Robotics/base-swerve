package frc.lib.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BuildTelemetry {
    public static NetworkTable buildTable = NetworkTableInstance.getDefault().getTable("Build");
    static {
        buildTable.getStringTopic("Project Name").publish().accept(BuildConstants.MAVEN_NAME);
        buildTable.getStringTopic("Build Date").publish().accept(BuildConstants.BUILD_DATE);
        buildTable.getStringTopic("Build Git Branch").publish().accept(BuildConstants.GIT_BRANCH);
        buildTable.getStringTopic("Build Git SHA").publish().accept(BuildConstants.GIT_SHA);
        buildTable.getStringTopic("Build Git Date").publish().accept(BuildConstants.GIT_DATE);
        buildTable.getIntegerTopic("Build Git Revision").publish().accept(BuildConstants.GIT_REVISION);
        buildTable.getStringTopic("Author").publish().accept("Sean lol (who else)");
    }
}
