# Setting Up A New (Swerve) Robot!!
#### Warning: The swerve libs are based off of CTRE's Libraries, which only support TalonFX-based motors and CANCoders with a Pigeon 2.
## Constants
The first things to do when setting up a new bot is to update the CAN ID's and Motor Offsets in the [Constants file](/src/main/java/frc/robot/Constants.java)
1. Set your CAN IDs to something logical in phoenix tuner.
2. Go into constants and find the lines that say "public static final CANIDs". Edit the constructor with your values. The order is "new CANIDs(\<drive motor id\>, \<steer motor id\>, \<CANCoder id\>);
3. enter the pigeon's can ID into the pigeonCANId constant
4. Measure the width and length of your robot and put those values into the Dimensions constructor
5. Enter your module's gear ratios into the Gearing constructor. There are predefined ratios for different SDS module drive ratios. Don't worry about the steer coupling ratio, it can stay 0.
6. Use a piece of extrusion that fits under your drivetrain to hold the wheels pointed perfectly forward. Use phoenix tuner to get the RAW absolute sensor position for each cancoder (WITHOUT the set offset). Enter this value NEGATED into the EncoderOffsets constructor

