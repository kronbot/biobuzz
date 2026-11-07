## Record & replay — Biobuzz

Implementarea urmează structura din `FTC_TestRobot`:

- `opmodes/teleop/RecorderOp.java`: moștenește clasa de condus, înregistrează direct
  CSV-ul și închide writer-ul la oprire.
- `opmodes/autonomous/ReplayOp.java`: `LinearOpMode` cu încărcare CSV, așteptare
  PLAY, bucla de interpolare/corecție din referință și oprire în `finally`.
- `utils/RecordingFormat.java`: antetul CSV, numele fișierelor și modurile de condus.

Cele patru clase Blue/Red Recorder/Replay sunt doar puncte de intrare în Driver
Station. Nu există `RecordingSession`, `ReplayController`, `Alliance`,
`InitializationState` sau `InitializationDiagnostics`.

| Alianță | Înregistrare | Redare | Fișier pe Control Hub |
| --- | --- | --- | --- |
| Albastră | Recorder - Blue Alliance | Replay - Blue Alliance | `/sdcard/robot_recording_blue.csv` |
| Roșie | Recorder - Red Alliance | Replay - Red Alliance | `/sdcard/robot_recording_red.csv` |

Directorul este `Constants.RECORDING_DIRECTORY`, implicit `/sdcard`. Numele sunt
`RecordingFormat.BLUE_FILE_NAME` și `RED_FILE_NAME`. Ambele opmode-uri folosesc
`RecordingFormat.file(...)` și afișează calea completă în telemetrie.

### Utilizare

1. Selectează recorderul alianței. La INIT se deschide direct CSV-ul final și se
   scrie antetul, ca în referință. **INIT începe o înregistrare nouă și înlocuiește
   fișierul anterior al aceleiași alianțe**, inclusiv dacă anulezi înainte de PLAY.
2. Selectează poziția cu D-pad stânga/dreapta sau jos pentru ultima poziție salvată.
3. Apasă PLAY și condu. Verifică `Recording: ACTIVE`, calea și numărul de cadre.
   Eșantionarea țintește 20 ms; schimbările de mecanisme, autoaim, Triangle și L3
   sunt salvate imediat. Bufferul este golit cel puțin o dată la 0,5 s.
4. Apasă STOP. Writer-ul este închis și mesajul devine `SAVED` sau `FAILED`.
   Fișierul există deja; nu se folosește `.tmp` și nu se redenumește nimic.
5. Pune robotul în aceeași poziție fizică și orientare, apoi selectează replay-ul
   aceleiași alianțe. Verifică numărul de cadre și durata, apoi apasă PLAY.

Dacă încărcarea CSV-ului sau inițializarea eșuează, replay-ul oprește motoarele
și păstrează pe ecran `Replay error` și calea fișierului până apeși STOP. PLAY nu
pornește un replay invalid. Eroarea completă este scrisă și în log cu tag-ul
`Replay`. Astfel, eroarea inițială nu mai este ascunsă de avertismentul SDK
`ended prematurely ... not monitoring for the start condition`.

Recorderul folosește exact comenzile TeleOp-ului corespunzător:
Triangle schimbă referința de condus, L2 peste 50% ține autoaim-ul activ,
R1/L1 țin intake-ul înainte/invers, iar eliberarea bumperelor îl oprește.
Transferul urmează bumperele numai când este montat și activat.
L3 resetează poziția; evenimentul este înregistrat și reprodus.

### Adaptări față de FTC_TestRobot

Primele 21 de coloane urmează formatul referinței. `AimingAtBlue` identifică
alianța recorderului; comenzile A/B sunt păstrate ca date, fără să adauge funcții
care nu există în TeleOp-ul Biobuzz. Sunt adăugate patru coloane:
`IntakePower,TransferPower,AimPipeline,PoseReset`.

Replay-ul folosește aceeași interpolare și aceiași coeficienți inițiali P/D din
referință; aceștia sunt declarați la începutul `ReplayOp.java`. Nu există un
controler de replay separat. Autoaim-ul folosește direct `DriveController`, cu
ținte după alianță, selecție automată după Y și Limelight cu revenire la odometrie.

Pe robotul actual, `Constants.TRANSFER_ENABLED = false`, deoarece motorul nu este
montat. Când îl montezi, configurează numele `transfer` și activează opțiunea
înainte de INIT. Un CSV care comandă transferul este refuzat dacă acesta este
oprit din configurația codului.

Fișierele vechi de 18 coloane din implementarea anterioară trebuie reînregistrate;
nu sunt interpretate ca noul format de 25 de coloane. Și traseele de 21 de coloane
din FTC_TestRobot trebuie reînregistrate pentru mecanismele Biobuzz.

TeleOp-urile și recorderele păstrează NextFTC și PsiKit, deja folosite de proiect.
Replay-ul are ciclul standard FTC `LinearOpMode`; bridge-ul PsiKit nu îl învelește.
În replay, cache-ul hub-urilor este pus în AUTO, inclusiv după un TeleOp care a
folosit cache MANUAL. Portul PsiKit rămâne 5801, separat de Limelight Proxy pe 5800.

### Verificare

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Testele verifică citirea fișierelor reale, formatul, ordinea cadrelor, datele
invalide, alegerea alianței și transferul opțional. Precizia pe teren se verifică
pe robot; compilarea și testele JVM nu pot confirma odometria sau alunecarea.

## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""
