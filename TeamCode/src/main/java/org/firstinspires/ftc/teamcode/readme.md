## Autonomie record & replay — Biobuzz

Implementarea pornește de la recorderul și replay-ul din `FTC_TestRobot`, adaptate
la `KronBot`, NextFTC, intake, transfer și autoaim-ul cu Limelight.

### OpMode-uri disponibile

Toate apar în grupul `KronBot`. Înregistrarea este un TeleOp, iar redarea este
un Autonomous.

| Alianță | TeleOp normal | Înregistrare | Autonomie | Fișier pe Control Hub |
| --- | --- | --- | --- | --- |
| Albastră | Driving - Blue Alliance | Recorder - Blue Alliance | Replay - Blue Alliance | `/sdcard/robot_recording_blue.csv` |
| Roșie | Driving - Red Alliance | Recorder - Red Alliance | Replay - Red Alliance | `/sdcard/robot_recording_red.csv` |

`BlueRecorderOp` moștenește `BlueAllianceTeleOp`, iar `RedRecorderOp` moștenește
`RedAllianceTeleOp`. Logica de condus este aceeași; recorderul adaugă salvarea
datelor și telemetria înregistrării. Configurațiile de coșuri și pipeline-uri
sunt comune în `utils/Alliance.java`, astfel încât TeleOp-ul și autonomia să
folosească aceleași setări ale alianței.

### Cum înregistrezi și redai

1. Așază robotul în poziția fizică de început și selectează recorderul alianței.
2. În INIT, alege poziția ca în TeleOp: D-pad stânga/dreapta pentru partea de
   start sau D-pad jos pentru ultima poziție salvată. INIT nu șterge înregistrarea
   precedentă.
3. Apasă PLAY și execută secvența dorită. Verifică `Recording: ACTIVE` și numărul
   de cadre din telemetrie. Cronometrul înregistrării pornește la PLAY.
4. Apasă STOP la sfârșitul secvenței. Motoarele se opresc, se închide fișierul
   temporar și se salvează înregistrarea. Există o singură înregistrare pe alianță:
   următoarea înregistrare salvată cu succes o înlocuiește pe cea anterioară.
5. Pune robotul înapoi în **aceeași poziție fizică și aceeași orientare**.
   Selectează replay-ul aceleiași alianțe. INIT afișează poziția înregistrată,
   numărul de cadre și durata; un fișier lipsă sau invalid apare ca eroare.
6. Apasă PLAY. Replay-ul setează originea odometriei la poziția primului cadru și
   execută secvența. La sfârșitul duratei înregistrate, oprește automat deplasarea,
   intake-ul și transferul. STOP întrerupe și el execuția. Poziția finală este
   salvată pentru TeleOp, iar TeleOp-ul alianței este preselectat.

Durata redată este durata integrală înregistrată; codul nu trunchiază fișierul
automat. Pentru o secvență de maximum 30 s, încheie înregistrarea înainte de 30 s.
Nu există oglindire automată a traseelor: fiecare alianță își folosește fișierul.

### Comenzile recorderului

| Comandă | Comportament, identic cu TeleOp-ul alianței |
| --- | --- |
| Stick stânga | Deplasare înainte/înapoi și lateral |
| Stick dreapta, axa X | Rotire când autoaim-ul este oprit |
| Triangle | Comută robot centric / field centric |
| R1 ținut | Intake și transfer înainte |
| L1 ținut | Intake și transfer invers; R1 are prioritate dacă ambele sunt apăsate |
| Eliberarea bumperelor | Oprește intake-ul și transferul |
| L2 peste 50% | Autoaim; coșul este ales automat după Y |
| L3 | Resetează poziția ca în TeleOp; evenimentul este salvat și reprodus |

### Ce se salvează și cum funcționează replay-ul

Formatul CSV are 18 coloane, definite în `utils/RecordingFormat.java`:

```text
Time,X,Y,Heading,Voltage,DriveFwd,DriveStr,DriveTurn,VelX,VelY,AngularVelocity,FieldCentric,AimingAtTarget,AimPipeline,IntakePower,TransferPower,PoseResetId,Alliance
```

Poziția este în inch, orientarea în radiani, vitezele în inch/s și rad/s, iar
puterile sunt în intervalul [-1, 1]. Sunt salvate comenzile efectiv trimise
de `DriveController`, după deadzone, viteza manuală și autoaim. Puterile
intake-ului și transferului sunt salvate separat, inclusiv semnul și oprirea.

Recorderul țintește un cadru la 20 ms, limitat de viteza reală a buclei robotului.
Schimbările de putere a mecanismelor, autoaim, pipeline, referință de condus și
resetările de poziție produc un cadru imediat, chiar dacă nu au trecut 20 ms.
Fișierul este golit din buffer la aproximativ 0,5 s. Se scrie mai întâi în
`robot_recording_<alianță>.csv.tmp`; fișierul final este înlocuit numai după
închiderea cu succes. La o eroare de scriere, se păstrează CSV-ul final anterior.
O oprire brutală a aplicației/alimentării poate lăsa fișierul `.tmp` nefinalizat.

La replay, ceasul selectează cadrele relevante, iar poziția, viteza și orientarea
sunt interpolate între ele. Orientarea folosește rotația cea mai scurtă când
trece prin ±π. Stările mecanismelor sunt menținute până la momentul schimbării;
nu sunt interpolate. Nici comenzile nu sunt interpolate peste schimbarea unei
stări discrete. Resetările L3 schimbă coordonatele la momentul înregistrat;
nu sunt interpretate ca deplasări fizice.

Comanda de deplasare combină:

- comanda înregistrată, ajustată după raportul dintre tensiunea înregistrată și
  cea curentă; factorul este limitat implicit la 0,90–1,10;
- corecția proporțională cu eroarea de poziție/orientare;
- corecția proporțională cu eroarea de viteză, filtrată pentru a reduce zgomotul.

Corecția de translație este limitată implicit la magnitudinea 0,35 în ambele
referințe de condus; corecția de rotire este limitată la ±0,30. Vectorul final
de translație este normalizat la maximum 1, iar rotirea este limitată la [-1, 1].

Când cadrul cere autoaim, replay-ul folosește `DriveController.getAimTurn()`:
același PID, aceleași coordonate ale coșurilor și aceeași alegere automată după Y
ca TeleOp-ul. Pipeline-ul urmărește coșul curent. O detecție Limelight validă este
folosită pentru țintire; altfel se folosește odometria. Astfel, țintirea este
recalculată în timpul autonomiei, pe lângă urmărirea orientării înregistrate.

CSV-urile vechi din `FTC_TestRobot` nu sunt compatibile: nu conțin stările
mecanismelor noi și folosesc altă schemă de țintire. Înregistrează traseele din
nou folosind recorderele Biobuzz. Încărcarea verifică antetul, toate valorile
numerice, limitele puterilor, alianța, pipeline-urile, ordinea timpilor și
secvența resetărilor înainte de inițializarea robotului pentru replay.

### Reglare și verificare

Coeficienții `REPLAY_*` se află în `utils/Constants.java` și sunt disponibili în
Panels. Valorile inițiale sunt cele din proiectul de referință; trebuie validate
pe șasiul nou. `REPLAY_TRANSLATION_KP/KD` corectează poziția și viteza liniară,
iar `REPLAY_ROTATION_KP/KD` corectează orientarea și viteza unghiulară.
Telemetria replay-ului afișează eroarea instantanee, medie și maximă de poziție,
eroarea de orientare și comenzile finale. PsiKit păstrează și erorile de replay.

Pentru prima probă pe robot, înregistrează o secvență scurtă cu mers înainte,
mers lateral, rotire, intake înainte/invers și eliberarea bumperelor. Testează
apoi separat autoaim-ul și trecerea dintre cele două coșuri. Păstrează aceeași
poziție de start și verifică odometria înainte de reglarea coeficienților.

Testele de pe calculator verifică schema CSV, datele invalide, tranzițiile,
unghiurile, transformarea coordonatelor, limitele corecției și o simulare
simplificată de urmărire. Ele nu măsoară precizia mecanică pe teren.

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

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
