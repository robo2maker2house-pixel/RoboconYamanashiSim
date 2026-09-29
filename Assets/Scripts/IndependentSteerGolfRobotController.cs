using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// 「ぺろりん」用のキーボード操作プログラムです。
/// このコンポーネントは、インポートしたURDFのルートへ追加してください。
/// </summary>
public class IndependentSteerGolfRobotController : MonoBehaviour
{
    // 10 bit ADC: 0～1023の1024段階。
    private const int CatchAnalogMinimum = 0;
    private const int CatchAnalogMaximum = 1023;

    // ---------------------------------------------------------------------
    // Inspectorから調整する値
    // ---------------------------------------------------------------------
    [Header("走行")]
    [SerializeField] private float maximumLinearSpeed = 0.09f;
    [SerializeField] private float maximumYawSpeed = 0.7f;
    [SerializeField] private float linearAcceleration = 0.25f;
    [SerializeField] private float yawAcceleration = 1.5f;
    [SerializeField] private float wheelRadius = 0.03f;
    [Tooltip("モデル正面とUnityの青いZ軸が異なる場合に調整します。")]
    [SerializeField] private float forwardYawOffsetDegrees;

    [Header("ステアリング・車輪")]
    [SerializeField] private float steeringStiffness = 250f;
    [SerializeField] private float steeringDamping = 25f;
    [SerializeField] private float steeringForceLimit = 20f;
    [SerializeField] private float wheelDamping = 5f;
    [Tooltip("20:40歯車後の車輪側駆動力として調整します。")]
    [SerializeField] private float wheelForceLimit = 10f;
    [Tooltip("20:40減速後の目視確認用上限。180 deg/s = 30 rpmです。")]
    [SerializeField] private float maximumWheelSpeedDegrees = 180f;

    [Header("ステアリング初期角度・ゼロ点補正")]
    [Tooltip("中央後輪の直進時角度です。")]
    [SerializeField] private float centerRearSteeringOffsetDegrees = 90f;
    [Tooltip("前方左輪の直進時角度です。")]
    [SerializeField] private float frontLeftSteeringOffsetDegrees = -30f;
    [Tooltip("前方右輪の直進時角度です。")]
    [SerializeField] private float frontRightSteeringOffsetDegrees = 30f;

    [Header("カメラ・目")]
    [SerializeField] private float cameraSpeedDegrees = 60f;
    [SerializeField] private float cameraLimitDegrees = 80f;
    [Tooltip("カメラ歯車は1:1なので通常は1です。")]
    [SerializeField] private float cameraDriveGearRatio = 1f;
    [SerializeField] private float eyeSpeedDegrees = 45f;
    [SerializeField] private float eyeLimitDegrees = 30f;
    [Tooltip("サーボ側歯車角 / 目側歯車角。歯数が判明したら設定します。")]
    [SerializeField] private float eyeDriveGearRatio = 1f;

    [Header("クラブ・キャッチのレール位置")]
    [Tooltip("クラブの最前端。part_334から求めた円弧上の初期位置です。")]
    [SerializeField] private float clubFrontDegrees = 0f;
    [Tooltip("キャッチ最上部。アナログ値0に対応します。")]
    [SerializeField] private float catchTopDegrees = 0f;
    [Tooltip("キャッチ最下部。アナログ値1023に対応し、クラブを捕捉します。")]
    [SerializeField] private float catchBottomDegrees = 90f;
    [Tooltip("クラブを引き上げられる最大角度です。")]
    [SerializeField] private float maximumClubPullDegrees = -75f;
    [SerializeField] private float clawOpenDegrees = 50f;
    [SerializeField] private float railRadius = 0.221f;
    [SerializeField] private float bearingRadius = 0.005f;

    [Header("キャッチ手動操作（10 bit / 1024段階）")]
    [Tooltip("I/Kキーで1秒間に変化するアナログ値です。")]
    [SerializeField] private float catchAnalogSpeedPerSecond = 512f;
    [Tooltip("この値以上まで下げると、爪が閉じている場合にクラブを捕捉します。")]
    [SerializeField, Range(0, 1023)] private int captureAnalogThreshold = 1000;
    [SerializeField] private float clawReleaseSeconds = 0.12f;
    [SerializeField] private float clawCloseDelaySeconds = 0.35f;
    [Tooltip("自由可動中の爪に与える、ごく小さい回転抵抗です。")]
    [SerializeField] private float passiveClawDamping = 0.03f;
    [Tooltip("自由可動中の爪の抵抗トルク上限です。0に近いほど軽く動きます。")]
    [SerializeField] private float passiveClawForceLimit = 0.02f;

    [Header("定荷重ばね（左右2本）")]
    [Tooltip("ばね1本がクラブをレール接線方向へ引く力です。実測値があれば置き換えてください。")]
    [SerializeField] private float constantForcePerSpringNewtons = 3f;
    [Tooltip("一定力を作る速度ドライブの目標速度です。実速度は力上限で決まります。")]
    [SerializeField] private float springReturnVelocityDegrees = 360f;
    [SerializeField] private float springDriveDamping = 4f;
    [Tooltip("base_link_visual_245/247 のばね巻取り半径です。")]
    [SerializeField] private float springReelRadius = 0.01f;

    [Header("表示")]
    [SerializeField] private bool showOperationGuide = true;
    [SerializeField, HideInInspector] private int configurationVersion;

    private GUIStyle operationGuideStyle;
    private readonly GUIContent operationGuideContent = new GUIContent();

    private ArticulationBody rootBody;
    private ArticulationBody cameraTilt;
    private ArticulationBody cameraDriveGear;
    private ArticulationBody leftEyeOutput;
    private ArticulationBody leftEyeDriveGear;
    private ArticulationBody rightEyeOutput;
    private ArticulationBody rightEyeDriveGear;
    private ArticulationBody rightClubSpringReel;
    private ArticulationBody leftClubSpringReel;
    private ArticulationBody tongueClub;
    private ArticulationBody catchCarriage;
    private ArticulationBody catchClaw;
    private readonly List<ArticulationBody> tongueBearings = new List<ArticulationBody>();
    private readonly List<ArticulationBody> catchBearings = new List<ArticulationBody>();

    private readonly List<WheelModule> wheelModules = new List<WheelModule>();

    private Vector2 translationInput;
    private float yawInput;
    private Vector2 currentTranslation;
    private float currentYawSpeed;
    private float cameraTarget;
    private float eyeTarget;
    private float catchAnalogTarget;
    private float potentiometerValue;
    private Coroutine clubRoutine;
    private bool clubIsCaptured;
    private bool clawIsOpen;
    private bool initialized;

    private sealed class WheelModule
    {
        public ArticulationBody Steering;
        public ArticulationBody Wheel;
        public float LastSteeringTarget;
        public float WheelDirection = 1f;
        public float SteeringOffsetDegrees;
    }

    private void OnValidate()
    {
        UpgradeSerializedSettings();
    }

    private void Awake()
    {
        UpgradeSerializedSettings();
        DisableConflictingControllers();
        initialized = FindRobotBodies();

        if (!initialized)
        {
            enabled = false;
            return;
        }

        ConfigureRobotDrives();
        ResetMechanismsImmediately();
    }

    private void UpgradeSerializedSettings()
    {
        // 古いScene/Prefabに保存された設定を、v7の初期値へ一度だけ更新します。
        if (configurationVersion >= 7)
            return;

        maximumLinearSpeed = 0.09f;
        maximumYawSpeed = 0.7f;
        linearAcceleration = 0.25f;
        yawAcceleration = 1.5f;
        maximumWheelSpeedDegrees = 180f;
        centerRearSteeringOffsetDegrees = 90f;
        frontLeftSteeringOffsetDegrees = -30f;
        frontRightSteeringOffsetDegrees = 30f;
        eyeSpeedDegrees = 45f;
        eyeLimitDegrees = 30f;
        clubFrontDegrees = 0f;
        catchTopDegrees = 0f;
        catchBottomDegrees = 90f;
        maximumClubPullDegrees = -75f;
        catchAnalogSpeedPerSecond = 512f;
        captureAnalogThreshold = 1000;
        clawReleaseSeconds = 0.12f;
        clawCloseDelaySeconds = 0.35f;
        passiveClawDamping = 0.03f;
        passiveClawForceLimit = 0.02f;
        railRadius = 0.221f;
        bearingRadius = 0.005f;
        constantForcePerSpringNewtons = 3f;
        springReturnVelocityDegrees = 360f;
        springDriveDamping = 4f;
        springReelRadius = 0.01f;
        configurationVersion = 7;
    }

    private void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
            return;

        // -----------------------------------------------------------------
        // 走行入力
        // -----------------------------------------------------------------
        translationInput = Vector2.zero;
        yawInput = 0f;

        // Imported model orientation: its visual front is opposite Unity local +Z.
        if (keyboard.wKey.isPressed) translationInput.y -= 1f;
        if (keyboard.sKey.isPressed) translationInput.y += 1f;
        // The CAD model's lateral axis is opposite Unity local X.
        if (keyboard.dKey.isPressed) translationInput.x -= 1f;
        if (keyboard.aKey.isPressed) translationInput.x += 1f;
        if (keyboard.eKey.isPressed) yawInput += 1f;
        if (keyboard.qKey.isPressed) yawInput -= 1f;

        translationInput = Vector2.ClampMagnitude(translationInput, 1f);

        // -----------------------------------------------------------------
        // カメラ・目の入力
        // -----------------------------------------------------------------
        float cameraInput = 0f;
        if (keyboard.upArrowKey.isPressed) cameraInput += 1f;
        if (keyboard.downArrowKey.isPressed) cameraInput -= 1f;
        cameraTarget = Mathf.Clamp(
            cameraTarget + cameraInput * cameraSpeedDegrees * Time.deltaTime,
            -cameraLimitDegrees,
            cameraLimitDegrees);

        float eyeInput = 0f;
        if (keyboard.rightArrowKey.isPressed) eyeInput += 1f;
        if (keyboard.leftArrowKey.isPressed) eyeInput -= 1f;
        eyeTarget = Mathf.Clamp(
            eyeTarget + eyeInput * eyeSpeedDegrees * Time.deltaTime,
            -eyeLimitDegrees,
            eyeLimitDegrees);

        SetPositionTarget(cameraTilt, cameraTarget);
        SetPositionTarget(cameraDriveGear, -cameraTarget * cameraDriveGearRatio);
        SetPositionTarget(leftEyeOutput, eyeTarget);
        SetPositionTarget(rightEyeOutput, eyeTarget);
        SetPositionTarget(leftEyeDriveGear, -eyeTarget * eyeDriveGearRatio);
        SetPositionTarget(rightEyeDriveGear, -eyeTarget * eyeDriveGearRatio);

        // -----------------------------------------------------------------
        // クラブキャッチの手動位置入力
        // I: 上へ（アナログ値を減らす） / K: 下へ（アナログ値を増やす）
        // -----------------------------------------------------------------
        if (clubRoutine == null)
        {
            float catchInput = 0f;
            if (keyboard.iKey.isPressed) catchInput -= 1f;
            if (keyboard.kKey.isPressed) catchInput += 1f;

            catchAnalogTarget = Mathf.Clamp(
                catchAnalogTarget + catchInput * catchAnalogSpeedPerSecond * Time.deltaTime,
                CatchAnalogMinimum,
                CatchAnalogMaximum);

            float catchTargetDegrees = AnalogToCatchDegrees(catchAnalogTarget);
            SetPositionTarget(catchCarriage, catchTargetDegrees);

            // 捕捉後は、キャッチを上げた量だけクラブもレール上で引き上げます。
            if (clubIsCaptured)
            {
                float clubTargetDegrees = clubFrontDegrees +
                    (catchTargetDegrees - catchBottomDegrees);
                clubTargetDegrees = Mathf.Clamp(
                    clubTargetDegrees,
                    Mathf.Min(maximumClubPullDegrees, clubFrontDegrees),
                    clubFrontDegrees);
                SetPositionTarget(tongueClub, clubTargetDegrees);
            }
        }

        // Spaceはキャッチを自動降下させません。現在位置で爪だけを開放します。
        if (keyboard.spaceKey.wasPressedThisFrame && clubRoutine == null)
        {
            if (clubIsCaptured)
                clubRoutine = StartCoroutine(ReleaseClub());
            else
                Debug.LogWarning("クラブ未捕捉です。Kキーでキャッチを最下部まで下げてください。", this);
        }

        // -----------------------------------------------------------------
        // リセット・表示
        // -----------------------------------------------------------------
        if (keyboard.rKey.wasPressedThisFrame)
        {
            if (clubRoutine != null)
                StopCoroutine(clubRoutine);

            clubRoutine = null;
            ResetMechanismsImmediately();
        }

        if (keyboard.f1Key.wasPressedThisFrame)
            showOperationGuide = !showOperationGuide;
    }

    private void FixedUpdate()
    {
        if (!initialized || rootBody == null)
            return;

        Vector2 desiredTranslation = translationInput * maximumLinearSpeed;
        currentTranslation = Vector2.MoveTowards(
            currentTranslation,
            desiredTranslation,
            linearAcceleration * Time.fixedDeltaTime);

        currentYawSpeed = Mathf.MoveTowards(
            currentYawSpeed,
            yawInput * maximumYawSpeed,
            yawAcceleration * Time.fixedDeltaTime);

        float clubDegrees = ReadJointDegrees(tongueClub);
        UpdateBearingTargets(tongueBearings, clubDegrees);
        UpdateBearingTargets(catchBearings, ReadJointDegrees(catchCarriage));
        UpdateSpringReelTargets(clubDegrees);

        potentiometerValue = ReadPotentiometerValue();
        TryCaptureClubAtBottom();

        DriveWheelModules(currentTranslation, currentYawSpeed);
        MoveArticulationRoot(currentTranslation, currentYawSpeed);
    }

    private bool FindRobotBodies()
    {
        Dictionary<string, ArticulationBody> bodies = new Dictionary<string, ArticulationBody>();
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>(true))
        {
            bodies[body.gameObject.name] = body;
            if (body.isRoot || body.gameObject.name == "base_link")
                rootBody = body;
        }

        // CAD上のfrontは実機の中央後輪、rear_left/rightは前方左右輪です。
        // 下記3値をInspectorへ出しているため、実機との差を個別調整できます。
        AddWheelModule(
            bodies,
            "front_steer_link",
            "front_wheel_link",
            centerRearSteeringOffsetDegrees);
        AddWheelModule(
            bodies,
            "rear_left_steer_link",
            "rear_left_wheel_link",
            frontLeftSteeringOffsetDegrees);
        AddWheelModule(
            bodies,
            "rear_right_steer_link",
            "rear_right_wheel_link",
            frontRightSteeringOffsetDegrees);

        bodies.TryGetValue("camera_tilt_link", out cameraTilt);
        bodies.TryGetValue("camera_drive_gear_link", out cameraDriveGear);
        bodies.TryGetValue("left_eye_output_link", out leftEyeOutput);
        bodies.TryGetValue("left_eye_drive_gear_link", out leftEyeDriveGear);
        bodies.TryGetValue("right_eye_output_link", out rightEyeOutput);
        bodies.TryGetValue("right_eye_drive_gear_link", out rightEyeDriveGear);
        bodies.TryGetValue("right_club_spring_reel_link", out rightClubSpringReel);
        bodies.TryGetValue("left_club_spring_reel_link", out leftClubSpringReel);
        bodies.TryGetValue("tongue_club_link", out tongueClub);
        bodies.TryGetValue("catch_carriage_link", out catchCarriage);
        bodies.TryGetValue("catch_claw_link", out catchClaw);

        for (int index = 0; index < 6; index++)
        {
            if (bodies.TryGetValue($"tongue_bearing_{index}_link", out ArticulationBody tongueBearing))
                tongueBearings.Add(tongueBearing);
            if (bodies.TryGetValue($"catch_bearing_{index}_link", out ArticulationBody catchBearing))
                catchBearings.Add(catchBearing);
        }

        if (rootBody == null || wheelModules.Count != 3)
        {
            Debug.LogError(
                "ロボットのArticulationBodyが見つかりません。" +
                "このスクリプトを independent_steer_golf_robot のルートへ追加してください。",
                this);
            return false;
        }

        return true;
    }

    private void AddWheelModule(
        Dictionary<string, ArticulationBody> bodies,
        string steeringName,
        string wheelName,
        float steeringOffsetDegrees)
    {
        if (!bodies.TryGetValue(steeringName, out ArticulationBody steering) ||
            !bodies.TryGetValue(wheelName, out ArticulationBody wheel))
        {
            Debug.LogWarning($"車輪ユニット {steeringName} / {wheelName} が見つかりません。", this);
            return;
        }

        wheelModules.Add(new WheelModule
        {
            Steering = steering,
            Wheel = wheel,
            LastSteeringTarget = steeringOffsetDegrees,
            SteeringOffsetDegrees = steeringOffsetDegrees
        });
    }

    private void ConfigureRobotDrives()
    {
        foreach (WheelModule module in wheelModules)
        {
            ConfigurePositionDrive(
                module.Steering,
                steeringStiffness,
                steeringDamping,
                steeringForceLimit);
            SetPositionTarget(module.Steering, module.SteeringOffsetDegrees);

            ArticulationDrive wheelDrive = module.Wheel.xDrive;
            wheelDrive.stiffness = 0f;
            wheelDrive.damping = wheelDamping;
            wheelDrive.forceLimit = wheelForceLimit;
            module.Wheel.xDrive = wheelDrive;
        }

        ConfigurePositionDrive(cameraTilt, 80f, 10f, 5f);
        ConfigurePositionDrive(cameraDriveGear, 40f, 6f, 2f);
        ConfigurePositionDrive(leftEyeOutput, 60f, 8f, 3f);
        ConfigurePositionDrive(rightEyeOutput, 60f, 8f, 3f);
        ConfigurePositionDrive(leftEyeDriveGear, 30f, 5f, 1f);
        ConfigurePositionDrive(rightEyeDriveGear, 30f, 5f, 1f);
        ConfigurePositionDrive(rightClubSpringReel, 20f, 2f, 0.10f);
        ConfigurePositionDrive(leftClubSpringReel, 20f, 2f, 0.10f);
        ConfigureClubSpringDrive();
        ConfigurePositionDrive(catchCarriage, 150f, 18f, 20f);
        ConfigureClawPassiveDrive();

        foreach (ArticulationBody bearing in tongueBearings)
            ConfigurePositionDrive(bearing, 8f, 1f, 0.08f);
        foreach (ArticulationBody bearing in catchBearings)
            ConfigurePositionDrive(bearing, 8f, 1f, 0.08f);
    }

    private static void ConfigurePositionDrive(
        ArticulationBody body,
        float stiffness,
        float damping,
        float forceLimit)
    {
        if (body == null)
            return;

        ArticulationDrive drive = body.xDrive;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        drive.targetVelocity = 0f;
        body.xDrive = drive;
    }

    private void ConfigureClubSpringDrive()
    {
        if (tongueClub == null)
            return;

        // The two spring forces act tangentially to the circular rail. Their
        // combined linear force therefore becomes an almost constant torque.
        float constantTorque =
            2f * Mathf.Max(0f, constantForcePerSpringNewtons) * Mathf.Max(railRadius, 0.001f);
        ArticulationDrive drive = tongueClub.xDrive;
        drive.stiffness = 0f;
        drive.damping = Mathf.Max(0.01f, springDriveDamping);
        drive.forceLimit = Mathf.Max(0.01f, constantTorque);
        drive.target = clubFrontDegrees;
        drive.targetVelocity = Mathf.Abs(springReturnVelocityDegrees);
        tongueClub.xDrive = drive;
        tongueClub.WakeUp();
    }

    private void ConfigureClubPullDrive()
    {
        ConfigurePositionDrive(tongueClub, 420f, 24f, 45f);
    }

    private void ConfigureClawPassiveDrive()
    {
        if (catchClaw == null)
            return;

        // 平常時は位置目標を持たせません。地面やクラブに触れると、爪が
        // 関節軸まわりに押し上げられ、その後は自重で下へ戻ります。
        ArticulationDrive drive = catchClaw.xDrive;
        drive.stiffness = 0f;
        drive.damping = Mathf.Max(0f, passiveClawDamping);
        drive.forceLimit = Mathf.Max(0f, passiveClawForceLimit);
        drive.targetVelocity = 0f;
        catchClaw.xDrive = drive;
        catchClaw.WakeUp();
    }

    private void ConfigureClawLiftDrive()
    {
        // クラブ開放時だけサーボを有効にし、爪を上位置へ固定します。
        ConfigurePositionDrive(catchClaw, 80f, 8f, 5f);
        SetPositionTarget(catchClaw, clawOpenDegrees);
    }

    private void DriveWheelModules(Vector2 translation, float yawSpeed)
    {
        bool hasCommand = translation.sqrMagnitude > 0.0001f || Mathf.Abs(yawSpeed) > 0.001f;

        foreach (WheelModule module in wheelModules)
        {
            Vector3 localPosition = rootBody.transform.InverseTransformPoint(
                module.Steering.transform.position);

            // Vector2 uses X = robot right, Y = robot forward.
            Vector2 moduleVelocity = translation + new Vector2(
                yawSpeed * localPosition.z,
                -yawSpeed * localPosition.x);

            if (hasCommand && moduleVelocity.sqrMagnitude > 0.0001f)
            {
                float requestedAngle =
                    Mathf.Atan2(moduleVelocity.x, moduleVelocity.y) * Mathf.Rad2Deg +
                    module.SteeringOffsetDegrees;
                requestedAngle = OptimizeSteeringAngle(
                    requestedAngle,
                    module.LastSteeringTarget,
                    out float wheelDirection);

                module.LastSteeringTarget = Mathf.Clamp(requestedAngle, -90f, 90f);
                module.WheelDirection = wheelDirection;
                SetPositionTarget(module.Steering, module.LastSteeringTarget);

                float linearSpeed = moduleVelocity.magnitude * module.WheelDirection;
                float wheelDegreesPerSecond = linearSpeed / Mathf.Max(wheelRadius, 0.001f) * Mathf.Rad2Deg;
                SetVelocityTarget(
                    module.Wheel,
                    Mathf.Clamp(wheelDegreesPerSecond, -maximumWheelSpeedDegrees, maximumWheelSpeedDegrees));
            }
            else
            {
                SetVelocityTarget(module.Wheel, 0f);
            }
        }
    }

    private static float OptimizeSteeringAngle(
        float requestedAngle,
        float currentAngle,
        out float wheelDirection)
    {
        requestedAngle = Mathf.DeltaAngle(0f, requestedAngle);
        float directDifference = Mathf.Abs(Mathf.DeltaAngle(currentAngle, requestedAngle));
        float reversedAngle = Mathf.DeltaAngle(0f, requestedAngle + 180f);
        float reversedDifference = Mathf.Abs(Mathf.DeltaAngle(currentAngle, reversedAngle));

        if (reversedDifference < directDifference)
        {
            wheelDirection = -1f;
            return reversedAngle;
        }

        wheelDirection = 1f;
        return requestedAngle;
    }

    private void MoveArticulationRoot(Vector2 translation, float yawSpeed)
    {
        rootBody.WakeUp();

        Quaternion offset = Quaternion.AngleAxis(forwardYawOffsetDegrees, Vector3.up);
        Vector3 forward = Vector3.ProjectOnPlane(offset * rootBody.transform.forward, Vector3.up).normalized;
        Vector3 right = Vector3.ProjectOnPlane(offset * rootBody.transform.right, Vector3.up).normalized;
        Vector3 desiredHorizontalVelocity =
            forward * translation.y + right * translation.x;

        Vector3 velocity = rootBody.linearVelocity;
        rootBody.linearVelocity = new Vector3(
            desiredHorizontalVelocity.x,
            velocity.y,
            desiredHorizontalVelocity.z);

        Vector3 angularVelocity = rootBody.angularVelocity;
        angularVelocity.y = yawSpeed;
        rootBody.angularVelocity = angularVelocity;
    }

    // ---------------------------------------------------------------------
    // クラブ捕捉・開放
    // ---------------------------------------------------------------------

    private void TryCaptureClubAtBottom()
    {
        if (clubIsCaptured || clawIsOpen || clubRoutine != null)
            return;

        // キャッチを最下部まで手動で下げると、自由可動爪がクラブに掛かります。
        if (potentiometerValue < captureAnalogThreshold)
            return;

        clubIsCaptured = true;
        ConfigureClubPullDrive();
        Debug.Log("クラブを捕捉しました。Iキーでキャッチを引き上げ、Spaceで開放できます。", this);
    }

    private IEnumerator ReleaseClub()
    {
        // Spaceではキャッチ位置を変更せず、爪だけを開いてクラブを放します。
        clawIsOpen = true;
        ConfigureClawLiftDrive();
        yield return new WaitForSeconds(clawReleaseSeconds);

        // 捕捉を解除すると、左右の定荷重ばねがクラブを前端へ戻します。
        clubIsCaptured = false;
        ConfigureClubSpringDrive();
        yield return new WaitForSeconds(clawCloseDelaySeconds);

        // サーボによる固定を解除します。爪は再び自由可動になり、
        // 次回下降時には接触に合わせて受動的に持ち上がります。
        clawIsOpen = false;
        ConfigureClawPassiveDrive();
        clubRoutine = null;
    }

    private void ResetMechanismsImmediately()
    {
        cameraTarget = 0f;
        eyeTarget = 0f;
        SetPositionTarget(cameraTilt, 0f);
        SetPositionTarget(cameraDriveGear, 0f);
        SetPositionTarget(leftEyeOutput, 0f);
        SetPositionTarget(rightEyeOutput, 0f);
        SetPositionTarget(leftEyeDriveGear, 0f);
        SetPositionTarget(rightEyeDriveGear, 0f);

        // キャッチ初期位置は最上部 = アナログ値0です。
        catchAnalogTarget = CatchAnalogMinimum;
        potentiometerValue = CatchAnalogMinimum;
        clubIsCaptured = false;
        clawIsOpen = false;
        ConfigureClubSpringDrive();
        ConfigureClawPassiveDrive();
        SetPositionTarget(catchCarriage, catchTopDegrees);

        // 停止状態でも、各車輪を直進用の初期角度へ合わせます。
        foreach (WheelModule module in wheelModules)
        {
            module.LastSteeringTarget = module.SteeringOffsetDegrees;
            SetPositionTarget(module.Steering, module.SteeringOffsetDegrees);
        }
    }

    private float ReadPotentiometerValue()
    {
        float carriageDegrees = ReadJointDegrees(catchCarriage);
        float position01 = Mathf.InverseLerp(
            catchTopDegrees,
            catchBottomDegrees,
            carriageDegrees);

        return Mathf.Lerp(CatchAnalogMinimum, CatchAnalogMaximum, position01);
    }

    private float AnalogToCatchDegrees(float analogValue)
    {
        float position01 = Mathf.InverseLerp(
            CatchAnalogMinimum,
            CatchAnalogMaximum,
            analogValue);
        return Mathf.Lerp(catchTopDegrees, catchBottomDegrees, position01);
    }

    private static float ReadJointDegrees(ArticulationBody body)
    {
        if (body == null || body.jointPosition.dofCount == 0)
            return 0f;

        return body.jointPosition[0] * Mathf.Rad2Deg;
    }

    private void UpdateBearingTargets(List<ArticulationBody> bearings, float carrierDegrees)
    {
        float rollingRatio = railRadius / Mathf.Max(bearingRadius, 0.001f);
        float bearingDegrees = -carrierDegrees * rollingRatio;

        foreach (ArticulationBody bearing in bearings)
            SetPositionTarget(bearing, bearingDegrees);
    }

    private void UpdateSpringReelTargets(float clubDegrees)
    {
        float windingRatio = railRadius / Mathf.Max(springReelRadius, 0.001f);
        float reelDegrees = -(clubDegrees - clubFrontDegrees) * windingRatio;

        // The two reels are mirrored, so their visible winding directions are
        // opposite although both springs pull the club toward the front.
        SetPositionTarget(rightClubSpringReel, reelDegrees);
        SetPositionTarget(leftClubSpringReel, -reelDegrees);
    }

    private static void SetPositionTarget(ArticulationBody body, float degrees)
    {
        if (body == null)
            return;

        body.WakeUp();
        ArticulationDrive drive = body.xDrive;
        drive.target = degrees;
        body.xDrive = drive;
    }

    private static void SetVelocityTarget(ArticulationBody body, float degreesPerSecond)
    {
        if (body == null)
            return;

        body.WakeUp();
        ArticulationDrive drive = body.xDrive;
        drive.targetVelocity = degreesPerSecond;
        body.xDrive = drive;
    }

    private void DisableConflictingControllers()
    {
        foreach (MonoBehaviour behaviour in GetComponents<MonoBehaviour>())
        {
            if (behaviour == null || behaviour == this)
                continue;

            string fullName = behaviour.GetType().FullName;
            if (fullName == "Unity.Robotics.UrdfImporter.Control.Controller" ||
                behaviour.GetType().Name == "TurtleBotDrive")
            {
                behaviour.enabled = false;
                Debug.Log($"競合を避けるため {behaviour.GetType().Name} を停止しました。", this);
            }
        }
    }

    private void OnGUI()
    {
        if (!showOperationGuide)
            return;

        if (operationGuideStyle == null)
        {
            operationGuideStyle = new GUIStyle(GUI.skin.box)
            {
                alignment = TextAnchor.UpperLeft,
                fontSize = 13,
                wordWrap = true,
                padding = new RectOffset(10, 10, 8, 8)
            };
        }

        string catchState = clubIsCaptured ? "捕捉中" : "未捕捉";

        string guide =
            "ぺろりん 操作ガイド\n" +
            "W / S : 前進・後退    A / D : 横移動\n" +
            "Q / E : 左右旋回      ↑ / ↓ : カメラ\n" +
            "← / → : 目            I / K : キャッチを上げる・下げる\n" +
            "Space : 現在位置でクラブを開放\n" +
            $"キャッチ実測 {potentiometerValue:0} / 指令 {catchAnalogTarget:0}  （{catchState}）\n" +
            "R : 機構リセット      F1 : この表示を隠す";

        operationGuideContent.text = guide;

        const float margin = 10f;
        float width = Mathf.Min(430f, Mathf.Max(220f, Screen.width - margin * 2f));
        float height = operationGuideStyle.CalcHeight(operationGuideContent, width);
        height = Mathf.Min(height, Mathf.Max(80f, Screen.height - margin * 2f));

        GUI.Box(new Rect(margin, margin, width, height), operationGuideContent, operationGuideStyle);
    }
}
