        // QuadDroneController.cpp

        #include "QuadDroneController.h"
        #include "QuadPawn.h"
        #include "DrawDebugHelpers.h"
        #include "imgui.h"
        #include "implot.h"
        #include "string"
        #include <zmq.hpp>
        #include <zmq_addon.hpp>
        #include "ImGuiUtil.h"
        #include "ImageUtils.h"
        #include "EngineUtils.h"
        #include "Math/UnrealMathUtility.h"

        zmq::context_t* UQuadDroneController::SharedZMQContext = nullptr;
        zmq::socket_t* UQuadDroneController::SharedZMQSocket = nullptr;
        int32 UQuadDroneController::SharedResourceRefCount = 0;
        #define ACCEPTABLE_DIST 200

        const FVector start = FVector(0, 0, 1000);

        static TArray<FVector> make_test_dests()
        {
            const int step = 1000;
            const int z_step = 200;
            TArray<FVector> xyzSetpoint;
            xyzSetpoint.Add(start);
            for (int i = 0; i < 1000; i++)
            {
                bool z = FMath::RandBool();
                bool x = FMath::RandBool();
                bool y = FMath::RandBool();
                FVector last = xyzSetpoint[xyzSetpoint.Num() - 1];
                float z_base = 1000;
                xyzSetpoint.Add(FVector(last.X + (x ? step : -step), last.Y + (y ? step : -step), z ? z_base + z_step : z_base - z_step));
            }
            return xyzSetpoint;
        }

        static TArray<FVector> spiralWaypoints()
        {
            TArray<FVector> points;
            const float baseHeight = 1000.0f;  // Height to rise from current position (cm)
            const float radius = 1500.0f;      // Radius of movements (cm)

            // Get current drone position
            AQuadPawn* drone = nullptr;
            FVector currentPos;

            // Find the drone in the world
            for (TActorIterator<AQuadPawn> ActorItr(GWorld); ActorItr; ++ActorItr)
            {
                drone = *ActorItr;
                if (drone)
                {
                    currentPos = drone->GetActorLocation();
                    break;
                }
            }

            // Define the working height once
            float workingHeight = currentPos.Z + baseHeight;

            // Rise up to testing height
            points.Add(FVector(currentPos.X, currentPos.Y, workingHeight));

            // Square pattern for clear yaw changes
            points.Add(FVector(currentPos.X + radius, currentPos.Y, workingHeight));         // Forward
            points.Add(FVector(currentPos.X + radius, currentPos.Y + radius, workingHeight));    // Right
            points.Add(FVector(currentPos.X - radius, currentPos.Y + radius, workingHeight));   // Back
            points.Add(FVector(currentPos.X - radius, currentPos.Y - radius, workingHeight));  // Left
            points.Add(FVector(currentPos.X + radius, currentPos.Y - radius, workingHeight));   // Forward

            // Octagon pattern for smoother yaw transitions
            const int octagonPoints = 8;
            const float angleStep = 2.0f * PI / octagonPoints;
            for (int i = 0; i <= octagonPoints; i++)
            {
                float angle = i * angleStep;
                float x = currentPos.X + (radius * 1.5f * FMath::Cos(angle));
                float y = currentPos.Y + (radius * 1.5f * FMath::Sin(angle));
                points.Add(FVector(x, y, workingHeight + 500.0f)); // Slightly higher
            }

            // Cross pattern for quick direction changes
            points.Add(FVector(currentPos.X + radius * 2, currentPos.Y, workingHeight));     // Right
            points.Add(FVector(currentPos.X, currentPos.Y, workingHeight));                  // Center
            points.Add(FVector(currentPos.X - radius * 2, currentPos.Y, workingHeight));     // Left
            points.Add(FVector(currentPos.X, currentPos.Y, workingHeight));                  // Center
            points.Add(FVector(currentPos.X, currentPos.Y + radius * 2, workingHeight));     // Forward
            points.Add(FVector(currentPos.X, currentPos.Y, workingHeight));                  // Center
            points.Add(FVector(currentPos.X, currentPos.Y - radius * 2, workingHeight));     // Back
            points.Add(FVector(currentPos.X, currentPos.Y, workingHeight));                  // Center

            return points;
        }
        // ---------------------- Constructor ------------------------

        UQuadDroneController::UQuadDroneController(const FObjectInitializer& ObjectInitializer)
            : dronePawn(nullptr)
            , Thrusts({ 0, 0, 0, 0 })
            , desiredYaw(0.f)
            , bDesiredYawInitialized(false)
            , desiredAltitude(0.0f)
            , bDesiredAltitudeInitialized(false)
            , currentFlightMode(FlightMode::None)
            , currentNav(nullptr)
            , curPos(0)
            , AutoWaypointHUD(nullptr)
            , VelocityHUD(nullptr)
            , JoyStickHUD(nullptr)
            , ManualThrustHUD(nullptr)
            , desiredNewVelocity(FVector::ZeroVector)
            , maxVelocity(250.0f)
            , maxAngle(15.f)
            , initialTakeoff(true)
            , altitudeReached(false)
            , Debug_DrawDroneCollisionSphere(true)
            , Debug_DrawDroneWaypoint(true)
            , thrustInput(0.0f)
            , yawInput(0.0f)
            , pitchInput(0.0f)
            , rollInput(0.0f)
            , hoverThrust(0.0f)
            , bHoverThrustInitialized(false)
            , ZMQSocket(nullptr)
            , CommandSocket(nullptr)
            , SceneCaptureComponent(nullptr)
            , RenderTarget(nullptr)
            , UpdateInterval(0.0001f)
            , TimeSinceLastUpdate(0.0f)
            , bIsActive(true)
            , DroneID(TEXT("DefaultDrone"))
        {
            SharedResourceRefCount++;
            UE_LOG(LogTemp, Display, TEXT("QuadDroneController constructor called, SharedResourceRefCount: %d"), SharedResourceRefCount);

            Thrusts.SetNum(4);

            // PID stuff
            xPID = MakeUnique<QuadPIDController>();
            xPID->SetLimits(-maxPIDOutput, maxPIDOutput);
            xPID->SetGains(1.f,  0.f, 0.1f);
    
            yPID = MakeUnique<QuadPIDController>();
            yPID->SetLimits(-maxPIDOutput, maxPIDOutput);
            yPID->SetGains(1.f,  0.f, 0.1f);
    
            zPID = MakeUnique<QuadPIDController>();
            zPID->SetLimits(-maxPIDOutput, maxPIDOutput);
            zPID->SetGains(5.f,  1.f, 0.1f);
    
            pitchAttitudePID = MakeUnique<QuadPIDController>();
            pitchAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
            pitchAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    
            rollAttitudePID = MakeUnique<QuadPIDController>();
            rollAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
            rollAttitudePID->SetGains(2.934f, 0.297f, 3.633f);
    
            yawAttitudePID = MakeUnique<QuadPIDController>();
            yawAttitudePID->SetLimits(-maxPIDOutput, maxPIDOutput);
            yawAttitudePID->SetGains(0.f, 0.f, 0.f);


            // Move by Velocity
            xPIDVelocity = MakeUnique<QuadPIDController>();
            xPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            xPIDVelocity->SetGains(1.f, 0.f, 0.1f);

            yPIDVelocity = MakeUnique<QuadPIDController>();
            yPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            yPIDVelocity->SetGains(1.f, 0.f, 0.1f);

            zPIDVelocity = MakeUnique<QuadPIDController>();
            zPIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            zPIDVelocity->SetGains(5.f, 1.f, 0.1f);

            pitchAttitudePIDVelocity = MakeUnique<QuadPIDController>();
            pitchAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            pitchAttitudePIDVelocity->SetGains(8.f, 0.3f, 3.7);

            rollAttitudePIDVelocity = MakeUnique<QuadPIDController>();
            rollAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            rollAttitudePIDVelocity->SetGains(8.f, 0.3f, 3.7);

            yawAttitudePIDVelocity = MakeUnique<QuadPIDController>();
            yawAttitudePIDVelocity->SetLimits(-maxPIDOutput, maxPIDOutput);
            yawAttitudePIDVelocity->SetGains(1.8f, 0.15f, 1.5f);

            // Move by Controller
            xPIDJoyStick = MakeUnique<QuadPIDController>();
            xPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            xPIDJoyStick->SetGains(8, 3.626f, 1.832f);

            yPIDJoyStick = MakeUnique<QuadPIDController>();
            yPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            yPIDJoyStick->SetGains(2.329f, 3.626f, 1.832f);

            zPIDJoyStick = MakeUnique<QuadPIDController>();
            zPIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            zPIDJoyStick->SetGains(5.344f, 1.f, 0.1f);

            pitchAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
            pitchAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            pitchAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

            rollAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
            rollAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            rollAttitudePIDJoyStick->SetGains(11.755f, 5.267f, 9.008f);

            yawAttitudePIDJoyStick = MakeUnique<QuadPIDController>();
            yawAttitudePIDJoyStick->SetLimits(-maxPIDOutput, maxPIDOutput);
            yawAttitudePIDJoyStick->SetGains(0.f, 0.f, 0.f);



            // Initialize ImGuiUtil instances using MakeUnique
            AutoWaypointHUD = MakeUnique<ImGuiUtil>(
                dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
                Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
                altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
                xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
                pitchAttitudePID.Get(), yawAttitudePID.Get(),
                xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
                rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
                yawAttitudePIDVelocity.Get(),
                xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
                rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
                yawAttitudePIDJoyStick.Get()
            );

            VelocityHUD = MakeUnique<ImGuiUtil>(
                dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
                Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
                altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
                xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
                pitchAttitudePID.Get(), yawAttitudePID.Get(),
                xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
                rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
                yawAttitudePIDVelocity.Get(),
                xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
                rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
                yawAttitudePIDJoyStick.Get()
            );

            JoyStickHUD = MakeUnique<ImGuiUtil>(
                dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
                Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
                altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
                xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
                pitchAttitudePID.Get(), yawAttitudePID.Get(),
                xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
                rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
                yawAttitudePIDVelocity.Get(),
                xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
                rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
                yawAttitudePIDJoyStick.Get()
            );

            ManualThrustHUD = MakeUnique<ImGuiUtil>(
                dronePawn, this, desiredNewVelocity, initialTakeoff, altitudeReached,
                Debug_DrawDroneCollisionSphere, Debug_DrawDroneWaypoint, maxPIDOutput,
                altitudeThresh, minAltitudeLocal, maxVelocity, maxAngle,
                xPID.Get(), yPID.Get(), zPID.Get(), rollAttitudePID.Get(),
                pitchAttitudePID.Get(), yawAttitudePID.Get(),
                xPIDVelocity.Get(), yPIDVelocity.Get(), zPIDVelocity.Get(),
                rollAttitudePIDVelocity.Get(), pitchAttitudePIDVelocity.Get(),
                yawAttitudePIDVelocity.Get(),
                xPIDJoyStick.Get(), yPIDJoyStick.Get(), zPIDJoyStick.Get(),
                rollAttitudePIDJoyStick.Get(), pitchAttitudePIDJoyStick.Get(),
                yawAttitudePIDJoyStick.Get()
            );
        }

        UQuadDroneController::~UQuadDroneController()
        {
            // Cleanup ZMQ
            bIsActive = false;

            if (CommandSocket)
            {
                CommandSocket->close();
                delete CommandSocket;
                CommandSocket = nullptr;
                UE_LOG(LogTemp, Display, TEXT("Command SUB Socket closed"));
            }

            SharedResourceRefCount--;

            if (SharedResourceRefCount == 0)
            {
                if (SharedZMQSocket)
                {
                    SharedZMQSocket->close();
                    delete SharedZMQSocket;
                    SharedZMQSocket = nullptr;
                    UE_LOG(LogTemp, Display, TEXT("Shared PUB Socket closed"));
                }

                if (SharedZMQContext)
                {
                    SharedZMQContext->close();
                    delete SharedZMQContext;
                    SharedZMQContext = nullptr;
                    UE_LOG(LogTemp, Display, TEXT("Shared ZMQ Context closed"));
                }
            }

            // Cleanup image capture components
            if (SceneCaptureComponent)
            {
                SceneCaptureComponent->UnregisterComponent();
                SceneCaptureComponent = nullptr;
            }

            RenderTarget = nullptr;

            UE_LOG(LogTemp, Display, TEXT("QuadDroneController Destructor completed"));
        } 
        void UQuadDroneController::Initialize(AQuadPawn* InPawn)
        {
            if (!InPawn)
            {
                UE_LOG(LogTemp, Error, TEXT("Initialize called with null pawn"));
                return;
            }

            if (dronePawn != InPawn)
            {
                UE_LOG(LogTemp, Display, TEXT("Initializing controller for pawn: %s"), *InPawn->GetName());
                dronePawn = InPawn;
            }

            if (!CommandSocket)
            {
                BeginZMQController();
            }

            if (!CommandSocket)
            {
                UE_LOG(LogTemp, Error, TEXT("Failed to initialize ZMQ for pawn: %s"), *InPawn->GetName());
            }
            else
            {
                UE_LOG(LogTemp, Display, TEXT("Successfully initialized controller for pawn: %s"),
                    *InPawn->GetName());
            }
        }
        // ------------ Setter and Getter -------------------
        void UQuadDroneController::SetDesiredVelocity(const FVector& NewVelocity)
        {
            desiredNewVelocity = NewVelocity;
            UE_LOG(LogTemp, Display, TEXT("SetDesiredVelocity called: %s"), *desiredNewVelocity.ToString());

        }

        void UQuadDroneController::SetFlightMode(FlightMode NewMode)
        {
            currentFlightMode = NewMode;
        }

        UQuadDroneController::FlightMode UQuadDroneController::GetFlightMode() const
        {
            return currentFlightMode;
        }


        // ---------------------- Waypoint Nav ------------------------

        void UQuadDroneController::AddNavPlan(FString name, TArray<FVector> waypoints)
        {
            NavPlan plan;
            plan.name = name;
            plan.waypoints = waypoints;
            setPointNavigation.Add(plan);
        }

        void UQuadDroneController::SetNavPlan(FString name)
        {
            for (int i = 0; i < setPointNavigation.Num(); i++) {
                if (setPointNavigation[i].name == name) {
                    currentNav = &setPointNavigation[i];
                    curPos = 0;
                    return;
                }
            }
        }

        // ---------------------- Reset PIDs ------------------------

        void UQuadDroneController::ResetPID()
        {
            xPID->Reset();
            yPID->Reset();
            zPID->Reset();
            rollAttitudePID->Reset();
            pitchAttitudePID->Reset();
            yawAttitudePID->Reset();

            xPIDVelocity->Reset();
            yPIDVelocity->Reset();
            zPIDVelocity->Reset();
            rollAttitudePIDVelocity->Reset();
            pitchAttitudePIDVelocity->Reset();
            yawAttitudePIDVelocity->Reset();

            xPIDJoyStick->Reset();
            yPIDJoyStick->Reset();
            zPIDJoyStick->Reset();
            rollAttitudePIDJoyStick->Reset();
            pitchAttitudePIDJoyStick->Reset();
            yawAttitudePIDJoyStick->Reset();
            curPos = 0;
            altitudeReached = false;
        }

        // ---------------------- Thrust, Desired Vel, Pitch, and Roll ------------------------

        void UQuadDroneController::ThrustMixer(float xOutput,float yOutput,float zOutput,float rollOutput,float pitchOutput)
        {
            //Order: FL,FR,BL,BR

            switch (currentFlightMode)
            {
            case FlightMode::AutoWaypoint :
                Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
                Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
                Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
                Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
                break;
            case FlightMode::VelocityControl:
                Thrusts[0] = zOutput - xOutput + yOutput + rollOutput + pitchOutput;
                Thrusts[1] = zOutput - xOutput - yOutput - rollOutput + pitchOutput;
                Thrusts[2] = zOutput + xOutput + yOutput + rollOutput - pitchOutput;
                Thrusts[3] = zOutput + xOutput - yOutput - rollOutput - pitchOutput;
                break;
            case FlightMode::ManualFlightControl:
                Thrusts[0] = zOutput + rollOutput + pitchOutput; 
                Thrusts[1] = zOutput - rollOutput + pitchOutput;
                Thrusts[2] = zOutput + rollOutput - pitchOutput; 
                Thrusts[3] = zOutput - rollOutput - pitchOutput; 
                break;
            }


    
            for (int i = 0; i < Thrusts.Num(); ++i) {
                Thrusts[i] = FMath::Clamp(Thrusts[i], 0.0f, 600.f);
            }
        }

        FVector UQuadDroneController::CalculateDesiredVelocity(const FVector& error, float InMaxVelocity)
        {
            // Normalizing position error and multiplying with maxVel to get a smaller desiredVel
            FVector desired_velocity = error.GetSafeNormal() * InMaxVelocity;
            return desired_velocity;
        }

        float UQuadDroneController::CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
        {
            FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
            FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

            if (FMath::Abs(normalizedError.Z) > altitudeThreshold)
            {
                return 0.0f;
            }
            else
            {
                float calculatedRoll = FMath::Atan2(normalizedError.Y, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
                return FMath::Clamp(calculatedRoll, -maxTilt, maxTilt);
            }
        }

        float UQuadDroneController::CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float maxTilt, float altitudeThreshold)
        {
            FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
            FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

            if (FMath::Abs(normalizedError.Z) > altitudeThreshold)
            {
                return 0.0f;
            }
            else
            {
                float calculatedPitch = FMath::Atan2(-normalizedError.X, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
                return FMath::Clamp(calculatedPitch, -maxTilt, maxTilt);
            }
        } 


        // ---------------------- Update ------------------------

        void UQuadDroneController::Update(double a_deltaTime)
        {

            AddNavPlan("TestPlan", spiralWaypoints());
            SetNavPlan("TestPlan");

            //UE_LOG(LogTemp, Display, TEXT("uPDATE called: %s"), *desiredNewVelocity.ToString());

            ImGui::Begin("Flight Mode Selector");

            if (ImGui::Button("Auto Waypoint", ImVec2(200, 50)))
            {
                currentFlightMode = FlightMode::AutoWaypoint;
                curPos = 0; // Reset to start navigation
            }
            if (ImGui::Button("Manual Thrust Control", ImVec2(200, 50)))
            {
                currentFlightMode = FlightMode::ManualThrustControl;
            }
            if (ImGui::Button("Manual Waypoint", ImVec2(200, 50)))
            {
                currentFlightMode = FlightMode::ManualWaypoint;
            }
            if (ImGui::Button("Move By Velocity", ImVec2(200, 50)))
            {
                currentFlightMode = FlightMode::VelocityControl;
            }
            if (ImGui::Button("Apply Controller Input", ImVec2(200, 50)))
            {
                currentFlightMode = FlightMode::ManualFlightControl;
            }

            // Adjust ImGui layout
            ImGui::End();


            switch (currentFlightMode)
            {
            case FlightMode::None:
                return;
            case FlightMode::AutoWaypoint:
                AutoWaypointControl(a_deltaTime);
                break;
            case FlightMode::ManualWaypoint:
                //ManualWaypointControl(a_deltaTime);
                break;
            case FlightMode::ManualFlightControl:
                ApplyControllerInput(a_deltaTime);
                break;
            case FlightMode::ManualThrustControl:
                ManualThrustControl(a_deltaTime);
                break;
            case FlightMode::VelocityControl:
                VelocityControl(a_deltaTime);
                break;
            }
   

        }

        void UQuadDroneController::ApplyControllerInput(double a_deltaTime)
        {
            if (!dronePawn) return;

            float droneMass = dronePawn->DroneBody->GetMass();
            const float mult = 0.5f;

            FVector currentPosition = dronePawn->GetActorLocation();
            FRotator currentRotation = dronePawn->GetActorRotation();

            // Initialize desiredAltitude and desiredYaw on the first run
            if (!bDesiredAltitudeInitialized)
            {
                desiredAltitude = currentPosition.Z;
                bDesiredAltitudeInitialized = true;
            }

            if (!bDesiredYawInitialized)
            {
                desiredYaw = currentRotation.Yaw;
                bDesiredYawInitialized = true;
            }

            // ------ Altitude Control ---------
            // Modify desired altitude based on thrust input
            float altitudeRate = 400.0f; // Units per second for altitude change
            desiredAltitude += thrustInput * altitudeRate * a_deltaTime;

            // Calculate error between desired and current altitude
            float z_error = desiredAltitude - currentPosition.Z;
            float z_output = zPIDJoyStick->Calculate(z_error, a_deltaTime);

            // ------ Attitude Control ---------
            float desiredRoll = rollInput * maxAngle;
            float roll_error = desiredRoll - currentRotation.Roll;
            float roll_output = rollAttitudePIDJoyStick->Calculate(roll_error, a_deltaTime);

            float desiredPitch = pitchInput * maxAngle;
            float pitch_error = desiredPitch - currentRotation.Pitch;
            float pitch_output = pitchAttitudePIDJoyStick->Calculate(pitch_error, a_deltaTime);

            // Yaw control 
            float yawRate = 90.0f; // Degrees per second
            desiredYaw += yawInput * yawRate * a_deltaTime;
            desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

            float yaw_error = desiredYaw - currentRotation.Yaw;
            yaw_error = FMath::UnwindDegrees(yaw_error);
            float yaw_output = yawAttitudePIDJoyStick->Calculate(yaw_error, a_deltaTime);

            // Apply yaw torque
            FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
            dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

            // Thrust Mixing
            ThrustMixer(0, 0, z_output, roll_output, pitch_output);

            // Apply thrusts to rotors
            for (int i = 0; i < Thrusts.Num(); ++i)
            {
                dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
            }

            // Collect data for ImGui display
            TArray<float> ThrustsVal = Thrusts;
            FVector waypoint(0, 0, desiredAltitude);
            FVector error(0, 0, z_error);
            FVector desiredVelocity(0, 0, 0);
            float xOutput = 0.0f;
            float yOutput = 0.0f;

            //JoyStickHUD->JoyStickHud(ThrustsVal, roll_error, pitch_error, currentRotation, waypoint, currentPosition, error, desiredVelocity, dronePawn->GetVelocity(), xOutput, yOutput, z_output, a_deltaTime);
        }

        void UQuadDroneController::AutoWaypointControl(double a_deltaTime)
        {
            if (!currentNav || curPos >= currentNav->waypoints.Num()) return;
    
            // Unreal Feedback and data collection
            float droneMass = dronePawn->DroneBody->GetMass();
            const float mult = 0.5f;
            FVector currentPosition = dronePawn->GetActorLocation();
            FRotator currentRotation = dronePawn->GetActorRotation();
            FVector currentVelocity = dronePawn->GetVelocity();
    
            FVector setPoint = currentNav->waypoints[curPos];

            if (!altitudeReached)
            {
                // Drone must ascend to minAltitudeLocal
                setPoint = FVector(currentPosition.X, currentPosition.Y, minAltitudeLocal); // Set waypoint directly above the drone
            }
            else
            {
                // Proceed with current waypoint
                setPoint = currentNav->waypoints[curPos];
            }

            // Calculate position error and other variables based on the current setPoint
            FVector positionError = setPoint - currentPosition;

            // Check if the drone has reached the setPoint, if so then reset integral sums    
            if (positionError.Size() < ACCEPTABLE_DIST)
            {
                if (!altitudeReached)
                {
                    altitudeReached = true;
                    // After reaching minAltitudeLocal, update the setPoint to the next waypoint
                    setPoint = currentNav->waypoints[curPos];
                    positionError = setPoint - currentPosition;

                    // Reset the integral sums of PID controllers
                    xPID->ResetIntegral();
                    yPID->ResetIntegral();
                    zPID->ResetIntegral();
                    rollAttitudePID->ResetIntegral();
                    pitchAttitudePID->ResetIntegral();
                    yawAttitudePID->ResetIntegral();
                }
                else
                {
                    // Move to the next waypoint
                    curPos++;
                    if (curPos >= currentNav->waypoints.Num())
                    {
                        // Reached the end of the nav plan
                        currentNav = nullptr;
                        curPos = 0;
                        // Stop updating if no more waypoints
                        return;
                    }
                    else
                    {
                        // Update setPoint and positionError for the new waypoint
                        setPoint = currentNav->waypoints[curPos];
                        positionError = setPoint - currentPosition;

                        // Reset the integral sums of PID controllers
                        xPID->ResetIntegral();
                        yPID->ResetIntegral();
                        zPID->ResetIntegral();
                        rollAttitudePID->ResetIntegral();
                        pitchAttitudePID->ResetIntegral();
                        yawAttitudePID->ResetIntegral();
                    }
                }
            }

            // Continue with calculations using the updated positionError
            FVector normalizedError = positionError.GetSafeNormal();
            FVector droneForwardVector = dronePawn->GetActorForwardVector();
            FVector desiredVelocity = CalculateDesiredVelocity(positionError, maxVelocity);

            DrawDebugVisuals(currentPosition, setPoint);

            // ------------------- Yaw Control ---------------------

            // Calculate desiredYaw based on the direction to the waypoint
            desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(positionError.Y, positionError.X));

            // Normalize desiredYaw to [-180, 180]
            desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

            // Compute yaw error
            float yaw_error = desiredYaw - currentRotation.Yaw;

            // Normalize yaw error to [-180, 180]
            yaw_error = FMath::UnwindDegrees(yaw_error);

            // Use yaw PID controller to calculate yaw torque
            float yaw_output = yawAttitudePID->Calculate(yaw_error, a_deltaTime);

            // Apply yaw torque directly to the drone body
            FVector yawTorque = FVector(0.0f, 0.0f, yaw_output); // Torque around Z-axis
            dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);
    
            // ------------------- Attitude CONTROL ---------------------
            //float yawTorque = CalculateYawTorque(setPoint, currentPosition, currentRotation, a_deltaTime);
            float desiredRoll = CalculateDesiredRoll(normalizedError, droneForwardVector, maxAngle, altitudeThresh);
            float desiredPitch = CalculateDesiredPitch(normalizedError, droneForwardVector, maxAngle, altitudeThresh);

            float roll_error = desiredRoll - currentRotation.Roll;
            float pitch_error = desiredPitch - currentRotation.Pitch;

            float roll_output = rollAttitudePID->Calculate(roll_error, a_deltaTime);
            float pitch_output = pitchAttitudePID->Calculate(pitch_error, a_deltaTime);
    
            // ------------------- POSITION CONTROL ---------------------
            float x_output = xPID->Calculate(desiredVelocity.X - currentVelocity.X, a_deltaTime);
            float y_output = yPID->Calculate(desiredVelocity.Y - currentVelocity.Y, a_deltaTime);
            float z_output = zPID->Calculate(desiredVelocity.Z - currentVelocity.Z, a_deltaTime);    
            //------------------- Thrust Mixing -------------
    
            ThrustMixer(x_output,y_output,z_output,roll_output,pitch_output);
            //ApplyTorqueToRotors(yawTorque);
            // Update the thrust of each rotor
            for (int i = 0; i < Thrusts.Num(); ++i) {
                dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
            }
    
            AutoWaypointHUD->AutoWaypointHud(Thrusts, roll_error, pitch_error, currentRotation, setPoint, currentPosition, positionError, currentVelocity, x_output, y_output, z_output,a_deltaTime); //****
            AutoWaypointHUD->RenderImPlot(Thrusts,a_deltaTime);

        }

        void UQuadDroneController::ManualThrustControl(double a_deltaTime)
        {
            if (!dronePawn) return;

            float droneMass = dronePawn->DroneBody->GetMass();
            const float mult = 0.5f;

            // Reset net torque
            FVector NetTorque = FVector::ZeroVector;

            // Apply thrusts and calculate torque
            for (int i = 0; i < Thrusts.Num(); ++i)
            {
                // Apply thrust
                dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];

                // Calculate rotor torque (assumed proportional to thrust)
                //float rotorTorque = rotorSpinDirections[i] * Thrusts[i] * rotorTorqueConstant;

                // Sum net torque around Z-axis (yaw)
                NetTorque += FVector(0.0f, 0.0f, 0.f);
            }

            // Apply net torque to the drone body
            dronePawn->DroneBody->AddTorqueInRadians(NetTorque, NAME_None, true);

            // Optional: Render ImGui for debugging and visualization
            FVector currentPosition = dronePawn->GetActorLocation();
            FRotator currentRotation = dronePawn->GetActorRotation();
            FVector currentVelocity = dronePawn->GetVelocity();

            FVector waypoint = currentPosition; // No waypoint in manual thrust control
            FVector error(0, 0, 0);
            FVector desiredVelocity(0, 0, 0);
            float xOutput = 0.0f;
            float yOutput = 0.0f;
            float zOutput = 0.0f;

            // Call RenderImGui
            //ManualThrustHUD->AutoWaypointHud(Thrusts, 0.0f, 0.0f, currentRotation, waypoint, currentPosition, error, desiredVelocity, currentVelocity, xOutput, yOutput, zOutput, a_deltaTime);
        }

        void UQuadDroneController::VelocityControl(double a_deltaTime)
        {
            if (!dronePawn) return;
            // ZMQ

            if (!bIsActive) return;

            if (UpdateInterval == 0.0) {
                CaptureAndSendImage();
            }
            else {
                TimeSinceLastUpdate += a_deltaTime;

                // Check if it's time to capture and send a new image
                if (TimeSinceLastUpdate >= UpdateInterval)
                {
                    CaptureAndSendImage();
                    TimeSinceLastUpdate = 0.0f; // Reset the timer
                }
            }

            ReceiveVelocityCommand();

            

            float droneMass = dronePawn->DroneBody->GetMass();
            const float mult = 0.5f;

            FVector currentVelocity = dronePawn->GetVelocity();
            FRotator currentRotation = dronePawn->GetActorRotation();

            // Calculate velocity error
            FVector velocityError = desiredNewVelocity - currentVelocity;

            // Use PID controllers to calculate outputs
            float x_output = xPIDVelocity->Calculate(velocityError.X, a_deltaTime);
            float y_output = yPIDVelocity->Calculate(velocityError.Y, a_deltaTime);
            float z_output = zPIDVelocity->Calculate(velocityError.Z, a_deltaTime);

            // Attitude stabilization (keeping roll and pitch at zero)
            float roll_error = -currentRotation.Roll;
            float pitch_error = -currentRotation.Pitch;

            float roll_output = rollAttitudePIDVelocity->Calculate(roll_error, a_deltaTime);
            float pitch_output = pitchAttitudePIDVelocity->Calculate(pitch_error, a_deltaTime);

            // Calculate desired yaw based on velocity direction
            FVector horizontalVelocity = desiredNewVelocity;
            horizontalVelocity.Z = 0; // Ignore vertical component for yaw calculation

            // Only update desired yaw if we have significant horizontal velocity
            const float MIN_VELOCITY_FOR_YAW = 10.0f; // Adjust this threshold as needed
            if (horizontalVelocity.SizeSquared() > MIN_VELOCITY_FOR_YAW * MIN_VELOCITY_FOR_YAW)
            {
                // Calculate the desired yaw angle based on velocity direction
                desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(horizontalVelocity.Y, horizontalVelocity.X));
            }
            else
            {
                // If velocity is too low, maintain current yaw to prevent erratic rotation
                desiredYaw = currentRotation.Yaw;
            }

            // Normalize desiredYaw to [-180, 180]
            desiredYaw = FMath::Fmod(desiredYaw + 180.0f, 360.0f) - 180.0f;

            // Calculate yaw error
            float yaw_error = desiredYaw - currentRotation.Yaw;
            yaw_error = FMath::UnwindDegrees(yaw_error);  // Normalize the error to [-180, 180]

            // Calculate yaw output using PID
            float yaw_output = yawAttitudePIDVelocity->Calculate(yaw_error, a_deltaTime);

            // Apply yaw torque directly to the drone body
            FVector yawTorque = FVector(0.0f, 0.0f, yaw_output);
            dronePawn->DroneBody->AddTorqueInDegrees(yawTorque, NAME_None, true);

            // Apply thrust mixing
            ThrustMixer(x_output, y_output, z_output, roll_output, pitch_output);

            // Update rotors
            for (int i = 0; i < Thrusts.Num(); ++i)
            {
                dronePawn->Rotors[i].Thruster->ThrustStrength = droneMass * mult * Thrusts[i];
            }

            // Debug visualization for velocity direction
            if (Debug_DrawDroneWaypoint)
            {
                FVector dronePos = dronePawn->GetActorLocation();
                FVector velocityDirection = desiredNewVelocity.GetSafeNormal() * 200.0f; // Scale for visualization

                DrawDebugLine(
                    dronePawn->GetWorld(),
                    dronePos,
                    dronePos + velocityDirection,
                    FColor::Yellow,
                    false,
                    -1.0f,
                    0,
                    2.0f
                );
            }
            VelocityHUD->VelocityHud(Thrusts, roll_error, pitch_error, currentRotation, FVector::ZeroVector, dronePawn->GetActorLocation(), FVector::ZeroVector, currentVelocity, x_output, y_output, z_output, a_deltaTime);
            VelocityHUD->RenderImPlot(Thrusts, a_deltaTime);

        }

        // ---------------------- Controller Handling ------------------------

        void UQuadDroneController::HandleThrustInput(float Value)
        {
            // Value ranges from -1 to 1
            thrustInput = Value; // Store the input value for use in ApplyControllerInput
        }

        void UQuadDroneController::HandleYawInput(float Value)
        {
            yawInput = Value; // Store the raw input for later use with interpolation
        }

        void UQuadDroneController::HandlePitchInput(float Value)
        {
            pitchInput = Value; // Map Value to desired pitch range
        }

        void UQuadDroneController::HandleRollInput(float Value)
        {
            rollInput = Value; // Map Value to desired roll range
        }

        void UQuadDroneController::DrawDebugVisuals(const FVector& currentPosition, const FVector& setPoint) const
        {
            if (Debug_DrawDroneCollisionSphere)
            {
                // Get the mesh bounds to calculate the vertical offset
                FBoxSphereBounds MeshBounds = dronePawn->DroneBody->CalcBounds(dronePawn->DroneBody->GetComponentTransform());
                float VerticalOffset = MeshBounds.BoxExtent.Z;

                // Adjust the position upwards
                FVector AdjustedPosition = currentPosition + FVector(0.0f, 0.0f, VerticalOffset);

                // Draw the debug sphere at the adjusted position
                DrawDebugSphere(
                    dronePawn->GetWorld(),
                    AdjustedPosition,
                    dronePawn->DroneBody->GetCollisionShape().GetSphereRadius(),
                    10,
                    FColor::Red,
                    false,     // bPersistentLines
                    0.0f       // LifeTime
                );
            }

            if (Debug_DrawDroneWaypoint)
            {
                // Existing code remains the same
                DrawDebugSphere(
                    dronePawn->GetWorld(),
                    setPoint,
                    ACCEPTABLE_DIST,
                    10,
                    FColor::Red,
                    false,     // bPersistentLines
                    0.0f       // LifeTime
                );
                DrawDebugLine(
                    dronePawn->GetWorld(),
                    currentPosition,
                    setPoint,
                    FColor::Green,
                    false,     // bPersistentLines
                    0.0f       // LifeTime
                );
            }
        }

        void UQuadDroneController::IncreaseAllThrusts(float Amount)
        {
            for (int i = 0; i < Thrusts.Num(); ++i)
            {
                Thrusts[i] += Amount;
                // Clamp the thrust value to ensure it stays within acceptable limits
                Thrusts[i] = FMath::Clamp(Thrusts[i], -maxPIDOutput, maxPIDOutput);
            }
        }

        // ------------------------ ZMQ Handling ----------------------------

        void UQuadDroneController::BeginZMQController()
        {
            UE_LOG(LogTemp, Display, TEXT("BeginZMQController called"));

            // Check if ZMQ is already initialized
            if (CommandSocket)
            {
                UE_LOG(LogTemp, Warning, TEXT("CommandSocket already initialized, skipping initialization"));
                return;
            }

            // Initialize ZMQ Context first
            if (!SharedZMQContext)
            {
                try
                {
                    SharedZMQContext = new zmq::context_t(1);
                    UE_LOG(LogTemp, Display, TEXT("Created new ZMQ context"));
                }
                catch (const zmq::error_t& e)
                {
                    UE_LOG(LogTemp, Error, TEXT("Failed to create ZMQ context: %s"), UTF8_TO_TCHAR(e.what()));
                    return;
                }
            }

            // Initialize Command Socket first
            try
            {
                UE_LOG(LogTemp, Display, TEXT("Creating Command Socket..."));
                CommandSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::sub);

                FString Address = TEXT("tcp://localhost:5556");
                UE_LOG(LogTemp, Display, TEXT("Connecting to %s..."), *Address);

                CommandSocket->connect(TCHAR_TO_UTF8(*Address));
                CommandSocket->set(zmq::sockopt::subscribe, "");

                UE_LOG(LogTemp, Display, TEXT("Successfully created and connected Command Socket"));
            }
            catch (const zmq::error_t& e)
            {
                UE_LOG(LogTemp, Error, TEXT("Failed to create/connect Command Socket: %s"), UTF8_TO_TCHAR(e.what()));
                if (CommandSocket)
                {
                    delete CommandSocket;
                    CommandSocket = nullptr;
                }
                return;
            }

            // Then initialize Publisher Socket
            if (!SharedZMQSocket)
            {
                try
                {
                    SharedZMQSocket = new zmq::socket_t(*SharedZMQContext, zmq::socket_type::pub);
                    FString Address = TEXT("tcp://*:5555");
                    SharedZMQSocket->bind(TCHAR_TO_UTF8(*Address));
                    UE_LOG(LogTemp, Display, TEXT("ZeroMQ PUB Socket bound to %s"), *Address);
                }
                catch (const zmq::error_t& e)
                {
                    UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during publisher socket setup: %s"), UTF8_TO_TCHAR(e.what()));
                    return;
                }
            }
            ZMQSocket = SharedZMQSocket;

            // Rest of BeginZMQController implementation...
        }

        void UQuadDroneController::ReceiveVelocityCommand()
        {
            if (!CommandSocket)
            {
                UE_LOG(LogTemp, Warning, TEXT("CommandSocket is null"));
                return;
            }

            zmq::multipart_t multipart;
            try
            {
                bool result = multipart.recv(*CommandSocket, static_cast<int>(zmq::recv_flags::dontwait));
                if (result)
                {
                    // Message received
                    if (multipart.empty())
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Received empty multipart message"));
                        return;
                    }

                    // Extract topic
                    std::string topic = multipart.popstr();

                    if (multipart.empty())
                    {
                        UE_LOG(LogTemp, Warning, TEXT("No message part after topic"));
                        return;
                    }

                    // Extract message
                    zmq::message_t message = multipart.pop();

                    // Ensure the message size is correct
                    if (message.size() == sizeof(float) * 3)
                    {
                        // Ensure message.data() is not null
                        if (message.data())
                        {
                            float* velocityArray = reinterpret_cast<float*>(message.data());
                            FVector newVelocity(velocityArray[0], velocityArray[1], velocityArray[2]);

                            // Direct access to our own methods since we are the controller
                            SetDesiredVelocity(newVelocity);
                            SetFlightMode(FlightMode::VelocityControl);
                            UE_LOG(LogTemp, Display, TEXT("Received velocity command: %s"), *newVelocity.ToString());
                        }
                        else
                        {
                            UE_LOG(LogTemp, Warning, TEXT("Received message data is null"));
                        }
                    }
                    else
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Received invalid velocity command size: %zu bytes"), message.size());
                    }
                }
                else
                {
                    // No message received; this is expected in non-blocking mode
                    // You can optionally log this or simply continue
                }
            }
            catch (const zmq::error_t& e)
            {
                UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during receive: %s"), UTF8_TO_TCHAR(e.what()));
            }
        }

        void UQuadDroneController::CaptureAndSendImage()
        {
            if (!SceneCaptureComponent || !RenderTarget)
            {
                return;
            }

            FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
            if (!RenderTargetResource)
            {
                return;
            }

            TArray<FColor> Bitmap;
            if (!RenderTargetResource->ReadPixels(Bitmap))
            {
                return;
            }

            int32 NewSizeX = RenderTarget->SizeX;
            int32 NewSizeY = RenderTarget->SizeY;
            TArray<FColor> ResizedBitmap;
            FImageUtils::ImageResize(RenderTarget->SizeX, RenderTarget->SizeY, Bitmap, NewSizeX, NewSizeY, ResizedBitmap, false);

            TArray64<uint8> CompressedBitmap;
            FImageUtils::PNGCompressImageArray(NewSizeX, NewSizeY, ResizedBitmap, CompressedBitmap);

            TArray<uint8> CompressedBitmapArray(CompressedBitmap.GetData(), CompressedBitmap.Num());

            SendImageData(CompressedBitmapArray);
        }

        void UQuadDroneController::SendImageData(const TArray<uint8>& CompressedBitmap)
        {
            if (!ZMQSocket)
            {
                return;
            }

            zmq::multipart_t multipart;
            multipart.addstr(TCHAR_TO_UTF8(*DroneID));
            multipart.addmem(CompressedBitmap.GetData(), CompressedBitmap.Num());

            try
            {
                multipart.send(*ZMQSocket, static_cast<int>(zmq::send_flags::none));
                UE_LOG(LogTemp, Verbose, TEXT("Sent image data from drone %s of size: %d bytes"), *DroneID, CompressedBitmap.Num());
            }
            catch (const zmq::error_t& e)
            {
                UE_LOG(LogTemp, Error, TEXT("ZeroMQ Error during sending image data: %s"), UTF8_TO_TCHAR(e.what()));
            }
        }


        void UQuadDroneController::StoreInitialPosition()
        {
            if (dronePawn)
            {
                initialDronePosition = dronePawn->GetActorLocation();
                UE_LOG(LogTemp, Display, TEXT("Stored initial drone position: %s"), *initialDronePosition.ToString());
            }
        }

 

       void UQuadDroneController::ResetDroneHigh()
       {
           if (dronePawn)
           {
               // Reset to high position (10000cm up)
               dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10000.0f), false, nullptr, ETeleportType::TeleportPhysics);
               dronePawn->SetActorRotation(FRotator::ZeroRotator, ETeleportType::TeleportPhysics);

               if (dronePawn->DroneBody)
               {
                   dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                   dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                   dronePawn->DroneBody->WakeAllRigidBodies();
               }

               // Reset controller states
               ResetPID();
               desiredNewVelocity = FVector::ZeroVector;
               initialTakeoff = true;
               altitudeReached = false;
           }
       }

       void UQuadDroneController::ResetDroneOrigin()
       {
           if (dronePawn)
           {
               // Reset to origin (10cm up)
               dronePawn->SetActorLocation(FVector(0.0f, 0.0f, 10.0f), false, nullptr, ETeleportType::TeleportPhysics);
               dronePawn->SetActorRotation(FRotator::ZeroRotator, ETeleportType::TeleportPhysics);

               if (dronePawn->DroneBody)
               {
                   dronePawn->DroneBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
                   dronePawn->DroneBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
                   dronePawn->DroneBody->WakeAllRigidBodies();
               }

               // Reset controller states
               ResetPID();
               desiredNewVelocity = FVector::ZeroVector;
               initialTakeoff = true;
               altitudeReached = false;
           }
       }