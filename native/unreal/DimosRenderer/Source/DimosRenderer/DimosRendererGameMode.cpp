#include "DimosRendererGameMode.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/DirectionalLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformFileManager.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/Material.h"
#include "MaterialShared.h"
#include "MaterialShaderPrecompileMode.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/PackageName.h"
#include "ProceduralMeshComponent.h"
#include "RenderingThread.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "UnrealClient.h"

#include <limits>

#if WITH_EDITOR
#include "ShaderCompiler.h"
#include "StaticMeshCompiler.h"
#include "TextureCompiler.h"
#endif

namespace
{
constexpr uint32 MaxMessageBytes = 1024 * 1024;

TSharedPtr<FJsonObject> Message(const TCHAR* Type)
{
    auto Object = MakeShared<FJsonObject>();
    Object->SetStringField(TEXT("type"), Type);
    Object->SetNumberField(TEXT("version"), 1);
    return Object;
}

FVector Vector(const TArray<TSharedPtr<FJsonValue>>& Values)
{
    return FVector(Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber());
}

bool IsArray(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, int32 Size)
{
    const TArray<TSharedPtr<FJsonValue>>* Values;
    if (!Object || !Object->TryGetArrayField(Name, Values) || Values->Num() != Size) return false;
    for (const auto& Value : *Values)
    {
        double Number;
        if (!Value->TryGetNumber(Number) || !FMath::IsFinite(Number)) return false;
    }
    return true;
}

bool Number(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, double& Value)
{
    return Object && Object->TryGetNumberField(Name, Value) && FMath::IsFinite(Value);
}

bool Identifier(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, int32& Value)
{
    double Parsed;
    if (!Number(Object, Name, Parsed) || Parsed < 0 || Parsed > MAX_int32 || FMath::Floor(Parsed) != Parsed) return false;
    Value = static_cast<int32>(Parsed);
    return true;
}

bool ValidPose(const TSharedPtr<FJsonObject>& Object)
{
    if (!Object || !IsArray(Object, TEXT("position"), 3) || !IsArray(Object, TEXT("quaternion"), 4)) return false;
    const auto& Q = Object->GetArrayField(TEXT("quaternion"));
    double Length2 = 0;
    for (const auto& Value : Q) Length2 += FMath::Square(Value->AsNumber());
    return FMath::Abs(Length2 - 1) < 0.001;
}

const FMaterialResource* NativeMaterialResource(UMaterialInterface* Material)
{
    const auto* Resource = Material->GetMaterialResource(GMaxRHIShaderPlatform);
    if ((!Resource || !Resource->GetMaterial()) && Material->GetMaterial())
        Resource = Material->GetMaterial()->GetMaterialResource(GMaxRHIShaderPlatform);
    return Resource;
}

#if WITH_EDITOR
void FinishMaterial(UMaterialInterface* Material)
{
    const auto* Resource = NativeMaterialResource(Material);
    if (!Resource || !Resource->IsGameThreadShaderMapComplete())
        Material->ForceRecompileForRendering(EMaterialShaderPrecompileMode::Synchronous);
}
#endif

bool PrepareStaticMeshes(const TArray<UStaticMesh*>& Meshes, FString& Error)
{
#if WITH_EDITOR
    FStaticMeshCompilingManager::Get().FinishCompilation(Meshes);
#endif
    TSet<UMaterialInterface*> Materials;
    TSet<UMaterial*> BaseMaterials;
    TArray<UTexture*> Textures;
    for (auto* Mesh : Meshes)
    {
        if (!Mesh->HasValidRenderData() || Mesh->GetStaticMaterials().IsEmpty())
        { Error = FString::Printf(TEXT("Static mesh has no render data or materials: %s"), *Mesh->GetPathName()); return false; }
        for (const auto& Slot : Mesh->GetStaticMaterials())
        {
            auto* Material = Slot.MaterialInterface.Get();
            if (!Material || Material->GetBlendMode() != BLEND_Opaque)
            { Error = FString::Printf(TEXT("Static mesh requires opaque materials in every slot: %s"), *Mesh->GetPathName()); return false; }
            Materials.Add(Material);
            BaseMaterials.Add(Material->GetMaterial());
        }
    }
#if WITH_EDITOR
    // Instances without their own static permutation use their parent's shader
    // resource. Compile missing parents too, synchronously rather than queuing
    // DDC work. Reuse complete resources on reset instead of invalidating them.
    for (auto* Material : BaseMaterials)
        if (Material) FinishMaterial(Material);
#endif
    for (auto* Material : Materials)
    {
        TArray<UTexture*> UsedTextures;
        Material->GetUsedTextures(UsedTextures);
        for (auto* Texture : UsedTextures) if (Texture) Textures.AddUnique(Texture);
#if WITH_EDITOR
        // Editor -game must finish the source asset's permutations before the
        // first RGB-D capture; otherwise Unreal silently draws its gray fallback.
        FinishMaterial(Material);
#endif
    }
#if WITH_EDITOR
    FTextureCompilingManager::Get().FinishCompilation(Textures);
    if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
#endif
    for (auto* Material : Materials)
    {
        const auto* Resource = NativeMaterialResource(Material);
        if (!Resource || !Resource->IsGameThreadShaderMapComplete())
        {
            Error = FString::Printf(TEXT("Static mesh material shaders are unavailable: %s (platform=%d, resource=%s)"),
                *Material->GetPathName(), static_cast<int32>(GMaxRHIShaderPlatform), Resource ? TEXT("incomplete") : TEXT("missing"));
            return false;
        }
    }
    for (auto* Texture : Textures)
    {
        Texture->SetForceMipLevelsToBeResident(30.0f);
        Texture->WaitForStreaming();
    }
    FlushRenderingCommands();
    return true;
}
}

ADimosRendererGameMode::ADimosRendererGameMode()
{
    PrimaryActorTick.bCanEverTick = true;
    DefaultPawnClass = nullptr;
    SetActorHiddenInGame(false);
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    ColorCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ColorCamera"));
    DepthCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DepthCamera"));
    for (auto* Camera : {ColorCapture.Get(), DepthCapture.Get()})
    {
        Camera->SetupAttachment(RootComponent);
        Camera->bCaptureEveryFrame = false;
        Camera->bCaptureOnMovement = false;
        Camera->bAlwaysPersistRenderingState = false;
        Camera->bUseCustomProjectionMatrix = true;
        Camera->ShowFlags.SetMotionBlur(false);
        Camera->ShowFlags.SetTemporalAA(false);
        Camera->ShowFlags.SetAntiAliasing(false);
        Camera->ShowFlags.SetAtmosphere(false);
        Camera->ShowFlags.SetFog(false);
        Camera->ShowFlags.SetBloom(false);
        Camera->ShowFlags.SetLensFlares(false);
    }
    ColorCapture->CaptureSource = SCS_FinalColorLDR;
    ColorCapture->PostProcessSettings.bOverride_AutoExposureMethod = true;
    ColorCapture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
    ColorCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
    ColorCapture->PostProcessSettings.AutoExposureBias = 0;
    ColorCapture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    ColorCapture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = false;
    DepthCapture->CaptureSource = SCS_SceneDepth;
}

void ADimosRendererGameMode::StartPlay()
{
    Super::StartPlay();
    MeshMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Dimos.M_Dimos"));
    if (!MeshMaterial)
    {
        UE_LOG(LogTemp, Error, TEXT("DimOS material missing; run Scripts/prepare_assets.py before cooking."));
        FPlatformMisc::RequestExit(false);
        return;
    }
#if WITH_EDITOR
    // Editor -game can otherwise return gray fallback pixels while the newly
    // generated material compiles. Packaged builds already contain its shaders.
    MeshMaterial->ForceRecompileForRendering();
    if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
#endif
    for (int32 Index = 0; Index < 2; ++Index)
    {
        auto* Light = GetWorld()->SpawnActor<ADirectionalLight>();
        Light->GetLightComponent()->SetMobility(EComponentMobility::Movable);
        Light->SetActorRotation(Index == 0 ? FRotator(-45, -35, 0) : FRotator(-25, 145, 0));
        Light->GetLightComponent()->SetIntensity(Index == 0 ? 5.0f : 1.0f);
    }
    int32 Port = 0;
    if (!FParse::Value(FCommandLine::Get(), TEXT("DimosPort="), Port) || Port < 1 || Port > 65535)
    {
        UE_LOG(LogTemp, Error, TEXT("A valid -DimosPort is required."));
        FPlatformMisc::RequestExit(false);
        return;
    }
    auto* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    Socket = Subsystem->CreateSocket(NAME_Stream, TEXT("DimOS renderer"), false);
    auto Address = Subsystem->CreateInternetAddr();
    bool bValid;
    Address->SetIp(TEXT("127.0.0.1"), bValid);
    Address->SetPort(Port);
    Socket->SetNoDelay(true);
    int32 BufferSize;
    Socket->SetSendBufferSize(8 * 1024 * 1024, BufferSize);
    Socket->SetReceiveBufferSize(8 * 1024 * 1024, BufferSize);
    if (!Socket->Connect(*Address))
    {
        UE_LOG(LogTemp, Error, TEXT("Could not connect to the DimOS loopback socket."));
        FPlatformMisc::RequestExit(false);
        return;
    }
    Socket->SetNonBlocking(true);
    Reply(Message(TEXT("ready")));
}

void ADimosRendererGameMode::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Socket)
    {
        Socket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
        Socket = nullptr;
    }
    Super::EndPlay(Reason);
}

void ADimosRendererGameMode::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (Socket && !PumpSocket()) FPlatformMisc::RequestExit(false);
}

bool ADimosRendererGameMode::PumpSocket()
{
    while (SendOffset < SendBuffer.Num())
    {
        int32 Sent = 0;
        if (!Socket->Send(SendBuffer.GetData() + SendOffset, SendBuffer.Num() - SendOffset, Sent))
        {
            return ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode() == SE_EWOULDBLOCK;
        }
        if (Sent == 0) return false;
        SendOffset += Sent;
    }
    SendBuffer.Reset();
    SendOffset = 0;
    if (bExitAfterSend) return false;
    // Recv also observes EOF: HasPendingData alone hides a peer that closed idle.
    uint8 Chunk[64 * 1024];
    for (;;)
    {
        int32 Received = 0;
        if (!Socket->Recv(Chunk, sizeof(Chunk), Received)) return false;
        if (Received == 0) break; // UE nonblocking Recv reports would-block as success/0.
        if (ReceiveBuffer.Num() + Received > MaxMessageBytes + 4) return false;
        ReceiveBuffer.Append(Chunk, Received);
    }
    if (ReceiveBuffer.Num() < 4) return true;
    const uint32 Length = uint32(ReceiveBuffer[0]) | uint32(ReceiveBuffer[1]) << 8 |
                          uint32(ReceiveBuffer[2]) << 16 | uint32(ReceiveBuffer[3]) << 24;
    if (Length == 0 || Length > MaxMessageBytes) return false;
    if (ReceiveBuffer.Num() < int64(Length) + 4) return true;
    FUTF8ToTCHAR Text(reinterpret_cast<const ANSICHAR*>(ReceiveBuffer.GetData() + 4), Length);
    const FString Json(Text.Length(), Text.Get());
    ReceiveBuffer.RemoveAt(0, Length + 4, EAllowShrinking::No);
    TSharedPtr<FJsonObject> Command;
    FString Error;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Command) || !Command)
        Error = TEXT("Invalid JSON command");
    else if (HandleCommand(Command, Error))
        return true;
    Fail(Error);
    return true;
}

void ADimosRendererGameMode::Reply(const TSharedPtr<FJsonObject>& Header, const TArray<uint8>& Payload)
{
    FString Json;
    FJsonSerializer::Serialize(Header.ToSharedRef(), TJsonWriterFactory<>::Create(&Json));
    const FTCHARToUTF8 Text(*Json);
    const uint32 Length = Text.Length();
    SendBuffer.Reset(4 + Length + Payload.Num());
    for (int32 Byte = 0; Byte < 4; ++Byte) SendBuffer.Add((Length >> (Byte * 8)) & 255);
    SendBuffer.Append(reinterpret_cast<const uint8*>(Text.Get()), Length);
    SendBuffer.Append(Payload);
    SendOffset = 0;
}

void ADimosRendererGameMode::Fail(const FString& Error)
{
    auto Header = Message(TEXT("error"));
    Header->SetStringField(TEXT("message"), Error);
    Reply(Header);
}

void ADimosRendererGameMode::SetPose(USceneComponent* Component, const TSharedPtr<FJsonObject>& Pose)
{
    const auto& Q = Pose->GetArrayField(TEXT("quaternion"));
    Component->SetWorldLocationAndRotation(Vector(Pose->GetArrayField(TEXT("position"))),
        FQuat(Q[0]->AsNumber(), Q[1]->AsNumber(), Q[2]->AsNumber(), Q[3]->AsNumber()), false, nullptr,
        ETeleportType::TeleportPhysics);
}

bool ADimosRendererGameMode::HandleCommand(const TSharedPtr<FJsonObject>& Command, FString& Error)
{
    FString Type;
    if (!Command->TryGetStringField(TEXT("type"), Type)) { Error = TEXT("Missing command type"); return false; }
    if (Type == TEXT("load"))
    {
        FString Path;
        if (!Command->TryGetStringField(TEXT("manifest"), Path)) { Error = TEXT("Missing manifest path"); return false; }
        if (!LoadScene(Path, Error)) return false;
        auto Header = Message(TEXT("loaded"));
        Header->SetNumberField(TEXT("geometries"), Geometries.Num());
        Reply(Header);
        return true;
    }
    if (Type == TEXT("capture")) return Capture(Command, Error);
    if (Type == TEXT("reset"))
    {
        double NextEpisode;
        if (!Number(Command, TEXT("episode"), NextEpisode) || NextEpisode < 0 ||
            NextEpisode > 9007199254740991.0 || FMath::Floor(NextEpisode) != NextEpisode)
        { Error = TEXT("Invalid episode"); return false; }
        Episode = static_cast<int64>(NextEpisode);
        ColorCapture->bCameraCutThisFrame = true;
        DepthCapture->bCameraCutThisFrame = true;
        auto Header = Message(TEXT("reset"));
        Header->SetNumberField(TEXT("episode"), Episode);
        Reply(Header);
        return true;
    }
    if (Type == TEXT("shutdown"))
    {
        Reply(Message(TEXT("shutdown")));
        bExitAfterSend = true;
        return true;
    }
    Error = TEXT("Unknown command type");
    return false;
}

bool ADimosRendererGameMode::LoadScene(const FString& Path, FString& Error)
{
    FString Json;
    TSharedPtr<FJsonObject> Manifest;
    // The manifest is external runtime input, not a cooked Unreal content asset.
    // Keep this lookup independent of cooked content and package mount paths.
    TUniquePtr<IFileHandle> File(IPlatformFile::GetPlatformPhysical().OpenRead(*Path));
    if (!File || File->Size() <= 0 || File->Size() > 256 * 1024 * 1024)
    { Error = TEXT("Cannot open scene manifest or manifest exceeds 256 MiB"); return false; }
    TArray<uint8> Bytes;
    Bytes.SetNumUninitialized(File->Size());
    if (!File->Read(Bytes.GetData(), Bytes.Num()))
    { Error = TEXT("Cannot read scene manifest"); return false; }
    FFileHelper::BufferToString(Json, Bytes.GetData(), Bytes.Num());
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Manifest))
    { Error = TEXT("Invalid scene manifest JSON"); return false; }
    const TArray<TSharedPtr<FJsonValue>>* Meshes;
    const TArray<TSharedPtr<FJsonValue>>* Geoms;
    double Version;
    if (!Number(Manifest, TEXT("version"), Version) || Version != 1)
    { Error = TEXT("Unsupported manifest version"); return false; }
    if (!Manifest->TryGetArrayField(TEXT("meshes"), Meshes) || !Manifest->TryGetArrayField(TEXT("geoms"), Geoms))
    { Error = TEXT("Manifest needs meshes and geoms"); return false; }
    TMap<FString, TSharedPtr<FJsonObject>> MeshMap;
    for (const auto& Value : *Meshes)
    {
        if (Value->Type != EJson::Object) { Error = TEXT("Invalid mesh"); return false; }
        auto Mesh = Value->AsObject();
        FString Id;
        const TArray<TSharedPtr<FJsonValue>>* Vertices;
        const TArray<TSharedPtr<FJsonValue>>* Normals;
        const TArray<TSharedPtr<FJsonValue>>* Triangles;
        if (!Mesh->TryGetStringField(TEXT("id"), Id) || Id.IsEmpty() || MeshMap.Contains(Id) ||
            !Mesh->TryGetArrayField(TEXT("vertices"), Vertices) || Vertices->IsEmpty() ||
            !Mesh->TryGetArrayField(TEXT("normals"), Normals) || Normals->Num() != Vertices->Num() ||
            !Mesh->TryGetArrayField(TEXT("triangles"), Triangles) || Triangles->IsEmpty() || Triangles->Num() % 3 != 0)
        { Error = TEXT("Invalid mesh arrays or duplicate mesh id"); return false; }
        for (const auto* Vectors : {Vertices, Normals})
        {
            for (const auto& Item : *Vectors)
            {
                const TArray<TSharedPtr<FJsonValue>>* Components;
                if (!Item->TryGetArray(Components) || Components->Num() != 3)
                { Error = TEXT("Mesh vector must contain three finite numbers"); return false; }
                for (const auto& Component : *Components)
                {
                    double Scalar;
                    if (!Component->TryGetNumber(Scalar) || !FMath::IsFinite(Scalar))
                    { Error = TEXT("Mesh vector must contain three finite numbers"); return false; }
                }
            }
        }
        for (const auto& Triangle : *Triangles)
        {
            double Index;
            if (!Triangle->TryGetNumber(Index) || !FMath::IsFinite(Index) || Index < 0 ||
                Index >= Vertices->Num() || FMath::Floor(Index) != Index)
            { Error = TEXT("Mesh triangle index is out of bounds"); return false; }
        }
        MeshMap.Add(Id, Mesh);
    }
    // Validate the complete request before mutating the current scene.
    TSet<int32> Ids;
    TMap<int32, UStaticMesh*> StaticMeshes;
    TArray<UStaticMesh*> UniqueStaticMeshes;
    for (const auto& Value : *Geoms)
    {
        if (Value->Type != EJson::Object) { Error = TEXT("Invalid geometry"); return false; }
        auto Geom = Value->AsObject();
        FString MeshId;
        int32 Id;
        if (!ValidPose(Geom) || !Identifier(Geom, TEXT("id"), Id) || Ids.Contains(Id))
        { Error = TEXT("Invalid geometry pose or duplicate id"); return false; }
        Ids.Add(Id);
        if (Geom->HasField(TEXT("static_mesh")))
        {
            FString AssetPath;
            if (!Geom->TryGetStringField(TEXT("static_mesh"), AssetPath) ||
                !FPackageName::IsValidObjectPath(AssetPath) ||
                (!AssetPath.StartsWith(TEXT("/Game/")) && !AssetPath.StartsWith(TEXT("/Engine/"))) ||
                !IsArray(Geom, TEXT("scale"), 3) || Geom->HasField(TEXT("mesh_id")) ||
                Geom->HasField(TEXT("rgba")) || Geom->HasField(TEXT("roughness")))
            { Error = TEXT("Static geometry needs a full content object path and scale, without procedural material fields"); return false; }
            for (const auto& Axis : Geom->GetArrayField(TEXT("scale")))
                if (Axis->AsNumber() <= 0)
                { Error = TEXT("Static mesh scale must be positive and finite"); return false; }
            auto* StaticMesh = LoadObject<UStaticMesh>(nullptr, *AssetPath);
            if (!StaticMesh)
            { Error = FString::Printf(TEXT("Static mesh asset not found: %s"), *AssetPath); return false; }
            StaticMeshes.Add(Id, StaticMesh);
            UniqueStaticMeshes.AddUnique(StaticMesh);
            continue;
        }
        if (!IsArray(Geom, TEXT("rgba"), 4) || !Geom->TryGetStringField(TEXT("mesh_id"), MeshId) ||
            !MeshMap.Contains(MeshId) || Geom->HasField(TEXT("scale")))
        { Error = TEXT("Invalid procedural geometry color or mesh reference"); return false; }
        const auto& RGBA = Geom->GetArrayField(TEXT("rgba"));
        for (const auto& Channel : RGBA)
            if (Channel->AsNumber() < 0 || Channel->AsNumber() > 1)
            { Error = TEXT("Geometry color must lie in [0, 1]"); return false; }
        if (RGBA[3]->AsNumber() != 1)
        { Error = TEXT("Only opaque geometry is supported"); return false; }
        double Roughness;
        if (Geom->HasField(TEXT("roughness")) &&
            (!Number(Geom, TEXT("roughness"), Roughness) || Roughness < 0 || Roughness > 1))
        { Error = TEXT("Roughness must lie in [0, 1]"); return false; }
    }
    if (!PrepareStaticMeshes(UniqueStaticMeshes, Error)) return false;
    for (auto& Pair : Geometries) Pair.Value->DestroyComponent();
    Geometries.Empty();
    for (const auto& Value : *Geoms)
    {
        auto Geom = Value->AsObject();
        const auto Id = static_cast<int32>(Geom->GetNumberField(TEXT("id")));
        if (auto* const* StaticMesh = StaticMeshes.Find(Id))
        {
            auto* Component = NewObject<UStaticMeshComponent>(this);
            Component->SetMobility(EComponentMobility::Movable);
            Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Component->SetGenerateOverlapEvents(false);
            Component->SetSimulatePhysics(false);
            Component->SetCanEverAffectNavigation(false);
            Component->SetStaticMesh(*StaticMesh);
            Component->SetupAttachment(RootComponent);
            Component->SetRelativeScale3D(Vector(Geom->GetArrayField(TEXT("scale"))));
            Component->RegisterComponent();
            SetPose(Component, Geom);
            Geometries.Add(Id, Component);
            continue;
        }
        auto Mesh = MeshMap[Geom->GetStringField(TEXT("mesh_id"))];
        TArray<FVector> Vertices, Normals;
        TArray<int32> Triangles;
        TArray<FVector2D> UV;
        for (const auto& V : Mesh->GetArrayField(TEXT("vertices"))) Vertices.Add(Vector(V->AsArray()));
        for (const auto& N : Mesh->GetArrayField(TEXT("normals"))) Normals.Add(Vector(N->AsArray()));
        for (const auto& T : Mesh->GetArrayField(TEXT("triangles"))) Triangles.Add(static_cast<int32>(T->AsNumber()));
        UV.Init(FVector2D::ZeroVector, Vertices.Num());
        auto* Component = NewObject<UProceduralMeshComponent>(this);
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetupAttachment(RootComponent);
        Component->RegisterComponent();
        Component->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV, {}, {}, false);
        auto* Material = UMaterialInstanceDynamic::Create(MeshMaterial, this);
        const auto& RGBA = Geom->GetArrayField(TEXT("rgba"));
        Material->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(RGBA[0]->AsNumber(), RGBA[1]->AsNumber(), RGBA[2]->AsNumber(), RGBA[3]->AsNumber()));
        double Roughness = 0.7;
        Geom->TryGetNumberField(TEXT("roughness"), Roughness);
        Material->SetScalarParameterValue(TEXT("Roughness"), Roughness);
        Component->SetMaterial(0, Material);
        SetPose(Component, Geom);
        Geometries.Add(Id, Component);
    }
    bHasScene = true;
    Episode = -1;
    return true;
}

bool ADimosRendererGameMode::Capture(const TSharedPtr<FJsonObject>& Command, FString& Error)
{
    if (!bHasScene) { Error = TEXT("Load a scene first"); return false; }
    const TSharedPtr<FJsonObject>* Camera;
    const TArray<TSharedPtr<FJsonValue>>* Poses;
    if (!Command->TryGetObjectField(TEXT("camera"), Camera) || !ValidPose(*Camera) ||
        !Command->TryGetArrayField(TEXT("poses"), Poses))
    { Error = TEXT("Invalid camera or pose list"); return false; }
    int32 Width, Height;
    double Fx, Fy, Cx, Cy;
    if (!Identifier(*Camera, TEXT("width"), Width) || !Identifier(*Camera, TEXT("height"), Height) ||
        !Number(*Camera, TEXT("fx"), Fx) || !Number(*Camera, TEXT("fy"), Fy) ||
        !Number(*Camera, TEXT("cx"), Cx) || !Number(*Camera, TEXT("cy"), Cy) ||
        Width < 1 || Height < 1 || Width > 4096 || Height > 4096 || Fx <= 0 || Fy <= 0)
    { Error = TEXT("Invalid camera dimensions or intrinsics"); return false; }
    double Near = 0.01, Far = 1000.0;
    if (((*Camera)->HasField(TEXT("near_m")) && !Number(*Camera, TEXT("near_m"), Near)) ||
        ((*Camera)->HasField(TEXT("far_m")) && !Number(*Camera, TEXT("far_m"), Far)) || Near <= 0 || Far <= Near)
    { Error = TEXT("Invalid camera clipping range"); return false; }
    for (const TCHAR* Key : {TEXT("episode"), TEXT("sequence"), TEXT("physics_tick"), TEXT("sim_time"), TEXT("timestamp")})
    {
        double Value;
        if (!Number(Command, Key, Value) || Value < 0)
        { Error = TEXT("Missing or invalid snapshot identity"); return false; }
    }
    for (const TCHAR* Key : {TEXT("episode"), TEXT("sequence"), TEXT("physics_tick")})
    {
        const double Value = Command->GetNumberField(Key);
        if (Value > 9007199254740991.0 || FMath::Floor(Value) != Value)
        { Error = TEXT("Snapshot counters must be exactly representable integers"); return false; }
    }
    if (Episode >= 0 && Command->GetNumberField(TEXT("episode")) != Episode)
    { Error = TEXT("Reset before capturing a new episode"); return false; }
    TSet<int32> Ids;
    for (const auto& Value : *Poses)
    {
        if (Value->Type != EJson::Object) { Error = TEXT("Invalid geometry pose"); return false; }
        auto Pose = Value->AsObject();
        int32 Id;
        if (!ValidPose(Pose) || !Identifier(Pose, TEXT("id"), Id) || !Geometries.Contains(Id) || Ids.Contains(Id))
        { Error = TEXT("Invalid or unknown geometry pose"); return false; }
        Ids.Add(Id);
    }
    if (Ids.Num() != Geometries.Num())
    { Error = TEXT("Capture requires a complete geometry snapshot"); return false; }
    if (!ColorTarget || ColorTarget->SizeX != Width || ColorTarget->SizeY != Height)
    {
        ColorTarget = NewObject<UTextureRenderTarget2D>(this);
        ColorTarget->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
        ColorTarget->UpdateResourceImmediate(true);
        DepthTarget = NewObject<UTextureRenderTarget2D>(this);
        DepthTarget->ClearColor = FLinearColor(MAX_flt, 0, 0, 0);
        DepthTarget->InitCustomFormat(Width, Height, PF_R32_FLOAT, true);
        DepthTarget->UpdateResourceImmediate(true);
        ColorCapture->TextureTarget = ColorTarget;
        DepthCapture->TextureTarget = DepthTarget;
    }
    const double Started = FPlatformTime::Seconds();
    for (const auto& Value : *Poses)
    {
        auto Pose = Value->AsObject();
        SetPose(Geometries[static_cast<int32>(Pose->GetNumberField(TEXT("id")))], Pose);
    }
    FMatrix Projection = FMatrix::Identity;
    Projection.M[0][0] = 2 * Fx / Width;
    Projection.M[1][1] = 2 * Fy / Height;
    Projection.M[2][0] = 2 * (Cx + 0.5) / Width - 1;
    Projection.M[2][1] = 1 - 2 * (Cy + 0.5) / Height;
    // Finite reversed-Z projection clips color and depth at the same far plane.
    Projection.M[2][2] = Near / (Near - Far);
    Projection.M[2][3] = 1;
    Projection.M[3][2] = 100 * Near * Far / (Far - Near);
    Projection.M[3][3] = 0;
    for (auto* CaptureComponent : {ColorCapture.Get(), DepthCapture.Get()})
    {
        SetPose(CaptureComponent, *Camera);
        CaptureComponent->CustomProjectionMatrix = Projection;
        CaptureComponent->FOVAngle = FMath::RadiansToDegrees(2 * FMath::Atan(Width / (2 * Fx)));
        CaptureComponent->bCameraCutThisFrame = true;
    }
    GetWorld()->SendAllEndOfFrameUpdates();
    FlushRenderingCommands();
    ColorCapture->CaptureScene();
    DepthCapture->CaptureScene();
    TArray<FColor> Colors;
    TArray<FLinearColor> Depth;
    FReadSurfaceDataFlags ColorFlags(RCM_UNorm);
    ColorFlags.SetLinearToGamma(false);
    FReadSurfaceDataFlags DepthFlags(RCM_MinMax);
    DepthFlags.SetLinearToGamma(false);
    if (!ColorTarget->GameThread_GetRenderTargetResource()->ReadPixels(Colors, ColorFlags) ||
        !DepthTarget->GameThread_GetRenderTargetResource()->ReadLinearColorPixels(Depth, DepthFlags) ||
        Colors.Num() != Width * Height || Depth.Num() != Width * Height)
    { Error = TEXT("GPU readback failed"); return false; }
    TArray<uint8> Payload;
    const int32 Pixels = Width * Height;
    Payload.SetNumUninitialized(Pixels * 8);
    for (int32 Index = 0; Index < Pixels; ++Index)
    {
        Payload[Index * 4] = Colors[Index].R;
        Payload[Index * 4 + 1] = Colors[Index].G;
        Payload[Index * 4 + 2] = Colors[Index].B;
        Payload[Index * 4 + 3] = 255;
        // UE 5.7 adds 1e-8 to the inverse-depth denominator when linearizing
        // device Z. Undo that bias, then classify the cleared far plane as no hit.
        const double RawCentimeters = Depth[Index].R;
        float Meters = 0.01 * RawCentimeters / (1.0 - 1e-8 * RawCentimeters);
        if (!FMath::IsFinite(Meters) || Meters <= 0 || Meters >= Far * (1.0 - 1e-6))
            Meters = std::numeric_limits<float>::infinity();
        FMemory::Memcpy(Payload.GetData() + Pixels * 4 + Index * 4, &Meters, 4);
    }
    auto Header = Message(TEXT("frame"));
    for (const TCHAR* Key : {TEXT("episode"), TEXT("sequence"), TEXT("physics_tick"), TEXT("sim_time"), TEXT("timestamp")})
        Header->SetField(Key, Command->TryGetField(Key));
    Header->SetNumberField(TEXT("width"), Width);
    Header->SetNumberField(TEXT("height"), Height);
    Header->SetNumberField(TEXT("rgba_bytes"), Pixels * 4);
    Header->SetNumberField(TEXT("depth_bytes"), Pixels * 4);
    Header->SetStringField(TEXT("depth_unit"), TEXT("m"));
    Header->SetStringField(TEXT("depth_kind"), TEXT("axial"));
    Header->SetNumberField(TEXT("capture_ms"), (FPlatformTime::Seconds() - Started) * 1000);
    Reply(Header, Payload);
    Episode = static_cast<int64>(Command->GetNumberField(TEXT("episode")));
    return true;
}
