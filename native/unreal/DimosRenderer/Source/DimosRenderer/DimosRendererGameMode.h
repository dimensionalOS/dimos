#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DimosRendererGameMode.generated.h"

class FSocket;
class FJsonObject;
class USceneComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UMaterialInterface;
struct FDimosCaptureReadback;

UCLASS()
class DIMOSRENDERER_API ADimosRendererGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    ADimosRendererGameMode();
    virtual void StartPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    bool HandleCommand(const TSharedPtr<FJsonObject>& Command, FString& Error);
    bool LoadScene(const FString& Path, FString& Error);
    bool Capture(const TSharedPtr<FJsonObject>& Command, FString& Error);
    void Reply(const TSharedPtr<FJsonObject>& Header, const TArray<uint8>& Payload = {});
    void Fail(const FString& Message);
    void SetPose(USceneComponent* Component, const TSharedPtr<FJsonObject>& Pose);
    bool PumpSocket();
    bool FlushSocket();
    void PollCapture();

    FSocket* Socket = nullptr;
    TArray<uint8> ReceiveBuffer;
    TArray<uint8> SendBuffer;
    int32 SendOffset = 0;
    FString CaptureEpoch;
    FString CaptureView;
    bool bExitAfterSend = false;
    bool bHasScene = false;
    bool bHeadless = false;
    TSharedPtr<FDimosCaptureReadback, ESPMode::ThreadSafe> PendingCapture;

    UPROPERTY() TMap<int32, TObjectPtr<USceneComponent>> Geometries;
    UPROPERTY() TObjectPtr<USceneCaptureComponent2D> ColorCapture;
    UPROPERTY() TObjectPtr<USceneCaptureComponent2D> DepthCapture;
    // Bounded resolution cache: switching cameras must not allocate GPU targets each frame.
    UPROPERTY() TArray<TObjectPtr<UTextureRenderTarget2D>> ColorTargets;
    UPROPERTY() TArray<TObjectPtr<UTextureRenderTarget2D>> DepthTargets;
    UPROPERTY() TObjectPtr<UTextureRenderTarget2D> ColorTarget;
    UPROPERTY() TObjectPtr<UTextureRenderTarget2D> DepthTarget;
    UPROPERTY() TObjectPtr<UMaterialInterface> MeshMaterial;
};
