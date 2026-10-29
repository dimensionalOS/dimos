#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DimosRendererGameMode.generated.h"

class FSocket;
class FJsonObject;
class UProceduralMeshComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UMaterialInterface;

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

    FSocket* Socket = nullptr;
    TArray<uint8> ReceiveBuffer;
    TArray<uint8> SendBuffer;
    int32 SendOffset = 0;
    int64 Episode = -1;
    bool bExitAfterSend = false;
    bool bHasScene = false;

    UPROPERTY() TMap<int32, TObjectPtr<UProceduralMeshComponent>> Geometries;
    UPROPERTY() TObjectPtr<USceneCaptureComponent2D> ColorCapture;
    UPROPERTY() TObjectPtr<USceneCaptureComponent2D> DepthCapture;
    UPROPERTY() TObjectPtr<UTextureRenderTarget2D> ColorTarget;
    UPROPERTY() TObjectPtr<UTextureRenderTarget2D> DepthTarget;
    UPROPERTY() TObjectPtr<UMaterialInterface> MeshMaterial;
};
