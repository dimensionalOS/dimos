#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "DimosAssetLibrary.generated.h"

class UStaticMesh;

UCLASS()
class DIMOSRENDERER_API UDimosAssetLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /** Describe native mesh materials and simple colliders; all lengths are UE centimetres. */
    UFUNCTION(BlueprintCallable, Category = "DimOS|Assets")
    static FString DescribeStaticMesh(UStaticMesh* Mesh);
};
