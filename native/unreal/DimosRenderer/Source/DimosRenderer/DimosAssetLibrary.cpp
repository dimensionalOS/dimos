#include "DimosAssetLibrary.h"

#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/BodySetup.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

#if WITH_EDITOR
#include "StaticMeshCompiler.h"
#endif

namespace
{
TArray<TSharedPtr<FJsonValue>> VectorJson(const FVector& Value)
{
    return {MakeShared<FJsonValueNumber>(Value.X), MakeShared<FJsonValueNumber>(Value.Y),
        MakeShared<FJsonValueNumber>(Value.Z)};
}

TSharedPtr<FJsonObject> TransformJson(const FTransform& Transform)
{
    auto Object = MakeShared<FJsonObject>();
    Object->SetArrayField(TEXT("translation"), VectorJson(Transform.GetTranslation()));
    const auto Rotation = Transform.GetRotation();
    Object->SetArrayField(TEXT("rotation_xyzw"), {
        MakeShared<FJsonValueNumber>(Rotation.X), MakeShared<FJsonValueNumber>(Rotation.Y),
        MakeShared<FJsonValueNumber>(Rotation.Z), MakeShared<FJsonValueNumber>(Rotation.W)});
    Object->SetArrayField(TEXT("scale"), VectorJson(Transform.GetScale3D()));
    return Object;
}

TSharedPtr<FJsonObject> Collider(const TCHAR* Type, const FTransform& Transform)
{
    auto Object = MakeShared<FJsonObject>();
    Object->SetStringField(TEXT("type"), Type);
    Object->SetObjectField(TEXT("transform"), TransformJson(Transform));
    return Object;
}

FString JsonString(const TSharedPtr<FJsonObject>& Object)
{
    FString Json;
    FJsonSerializer::Serialize(Object.ToSharedRef(), TJsonWriterFactory<>::Create(&Json));
    return Json;
}
}

FString UDimosAssetLibrary::DescribeStaticMesh(UStaticMesh* Mesh)
{
    auto Result = MakeShared<FJsonObject>();
    Result->SetNumberField(TEXT("version"), 1);
    if (!Mesh)
    {
        Result->SetStringField(TEXT("error"), TEXT("A static mesh asset is required"));
        return JsonString(Result);
    }
#if WITH_EDITOR
    TArray<UStaticMesh*> Meshes{Mesh};
    FStaticMeshCompilingManager::Get().FinishCompilation(Meshes);
#endif
    Result->SetStringField(TEXT("asset_path"), Mesh->GetPathName());
    const auto Bounds = Mesh->GetBounds();
    auto BoundsJson = MakeShared<FJsonObject>();
    BoundsJson->SetArrayField(TEXT("origin"), VectorJson(Bounds.Origin));
    BoundsJson->SetArrayField(TEXT("extent"), VectorJson(Bounds.BoxExtent));
    BoundsJson->SetNumberField(TEXT("sphere_radius"), Bounds.SphereRadius);
    Result->SetObjectField(TEXT("bounds"), BoundsJson);

    TArray<TSharedPtr<FJsonValue>> Materials;
    for (const auto& Slot : Mesh->GetStaticMaterials())
    {
        auto Material = MakeShared<FJsonObject>();
        Material->SetStringField(TEXT("slot"), Slot.MaterialSlotName.ToString());
        auto* Interface = Slot.MaterialInterface.Get();
        Material->SetStringField(TEXT("asset_path"), Interface ? Interface->GetPathName() : TEXT(""));
        Material->SetStringField(TEXT("blend_mode"), Interface ?
            StaticEnum<EBlendMode>()->GetNameStringByValue(Interface->GetBlendMode()) : TEXT("missing"));
        Materials.Add(MakeShared<FJsonValueObject>(Material));
    }
    Result->SetArrayField(TEXT("materials"), Materials);

    TArray<TSharedPtr<FJsonValue>> Colliders;
    int32 Unsupported = 0;
    auto* Body = Mesh->GetBodySetup();
    Result->SetBoolField(TEXT("has_body_setup"), Body != nullptr);
    if (Body)
    {
        Result->SetStringField(TEXT("collision_trace"),
            StaticEnum<ECollisionTraceFlag>()->GetNameStringByValue(Body->GetCollisionTraceFlag()));
        const auto& Geometry = Body->AggGeom;
        for (const auto& Box : Geometry.BoxElems)
        {
            auto Shape = Collider(TEXT("box"), Box.GetTransform());
            Shape->SetArrayField(TEXT("size"), VectorJson(FVector(Box.X, Box.Y, Box.Z)));
            Colliders.Add(MakeShared<FJsonValueObject>(Shape));
        }
        for (const auto& Sphere : Geometry.SphereElems)
        {
            auto Shape = Collider(TEXT("sphere"), Sphere.GetTransform());
            Shape->SetNumberField(TEXT("radius"), Sphere.Radius);
            Colliders.Add(MakeShared<FJsonValueObject>(Shape));
        }
        for (const auto& Capsule : Geometry.SphylElems)
        {
            auto Shape = Collider(TEXT("capsule"), Capsule.GetTransform());
            Shape->SetNumberField(TEXT("radius"), Capsule.Radius);
            // Unreal Length excludes the hemispherical caps; its long axis is local Z.
            Shape->SetNumberField(TEXT("length"), Capsule.Length);
            Colliders.Add(MakeShared<FJsonValueObject>(Shape));
        }
        for (const auto& Convex : Geometry.ConvexElems)
        {
            auto Shape = Collider(TEXT("convex"), Convex.GetTransform());
            TArray<TSharedPtr<FJsonValue>> Vertices, Indices;
            for (const auto& Vertex : Convex.VertexData)
                Vertices.Add(MakeShared<FJsonValueArray>(VectorJson(Vertex)));
            // Source assets may omit serialized indices; use the native hull's
            // triangulation when available, without changing the asset or transform.
            const auto Triangles = Convex.IndexData.IsEmpty() ? Convex.GetChaosConvexIndices() : Convex.IndexData;
            for (int32 Index : Triangles) Indices.Add(MakeShared<FJsonValueNumber>(Index));
            Shape->SetArrayField(TEXT("vertices"), Vertices);
            Shape->SetArrayField(TEXT("indices"), Indices);
            Colliders.Add(MakeShared<FJsonValueObject>(Shape));
        }
        Unsupported = Geometry.GetElementCount() - Colliders.Num();
    }
    Result->SetArrayField(TEXT("colliders"), Colliders);
    Result->SetNumberField(TEXT("unsupported_collision_count"), Unsupported);
    return JsonString(Result);
}
