// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Actor.h"
#include "Pathfinder.generated.h"

USTRUCT()
struct FGridNode {
    GENERATED_BODY()

    UPROPERTY()
    int32 X = 0;

    UPROPERTY()
    int32 Y = 0;

    UPROPERTY()
    bool bWalkable = true;

    UPROPERTY()
    float Cost = TNumericLimits<float>::Max();

    UPROPERTY()
    float HeuristicCost = 0.0f;

    UPROPERTY()
    float TotalCost = TNumericLimits<float>::Max();

    UPROPERTY()
    int32 ParentIndex = -1;

    UPROPERTY()
    bool bVisited = false;

    UPROPERTY()
    bool bInOpenSet = false;

    UPROPERTY()
    bool bInClosedSet = false;
};

UENUM(BlueprintType)
enum class EPathfindingAlgorithm : uint8 {
    Dijkstra UMETA(DisplayName = "Dijkstra"),
    AStar UMETA(DisplayName = "A*")
};

UENUM()
enum class ESearchDebugEventType : uint8 {
    AddedToOpenSet,
    AddedToClosedSet
};

USTRUCT()
struct FSearchDebugEvent {
    GENERATED_BODY()

    UPROPERTY()
    int32 NodeIndex = -1;

    UPROPERTY()
    ESearchDebugEventType EventType = ESearchDebugEventType::AddedToOpenSet;
};

UCLASS()
class HW1_MAZE_API APathfinder : public AActor {
    GENERATED_BODY()

  public:
    // Sets default values for this actor's properties
    APathfinder();

  protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

  public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;
    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    FVector GridOrigin = FVector(0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    int32 GridWidth = 20;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    int32 GridHeight = 20;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    float CellSize = 100.0f;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    float WallCheckHalfExtent = 40.0f;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    float WallCheckHalfHeight = 80.0f;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    float DebugZOffset = 30.0f;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    ATargetPoint *StartPoint = nullptr;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    ATargetPoint *FinishPoint = nullptr;

    UPROPERTY(EditAnywhere, Category = "Pathfinding")
    EPathfindingAlgorithm SearchAlgorithm = EPathfindingAlgorithm::Dijkstra;

    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawGridDebug = true;

    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawOnlyBlockedCells = false;

    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawStartFinishDebug = true;
    
    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawPathDebug = true;

    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawSearchStateDebug = true;

    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bAnimateSearchDebug = true;

    UPROPERTY(EditAnywhere, Category = "Debug", meta = (ClampMin = "0.1"))
    float DebugIterationsPerSecond = 8.0f;

  private:
    UPROPERTY()
    TArray<FGridNode> GridNodes;

    UPROPERTY()
    TArray<FSearchDebugEvent> SearchDebugEvents;

    int32 StartNodeIndex = -1;
    int32 FinishNodeIndex = -1;

    TArray<int32> FinalPath;
    TArray<bool> AnimatedOpenSetStates;
    TArray<bool> AnimatedClosedSetStates;
    float DebugAnimationElapsed = 0.0f;
    int32 RevealedSearchEventCount = 0;
    int32 RevealedPathNodeCount = 0;
    bool bAnimationFinished = false;

    FVector GridToWorld(int32 X, int32 Y) const;
    bool WorldToGrid(const FVector &WorldLocation, int32 &OutX,
                     int32 &OutY) const;
    int32 GetNodeIndex(int32 X, int32 Y) const;
    bool IsValidCell(int32 X, int32 Y) const;
    bool IsCellWalkable(int32 X, int32 Y) const;

    void BuildGrid();
    bool CheckWallAtCell(int32 X, int32 Y) const;
    bool ResolveStartAndFinish();

    void ResetPathfindingState();
    void ResetDebugAnimationState();
    void AddSearchDebugEvent(int32 NodeIndex, ESearchDebugEventType EventType);
    void UpdateDebugAnimation(float DeltaTime);
    void GetNeighborIndices(int32 NodeIndex, TArray<int32> &OutNeighbors) const;
    int32 GetBestOpenSetIndex(const TArray<int32> &OpenSet,
                              bool bUseHeuristic) const;
    float CalculateHeuristicCost(int32 NodeIndex) const;
    bool FindPathDijkstra();
    bool FindPathAStar();
    bool ReconstructPath();

    void DrawGridDebug(bool bPersistent = true, float LifeTime = 60.0f) const;
    void DrawSearchStateDebug(bool bPersistent = true,
                              float LifeTime = 60.0f) const;
    void DrawStartFinishDebug(bool bPersistent = true,
                              float LifeTime = 60.0f) const;
    void DrawPathDebug(bool bPersistent = true, float LifeTime = 60.0f) const;
};
