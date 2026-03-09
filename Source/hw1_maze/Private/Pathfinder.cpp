#include "Pathfinder.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"

APathfinder::APathfinder() { PrimaryActorTick.bCanEverTick = true; }

void APathfinder::BeginPlay() {
    Super::BeginPlay();

    ResetDebugAnimationState();

    BuildGrid();

    if (bDrawGridDebug && !bAnimateSearchDebug) {
        DrawGridDebug();
    }

    const bool bPointsResolved = ResolveStartAndFinish();

    if (bPointsResolved && bDrawStartFinishDebug && !bAnimateSearchDebug) {
        DrawStartFinishDebug();
    }

    if (bPointsResolved) {
        const bool bPathFound =
            SearchAlgorithm == EPathfindingAlgorithm::AStar
                ? FindPathAStar()
                : FindPathDijkstra();

        if (bPathFound) {
            UE_LOG(LogTemp, Warning,
                   TEXT("%s path found. Path length = %d"),
                   SearchAlgorithm == EPathfindingAlgorithm::AStar ? TEXT("A*")
                                                                   : TEXT("Dijkstra"),
                   FinalPath.Num());

            if (bDrawSearchStateDebug && !bAnimateSearchDebug) {
                DrawSearchStateDebug();
            }

            if (bDrawPathDebug && !bAnimateSearchDebug) {
                DrawPathDebug();
            }
        } else {
            UE_LOG(LogTemp, Error, TEXT("%s could not find a path"),
                   SearchAlgorithm == EPathfindingAlgorithm::AStar ? TEXT("A*")
                                                                   : TEXT("Dijkstra"));

            if (bDrawSearchStateDebug && !bAnimateSearchDebug) {
                DrawSearchStateDebug();
            }
        }
    }
}

void APathfinder::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    UpdateDebugAnimation(DeltaTime);
}

FVector APathfinder::GridToWorld(int32 X, int32 Y) const {
    const float WorldX = GridOrigin.X + X * CellSize + CellSize * 0.5f;
    const float WorldY = GridOrigin.Y + Y * CellSize + CellSize * 0.5f;
    const float WorldZ = GridOrigin.Z + DebugZOffset;

    return FVector(WorldX, WorldY, WorldZ);
}

bool APathfinder::WorldToGrid(const FVector &WorldLocation, int32 &OutX,
                              int32 &OutY) const {
    OutX = FMath::FloorToInt((WorldLocation.X - GridOrigin.X) / CellSize);
    OutY = FMath::FloorToInt((WorldLocation.Y - GridOrigin.Y) / CellSize);

    return IsValidCell(OutX, OutY);
}

int32 APathfinder::GetNodeIndex(int32 X, int32 Y) const {
    return Y * GridWidth + X;
}

bool APathfinder::IsValidCell(int32 X, int32 Y) const {
    return X >= 0 && X < GridWidth && Y >= 0 && Y < GridHeight;
}

bool APathfinder::IsCellWalkable(int32 X, int32 Y) const {
    if (!IsValidCell(X, Y)) {
        return false;
    }

    const int32 Index = GetNodeIndex(X, Y);
    if (!GridNodes.IsValidIndex(Index)) {
        return false;
    }

    return GridNodes[Index].bWalkable;
}

void APathfinder::BuildGrid() {
    GridNodes.Empty();
    GridNodes.Reserve(GridWidth * GridHeight);

    for (int32 Y = 0; Y < GridHeight; ++Y) {
        for (int32 X = 0; X < GridWidth; ++X) {
            FGridNode Node;
            Node.X = X;
            Node.Y = Y;

            const bool bHasWall = CheckWallAtCell(X, Y);
            Node.bWalkable = !bHasWall;

            GridNodes.Add(Node);
        }
    }
}

bool APathfinder::CheckWallAtCell(int32 X, int32 Y) const {
    UWorld *World = GetWorld();
    if (!World) {
        return false;
    }

    const FVector Center = GridToWorld(X, Y);
    const float ProbeHalfExtent =
        FMath::Max(1.0f, FMath::Min(WallCheckHalfExtent, CellSize * 0.2f));
    const FVector BoxHalfExtent(ProbeHalfExtent, ProbeHalfExtent,
                                WallCheckHalfHeight);

    TArray<FOverlapResult> Overlaps;

    FCollisionQueryParams Params;
    Params.AddIgnoredActor(this);

    const bool bHit = World->OverlapMultiByChannel(
        Overlaps, Center, FQuat::Identity, ECC_WorldStatic,
        FCollisionShape::MakeBox(BoxHalfExtent), Params);

    if (!bHit) {
        return false;
    }

    for (const FOverlapResult &Result : Overlaps) {
        AActor *HitActor = Result.GetActor();
        if (!HitActor) {
            continue;
        }

        if (HitActor->ActorHasTag(FName("Wall"))) {
            return true;
        }
    }

    return false;
}

bool APathfinder::ResolveStartAndFinish() {
    StartNodeIndex = -1;
    FinishNodeIndex = -1;

    if (!StartPoint || !FinishPoint) {
        UE_LOG(LogTemp, Error,
               TEXT("StartPoint or FinishPoint is not assigned"));
        return false;
    }

    int32 StartX = -1;
    int32 StartY = -1;
    int32 FinishX = -1;
    int32 FinishY = -1;

    const bool bStartValid =
        WorldToGrid(StartPoint->GetActorLocation(), StartX, StartY);
    const bool bFinishValid =
        WorldToGrid(FinishPoint->GetActorLocation(), FinishX, FinishY);

    if (!bStartValid || !bFinishValid) {
        UE_LOG(LogTemp, Error, TEXT("Start or Finish is outside the grid"));
        return false;
    }

    if (!IsCellWalkable(StartX, StartY)) {
        UE_LOG(LogTemp, Error, TEXT("Start cell is blocked: (%d, %d)"), StartX,
               StartY);
        return false;
    }

    if (!IsCellWalkable(FinishX, FinishY)) {
        UE_LOG(LogTemp, Error, TEXT("Finish cell is blocked: (%d, %d)"),
               FinishX, FinishY);
        return false;
    }

    StartNodeIndex = GetNodeIndex(StartX, StartY);
    FinishNodeIndex = GetNodeIndex(FinishX, FinishY);

    UE_LOG(LogTemp, Warning, TEXT("Start grid: (%d, %d), index=%d"), StartX,
           StartY, StartNodeIndex);
    UE_LOG(LogTemp, Warning, TEXT("Finish grid: (%d, %d), index=%d"), FinishX,
           FinishY, FinishNodeIndex);

    return true;
}

void APathfinder::ResetPathfindingState() {
    FinalPath.Empty();
    SearchDebugEvents.Empty();

    for (FGridNode &Node : GridNodes) {
        Node.Cost = TNumericLimits<float>::Max();
        Node.HeuristicCost = 0.0f;
        Node.TotalCost = TNumericLimits<float>::Max();
        Node.ParentIndex = -1;
        Node.bVisited = false;
        Node.bInOpenSet = false;
        Node.bInClosedSet = false;
    }
}

void APathfinder::ResetDebugAnimationState() {
    DebugAnimationElapsed = 0.0f;
    RevealedSearchEventCount = 0;
    RevealedPathNodeCount = 0;
    bAnimationFinished = false;
    AnimatedOpenSetStates.Empty();
    AnimatedClosedSetStates.Empty();
}

void APathfinder::AddSearchDebugEvent(int32 NodeIndex,
                                      ESearchDebugEventType EventType) {
    FSearchDebugEvent Event;
    Event.NodeIndex = NodeIndex;
    Event.EventType = EventType;
    SearchDebugEvents.Add(Event);
}

void APathfinder::UpdateDebugAnimation(float DeltaTime) {
    if (!bAnimateSearchDebug || !GetWorld() || (!bDrawGridDebug && !bDrawSearchStateDebug &&
                                                !bDrawStartFinishDebug && !bDrawPathDebug)) {
        return;
    }

    if (AnimatedOpenSetStates.Num() != GridNodes.Num()) {
        AnimatedOpenSetStates.Init(false, GridNodes.Num());
    }

    if (AnimatedClosedSetStates.Num() != GridNodes.Num()) {
        AnimatedClosedSetStates.Init(false, GridNodes.Num());
    }

    if (!bAnimationFinished) {
        DebugAnimationElapsed += DeltaTime;

        const int32 TotalAnimatedSteps = SearchDebugEvents.Num() + FinalPath.Num();
        const int32 TargetStepCount = FMath::Min(
            TotalAnimatedSteps,
            FMath::FloorToInt(DebugAnimationElapsed * DebugIterationsPerSecond));

        while (RevealedSearchEventCount < SearchDebugEvents.Num() &&
               RevealedSearchEventCount < TargetStepCount) {
            const FSearchDebugEvent &Event = SearchDebugEvents[RevealedSearchEventCount];
            if (Event.NodeIndex >= 0 && Event.NodeIndex < GridNodes.Num()) {
                if (Event.EventType == ESearchDebugEventType::AddedToOpenSet) {
                    AnimatedOpenSetStates[Event.NodeIndex] = true;
                } else {
                    AnimatedOpenSetStates[Event.NodeIndex] = false;
                    AnimatedClosedSetStates[Event.NodeIndex] = true;
                }
            }

            ++RevealedSearchEventCount;
        }

        const int32 TargetPathNodeCount =
            FMath::Clamp(TargetStepCount - SearchDebugEvents.Num(), 0, FinalPath.Num());
        RevealedPathNodeCount = TargetPathNodeCount;
        bAnimationFinished = TargetStepCount >= TotalAnimatedSteps;
    }

    if (bDrawGridDebug) {
        DrawGridDebug(false, 0.0f);
    }

    if (bDrawSearchStateDebug) {
        DrawSearchStateDebug(false, 0.0f);
    }

    if (bDrawStartFinishDebug && StartNodeIndex != -1 && FinishNodeIndex != -1) {
        DrawStartFinishDebug(false, 0.0f);
    }

    if (bDrawPathDebug && RevealedPathNodeCount > 0) {
        DrawPathDebug(false, 0.0f);
    }
}

void APathfinder::GetNeighborIndices(int32 NodeIndex,
                                    TArray<int32> &OutNeighbors) const {
    OutNeighbors.Reset();

    if (!GridNodes.IsValidIndex(NodeIndex)) {
        return;
    }

    const FGridNode &Node = GridNodes[NodeIndex];

    const int32 X = Node.X;
    const int32 Y = Node.Y;

    const int32 NeighborCoords[4][2] = {
        {X + 1, Y}, {X - 1, Y}, {X, Y + 1}, {X, Y - 1}};

    for (int32 i = 0; i < 4; ++i) {
        const int32 NX = NeighborCoords[i][0];
        const int32 NY = NeighborCoords[i][1];

        if (!IsValidCell(NX, NY)) {
            continue;
        }

        if (!IsCellWalkable(NX, NY)) {
            continue;
        }

        OutNeighbors.Add(GetNodeIndex(NX, NY));
    }
}

bool APathfinder::FindPathDijkstra() {
    if (!GridNodes.IsValidIndex(StartNodeIndex) ||
        !GridNodes.IsValidIndex(FinishNodeIndex)) {
        return false;
    }

    ResetPathfindingState();

    GridNodes[StartNodeIndex].Cost = 0.0f;
    GridNodes[StartNodeIndex].TotalCost = 0.0f;

    TArray<int32> OpenSet;
    OpenSet.Add(StartNodeIndex);
    GridNodes[StartNodeIndex].bInOpenSet = true;
    AddSearchDebugEvent(StartNodeIndex, ESearchDebugEventType::AddedToOpenSet);

    while (!OpenSet.IsEmpty()) {
        const int32 BestOpenSetPos = GetBestOpenSetIndex(OpenSet, false);
        const int32 CurrentIndex = OpenSet[BestOpenSetPos];

        OpenSet.RemoveAt(BestOpenSetPos);

        FGridNode &CurrentNode = GridNodes[CurrentIndex];
        CurrentNode.bInOpenSet = false;

        if (CurrentNode.bVisited) {
            continue;
        }

        CurrentNode.bVisited = true;
        CurrentNode.bInClosedSet = true;
        AddSearchDebugEvent(CurrentIndex, ESearchDebugEventType::AddedToClosedSet);

        if (CurrentIndex == FinishNodeIndex) {
            return ReconstructPath();
        }

        TArray<int32> Neighbors;
        GetNeighborIndices(CurrentIndex, Neighbors);

        for (const int32 NeighborIndex : Neighbors) {
            FGridNode &NeighborNode = GridNodes[NeighborIndex];

            if (NeighborNode.bVisited) {
                continue;
            }

            const float NewCost = CurrentNode.Cost + 1.0f;

            if (NewCost < NeighborNode.Cost) {
                NeighborNode.Cost = NewCost;
                NeighborNode.TotalCost = NewCost;
                NeighborNode.ParentIndex = CurrentIndex;

                if (!NeighborNode.bInOpenSet) {
                    OpenSet.Add(NeighborIndex);
                    NeighborNode.bInOpenSet = true;
                    AddSearchDebugEvent(NeighborIndex,
                                        ESearchDebugEventType::AddedToOpenSet);
                }
            }
        }
    }

    return false;
}

int32 APathfinder::GetBestOpenSetIndex(const TArray<int32> &OpenSet,
                                       bool bUseHeuristic) const {
    int32 BestOpenSetPos = 0;
    int32 CurrentIndex = OpenSet[0];
    float BestCost =
        bUseHeuristic ? GridNodes[CurrentIndex].TotalCost
                      : GridNodes[CurrentIndex].Cost;

    for (int32 i = 1; i < OpenSet.Num(); ++i) {
        const int32 CandidateIndex = OpenSet[i];
        const FGridNode &CandidateNode = GridNodes[CandidateIndex];
        const FGridNode &BestNode = GridNodes[CurrentIndex];
        const float CandidateCost =
            bUseHeuristic ? CandidateNode.TotalCost : CandidateNode.Cost;

        if (CandidateCost < BestCost ||
            (FMath::IsNearlyEqual(CandidateCost, BestCost) &&
             CandidateNode.HeuristicCost < BestNode.HeuristicCost)) {
            BestCost = CandidateCost;
            CurrentIndex = CandidateIndex;
            BestOpenSetPos = i;
        }
    }

    return BestOpenSetPos;
}

float APathfinder::CalculateHeuristicCost(int32 NodeIndex) const {
    if (!GridNodes.IsValidIndex(NodeIndex) ||
        !GridNodes.IsValidIndex(FinishNodeIndex)) {
        return 0.0f;
    }

    const FGridNode &Node = GridNodes[NodeIndex];
    const FGridNode &FinishNode = GridNodes[FinishNodeIndex];

    return FMath::Abs(Node.X - FinishNode.X) + FMath::Abs(Node.Y - FinishNode.Y);
}

bool APathfinder::FindPathAStar() {
    if (!GridNodes.IsValidIndex(StartNodeIndex) ||
        !GridNodes.IsValidIndex(FinishNodeIndex)) {
        return false;
    }

    ResetPathfindingState();

    FGridNode &StartNode = GridNodes[StartNodeIndex];
    StartNode.Cost = 0.0f;
    StartNode.HeuristicCost = CalculateHeuristicCost(StartNodeIndex);
    StartNode.TotalCost = StartNode.Cost + StartNode.HeuristicCost;

    TArray<int32> OpenSet;
    OpenSet.Add(StartNodeIndex);
    StartNode.bInOpenSet = true;
    AddSearchDebugEvent(StartNodeIndex, ESearchDebugEventType::AddedToOpenSet);

    while (!OpenSet.IsEmpty()) {
        const int32 BestOpenSetPos = GetBestOpenSetIndex(OpenSet, true);
        const int32 CurrentIndex = OpenSet[BestOpenSetPos];

        OpenSet.RemoveAt(BestOpenSetPos);

        FGridNode &CurrentNode = GridNodes[CurrentIndex];
        CurrentNode.bInOpenSet = false;

        if (CurrentNode.bVisited) {
            continue;
        }

        CurrentNode.bVisited = true;
        CurrentNode.bInClosedSet = true;
        AddSearchDebugEvent(CurrentIndex, ESearchDebugEventType::AddedToClosedSet);

        if (CurrentIndex == FinishNodeIndex) {
            return ReconstructPath();
        }

        TArray<int32> Neighbors;
        GetNeighborIndices(CurrentIndex, Neighbors);

        for (const int32 NeighborIndex : Neighbors) {
            FGridNode &NeighborNode = GridNodes[NeighborIndex];

            if (NeighborNode.bVisited) {
                continue;
            }

            const float TentativeCost = CurrentNode.Cost + 1.0f;

            if (TentativeCost < NeighborNode.Cost) {
                NeighborNode.Cost = TentativeCost;
                NeighborNode.HeuristicCost =
                    CalculateHeuristicCost(NeighborIndex);
                NeighborNode.TotalCost =
                    TentativeCost + NeighborNode.HeuristicCost;
                NeighborNode.ParentIndex = CurrentIndex;

                if (!NeighborNode.bInOpenSet) {
                    OpenSet.Add(NeighborIndex);
                    NeighborNode.bInOpenSet = true;
                    AddSearchDebugEvent(NeighborIndex,
                                        ESearchDebugEventType::AddedToOpenSet);
                }
            }
        }
    }

    return false;
}

bool APathfinder::ReconstructPath() {
    FinalPath.Empty();

    if (!GridNodes.IsValidIndex(FinishNodeIndex)) {
        return false;
    }

    int32 CurrentIndex = FinishNodeIndex;

    while (CurrentIndex != -1) {
        FinalPath.Add(CurrentIndex);

        if (CurrentIndex == StartNodeIndex) {
            break;
        }

        CurrentIndex = GridNodes[CurrentIndex].ParentIndex;
    }

    if (FinalPath.IsEmpty() || FinalPath.Last() != StartNodeIndex) {
        FinalPath.Empty();
        return false;
    }

    Algo::Reverse(FinalPath);
    return true;
}

void APathfinder::DrawGridDebug(bool bPersistent, float LifeTime) const {
    UWorld *World = GetWorld();
    if (!World) {
        return;
    }

    for (const FGridNode &Node : GridNodes) {
        if (bDrawOnlyBlockedCells && Node.bWalkable) {
            continue;
        }

        const FVector Center = GridToWorld(Node.X, Node.Y);
        const FVector Extent(CellSize * 0.5f, CellSize * 0.5f, 5.0f);
        const FColor Color = Node.bWalkable ? FColor::Silver : FColor::Red;

        DrawDebugBox(World, Center, Extent, Color, bPersistent, LifeTime, 0,
                     2.0f);
    }
}

void APathfinder::DrawSearchStateDebug(bool bPersistent, float LifeTime) const {
    UWorld *World = GetWorld();
    if (!World) {
        return;
    }

    const FVector Extent(CellSize * 0.35f, CellSize * 0.35f, 8.0f);

    for (const FGridNode &Node : GridNodes) {
        if (!Node.bWalkable) {
            continue;
        }

        const bool bInOpenSet = bAnimateSearchDebug
                                    ? AnimatedOpenSetStates.IsValidIndex(GetNodeIndex(Node.X, Node.Y)) &&
                                          AnimatedOpenSetStates[GetNodeIndex(Node.X, Node.Y)]
                                    : Node.bInOpenSet;
        const bool bInClosedSet = bAnimateSearchDebug
                                      ? AnimatedClosedSetStates.IsValidIndex(GetNodeIndex(Node.X, Node.Y)) &&
                                            AnimatedClosedSetStates[GetNodeIndex(Node.X, Node.Y)]
                                      : Node.bInClosedSet;

        if (!bInOpenSet && !bInClosedSet) {
            continue;
        }

        const FVector Center = GridToWorld(Node.X, Node.Y) + FVector(0.0f, 0.0f, 4.0f);
        const FColor Color = bInOpenSet ? FColor::Yellow : FColor(255, 140, 0);

        DrawDebugSolidBox(World, Center, Extent, Color, bPersistent, LifeTime,
                          0);
    }
}

void APathfinder::DrawStartFinishDebug(bool bPersistent, float LifeTime) const {
    UWorld *World = GetWorld();
    if (!World) {
        return;
    }

    if (GridNodes.IsValidIndex(StartNodeIndex)) {
        const FGridNode &StartNode = GridNodes[StartNodeIndex];
        DrawDebugSphere(World, GridToWorld(StartNode.X, StartNode.Y),
                        CellSize * 0.25f, 16, FColor::Green, bPersistent, LifeTime, 0,
                        3.0f);
    }

    if (GridNodes.IsValidIndex(FinishNodeIndex)) {
        const FGridNode &FinishNode = GridNodes[FinishNodeIndex];
        DrawDebugSphere(World, GridToWorld(FinishNode.X, FinishNode.Y),
                        CellSize * 0.25f, 16, FColor::Blue, bPersistent, LifeTime, 0,
                        3.0f);
    }
}

void APathfinder::DrawPathDebug(bool bPersistent, float LifeTime) const {
    UWorld *World = GetWorld();
    if (!World || FinalPath.Num() == 0) {
        return;
    }

    const int32 PathNodeCount = bAnimateSearchDebug
                                    ? FMath::Clamp(RevealedPathNodeCount, 0,
                                                   FinalPath.Num())
                                    : FinalPath.Num();

    for (int32 i = 0; i < PathNodeCount; ++i) {
        const int32 NodeIndex = FinalPath[i];
        if (!GridNodes.IsValidIndex(NodeIndex)) {
            continue;
        }

        const FGridNode &Node = GridNodes[NodeIndex];
        const FVector CurrentPos = GridToWorld(Node.X, Node.Y);

        DrawDebugSphere(World, CurrentPos, CellSize * 0.18f, 12,
                        FColor::Emerald, bPersistent, LifeTime, 0, 3.0f);

        if (i < PathNodeCount - 1) {
            const int32 NextIndex = FinalPath[i + 1];
            if (!GridNodes.IsValidIndex(NextIndex)) {
                continue;
            }

            const FGridNode &NextNode = GridNodes[NextIndex];
            const FVector NextPos = GridToWorld(NextNode.X, NextNode.Y);

            DrawDebugLine(World, CurrentPos, NextPos, FColor::Green, bPersistent,
                          LifeTime, 0, 4.0f);
        }
    }
}
