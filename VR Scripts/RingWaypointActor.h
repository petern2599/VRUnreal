// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "RingWaypointActor.generated.h"

UCLASS()
class ROS_VR_API ARingWaypointActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARingWaypointActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	//virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable, Category = "RingWaypoint")
		void SendDroneToWaypoint(FVector waypointLocation, int uasIndex);
	UFUNCTION(BlueprintCallable, Category = "RingWaypoint")
		void DisableDrone(int uasIndex);

public:
	UPROPERTY()
		class UTopic* VectorTopic;
	UPROPERTY()
		class UTopic* EnableTopic;

};
