// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "DroneTracker.generated.h"

UCLASS()
class ROS_VR_API ADroneTracker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADroneTracker();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	//public:
	//	UPROPERTY()
	//		class UTopic* DroneLocationTopic;
	//	UPROPERTY()
	//		class UTopic* DroneSelectionTopic6;
	//	UPROPERTY()
	//		class UTopic* DroneOffsetTopic;

public:
	UFUNCTION(BlueprintCallable, Category = "DroneTracker")
		void TrackDrone(FVector actorLocation);

	//public:	
	//	// Called every frame
	//	virtual void Tick(float DeltaTime) override;


};
