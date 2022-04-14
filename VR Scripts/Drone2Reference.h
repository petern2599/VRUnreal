// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "Drone2Reference.generated.h"

UCLASS()
class ROS_VR_API ADrone2Reference : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADrone2Reference();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	UPROPERTY()
		class UTopic* DronePositionTopic2;
	UPROPERTY()
		class UTopic* DroneOffsetTopic2;
	//public:	
		// Called every frame
		//virtual void Tick(float DeltaTime) override;

public:
	UFUNCTION(BlueprintCallable, Category = "Map3D")
		void GetDrone2Location();

};
