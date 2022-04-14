// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "LeftControllerReference.generated.h"

UCLASS()
class ROS_VR_API ALeftControllerReference : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ALeftControllerReference();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	UPROPERTY()
		class UTopic* ControllerRefTopic;

public:	
	// Called every frame
	/*virtual void Tick(float DeltaTime) override;*/
	UFUNCTION(BlueprintCallable, Category = "LeftController")
		void TrackController(float x, float y);

};